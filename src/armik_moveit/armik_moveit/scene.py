"""The palletizing cell: geometry and MoveIt planning-scene population.

A proper workcell sitting on a table in the arm's base_link frame: a table
surface, a supply bin and a pallet as distinct platforms, a separator wall
standing between them, and coloured parts in the bin. This module is the single
source of the cell geometry, the palletizing node imports from here.

    ros2 run armik_moveit populate_scene         # add the cell (default)
    ros2 run armik_moveit populate_scene --clear # remove it again
"""
import math
import sys

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, ObjectColor, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import ColorRGBA

BASE_FRAME = "base_link"

# The arm is mounted on a pedestal (see ur5e_robotiq.urdf.xacro); base_link is
# MOUNT_H above the floor. The cell geometry below is written floor-referenced
# (table top ~ 0) and dropped by MOUNT_H into base_link frame, so the arm works
# downward over the cell. MOUNT_H must match the pedestal height in the xacro.
MOUNT_H = 0.40

# --- cell geometry: id -> (size_xyz, center_xyz, rgb), floor-referenced (metres) ---
# Table top sits at floor z = 0; everything else rests on the table. x starts
# well in front of the base so the table never collides with the arm's base.
TABLE = ((0.66, 1.20, 0.04), (0.47, 0.0, -0.02), (0.55, 0.55, 0.58))
SUPPLY_BIN = ((0.26, 0.24, 0.08), (0.47, -0.32, 0.04), (0.30, 0.42, 0.60))
PALLET = ((0.34, 0.34, 0.06), (0.47, 0.36, 0.03), (0.62, 0.46, 0.26))
# The separator wall between the bin (-y) and the pallet (+y). Taller now (0.28 m
# wall); the bin and pallet are pushed further apart so the arm still clears it
# in the top-down placement posture.
SEPARATOR = ((0.34, 0.04, 0.28), (0.45, 0.0, 0.14), (0.90, 0.50, 0.10))

STRUCTURES = {
    "table": TABLE,
    "supply_bin": SUPPLY_BIN,
    "pallet": PALLET,
    "separator": SEPARATOR,
}

# Parts (0.04 m cubes) rest on the supply-bin top (z 0.08 + half part). A small
# clearance keeps them just above the surface so a part resting on the bin/pallet
# is not flagged as a collision once it is attached to the gripper.
PART_SIZE = 0.04
CLEARANCE = 0.006
BIN_TOP = SUPPLY_BIN[1][2] + SUPPLY_BIN[0][2] / 2      # 0.08 (floor-referenced)
PART_Z = BIN_TOP + PART_SIZE / 2 + CLEARANCE - MOUNT_H  # base_link frame
PART_COLORS = {
    "part_0": (0.85, 0.15, 0.15), "part_1": (0.15, 0.70, 0.20),
    "part_2": (0.15, 0.35, 0.85), "part_3": (0.90, 0.75, 0.10),
    "part_red": (0.85, 0.15, 0.15), "part_green": (0.15, 0.70, 0.20),
    "part_blue": (0.15, 0.35, 0.85),
}

# --- colour-sorting cell -----------------------------------------------------
# One infeed bin and THREE outfeed belts, one per colour, so a sorted part
# actually leaves on its own lane. The lanes fan out radially from the robot
# rather than sitting side by side: that is how a robotic divert sorter is
# built, with the arm at the hub, each lane carrying its colour away in a
# different direction to a different downstream station. Side by side lanes
# would waste the arm's reach on one side and put three belts in a line where a
# factory would have three destinations.
#
# All three drop points sit on an arc LANE_DROP_R from the base, comfortably
# inside the UR5e's REACH_MAX, and the fan is placed clear of the bin (which
# occupies roughly -34 deg).
SORT_COLOURS = ["red", "green", "blue"]

LANE_ANGLES_DEG = {"red": 0.0, "green": 35.0, "blue": 70.0}
LANE_DROP_R = 0.62          # radius of the place point on each lane
LANE_LEN = 0.50             # belt length, radially outward
LANE_WIDTH = 0.18
LANE_HEIGHT = 0.14          # top at floor z = 0.14, same as the old single belt
LANE_CENTRE_R = 0.66        # belt centre, so the drop sits in the inner third
# The belts run past the table edge and continue on the floor. Both surfaces
# are at z = 0, so a lane leaving the cell reads as a belt carrying parts away
# to a downstream station rather than a block sitting on a bench.
# Lane tints are deliberately DARK. The red lane sits inside the RGB-D camera's
# field of view, and detector.py segments parts by colour: a saturated belt would
# be a very large blob passing the same test as a 40 mm part and would swamp the
# detection. These read as red, green and blue lanes to a viewer while staying
# well below the detector's brightness bounds (r/g/b > 120, 100, 120).
LANE_RGB = {"red": (0.24, 0.08, 0.08),
            "green": (0.08, 0.22, 0.10),
            "blue": (0.08, 0.10, 0.26)}


def lane_axis(colour):
    """Unit vector pointing outward along a lane (the travel direction)."""
    a = math.radians(LANE_ANGLES_DEG[colour])
    return (math.cos(a), math.sin(a))


def lane_drop(colour):
    """Where the arm places this colour, in base_link x, y."""
    ax, ay = lane_axis(colour)
    return (LANE_DROP_R * ax, LANE_DROP_R * ay)


def lane_end(colour):
    """Far end of the belt, where a conveyed part leaves the cell."""
    ax, ay = lane_axis(colour)
    r = LANE_CENTRE_R + LANE_LEN / 2
    return (r * ax, r * ay)


# id -> (size, centre, rgb, yaw). Yaw turns each belt to run along its lane.
SORT_LANES = {
    f"conveyor_{c}": (
        (LANE_LEN, LANE_WIDTH, LANE_HEIGHT),
        (LANE_CENTRE_R * lane_axis(c)[0], LANE_CENTRE_R * lane_axis(c)[1],
         LANE_HEIGHT / 2),
        LANE_RGB[c],
        math.radians(LANE_ANGLES_DEG[c]),
    )
    for c in SORT_COLOURS
}

# The sorting cell gets its own table: the three lanes reach further out and
# further round than the palletizing layout, and TABLE is shared with the
# palletizing cell, which must not move. Kept clear of the pedestal (which
# occupies |x| <= 0.10, |y| <= 0.10) so the slab never collides with the base.
SORT_TABLE = ((0.82, 1.34, 0.04), (0.57, 0.13, -0.02), (0.55, 0.55, 0.58))

SORT_STRUCTURES = {"table": SORT_TABLE, "supply_bin": SUPPLY_BIN}
SORT_STRUCTURES.update(SORT_LANES)
SORT_PARTS = {  # legacy fixed layout (kept for reference)
    "red": (0.41, -0.38), "green": (0.47, -0.32), "blue": (0.53, -0.26),
}
# Random spawn region on the bin top (x_min, x_max, y_min, y_max), with a margin
# inside the bin footprint so parts land fully on the surface.
BIN_AREA = (0.40, 0.55, -0.41, -0.23)
CONVEYOR_TOP = LANE_HEIGHT - MOUNT_H  # belt top, base_link frame


def _grid(cx, cy, nx, ny, sx, sy):
    return [(cx + (i - (nx - 1) / 2) * sx, cy + (j - (ny - 1) / 2) * sy)
            for j in range(ny) for i in range(nx)]


# Pick cells on the bin, pallet slots (filled back-to-front), transit waypoint.
PICK_CELLS = [(x, y, PART_Z) for x, y in _grid(0.47, -0.32, 2, 2, 0.12, 0.12)]
PALLET_TOP = PALLET[1][2] + PALLET[0][2] / 2 - MOUNT_H  # base_link frame
PALLET_XY = sorted(_grid(0.47, 0.36, 2, 2, 0.15, 0.15), key=lambda p: (-p[1], p[0]))
# High central waypoint clear of the separator wall (floor z 0.52).
TRANSIT = (0.30, 0.0, 0.52 - MOUNT_H)
REACH_MAX = 0.82


def _box(object_id, size, center, yaw=0.0):
    obj = CollisionObject()
    obj.header.frame_id = BASE_FRAME
    obj.id = object_id
    prim = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[float(s) for s in size])
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = (float(c) for c in center)
    pose.orientation.z = math.sin(yaw / 2.0)
    pose.orientation.w = math.cos(yaw / 2.0)
    obj.primitives.append(prim)
    obj.primitive_poses.append(pose)
    obj.operation = CollisionObject.ADD
    return obj


def _color(object_id, rgb, alpha=1.0):
    oc = ObjectColor()
    oc.id = object_id
    oc.color = ColorRGBA(r=float(rgb[0]), g=float(rgb[1]), b=float(rgb[2]), a=alpha)
    return oc


def _structures_scene(structures, clear):
    scene = PlanningScene()
    scene.is_diff = True
    for object_id, entry in structures.items():
        # Most structures are axis aligned (size, centre, rgb); the sorting
        # lanes carry a fourth item, the yaw that turns them along the lane.
        size, center, rgb = entry[0], entry[1], entry[2]
        yaw = entry[3] if len(entry) > 3 else 0.0
        cx, cy, cz = center
        obj = _box(object_id, size, (cx, cy, cz - MOUNT_H), yaw)  # into base_link
        obj.operation = CollisionObject.REMOVE if clear else CollisionObject.ADD
        scene.world.collision_objects.append(obj)
        if not clear:
            scene.object_colors.append(_color(object_id, rgb))
    return scene


def build_scene(clear):
    """PlanningScene diff for the palletizing cell (table/bin/pallet/separator)."""
    return _structures_scene(STRUCTURES, clear)


def build_sort_scene(clear):
    """PlanningScene diff for the colour-sorting cell (table/bin/conveyor)."""
    return _structures_scene(SORT_STRUCTURES, clear)


def main():
    clear = "--clear" in sys.argv
    rclpy.init()
    node = Node("populate_scene")
    client = node.create_client(ApplyPlanningScene, "/apply_planning_scene")
    if not client.wait_for_service(timeout_sec=15.0):
        node.get_logger().error("/apply_planning_scene not available (is move_group up?)")
        rclpy.shutdown()
        sys.exit(1)
    req = ApplyPlanningScene.Request(scene=build_scene(clear))
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    ok = future.result() is not None and future.result().success
    print(f"planning scene {'cleared' if clear else 'added'}: "
          f"{'OK' if ok else 'FAILED'} ({', '.join(STRUCTURES)})")
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
