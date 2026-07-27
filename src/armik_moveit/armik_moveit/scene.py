"""The palletizing cell: geometry and MoveIt planning-scene population.

A proper workcell sitting on a table in the arm's base_link frame: a table
surface, a supply bin and a pallet as distinct platforms, a separator wall
standing between them, and coloured parts in the bin. This module is the single
source of the cell geometry, the palletizing node imports from here.

    ros2 run armik_moveit populate_scene         # add the cell (default)
    ros2 run armik_moveit populate_scene --clear # remove it again
"""
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

# --- colour-sorting cell: the same table + supply bin, but the place target is a
# conveyor belt. Three colour-coded parts sit on the bin; a button picks one.
CONVEYOR = ((0.24, 0.44, 0.14), (0.47, 0.36, 0.07), (0.16, 0.16, 0.18))
SORT_STRUCTURES = {"table": TABLE, "supply_bin": SUPPLY_BIN, "conveyor": CONVEYOR}
SORT_PARTS = {  # colour -> (x, y) on the bin top
    "red": (0.41, -0.38), "green": (0.47, -0.32), "blue": (0.53, -0.26),
}
CONVEYOR_TOP = CONVEYOR[1][2] + CONVEYOR[0][2] / 2 - MOUNT_H  # base_link frame
CONVEYOR_DROP = (0.47, 0.36)  # place x,y on the belt


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


def _box(object_id, size, center):
    obj = CollisionObject()
    obj.header.frame_id = BASE_FRAME
    obj.id = object_id
    prim = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[float(s) for s in size])
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = (float(c) for c in center)
    pose.orientation.w = 1.0
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
    for object_id, (size, center, rgb) in structures.items():
        cx, cy, cz = center
        obj = _box(object_id, size, (cx, cy, cz - MOUNT_H))  # drop into base_link frame
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
