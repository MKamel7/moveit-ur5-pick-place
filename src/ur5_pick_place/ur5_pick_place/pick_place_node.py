"""MoveIt 2 (moveit_py) pick-and-place for the UR5e.

Adds the table and target box to the planning scene, plans a top-down grasp
with OMPL, attaches the object, lifts, moves to a place location, and detaches.
The grasp geometry comes from ``ur5_pick_place.grasp`` (the unit-tested core).

The pick location comes from perception: it waits for a pose on
/detected_object_pose (published by the detector from the RGB-D camera). If no
detection arrives within the timeout it falls back to a fixed pose so the node
can also run against the plain (camera-less) sim.

Run via the launch file so the MoveIt parameters are loaded:
    ros2 launch ur5_pick_place pick_place.launch.py
"""
from __future__ import annotations

import os
import time

import rclpy
from moveit.planning import MoveItPy
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
from rclpy.logging import get_logger

from ur5_pick_place.grasp import pregrasp_pose, retreat_pose, top_down_grasp
from ur5_pick_place.moveit_env import (
    BASE_FRAME,
    EEF_LINK,
    PLANNING_GROUP,
    make_box,
    make_pose,
)

logger = get_logger("pick_place")

# Scene matches the world SDF (metres, base_link frame).
TABLE_ID = "table"
TABLE_SIZE = (0.40, 0.55, 0.20)
TABLE_POSE = (0.55, 0.15, 0.10)  # top at z = 0.20

CONVEYOR_ID = "conveyor"
CONVEYOR_SIZE = (0.30, 0.80, 0.15)
CONVEYOR_POSE = (0.50, -0.38, 0.075)  # top at z = 0.15

OBJECT_ID = "target_object"
OBJECT_SIZE = (0.05, 0.05, 0.05)
# Fallback pick top-centre if perception is unavailable (object top surface).
FALLBACK_PICK_TOP = (0.55, 0.17, 0.25)
# Object centre when placed on the conveyor belt (belt top 0.15 + half height).
PLACE_XYZ = (0.50, -0.38, 0.175)

READY_STATE = "up"  # SRDF named state: arm pointing up, clear of the table

# Grasp at the object's top surface plus a small clearance.
GRASP_CLEARANCE = 0.005
STANDOFF = 0.12  # pre-grasp / retreat height above the grasp

# Gazebo model name of the part being carried, e.g. "part_green". When set, the
# pick-and-place signals the part_animator over /carry_cmd so the part visibly
# follows the gripper and then rides the conveyor. Empty = planning only.
PART_MODEL = os.environ.get("PICK_PART_MODEL", "")

# Publisher for /carry_cmd, set up in main().
_carry_pub = None


def _carry_signal(op: str) -> None:
    """Tell the animator to attach/detach the carried part (best effort)."""
    if not PART_MODEL or _carry_pub is None:
        return
    from std_msgs.msg import String

    msg = String()
    msg.data = f"{op}:{PART_MODEL}"
    _carry_pub.publish(msg)


def get_perceived_pick_top(timeout_s: float = 10.0):
    """Wait for one /detected_object_pose and return its (x, y, z) top-centre.

    Returns None if no detection arrives within the timeout.
    """
    from geometry_msgs.msg import PoseStamped

    node = rclpy.create_node("pick_place_perception_client")
    result = {}

    def cb(msg: PoseStamped) -> None:
        result["xyz"] = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)

    node.create_subscription(PoseStamped, "/detected_object_pose", cb, 10)
    deadline = time.time() + timeout_s
    while rclpy.ok() and "xyz" not in result and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()
    return result.get("xyz")

# Grasp at the object's top surface (half height + a small clearance) so the
# flange rests on top of the box rather than penetrating the collision volume.
GRASP_Z_OFFSET = OBJECT_SIZE[2] / 2.0 + 0.005
STANDOFF = 0.12  # pre-grasp / retreat height above the grasp


def _plan_and_execute(robot: MoveItPy, arm, label: str, attempts: int = 3) -> bool:
    """Plan to the goal already set on ``arm`` and execute it, with retries."""
    for attempt in range(1, attempts + 1):
        arm.set_start_state_to_current_state()
        result = arm.plan()
        if result and result.trajectory is not None:
            logger.info(f"[{label}] planned on attempt {attempt}; executing")
            robot.execute(result.trajectory, controllers=[])
            time.sleep(0.5)
            return True
        logger.warn(f"[{label}] planning failed on attempt {attempt}/{attempts}")
        time.sleep(0.2)
    logger.error(f"[{label}] gave up after {attempts} attempts")
    return False


def _go_to_pose(robot: MoveItPy, arm, pose_msg, label: str) -> bool:
    from geometry_msgs.msg import PoseStamped

    goal = PoseStamped()
    goal.header.frame_id = BASE_FRAME
    goal.pose = pose_msg
    arm.set_goal_state(pose_stamped_msg=goal, pose_link=EEF_LINK)
    return _plan_and_execute(robot, arm, label)


def _go_to_named(robot: MoveItPy, arm, name: str, label: str) -> bool:
    arm.set_start_state_to_current_state()
    arm.set_goal_state(configuration_name=name)
    return _plan_and_execute(robot, arm, label)


def _apply_object(robot: MoveItPy, obj: CollisionObject) -> None:
    with robot.get_planning_scene_monitor().read_write() as scene:
        scene.apply_collision_object(obj)
        scene.current_state.update()


def _set_attached(robot: MoveItPy, object_id: str, attach: bool) -> None:
    aco = AttachedCollisionObject()
    aco.link_name = EEF_LINK
    aco.object.id = object_id
    aco.object.operation = CollisionObject.ADD if attach else CollisionObject.REMOVE
    aco.touch_links = [EEF_LINK, "wrist_3_link"]
    with robot.get_planning_scene_monitor().read_write() as scene:
        scene.process_attached_collision_object(aco)
        scene.current_state.update()


def _allow_collisions(robot: MoveItPy, object_id: str, surface_ids, allow: bool) -> None:
    """Allow (or disallow) collision between a held object and support surfaces.

    A just-grasped part still rests on the table, and a part being placed
    touches the belt, so those specific contacts must be permitted or the
    start-state collision check aborts the plan.
    """
    with robot.get_planning_scene_monitor().read_write() as scene:
        acm = scene.allowed_collision_matrix
        for sid in surface_ids:
            acm.set_entry(object_id, sid, allow)
        scene.current_state.update()


def run_pick_place(robot: MoveItPy, pick_top: tuple[float, float, float]) -> bool:
    """Execute the full pick-and-place given the object top-centre. True on success."""
    arm = robot.get_planning_component(PLANNING_GROUP)

    half_h = OBJECT_SIZE[2] / 2.0
    object_center = (pick_top[0], pick_top[1], pick_top[2] - half_h)
    grasp_z_offset = half_h + GRASP_CLEARANCE  # grasp tool0 at the top surface + clearance

    # 0. Move to a clear "ready" posture with an empty scene so the arm is not
    #    intersecting the table when it is added.
    if not _go_to_named(robot, arm, READY_STATE, "ready"):
        return False

    # 1. Build the scene (source table, conveyor belt, and the perceived object).
    _apply_object(robot, make_box(TABLE_ID, BASE_FRAME, TABLE_SIZE, TABLE_POSE))
    _apply_object(robot, make_box(CONVEYOR_ID, BASE_FRAME, CONVEYOR_SIZE, CONVEYOR_POSE))
    _apply_object(robot, make_box(OBJECT_ID, BASE_FRAME, OBJECT_SIZE, object_center))
    time.sleep(0.5)

    # 2. Grasp geometry from the unit-tested core.
    grasp = top_down_grasp(object_center, z_offset=grasp_z_offset)
    pre = pregrasp_pose(grasp, STANDOFF)
    lift = retreat_pose(grasp, STANDOFF)

    place = top_down_grasp(PLACE_XYZ, z_offset=grasp_z_offset)
    place_pre = pregrasp_pose(place, STANDOFF)

    steps = [
        ("pre-grasp", make_pose(pre.position, pre.orientation)),
        ("grasp", make_pose(grasp.position, grasp.orientation)),
    ]
    for label, pose in steps:
        if not _go_to_pose(robot, arm, pose, label):
            return False

    # 3. Attach and lift. The part still touches the table, so allow that
    #    contact (and the belt contact used at placing) before planning. Signal
    #    the animator so the gz part follows the gripper from here.
    _set_attached(robot, OBJECT_ID, attach=True)
    _allow_collisions(robot, OBJECT_ID, [TABLE_ID, CONVEYOR_ID], allow=True)
    _carry_signal("attach")
    if not _go_to_pose(robot, arm, make_pose(lift.position, lift.orientation), "lift"):
        return False

    # 4. Transfer to the conveyor, place, detach.
    transfer_pose = make_pose(place_pre.position, place_pre.orientation)
    if not _go_to_pose(robot, arm, transfer_pose, "transfer"):
        return False
    if not _go_to_pose(robot, arm, make_pose(place.position, place.orientation), "place"):
        return False
    _carry_signal("detach")  # drop it on the conveyor; the belt carries it away
    _set_attached(robot, OBJECT_ID, attach=False)
    if not _go_to_pose(robot, arm, make_pose(place_pre.position, place_pre.orientation), "retreat"):
        return False

    logger.info("pick-and-place complete: part placed on the conveyor")
    return True


def main() -> None:
    global _carry_pub
    rclpy.init()
    if PART_MODEL:
        from rclpy.qos import DurabilityPolicy, QoSProfile
        from std_msgs.msg import String

        _comm = rclpy.create_node("pick_place_comm")
        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        _carry_pub = _comm.create_publisher(String, "/carry_cmd", qos)

    pick_top = get_perceived_pick_top(timeout_s=10.0)
    if pick_top is None:
        logger.warn(f"no perception detection; using fallback pick {FALLBACK_PICK_TOP}")
        pick_top = FALLBACK_PICK_TOP
    else:
        logger.info(f"perceived pick top-centre: {tuple(round(c, 3) for c in pick_top)}")

    robot = MoveItPy(node_name="ur5_pick_place")
    logger.info("MoveItPy up; starting perception-driven pick-and-place")
    ok = False
    try:
        ok = run_pick_place(robot, pick_top)
        logger.info(f"result: {'SUCCESS' if ok else 'FAILURE'}")
    finally:
        # moveit_py can segfault during C++ teardown; guard so it never masks
        # the actual result.
        try:
            time.sleep(1.0)
            robot.shutdown()
        except Exception as exc:  # noqa: BLE001
            logger.warn(f"shutdown raised (ignored): {exc}")
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
