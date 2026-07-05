"""MoveIt 2 (moveit_py) pick-and-place for the UR5e.

Adds the table and target box to the planning scene, plans a top-down grasp
with OMPL, attaches the object, lifts, moves to a place location, and detaches.
The grasp geometry comes from ``ur5_pick_place.grasp`` (the unit-tested core).

The pick location comes from perception: the detector publishes each colour's
pose on /detected/<colour> (and the selected one on /detected_object_pose).

Two modes, selected by the PICK_MODE environment variable:
    single (default): pick the colour in PICK_COLOR (default "green").
    all:              sort red, then green, then blue onto the conveyor.
The arm returns to its ready posture at the end of either mode.

Run via the launch file so the MoveIt parameters are loaded:
    ros2 launch ur5_pick_place pick_place.launch.py                      # single/green
    PICK_MODE=all ros2 launch ur5_pick_place pick_place.launch.py        # sort all three
    PICK_COLOR=red ros2 launch ur5_pick_place pick_place.launch.py       # single/red
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

# Which coloured part to carry in single mode, e.g. "green". Empty = planning only.
SINGLE_COLOR = os.environ.get("PICK_COLOR", "green")
# "single" picks SINGLE_COLOR; "all" sorts red, then green, then blue.
PICK_MODE = os.environ.get("PICK_MODE", "single")
COLOR_ORDER = ("red", "green", "blue")

# Publisher for /carry_cmd, set up in main().
_carry_pub = None


def _carry_signal(op: str, part_model: str) -> None:
    """Tell the animator to attach/detach a carried part (best effort)."""
    if _carry_pub is None:
        return
    from std_msgs.msg import String

    msg = String()
    msg.data = f"{op}:{part_model}"
    _carry_pub.publish(msg)


def get_perceived_top(topic: str, timeout_s: float = 8.0):
    """Wait for one PoseStamped on ``topic`` and return its (x, y, z). None on timeout."""
    from geometry_msgs.msg import PoseStamped
    from rclpy.qos import DurabilityPolicy, QoSProfile

    qos = QoSProfile(depth=1)
    qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    node = rclpy.create_node("pick_place_perception_client")
    result = {}

    def cb(msg: PoseStamped) -> None:
        result["xyz"] = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)

    # /detected/<color> is latched; /detected_object_pose is not, so subscribe to both compatibly.
    node.create_subscription(PoseStamped, topic, cb, qos if topic.startswith("/detected/") else 10)
    deadline = time.time() + timeout_s
    while rclpy.ok() and "xyz" not in result and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()
    return result.get("xyz")


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


def _build_static_scene(robot: MoveItPy) -> None:
    _apply_object(robot, make_box(TABLE_ID, BASE_FRAME, TABLE_SIZE, TABLE_POSE))
    _apply_object(robot, make_box(CONVEYOR_ID, BASE_FRAME, CONVEYOR_SIZE, CONVEYOR_POSE))


# Keep the base joint in the front hemisphere: the table and belt are both in
# front of the robot, so this forbids the arm swinging the long way round behind
# itself (short-way motion only).
SHOULDER_PAN_LIMIT = 1.7  # radians either side of 0


def _apply_front_constraint(arm) -> None:
    from moveit_msgs.msg import Constraints, JointConstraint

    c = Constraints()
    c.name = "front_only"
    jc = JointConstraint()
    jc.joint_name = "shoulder_pan_joint"
    jc.position = 0.0
    jc.tolerance_above = SHOULDER_PAN_LIMIT
    jc.tolerance_below = SHOULDER_PAN_LIMIT
    jc.weight = 1.0
    c.joint_constraints.append(jc)
    arm.set_path_constraints(c)


def pick_one(robot: MoveItPy, arm, pick_top, part_model: str) -> bool:
    """Pick the part at ``pick_top`` and place it on the conveyor. True on success.

    Assumes the arm starts clear of the table (e.g. at the ready posture) and the
    static scene (table, belt) is already applied.
    """
    half_h = OBJECT_SIZE[2] / 2.0
    object_center = (pick_top[0], pick_top[1], pick_top[2] - half_h)
    grasp_z_offset = half_h + GRASP_CLEARANCE

    _apply_object(robot, make_box(OBJECT_ID, BASE_FRAME, OBJECT_SIZE, object_center))
    time.sleep(0.3)

    grasp = top_down_grasp(object_center, z_offset=grasp_z_offset)
    pre = pregrasp_pose(grasp, STANDOFF)
    lift = retreat_pose(grasp, STANDOFF)
    place = top_down_grasp(PLACE_XYZ, z_offset=grasp_z_offset)
    place_pre = pregrasp_pose(place, STANDOFF)

    for label, gp in (
        ("pre-grasp", make_pose(pre.position, pre.orientation)),
        ("grasp", make_pose(grasp.position, grasp.orientation)),
    ):
        if not _go_to_pose(robot, arm, gp, label):
            return False

    _set_attached(robot, OBJECT_ID, attach=True)
    _allow_collisions(robot, OBJECT_ID, [TABLE_ID, CONVEYOR_ID], allow=True)
    _carry_signal("attach", part_model)
    if not _go_to_pose(robot, arm, make_pose(lift.position, lift.orientation), "lift"):
        return False

    transfer = make_pose(place_pre.position, place_pre.orientation)
    if not _go_to_pose(robot, arm, transfer, "transfer"):
        return False
    if not _go_to_pose(robot, arm, make_pose(place.position, place.orientation), "place"):
        return False
    _carry_signal("detach", part_model)  # drop on the belt; the conveyor carries it away
    _set_attached(robot, OBJECT_ID, attach=False)
    if not _go_to_pose(robot, arm, make_pose(place_pre.position, place_pre.orientation), "retreat"):
        return False

    logger.info(f"{part_model} placed on the conveyor")
    return True


def run_single(robot: MoveItPy, color: str) -> bool:
    """Pick one selected colour and return the arm home."""
    arm = robot.get_planning_component(PLANNING_GROUP)
    _apply_front_constraint(arm)
    if not _go_to_named(robot, arm, READY_STATE, "ready"):
        return False
    _build_static_scene(robot)
    time.sleep(0.3)

    top = get_perceived_top(f"/detected/{color}") or get_perceived_top("/detected_object_pose")
    if top is None:
        logger.warn(f"no perception for '{color}'; using fallback pick")
        top = FALLBACK_PICK_TOP
    logger.info(f"picking '{color}' at {tuple(round(c, 3) for c in top)}")

    ok = pick_one(robot, arm, top, f"part_{color}")
    _go_to_named(robot, arm, READY_STATE, "home")  # return to initial posture
    return ok


def run_all(robot: MoveItPy) -> bool:
    """Sort all three parts onto the conveyor, then return the arm home."""
    arm = robot.get_planning_component(PLANNING_GROUP)
    _apply_front_constraint(arm)
    if not _go_to_named(robot, arm, READY_STATE, "ready"):
        return False
    _build_static_scene(robot)
    time.sleep(0.3)

    all_ok = True
    for color in COLOR_ORDER:
        top = get_perceived_top(f"/detected/{color}")
        if top is None:
            logger.warn(f"'{color}' not detected; skipping")
            all_ok = False
            continue
        logger.info(f"picking '{color}' at {tuple(round(c, 3) for c in top)}")
        # After placing, the next part's pre-grasp takes the arm straight back to
        # the table (no detour through the home pose).
        if not pick_one(robot, arm, top, f"part_{color}"):
            all_ok = False
    _go_to_named(robot, arm, READY_STATE, "home")  # return to initial posture at the end
    return all_ok


def main() -> None:
    global _carry_pub
    rclpy.init()
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from std_msgs.msg import String

    _comm = rclpy.create_node("pick_place_comm")
    qos = QoSProfile(depth=1)
    qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    _carry_pub = _comm.create_publisher(String, "/carry_cmd", qos)

    robot = MoveItPy(node_name="ur5_pick_place")
    logger.info(f"MoveItPy up; mode={PICK_MODE}")
    ok = False
    try:
        ok = run_all(robot) if PICK_MODE == "all" else run_single(robot, SINGLE_COLOR)
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
