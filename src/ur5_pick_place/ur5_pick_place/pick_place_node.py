"""MoveIt 2 (moveit_py) pick-and-place for the UR5e.

Stage 1 of the project: given a fixed object pose, add a table and the object
to the planning scene, plan a top-down grasp with OMPL, attach the object,
lift, move to a place location, and detach. The grasp geometry comes from
``ur5_pick_place.grasp`` (the unit-tested core).

Run via the launch file so the MoveIt parameters are loaded:
    ros2 launch ur5_pick_place pick_place.launch.py
"""
from __future__ import annotations

import time

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

# Fixed scene for Stage 1 (metres, base_link frame).
# The table is a compact pedestal spanning only the pick/place strip so it does
# not envelop the arm at start-up.
TABLE_ID = "table"
TABLE_SIZE = (0.30, 0.55, 0.20)
TABLE_POSE = (0.50, -0.025, 0.10)  # top at z = 0.20

OBJECT_ID = "target_object"
OBJECT_SIZE = (0.05, 0.05, 0.06)
PICK_XYZ = (0.50, 0.15, 0.23)  # sitting on the table top (top at 0.20 + half height)
PLACE_XYZ = (0.50, -0.20, 0.23)

READY_STATE = "up"  # SRDF named state: arm pointing up, clear of the table

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


def run_pick_place(robot: MoveItPy) -> bool:
    """Execute the full fixed-pose pick-and-place. Returns True on success."""
    arm = robot.get_planning_component(PLANNING_GROUP)

    # 0. Move to a clear "ready" posture with an empty scene so the arm is not
    #    intersecting the table when it is added.
    if not _go_to_named(robot, arm, READY_STATE, "ready"):
        return False

    # 1. Build the scene.
    _apply_object(robot, make_box(TABLE_ID, BASE_FRAME, TABLE_SIZE, TABLE_POSE))
    _apply_object(robot, make_box(OBJECT_ID, BASE_FRAME, OBJECT_SIZE, PICK_XYZ))
    time.sleep(0.5)

    # 2. Grasp geometry from the unit-tested core.
    grasp = top_down_grasp(PICK_XYZ, z_offset=GRASP_Z_OFFSET)
    pre = pregrasp_pose(grasp, STANDOFF)
    lift = retreat_pose(grasp, STANDOFF)

    place = top_down_grasp(PLACE_XYZ, z_offset=GRASP_Z_OFFSET)
    place_pre = pregrasp_pose(place, STANDOFF)

    steps = [
        ("pre-grasp", make_pose(pre.position, pre.orientation)),
        ("grasp", make_pose(grasp.position, grasp.orientation)),
    ]
    for label, pose in steps:
        if not _go_to_pose(robot, arm, pose, label):
            return False

    # 3. Attach and lift.
    _set_attached(robot, OBJECT_ID, attach=True)
    if not _go_to_pose(robot, arm, make_pose(lift.position, lift.orientation), "lift"):
        return False

    # 4. Transfer, place, detach.
    transfer_pose = make_pose(place_pre.position, place_pre.orientation)
    if not _go_to_pose(robot, arm, transfer_pose, "transfer"):
        return False
    if not _go_to_pose(robot, arm, make_pose(place.position, place.orientation), "place"):
        return False
    _set_attached(robot, OBJECT_ID, attach=False)
    if not _go_to_pose(robot, arm, make_pose(place_pre.position, place_pre.orientation), "retreat"):
        return False

    logger.info("pick-and-place complete")
    return True


def main() -> None:
    robot = MoveItPy(node_name="ur5_pick_place")
    logger.info("MoveItPy up; starting fixed-pose pick-and-place")
    ok = False
    try:
        ok = run_pick_place(robot)
        logger.info(f"result: {'SUCCESS' if ok else 'FAILURE'}")
    finally:
        # moveit_py can segfault during C++ teardown; guard so it never masks
        # the actual result.
        try:
            time.sleep(1.0)
            robot.shutdown()
        except Exception as exc:  # noqa: BLE001
            logger.warn(f"shutdown raised (ignored): {exc}")


if __name__ == "__main__":
    main()
