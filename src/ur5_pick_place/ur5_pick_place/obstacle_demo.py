"""Stage 2: collision-aware planning around an obstacle, with measured metrics.

Places a thin, tall wall between two end-effector poses. A naive straight-line
move between them would pass through the wall (proved with the tested
``segment_aabb_intersection``); OMPL instead routes the arm around it. The demo
plans the move with and without the wall and reports plan time and joint-space
path length so the detour is quantified.

Run against a live sim:
    ros2 launch ur_simulation_gz ur_sim_moveit.launch.py ur_type:=ur5e
    ros2 launch ur5_pick_place obstacle_demo.launch.py
"""
from __future__ import annotations

import time

from geometry_msgs.msg import PoseStamped
from moveit.planning import MoveItPy
from rclpy.logging import get_logger

from ur5_pick_place.grasp import top_down_grasp
from ur5_pick_place.metrics import joint_path_length, segment_aabb_intersection
from ur5_pick_place.moveit_env import BASE_FRAME, EEF_LINK, PLANNING_GROUP, make_box, make_pose

logger = get_logger("obstacle_demo")

POSE_A = (0.40, 0.25, 0.35)
POSE_B = (0.40, -0.25, 0.35)

WALL_ID = "wall"
WALL_CENTER = (0.40, 0.0, 0.30)
WALL_SIZE = (0.12, 0.02, 0.40)  # thin in y, 0.40 m tall (top at z = 0.50)

READY_STATE = "up"


def _pose_goal(xyz):
    g = top_down_grasp(xyz)
    ps = PoseStamped()
    ps.header.frame_id = BASE_FRAME
    ps.pose = make_pose(g.position, g.orientation)
    return ps


def _plan(arm, goal_ps, attempts=5):
    """Plan to a pose goal; return (trajectory_msg, plan_seconds) or (None, elapsed)."""
    arm.set_start_state_to_current_state()
    arm.set_goal_state(pose_stamped_msg=goal_ps, pose_link=EEF_LINK)
    start = time.perf_counter()
    for _ in range(attempts):
        result = arm.plan()
        if result and result.trajectory is not None:
            elapsed = time.perf_counter() - start
            return result.trajectory, elapsed
    return None, time.perf_counter() - start


def _joint_length(traj) -> float:
    msg = traj.get_robot_trajectory_msg()
    pts = [list(p.positions) for p in msg.joint_trajectory.points]
    return joint_path_length(pts)


def _move_to(robot, arm, goal_ps, label) -> bool:
    traj, _ = _plan(arm, goal_ps)
    if traj is None:
        logger.error(f"[{label}] could not reach start pose")
        return False
    robot.execute(traj, controllers=[])
    time.sleep(0.5)
    return True


def _apply(robot, obj):
    with robot.get_planning_scene_monitor().read_write() as scene:
        scene.apply_collision_object(obj)
        scene.current_state.update()


def _remove(robot, object_id):
    from moveit_msgs.msg import CollisionObject

    obj = CollisionObject()
    obj.id = object_id
    obj.header.frame_id = BASE_FRAME
    obj.operation = CollisionObject.REMOVE
    _apply(robot, obj)


def run_demo(robot: MoveItPy) -> bool:
    arm = robot.get_planning_component(PLANNING_GROUP)

    # Ready posture, empty scene.
    arm.set_start_state_to_current_state()
    arm.set_goal_state(configuration_name=READY_STATE)
    r = arm.plan()
    if r and r.trajectory is not None:
        robot.execute(r.trajectory, controllers=[])
        time.sleep(0.5)

    # Geometric proof: the straight A->B end-effector segment passes through the wall.
    naive_hits = segment_aabb_intersection(POSE_A, POSE_B, WALL_CENTER, WALL_SIZE)
    logger.info(f"naive straight-line A->B intersects wall: {naive_hits}")
    if not naive_hits:
        logger.error("test geometry is wrong: naive line should hit the wall")
        return False

    # Move to A.
    if not _move_to(robot, arm, _pose_goal(POSE_A), "goto-A"):
        return False

    # Baseline: plan A->B with NO obstacle.
    _remove(robot, WALL_ID)
    time.sleep(0.3)
    traj_free, t_free = _plan(arm, _pose_goal(POSE_B))
    if traj_free is None:
        logger.error("baseline plan (no wall) failed")
        return False
    len_free = _joint_length(traj_free)

    # With the wall present, plan A->B again (must route around).
    _apply(robot, make_box(WALL_ID, BASE_FRAME, WALL_SIZE, WALL_CENTER))
    time.sleep(0.3)
    arm.set_start_state_to_current_state()  # still at A
    traj_wall, t_wall = _plan(arm, _pose_goal(POSE_B))
    if traj_wall is None:
        logger.error("obstacle-aware plan (with wall) failed")
        return False
    len_wall = _joint_length(traj_wall)

    # Execute the collision-aware plan so it is visible in the sim.
    robot.execute(traj_wall, controllers=[])
    time.sleep(0.5)

    logger.info("=== Stage 2 measured results ===")
    logger.info("straight-line EE distance A->B (chord): 0.500 m")
    logger.info(f"baseline (no wall)  : plan {t_free:.3f} s, joint path {len_free:.3f} rad")
    logger.info(f"around wall         : plan {t_wall:.3f} s, joint path {len_wall:.3f} rad")
    logger.info(f"detour factor (joint path): {len_wall / len_free:.2f}x")
    logger.info("obstacle-aware plan executed collision-free")
    return True


def main() -> None:
    robot = MoveItPy(node_name="obstacle_demo")
    ok = False
    try:
        ok = run_demo(robot)
        logger.info(f"result: {'SUCCESS' if ok else 'FAILURE'}")
    finally:
        try:
            time.sleep(1.0)
            robot.shutdown()
        except Exception as exc:  # noqa: BLE001
            logger.warn(f"shutdown raised (ignored): {exc}")


if __name__ == "__main__":
    main()
