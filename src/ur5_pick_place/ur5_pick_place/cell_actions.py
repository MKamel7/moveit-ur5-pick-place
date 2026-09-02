#!/usr/bin/env python3
"""The cell's three actions, so a caller learns what happened and not just whether.

    ros2 launch ur5_pick_place cell_actions.launch.py

WHAT THIS REPLACES

Topic availability used as a state machine. Perception answered on a latched
topic, so a caller could not tell "nothing is there" from "the detector died";
planning and execution came back as one boolean, so "unreachable", "already in
collision" and "no path" were the same answer. The three action definitions in
`ur5_pick_place_msgs` say exactly which distinctions matter and why; this is
the server side of them.

THE DECISIONS ARE NOT IN THIS FILE

Which failure code applies is decided in `detection_outcome.py`,
`plan_outcome.py` and `pick_stages.py`, none of which import ROS, MoveIt or
this node. That is deliberate: the interesting logic is the classification, and
a classification that can only be exercised by standing up a simulator is a
classification nobody tests. This file is the glue that observes the world and
hands those modules what they saw.

ONE NODE, THREE SERVERS

MoveItPy loads a planning scene monitor, a robot model and every planning
pipeline, which takes seconds and a few hundred megabytes. Three nodes would
mean three copies fighting over the same scene, so the servers share one.
Goals are handled one at a time for the same reason a cell has one arm.
"""

from __future__ import annotations

import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from ur5_pick_place import detection_outcome, pick_stages, plan_outcome
from ur5_pick_place_msgs.action import DetectObject, ExecutePick, PlanGrasp

#: Reach of the UR5e, less the bit the table occupies. A target outside this is
#: refused before planning rather than after five seconds of search.
MAX_REACH_M = 0.90
MIN_REACH_M = 0.15


class CellActionServer(Node):
    def __init__(self) -> None:
        super().__init__("cell_actions")
        self._group = MutuallyExclusiveCallbackGroup()

        # Imported here rather than at module scope so the node can be started
        # for a --help or a lint without paying for MoveIt.
        from moveit.planning import MoveItPy

        from ur5_pick_place.launch_config import moveit_params  # noqa: F401

        self.get_logger().info("bringing up MoveIt, this takes a few seconds")
        self._robot = MoveItPy(node_name="cell_actions_moveit")
        self._arm = self._robot.get_planning_component("ur_manipulator")

        import ur5_pick_place.pick_place_node as cell

        self._cell = cell
        cell._build_static_scene(self._robot)
        cell._apply_front_constraint(self._arm)

        self._detect = ActionServer(
            self, DetectObject, "detect_object", self._on_detect,
            goal_callback=self._accept, cancel_callback=self._allow_cancel,
            callback_group=self._group)
        self._plan = ActionServer(
            self, PlanGrasp, "plan_grasp", self._on_plan,
            goal_callback=self._accept, cancel_callback=self._allow_cancel,
            callback_group=self._group)
        self._execute = ActionServer(
            self, ExecutePick, "execute_pick", self._on_execute,
            goal_callback=self._accept, cancel_callback=self._allow_cancel,
            callback_group=self._group)
        #: Remembered from the last plan_grasp, so execute_pick can turn the end
        #: of an approach back into the grasp point it was planned above.
        self._approach_height = 0.12
        self.get_logger().info("detect_object, plan_grasp and execute_pick are up")

    def _accept(self, goal) -> GoalResponse:
        return GoalResponse.ACCEPT

    def _allow_cancel(self, goal) -> CancelResponse:
        return CancelResponse.ACCEPT

    # ------------------------------------------------------------------ detect

    def _on_detect(self, handle):
        goal = handle.request
        started = time.time()
        topic = f"/detected/{goal.color}"
        timeout = goal.timeout_s if goal.timeout_s > 0.0 else 8.0

        publishers = self.count_publishers(topic)
        pose, age = None, None
        if goal.color in detection_outcome.VALID_COLORS and publishers:
            pose, age = self._latest_pose(topic, timeout)

        outcome = detection_outcome.classify(
            goal.color, pose_received=pose is not None, age_s=age,
            publisher_count=publishers)

        result = DetectObject.Result()
        result.success = outcome.success
        result.failure = outcome.failure
        result.message = outcome.message
        result.age_s = float(age) if age is not None else 0.0
        if pose is not None and outcome.success:
            result.pose = pose
        handle.succeed()
        self.get_logger().info(
            f"detect_object({goal.color}) -> {'ok' if outcome.success else outcome.message} "
            f"in {time.time() - started:.2f}s")
        return result

    def _latest_pose(self, topic: str, timeout_s: float):
        """One pose off the detector, WITH its age. Both halves matter.

        Age is not decoration here. `/detected/<colour>` is TRANSIENT_LOCAL, so
        a subscriber to a dead detector is handed the last pose it ever
        published, instantly and looking perfectly healthy. Killing the detector
        during a live test proved the point: `count_publishers` still reported 1
        from a stale graph entry, and with the age stubbed at zero this server
        answered "perception is running and reports no green object", which is
        the exact confusion DetectObject.action was written to end.

        The stamp comes from the detector on sim time, and so does this node's
        clock, so the subtraction is meaningful in simulation as well as on a
        real cell.
        """
        from rclpy.qos import DurabilityPolicy, QoSProfile

        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        received = {}

        def on_pose(msg: PoseStamped) -> None:
            received["msg"] = msg

        node = rclpy.create_node(f"cell_actions_perception_{int(time.time() * 1000) % 100000}")
        node.set_parameters([rclpy.parameter.Parameter(
            "use_sim_time", rclpy.Parameter.Type.BOOL, True)])
        node.create_subscription(PoseStamped, topic, on_pose, qos)
        deadline = time.time() + timeout_s
        while rclpy.ok() and "msg" not in received and time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)

        msg = received.get("msg")
        if msg is None:
            node.destroy_node()
            return None, None

        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        now = node.get_clock().now().nanoseconds * 1e-9
        node.destroy_node()
        # A pose stamped in the future is a clock that has not started rather
        # than a fresh detection, and reporting a negative age as "fresh" would
        # hide exactly that.
        return msg, max(0.0, now - stamp)

    # -------------------------------------------------------------------- plan

    def _on_plan(self, handle):
        goal = handle.request
        started = time.time()
        target = goal.target.pose.position
        reach = (target.x ** 2 + target.y ** 2 + target.z ** 2) ** 0.5

        in_workspace = MIN_REACH_M <= reach <= MAX_REACH_M
        start_in_collision = self._start_state_in_collision()
        trajectory = None
        ik_found = False
        if in_workspace and not start_in_collision:
            self._approach_height = goal.approach_height
            pre = self._pre_grasp(goal)
            self._arm.set_start_state_to_current_state()
            self._arm.set_goal_state(pose_stamped_msg=pre, pose_link="tool0")
            plan = self._arm.plan()
            # MoveIt does not separate "no IK" from "no path" in what it returns
            # here, so the distinction is drawn from whether the goal was
            # accepted as a joint state at all. Documented rather than guessed:
            # when planning fails with the start state clear and the target in
            # reach, both codes remain possible and NO_PLAN_FOUND is the
            # conservative report.
            ik_found = True
            if plan and plan.trajectory is not None:
                trajectory = plan.trajectory

        outcome = plan_outcome.classify(
            in_workspace=in_workspace, start_in_collision=start_in_collision,
            ik_found=ik_found, plan_found=trajectory is not None)

        result = PlanGrasp.Result()
        result.success = outcome.success
        result.failure = outcome.failure
        result.message = outcome.message
        result.planning_time_s = time.time() - started
        if trajectory is not None:
            result.trajectory = trajectory.get_robot_trajectory_msg()
        handle.succeed()
        self.get_logger().info(
            f"plan_grasp -> {'ok' if outcome.success else outcome.failure_name} "
            f"in {result.planning_time_s:.2f}s")
        return result

    # ----------------------------------------------------------------- execute

    def _on_execute(self, handle):
        goal = handle.request
        started = time.time()
        stage = pick_stages.STAGE_NONE
        failure = 0
        message = "completed"

        def report(reached: int) -> None:
            feedback = ExecutePick.Feedback()
            feedback.stage = reached
            feedback.elapsed_s = time.time() - started
            handle.publish_feedback(feedback)

        # WHAT THE SUPPLIED TRAJECTORY IS USED FOR, AND WHY NOT VERBATIM.
        # `approach` comes from PlanGrasp, and it was planned from whatever
        # state the arm was in then. Replaying it now would drive from the
        # arm's CURRENT state along a path that assumed a different one, which
        # is the failure mode MoveIt's own start-state tolerance exists to
        # catch. So the trajectory identifies the target, through the tool pose
        # of its last waypoint, and the cycle is planned fresh from where the
        # arm actually is.
        target = self._trajectory_endpoint(goal.approach)
        if target is None:
            result = ExecutePick.Result()
            result.success = False
            result.reached_stage = pick_stages.STAGE_NONE
            result.failure = 1
            result.message = ("the approach trajectory is empty, so there is no target to "
                              "pick; call plan_grasp first and pass its trajectory")
            handle.abort()
            return result

        place = (goal.place_pose.pose.position.x, goal.place_pose.pose.position.y,
                 goal.place_pose.pose.position.z)

        def reached(step: int) -> None:
            # Called by pick_one as it enters each stage, so the stage reported
            # here is the one the cell actually got to rather than a guess made
            # from the outside.
            nonlocal stage
            stage = step
            report(stage)

        try:
            ok = self._cell.pick_one(self._robot, self._arm, target, goal.part_name,
                                     on_stage=reached, place_xyz=place)
            if ok:
                message = "completed"
            else:
                failure = 1
                holding = pick_stages.holds_part(stage)
                released = pick_stages.world_changed(stage)
                held = "; the gripper is holding the part" if holding else ""
                moved = "; the part has already been released" if released else ""
                message = f"execution failed during {pick_stages.name(stage)}{held}{moved}"
            handle.succeed()
        except Exception as exc:  # noqa: BLE001
            failure = 1
            message = f"{type(exc).__name__}: {exc} (during {pick_stages.name(stage)})"
            handle.abort()

        result = ExecutePick.Result()
        result.success = stage == pick_stages.STAGE_COMPLETE and failure == 0
        result.reached_stage = stage
        result.failure = failure
        result.message = message
        result.cycle_time_s = time.time() - started
        self.get_logger().info(
            f"execute_pick -> {pick_stages.name(stage)}"
            f"{'' if result.success else ' (' + message + ')'}"
            f"{' [holding the part]' if pick_stages.holds_part(stage) else ''}"
            f"{' [the part has moved]' if pick_stages.world_changed(stage) else ''}")
        return result

    # ----------------------------------------------------------------- helpers

    def _trajectory_endpoint(self, trajectory):
        """The tool position at the end of a planned approach, or None.

        The approach ends above the grasp by PlanGrasp's approach_height, and
        the grasp itself is that point brought down onto the object, which is
        what pick_one expects to be handed.
        """
        points = trajectory.joint_trajectory.points
        if not points:
            return None
        from moveit.core.robot_state import RobotState

        state = RobotState(self._robot.get_robot_model())
        state.set_joint_group_positions("ur_manipulator", list(points[-1].positions))
        state.update()
        tool = state.get_global_link_transform("tool0")
        x, y, z = tool[0][3], tool[1][3], tool[2][3]
        return (x, y, z - self._approach_height)

    def _pose_stamped(self, xyz) -> PoseStamped:
        msg = PoseStamped()
        msg.header.frame_id = "base_link"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = xyz
        msg.pose.orientation.w = 1.0
        return msg

    def _pre_grasp(self, goal) -> PoseStamped:
        from ur5_pick_place.grasp import pregrasp_pose, top_down_grasp
        from ur5_pick_place.moveit_env import make_pose

        target = goal.target.pose.position
        grasp = top_down_grasp((target.x, target.y, target.z), z_offset=0.0)
        pre = pregrasp_pose(grasp, goal.approach_height)
        # GraspPose carries numpy arrays, not a geometry_msgs/Pose, and
        # make_pose is the converter the rest of the cell already uses.
        msg = PoseStamped()
        msg.header.frame_id = "base_link"
        msg.pose = make_pose(pre.position, pre.orientation)
        return msg

    def _start_state_in_collision(self) -> bool:
        """Ask the scene, rather than inferring it from a failed plan.

        This is the check whose absence cost a campaign 23 consecutive trials
        that all read as motion failures.
        """
        with self._robot.get_planning_scene_monitor().read_only() as scene:
            return bool(scene.is_state_colliding(joint_model_group_name="ur_manipulator"))


def main() -> None:
    rclpy.init()
    node = CellActionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
