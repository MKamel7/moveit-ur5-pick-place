"""Headless Plan-and-Execute smoke test for the UR5e MoveIt bringup (Phase 2).

Reads the current joint state, asks MoveGroup (OMPL) to plan to a nearby
reachable joint target, executes it on the (mock) hardware, and verifies the
arm actually reached the target. This is the scriptable equivalent of dragging
the RViz interactive marker and hitting Plan & Execute, so the toolchain can be
checked in CI or over SSH without a display.

Run against a live ``ur5e_moveit.launch.py``:
    ros2 run armik_moveit plan_execute_smoke

Exit code 0 on success (planned, executed, reached target), 1 otherwise.
"""
import sys
import time

import rclpy
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MotionPlanRequest, PlanningOptions
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState

GROUP = "ur_manipulator"
JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]
TOLERANCE = 0.05  # rad; execution counts as reached within this of the target


class PlanExecuteSmoke(Node):
    def __init__(self):
        super().__init__("plan_execute_smoke")
        self.current: dict[str, float] = {}
        self.create_subscription(JointState, "/joint_states", self._on_js, 10)
        self.client = ActionClient(self, MoveGroup, "/move_action")

    def _on_js(self, msg: JointState) -> None:
        for name, pos in zip(msg.name, msg.position):
            self.current[name] = pos

    def wait_for_state(self, timeout: float = 15.0) -> bool:
        end = time.time() + timeout
        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if all(j in self.current for j in JOINTS):
                return True
        return False

    def plan_and_execute(self, target: dict[str, float]):
        goal = MoveGroup.Goal()
        req = MotionPlanRequest()
        req.group_name = GROUP
        req.num_planning_attempts = 10
        req.allowed_planning_time = 10.0
        req.max_velocity_scaling_factor = 0.3
        req.max_acceleration_scaling_factor = 0.3

        constraints = Constraints()
        constraints.name = "target"
        for joint in JOINTS:
            jc = JointConstraint()
            jc.joint_name = joint
            jc.position = target[joint]
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        req.goal_constraints.append(constraints)
        goal.request = req

        opts = PlanningOptions()
        opts.plan_only = False  # plan AND execute
        goal.planning_options = opts

        if not self.client.wait_for_server(timeout_sec=30.0):
            self.get_logger().error("move_action server never appeared")
            return None
        send_future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error("goal was rejected")
            return None
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return result_future.result().result


def main() -> None:
    rclpy.init()
    node = PlanExecuteSmoke()
    try:
        if not node.wait_for_state():
            print("RESULT: FAIL (never received a full joint state)")
            sys.exit(1)

        start = {j: node.current[j] for j in JOINTS}
        # Nearby, reachable, collision-free target: nudge the first three joints.
        target = dict(start)
        target["shoulder_pan_joint"] = start["shoulder_pan_joint"] + 0.4
        target["shoulder_lift_joint"] = -1.2
        target["elbow_joint"] = 0.8
        print("start :", {j: round(start[j], 3) for j in JOINTS})
        print("target:", {j: round(target[j], 3) for j in JOINTS})

        result = node.plan_and_execute(target)
        if result is None:
            print("RESULT: FAIL (no result from move_group)")
            sys.exit(1)

        code = result.error_code.val
        print(f"MoveItErrorCode: {code} (1 == SUCCESS)")

        settle = time.time() + 6.0
        while rclpy.ok() and time.time() < settle:
            rclpy.spin_once(node, timeout_sec=0.1)
        reached = {j: node.current[j] for j in JOINTS}
        max_err = max(abs(reached[j] - target[j]) for j in JOINTS)
        print("reached:", {j: round(reached[j], 3) for j in JOINTS})
        print(f"max joint error vs target: {max_err:.4f} rad")

        ok = code == 1 and max_err < TOLERANCE
        print("RESULT:", "PASS" if ok else "FAIL")
        sys.exit(0 if ok else 1)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
