#!/usr/bin/env python3
"""Drive one cycle through the three actions, and print what each one said.

    ros2 launch ur5_pick_place demo_bringup.launch.py gazebo_gui:=false
    ros2 launch ur5_pick_place cell_actions.launch.py
    python3 tools/cell_action_client.py --color green

WHY THIS EXISTS RATHER THAN A README SNIPPET

The point of the typed interfaces is that a caller can tell outcomes apart, and
the only way to show that is to be a caller. This chains detect_object into
plan_grasp into execute_pick, passing the planned trajectory along, and prints
the failure code and message at each step rather than a boolean. It is also the
smoke test: if the servers stop answering in the terms their .action files
promise, running this says so in one screen.
"""

from __future__ import annotations

import argparse
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from ur5_pick_place import detection_outcome, pick_stages, plan_outcome
from ur5_pick_place_msgs.action import DetectObject, ExecutePick, PlanGrasp

DETECT_FAILURES = {
    detection_outcome.NONE: "none",
    detection_outcome.NO_DETECTION: "no detection",
    detection_outcome.PERCEPTION_UNAVAILABLE: "perception unavailable",
    detection_outcome.STALE: "stale pose",
    detection_outcome.INVALID_COLOR: "invalid colour",
}


class CellClient(Node):
    def __init__(self) -> None:
        super().__init__("cell_action_client")
        self.detect = ActionClient(self, DetectObject, "detect_object")
        self.plan = ActionClient(self, PlanGrasp, "plan_grasp")
        self.execute = ActionClient(self, ExecutePick, "execute_pick")

    def call(self, client, goal, name: str, timeout_s: float = 300.0):
        if not client.wait_for_server(timeout_sec=10.0):
            print(f"{name}: no server. Is cell_actions.launch.py running?")
            return None
        send = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send, timeout_sec=30.0)
        handle = send.result()
        if handle is None or not handle.accepted:
            print(f"{name}: goal rejected")
            return None
        result = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result, timeout_sec=timeout_s)
        return None if result.result() is None else result.result().result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--color", default="green")
    parser.add_argument("--place", nargs=3, type=float, default=[0.50, -0.38, 0.175],
                        metavar=("X", "Y", "Z"), help="where to put the part down")
    args = parser.parse_args()

    rclpy.init()
    client = CellClient()

    goal = DetectObject.Goal()
    goal.color = args.color
    goal.timeout_s = 8.0
    detected = client.call(client.detect, goal, "detect_object")
    if detected is None:
        return 2
    print(f"detect_object  success={detected.success} "
          f"failure={DETECT_FAILURES.get(detected.failure, detected.failure)} "
          f"age={detected.age_s:.2f}s  {detected.message}")
    if not detected.success:
        return 1

    plan_goal = PlanGrasp.Goal()
    plan_goal.target = detected.pose
    plan_goal.approach_height = 0.12
    plan_goal.front_constraint = True
    planned = client.call(client.plan, plan_goal, "plan_grasp")
    if planned is None:
        return 2
    outcome = plan_outcome.PlanOutcome(planned.success, planned.failure, planned.message)
    print(f"plan_grasp     success={planned.success} failure={outcome.failure_name} "
          f"in {planned.planning_time_s:.2f}s  {planned.message}")
    if not planned.success:
        return 1

    pick_goal = ExecutePick.Goal()
    pick_goal.part_name = f"part_{args.color}"
    pick_goal.approach = planned.trajectory
    pick_goal.place_pose.header.frame_id = "base_link"
    (pick_goal.place_pose.pose.position.x,
     pick_goal.place_pose.pose.position.y,
     pick_goal.place_pose.pose.position.z) = args.place
    pick_goal.place_pose.pose.orientation.w = 1.0
    picked = client.call(client.execute, pick_goal, "execute_pick")
    if picked is None:
        return 2
    print(f"execute_pick   success={picked.success} "
          f"reached={pick_stages.name(picked.reached_stage)} "
          f"in {picked.cycle_time_s:.1f}s  {picked.message}")
    if not picked.success:
        # The thing a boolean could not tell an operator.
        print(f"               holding the part: {pick_stages.holds_part(picked.reached_stage)}, "
              f"part has moved: {pick_stages.world_changed(picked.reached_stage)}")
        return 1

    rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
