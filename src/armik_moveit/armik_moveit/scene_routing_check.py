"""Verify the planning scene forces collision-aware routing (integration test).

Against a live ur5e_gripper_moveit bringup with the cell populated
(populate_scene), this:
  1. confirms the bin/pallet/separator collision objects are in move_group's scene,
  2. homes the arm, moves to a pose above the supply bin,
  3. plans+executes a transfer to above the pallet, with the separator in between
     (success proves OMPL routed around it, since MoveIt only returns
     collision-free plans),
  4. as a control, confirms a goal inside the separator is rejected.

This is the ROS equivalent of the collision-aware routing in the Phase 1 MuJoCo
palletizing cell.

    ros2 launch armik_moveit ur5e_gripper_moveit.launch.py launch_rviz:=false
    ros2 run armik_moveit populate_scene
    ros2 run armik_moveit scene_routing_check
"""
import sys
import time

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    MotionPlanRequest,
    PlanningOptions,
    PositionConstraint,
)
from moveit_msgs.srv import GetPlanningScene
from rclpy.action import ActionClient
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive

GROUP = "ur_manipulator"
EEF = "tool0"
BASE = "base_link"
SUCCESS = 1

HOME = {  # SRDF "up": arm pointing up, clear of the whole cell
    "shoulder_pan_joint": 0.0, "shoulder_lift_joint": -1.5707, "elbow_joint": 0.0,
    "wrist_1_joint": -1.5707, "wrist_2_joint": 0.0, "wrist_3_joint": 0.0,
}


def _point(x, y, z) -> Pose:
    p = Pose()
    p.position.x, p.position.y, p.position.z = x, y, z
    p.orientation.w = 1.0
    return p


class SceneRoutingCheck(Node):
    def __init__(self):
        super().__init__("scene_routing_check")
        self.mg = ActionClient(self, MoveGroup, "/move_action")
        self.scene_cli = self.create_client(GetPlanningScene, "/get_planning_scene")

    def _send(self, req, execute):
        goal = MoveGroup.Goal()
        goal.request = req
        opts = PlanningOptions()
        opts.plan_only = not execute
        goal.planning_options = opts
        self.mg.wait_for_server(timeout_sec=30.0)
        f = self.mg.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, f)
        handle = f.result()
        if handle is None or not handle.accepted:
            return None
        rf = handle.get_result_async()
        rclpy.spin_until_future_complete(self, rf)
        return rf.result().result.error_code.val

    def scene_objects(self):
        self.scene_cli.wait_for_service(timeout_sec=10.0)
        fut = self.scene_cli.call_async(GetPlanningScene.Request())
        rclpy.spin_until_future_complete(self, fut)
        return [o.id for o in fut.result().scene.world.collision_objects]

    def _base_request(self, attempts=20, seconds=30.0):
        req = MotionPlanRequest()
        req.group_name = GROUP
        req.num_planning_attempts = attempts
        req.allowed_planning_time = seconds
        req.max_velocity_scaling_factor = 0.4
        req.max_acceleration_scaling_factor = 0.4
        return req

    def go_home(self):
        req = self._base_request(attempts=10, seconds=15.0)
        c = Constraints()
        for joint, value in HOME.items():
            jc = JointConstraint()
            jc.joint_name = joint
            jc.position = value
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            c.joint_constraints.append(jc)
        req.goal_constraints.append(c)
        return self._send(req, execute=True)

    def go_to_point(self, point, execute):
        req = self._base_request()
        c = Constraints()
        pc = PositionConstraint()
        pc.header.frame_id = BASE
        pc.link_name = EEF
        pc.constraint_region.primitives.append(
            SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.05]))
        region_pose = Pose()
        region_pose.position = point.position
        region_pose.orientation.w = 1.0
        pc.constraint_region.primitive_poses.append(region_pose)
        pc.weight = 1.0
        # Position-only goal: any reaching IK is fine, isolating routing from
        # wrist reachability.
        c.position_constraints.append(pc)
        req.goal_constraints.append(c)
        return self._send(req, execute=execute)


def main():
    rclpy.init()
    node = SceneRoutingCheck()
    try:
        ids = node.scene_objects()
        print("scene objects:", ids)
        have = all(k in ids for k in ("supply_bin", "pallet", "separator"))
        print("bin/pallet/separator present:", have)

        print("reset to home (up) ...")
        home = node.go_home()
        print("  home result:", home)
        time.sleep(1.0)

        print("move to above bin ...")
        above_bin = node.go_to_point(_point(0.47, -0.30, 0.35), execute=True)
        print("  above-bin result:", above_bin)
        time.sleep(1.0)

        print("plan+execute transfer to above pallet (separator in between) ...")
        transfer = node.go_to_point(_point(0.47, 0.30, 0.35), execute=True)
        print("  transfer result:", transfer, "(SUCCESS => routed around separator)")

        print("control: goal inside the separator should be rejected ...")
        in_fixture = node.go_to_point(_point(0.45, 0.0, 0.07), execute=False)
        print("  in-separator goal result:", in_fixture, "(expect not SUCCESS)")

        ok = (have and home == SUCCESS and above_bin == SUCCESS
              and transfer == SUCCESS and in_fixture != SUCCESS)
        print("RESULT:", "PASS" if ok else "FAIL")
        sys.exit(0 if ok else 1)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
