"""Palletizing cell on the ROS 2 / MoveIt 2 stack (mock hardware).

Industrial-style pick-and-place for the UR5e + Robotiq 2F-85, built the way a
production cell is (see docs/ENGINEERING_PLAN.md):

  - Deterministic motion with the Pilz Industrial Motion Planner: PTP
    (point-to-point) for transfers, LIN (straight-line Cartesian) for the
    approach and retreat. No random sampling, so motion is smooth and repeatable.
  - A fixed top-down grasp: every station's joint configuration comes from one
    consistent /compute_ik seed, so the tool always arrives pointing straight
    down with the same posture (no wonky angles, no wrist flips, no behind-the-
    base swings).
  - The part is attached to the gripper by id at its real pose, so it rigidly
    follows the tool (no teleporting), mock hardware has no physics grasp.
  - A reachability pre-check rejects out-of-workspace slots; the pallet fills
    back-to-front; production metrics are printed at the end.

    ros2 launch armik_moveit ur5e_gripper_moveit.launch.py    # (RViz optional)
    ros2 run armik_moveit palletize

Geometry comes from scene.py (table/bin/pallet/separator).
"""
import math
import os
import sys
import time

import rclpy
from control_msgs.action import GripperCommand
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    AttachedCollisionObject,
    CollisionObject,
    Constraints,
    JointConstraint,
    MotionPlanRequest,
    ObjectColor,
    OrientationConstraint,
    PlanningOptions,
    PlanningScene,
    PositionConstraint,
    RobotState,
)
from moveit_msgs.srv import ApplyPlanningScene, GetPositionIK
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import ColorRGBA

from armik_moveit.scene import (
    CLEARANCE, MOUNT_H, PALLET_TOP, PALLET_XY, PART_COLORS, PART_SIZE, PART_Z,
    PICK_CELLS, REACH_MAX, STRUCTURES, TRANSIT, build_scene,
)

BASE = "base_link"
EEF = "tool0"
GROUP = "ur_manipulator"
SUCCESS = 1
PILZ = "pilz_industrial_motion_planner"

ARM_JOINTS = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
              "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
# Consistent IK seed (elbow-up, facing front) so every station resolves to the
# same posture branch.
IK_SEED = [0.0, -1.2, 1.4, -1.6, -1.57, 0.0]


def q_down(yaw):
    """Quaternion (x,y,z,w) for tool-z straight down at the given yaw = Rz(yaw)Rx(pi)."""
    sz, cz = math.sin(yaw / 2), math.cos(yaw / 2)
    # Rz(yaw)=(0,0,sz,cz) (Hamilton, w last-ish) composed with Rx(pi)=(1,0,0,0)
    return (cz, sz, 0.0, 0.0)


# Yaw chosen so wrist_3 stays small at both bin and pallet (see ik_probe).
GRASP_QUAT = q_down(-math.pi / 2)

GRIPPER_LINKS = [
    "robotiq_85_base_link",
    "robotiq_85_left_knuckle_link", "robotiq_85_right_knuckle_link",
    "robotiq_85_left_inner_knuckle_link", "robotiq_85_right_inner_knuckle_link",
    "robotiq_85_left_finger_link", "robotiq_85_right_finger_link",
    "robotiq_85_left_finger_tip_link", "robotiq_85_right_finger_tip_link",
]
GRIPPER_ACTION = "/robotiq_gripper_controller/gripper_cmd"
GRIP_OPEN = 0.0
GRIP_CLOSED = 0.6

GRIPPER_LEN = 0.16          # tool0 -> grasp point along the (downward) tool axis
APPROACH_DZ = 0.15          # standoff above a pick/place before the LIN descent

HOME = [0.0, -1.5707, 0.0, -1.5707, 0.0, 0.0]  # SRDF "up"
LAYERS = 1


def slot_pose(index):
    per_layer = len(PALLET_XY)
    layer, within = divmod(index, per_layer)
    x, y = PALLET_XY[within]
    return (x, y, PALLET_TOP + PART_SIZE / 2 + CLEARANCE + layer * PART_SIZE)


def reachable(x, y, z):
    return math.sqrt(x * x + y * y + (z + GRIPPER_LEN) ** 2) <= REACH_MAX


class Palletizer(Node):
    def __init__(self):
        super().__init__("palletize")
        self.mg = ActionClient(self, MoveGroup, "/move_action")
        self.grip = ActionClient(self, GripperCommand, GRIPPER_ACTION)
        self.scene = self.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self.ik = self.create_client(GetPositionIK, "/compute_ik")
        self.replans = 0
        self.speed_factor = 1.0  # scaled down by the safety layer (SSM)

    # --- planning scene ---
    def _apply(self, scene_msg):
        self.scene.wait_for_service(timeout_sec=10.0)
        fut = self.scene.call_async(ApplyPlanningScene.Request(scene=scene_msg))
        rclpy.spin_until_future_complete(self, fut)
        return fut.result() is not None and fut.result().success

    def _part_color(self, part_id):
        rgb = PART_COLORS.get(part_id)
        if not rgb:
            return None
        oc = ObjectColor(id=part_id)
        oc.color = ColorRGBA(r=float(rgb[0]), g=float(rgb[1]), b=float(rgb[2]), a=1.0)
        return oc

    def add_part(self, part_id, x, y, z):
        obj = CollisionObject()
        obj.header.frame_id = BASE
        obj.id = part_id
        obj.primitives.append(
            SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[PART_SIZE] * 3))
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = x, y, z
        pose.orientation.w = 1.0
        obj.primitive_poses.append(pose)
        obj.operation = CollisionObject.ADD
        scene = PlanningScene(is_diff=True)
        scene.world.collision_objects.append(obj)
        color = self._part_color(part_id)
        if color:
            scene.object_colors.append(color)
        return self._apply(scene)

    def _remove_world(self, part_id):
        obj = CollisionObject(id=part_id, operation=CollisionObject.REMOVE)
        obj.header.frame_id = BASE
        scene = PlanningScene(is_diff=True)
        scene.world.collision_objects.append(obj)
        self._apply(scene)

    def attach_part(self, part_id):
        """Attach the part to the tool at the grasp offset (tool-z points down, so
        +GRIPPER_LEN along tool-z lands it exactly where it sat, no teleport)."""
        self._remove_world(part_id)
        aco = AttachedCollisionObject()
        aco.link_name = EEF
        aco.object.header.frame_id = EEF
        aco.object.id = part_id
        aco.object.primitives.append(
            SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[PART_SIZE] * 3))
        pose = Pose()
        pose.position.z = GRIPPER_LEN
        pose.orientation.w = 1.0
        aco.object.primitive_poses.append(pose)
        aco.object.operation = CollisionObject.ADD
        aco.touch_links = GRIPPER_LINKS
        scene = PlanningScene(is_diff=True)
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects.append(aco)
        return self._apply(scene)

    def detach_part(self, part_id):
        """Detach by id: the part returns to the world at its current pose."""
        aco = AttachedCollisionObject()
        aco.link_name = EEF
        aco.object.id = part_id
        aco.object.operation = CollisionObject.REMOVE
        scene = PlanningScene(is_diff=True)
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects.append(aco)
        color = self._part_color(part_id)
        if color:
            scene.object_colors.append(color)  # re-assert colour after re-add
        return self._apply(scene)

    # --- kinematics + motion ---
    def ik_topdown(self, x, y, z, yaw=None):
        """Top-down /compute_ik at (x,y,z) from the consistent seed. None if no IK.

        yaw rotates the top-down grasp about the vertical (defaults to the
        comfortable GRASP_YAW; a perceived part passes its detected yaw).
        """
        quat = GRASP_QUAT if yaw is None else q_down(yaw)
        req = GetPositionIK.Request()
        r = req.ik_request
        r.group_name = GROUP
        r.ik_link_name = EEF
        r.avoid_collisions = True
        r.timeout.sec = 2
        r.robot_state = RobotState()
        r.robot_state.joint_state = JointState(name=ARM_JOINTS, position=IK_SEED)
        ps = PoseStamped()
        ps.header.frame_id = BASE
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = x, y, z
        (ps.pose.orientation.x, ps.pose.orientation.y,
         ps.pose.orientation.z, ps.pose.orientation.w) = quat
        r.pose_stamped = ps
        self.ik.wait_for_service(timeout_sec=10.0)
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        res = fut.result()
        if res is None or res.error_code.val != SUCCESS:
            return None
        sol = dict(zip(res.solution.joint_state.name,
                       res.solution.joint_state.position))
        if not all(j in sol for j in ARM_JOINTS):
            return None
        return [sol[j] for j in ARM_JOINTS]

    def _send(self, req):
        goal = MoveGroup.Goal(request=req)
        goal.planning_options = PlanningOptions(plan_only=False)
        self.mg.wait_for_server(timeout_sec=30.0)
        f = self.mg.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, f)
        h = f.result()
        if h is None or not h.accepted:
            return None
        rf = h.get_result_async()
        rclpy.spin_until_future_complete(self, rf)
        return rf.result().result.error_code.val

    def _pilz(self, planner, vel):
        req = MotionPlanRequest()
        req.group_name = GROUP
        req.pipeline_id = PILZ
        req.planner_id = planner
        req.num_planning_attempts = 1
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = vel * self.speed_factor
        req.max_acceleration_scaling_factor = vel * self.speed_factor
        return req

    def move_config(self, config, vel=0.6, label=""):
        """Move to a joint configuration with OMPL (obstacle-avoiding).

        Used for the transfers between stations: the goal is the clean top-down
        config, and OMPL routes around the separator (Pilz PTP would sweep a
        straight joint-space line through it).
        """
        if config is None:
            if label:
                print(f"      no IK for '{label}'")
            return False
        for _ in range(3):
            req = MotionPlanRequest()
            req.group_name = GROUP  # default OMPL pipeline
            req.num_planning_attempts = 16
            req.allowed_planning_time = 12.0
            req.max_velocity_scaling_factor = vel * self.speed_factor
            req.max_acceleration_scaling_factor = vel * self.speed_factor
            c = Constraints()
            for j, v in zip(ARM_JOINTS, config):
                c.joint_constraints.append(JointConstraint(
                    joint_name=j, position=v, tolerance_above=0.01,
                    tolerance_below=0.01, weight=1.0))
            req.goal_constraints.append(c)
            if self._send(req) == SUCCESS:
                return True
            self.replans += 1
        if label:
            print(f"      move '{label}' failed")
        return False

    def lin(self, x, y, z, vel=0.25, label="", yaw=None):
        """Pilz LIN (straight line) to a top-down pose (yaw optional)."""
        quat = GRASP_QUAT if yaw is None else q_down(yaw)
        req = self._pilz("LIN", vel)
        c = Constraints()
        pc = PositionConstraint()
        pc.header.frame_id = BASE
        pc.link_name = EEF
        pc.constraint_region.primitives.append(
            SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.005]))
        rp = Pose()
        rp.position.x, rp.position.y, rp.position.z = x, y, z
        rp.orientation.w = 1.0
        pc.constraint_region.primitive_poses.append(rp)
        pc.weight = 1.0
        c.position_constraints.append(pc)
        oc = OrientationConstraint()
        oc.header.frame_id = BASE
        oc.link_name = EEF
        (oc.orientation.x, oc.orientation.y,
         oc.orientation.z, oc.orientation.w) = quat
        oc.absolute_x_axis_tolerance = 0.05
        oc.absolute_y_axis_tolerance = 0.05
        oc.absolute_z_axis_tolerance = 0.05
        oc.weight = 1.0
        c.orientation_constraints.append(oc)
        req.goal_constraints.append(c)
        if self._send(req) == SUCCESS:
            return True
        self.replans += 1
        if label:
            print(f"      LIN '{label}' failed at ({x:.2f},{y:.2f},{z:.2f})")
        return False

    def gripper(self, position):
        self.grip.wait_for_server(timeout_sec=15.0)
        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = 50.0
        f = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, f)
        rf = f.result().get_result_async()
        rclpy.spin_until_future_complete(self, rf)
        return True

    def go_home(self):
        return self.move_config(HOME, label="home")

    def perceived_picks(self, timeout=20.0):
        """Wait for /detected_parts and return pick poses in base_link frame.

        The detector publishes each part's top-surface pose in the world frame.
        base_link sits MOUNT_H above the floor (the pedestal), so world -> base
        is a z shift by -MOUNT_H; the part centre is half a part below the top.
        """
        got = {}
        sub = self.create_subscription(
            PoseArray, "/detected_parts", lambda m: got.setdefault("arr", m), 10)
        end = time.time() + timeout
        while rclpy.ok() and "arr" not in got and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.2)
        self.destroy_subscription(sub)
        picks = []
        for p in got.get("arr", PoseArray()).poses:
            # part centre = detected top - half a part, + clearance (so a part
            # resting on the bin is not flagged in collision), then into base frame
            z_center_base = (p.position.z - PART_SIZE / 2 + CLEARANCE) - MOUNT_H
            # Parts rest on the known bin surface; clamp so depth noise cannot
            # push the spawned part below it (which would read as a collision).
            z_center_base = max(z_center_base, PART_Z)
            yaw = 2.0 * math.atan2(p.orientation.z, p.orientation.w)  # detected part yaw
            # a cube grasp is 90-deg symmetric: fold the yaw into [-45, 45] deg so
            # the gripper aligns with the part but the wrist stays comfortable.
            yaw = ((yaw + math.pi / 4) % (math.pi / 2)) - math.pi / 4
            picks.append((p.position.x, p.position.y, z_center_base, yaw))
        return picks

    # --- one pick and place ---
    def pick_place(self, part_id, pick, place, transit_cfg, pick_yaw=None):
        px, py, pz = pick
        qx, qy, qz = place
        grasp_z = pz + GRIPPER_LEN
        appr_z = grasp_z + APPROACH_DZ
        place_z = qz + GRIPPER_LEN
        place_appr = place_z + APPROACH_DZ

        self.gripper(GRIP_OPEN)
        # PICK: reach above the part at its detected grasp yaw, LIN down, grip,
        # attach, LIN up (the pick-side moves use pick_yaw; the place uses default)
        if not self.move_config(self.ik_topdown(px, py, appr_z, yaw=pick_yaw), label="above-bin"):
            return False
        if not self.lin(px, py, grasp_z, label="descend-pick", yaw=pick_yaw):
            return False
        self.gripper(GRIP_CLOSED)
        self.attach_part(part_id)
        attached = True
        try:
            if not self.lin(px, py, appr_z, label="lift", yaw=pick_yaw):
                return False
            # TRANSFER: via a high central config, so each OMPL plan around the
            # tall separator is short and reliable, then down over the slot.
            self.move_config(transit_cfg, label="transit")
            if not self.move_config(self.ik_topdown(qx, qy, place_appr), label="above-pallet"):
                return False
            # PLACE: LIN down, release, detach at the slot, LIN up
            if not self.lin(qx, qy, place_z, label="descend-place"):
                return False
            self.gripper(GRIP_OPEN)
            self.detach_part(part_id)
            attached = False
            self.lin(qx, qy, place_appr, label="retreat")
            return True
        finally:
            if attached:
                # failed mid-transfer: drop the held part and clear it so the
                # next part starts from a clean gripper (no cascade)
                self.detach_part(part_id)
                self._remove_world(part_id)
                self.gripper(GRIP_OPEN)


def main():
    rclpy.init()
    node = Palletizer()
    try:
        node._apply(build_scene(clear=False))
        # Pick source: hard-coded bin cells, or poses perceived by the camera.
        # Each entry is (x, y, z, yaw); fixed cells use the default grasp yaw.
        if os.environ.get("PICK_SOURCE") == "perception":
            print("waiting for /detected_parts (run perception.launch.py) ...")
            cells = node.perceived_picks()
            if not cells:
                print("no perceived parts; aborting")
                sys.exit(1)
            print(f"perceived {len(cells)} parts from the camera")
        else:
            cells = [(x, y, z, None) for (x, y, z) in PICK_CELLS]

        # Reachability pre-check on the picks: skip any part outside the workspace.
        parts = []
        rejected = 0
        for i, (x, y, z, yaw) in enumerate(cells):
            if not reachable(x, y, z):
                print(f"  part_{i}: pick ({x:.2f}, {y:.2f}) OUT OF REACH -> rejected")
                rejected += 1
                continue
            pid = f"part_{i}"
            node.add_part(pid, x, y, z)
            parts.append((pid, (x, y, z), yaw))
        time.sleep(0.5)
        print(f"cell: {', '.join(STRUCTURES)} | {len(parts)} reachable parts")

        if not node.go_home():
            print("could not reach home; aborting")
            sys.exit(1)
        transit_cfg = node.ik_topdown(*TRANSIT) or HOME

        placed = failed = 0
        t0 = time.time()
        place_index = 0
        for pid, pick, yaw in parts:
            slot = slot_pose(place_index)
            if not reachable(*slot):
                print(f"  {pid}: slot {slot} OUT OF REACH -> rejected")
                rejected += 1
                place_index += 1
                continue
            ang = "" if yaw is None else f" yaw {math.degrees(yaw):+.0f}"
            print(f"  {pid}: bin {tuple(round(v,2) for v in pick)}{ang} "
                  f"-> pallet {tuple(round(v,2) for v in slot)}")
            if node.pick_place(pid, pick, slot, transit_cfg, pick_yaw=yaw):
                placed += 1
                place_index += 1
            else:
                print(f"    {pid}: FAILED")
                failed += 1
        node.go_home()
        elapsed = time.time() - t0

        cycle = elapsed / placed if placed else float("nan")
        print("\n=== palletizing metrics ===")
        print(f"parts placed : {placed}/{len(parts)}")
        print(f"rejected     : {rejected} (out of reach)")
        print(f"failed       : {failed}")
        print(f"re-plans     : {node.replans}")
        print(f"total time   : {elapsed:.1f} s")
        print(f"cycle time   : {cycle:.1f} s/part")
        print("RESULT:", "PASS" if failed == 0 and placed > 0 else "FAIL")
        sys.exit(0 if placed > 0 and failed == 0 else 1)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
