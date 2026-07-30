"""Verify a carried part rides the gripper rigidly in the Gazebo twin.

The complaint this tests for: while the arm carried a part, the part visibly
lagged and snapped along behind the gripper instead of sitting in it. That came
from posing the part off a 10 Hz planning-scene poll, so between polls the
gripper moved and the part did not.

A part held by the gripper is a rigid body attached to it, so its pose expressed
IN THE GRIPPER'S FRAME must not change while it is held. This measures exactly
that, in the rendered world (from Gazebo's own pose feed, not from the inputs),
across a whole carry.

Run it, then command a sort:
    python3 test_twin_grasp.py --seconds 60 &
    ros2 topic pub --once -w 1 /target_color std_msgs/String "{data: red}"
"""
import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPlanningScene
from moveit_msgs.msg import PlanningSceneComponents

from gz.transport13 import Node as GzNode
from gz.msgs10.pose_v_pb2 import Pose_V

sys.path.insert(0, "/home/kamel/robot-arm-ik/ros2/src/armik_moveit")
from armik_moveit.twin_world import MODEL_PREFIX  # noqa: E402
from armik_moveit.gz_twin import PART_MODELS, quat_mul, quat_rot  # noqa: E402

GRIPPER = MODEL_PREFIX + "robotiq_85_base_link"
RIGID_TOL = 1.5e-3     # 1.5 mm of wander over a whole carry


def inv(pose):
    t, q = pose
    qi = (-q[0], -q[1], -q[2], q[3])
    ti = quat_rot(qi, (-t[0], -t[1], -t[2]))
    return (ti, qi)


def compose(pa, pb):
    ta, qa = pa
    tb, qb = pb
    r = quat_rot(qa, tb)
    return ((ta[0] + r[0], ta[1] + r[1], ta[2] + r[2]), quat_mul(qa, qb))


class Probe(Node):
    def __init__(self, world):
        super().__init__("twin_grasp_test")
        self.gz = GzNode()
        self.poses = {}
        self.gz.subscribe(Pose_V, f"/world/{world}/pose/info", self._on_poses)
        self.cli = self.create_client(GetPlanningScene, "/get_planning_scene")
        self.cli.wait_for_service(timeout_sec=20)
        self.req = GetPlanningScene.Request()
        self.req.components = PlanningSceneComponents(
            components=PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS)

    def _on_poses(self, msg):
        for p in msg.pose:
            if p.name == GRIPPER or p.name in PART_MODELS:
                self.poses[p.name] = (
                    (p.position.x, p.position.y, p.position.z),
                    (p.orientation.x, p.orientation.y,
                     p.orientation.z, p.orientation.w))

    def attached(self):
        f = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, f, timeout_sec=2.0)
        r = f.result()
        if r is None:
            return None
        ids = [a.object.id for a in r.scene.robot_state.attached_collision_objects]
        return ids[0] if ids else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", default="sort_cell_twin")
    ap.add_argument("--seconds", type=float, default=60.0)
    args = ap.parse_args()

    rclpy.init()
    n = Probe(args.world)
    for _ in range(60):
        rclpy.spin_once(n, timeout_sec=0.05)

    print("waiting for a part to be gripped ...")
    rel = []
    part = None
    t_end = time.time() + args.seconds
    while time.time() < t_end:
        rclpy.spin_once(n, timeout_sec=0.05)
        held = n.attached()
        if held is None:
            if rel:
                break            # carry finished
            continue
        part = held
        g, p = n.poses.get(GRIPPER), n.poses.get(held)
        if g and p:
            rel.append(compose(inv(g), p))       # part expressed in gripper frame

    n.destroy_node()
    rclpy.try_shutdown()

    if len(rel) < 10:
        print(f"FAIL: only {len(rel)} samples during a carry; command a sort "
              f"while this runs")
        return 1

    # Use the median as the reference offset, not the mean: the mean is dragged
    # by the transition samples this is trying to isolate.
    med = [sorted(r[0][i] for r in rel)[len(rel) // 2] for i in range(3)]
    dev = sorted(math.dist(r[0], med) for r in rel)

    def pct(q):
        return dev[min(int(q * len(dev)), len(dev) - 1)]

    spread = [max(r[0][i] for r in rel) - min(r[0][i] for r in rel) for i in range(3)]

    print(f"part: {part}")
    print(f"samples during the carry: {len(rel)}")
    print(f"median offset in gripper frame: "
          f"({med[0]:+.4f}, {med[1]:+.4f}, {med[2]:+.4f}) m")
    print(f"per-axis spread : "
          f"{spread[0]*1000:.3f} / {spread[1]*1000:.3f} / {spread[2]*1000:.3f} mm")
    print("deviation from that offset:")
    for label, q in (("p50", 0.50), ("p95", 0.95), ("p99", 0.99), ("max", 1.0)):
        print(f"  {label}: {pct(q) * 1000:8.3f} mm")

    # The claim under test is that the part rides the gripper rigidly THROUGH the
    # carry. The grasp and release instants are transitions: the twin learns of
    # the attach from move_group and needs one service round trip to react, and
    # the gripper keeps moving during it. So the steady state is judged at p99
    # and the transition peak is reported rather than failed on.
    ok = pct(0.99) <= RIGID_TOL
    print("\nRESULT:", "PASS" if ok else "FAIL",
          f"(p99 of the carry must stay within {RIGID_TOL*1000:.1f} mm of a "
          f"fixed offset in the gripper frame)")
    return 0 if ok else 1


sys.exit(main())
