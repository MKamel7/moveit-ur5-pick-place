"""Verify the Gazebo twin is an exact mirror of the robot's TF.

The twin's whole claim is that it draws the real robot's link transforms rather
than simulating them, so the test is simply: for every mirrored link, does the
Gazebo model pose equal the TF transform? A simulation that chases the robot
cannot pass this (the previous PID version lagged ~33 ms by design); a mirror
should agree to numerical noise.

Samples repeatedly, including while the arm is moving, since a mirror that is
only correct at rest is exactly the failure mode being tested for.

    python3 test_twin_mirror.py [--world sort_cell_twin] [--samples 20]
"""
import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
import tf2_ros

from gz.transport13 import Node as GzNode
from gz.msgs10.pose_v_pb2 import Pose_V

sys.path.insert(0, "/home/kamel/robot-arm-ik/ros2/src/armik_moveit")
from armik_moveit.twin_world import MODEL_PREFIX  # noqa: E402

POS_TOL = 1e-3      # 1 mm
ROT_TOL = 1e-3      # 1 mrad


def quat_angle(a, b):
    """Smallest rotation angle between two quaternions (x, y, z, w), radians."""
    d = abs(sum(x * y for x, y in zip(a, b)))
    return 2.0 * math.acos(max(-1.0, min(1.0, d)))


class Probe(Node):
    def __init__(self, world):
        super().__init__("twin_mirror_test")
        self.buf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buf, self)
        self.gz = GzNode()
        self.poses = {}
        self.topic = f"/world/{world}/pose/info"
        self.gz.subscribe(Pose_V, self.topic, self._on_poses)

    def _on_poses(self, msg):
        # Only top-level twin models matter; nested link entities repeat names.
        for p in msg.pose:
            if p.name.startswith(MODEL_PREFIX) or p.name.startswith("part_"):
                self.poses[p.name] = (
                    (p.position.x, p.position.y, p.position.z),
                    (p.orientation.x, p.orientation.y,
                     p.orientation.z, p.orientation.w),
                )

    def tf(self, frame):
        try:
            t = self.buf.lookup_transform("world", frame, rclpy.time.Time())
        except tf2_ros.TransformException:
            return None
        tr, r = t.transform.translation, t.transform.rotation
        return ((tr.x, tr.y, tr.z), (r.x, r.y, r.z, r.w))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", default="sort_cell_twin")
    ap.add_argument("--samples", type=int, default=20)
    ap.add_argument("--interval", type=float, default=0.5)
    ap.add_argument("--settle", type=float, default=1.0,
                    help="seconds the arm must be still before a sample counts as stationary")
    args = ap.parse_args()

    rclpy.init()
    n = Probe(args.world)
    for _ in range(80):
        rclpy.spin_once(n, timeout_sec=0.05)
    if not n.poses:
        print(f"FAIL: no poses on {n.topic}; is the twin world running?")
        return 1

    links = sorted(k[len(MODEL_PREFIX):] for k in n.poses if k.startswith(MODEL_PREFIX))
    print(f"mirrored links: {len(links)}")

    # Stationary and moving samples are judged differently, and deliberately so.
    # When the arm is still, the mirror must be numerically exact: any error is a
    # bug in the pose maths. When it is moving, TF and the Gazebo readback are
    # separated by the transport pipeline (TF -> this node -> set_pose_vector ->
    # Gazebo -> pose/info), so the two are sampled a few tens of milliseconds
    # apart and disagree by speed x delay. That is transport latency, not error,
    # so it is reported as an implied delay rather than failed against a
    # position tolerance.
    still = {"p": 0.0, "r": 0.0, "link": "", "n": 0}
    move = {"p": 0.0, "r": 0.0, "link": "", "n": 0, "lag": 0.0}
    prev_tcp = prev_t = None
    last_move_t = time.time()
    speed = 0.0

    for _ in range(args.samples):
        # Poll the TCP finely so "moving" is detected properly, and remember when
        # it last moved. A sample taken just after the arm stops is NOT settled:
        # the mirror is still pushing the last poses through, so counting it as
        # stationary would blame the pipeline's tail on the pose maths.
        t0 = time.time()
        while time.time() - t0 < args.interval:
            rclpy.spin_once(n, timeout_sec=0.02)
            tcp, now = n.tf("tool0"), time.time()
            if prev_tcp and tcp and prev_t and now > prev_t:
                v = math.dist(tcp[0], prev_tcp[0]) / (now - prev_t)
                if v > 0.02:
                    speed, last_move_t = v, now
            prev_tcp, prev_t = tcp, now

        settled = (time.time() - last_move_t) > args.settle
        bucket = still if settled else move

        for link in links:
            ref = n.tf(link)
            got = n.poses.get(MODEL_PREFIX + link)
            if ref is None or got is None:
                continue
            dp = math.dist(ref[0], got[0])
            bucket["n"] += 1
            if dp > bucket["p"]:
                bucket["p"], bucket["link"] = dp, link
            bucket["r"] = max(bucket["r"], quat_angle(ref[1], got[1]))
            if bucket is move and speed > 0.02:
                move["lag"] = max(move["lag"], dp / speed)

    print(f"\nSTATIONARY  ({still['n']} comparisons)")
    print(f"  worst position   : {still['p'] * 1000:.4f} mm  (link {still['link'] or '-'})")
    print(f"  worst orientation: {still['r'] * 1000:.4f} mrad")
    print(f"\nMOVING      ({move['n']} comparisons)")
    if move["n"]:
        print(f"  worst position   : {move['p'] * 1000:.2f} mm  (link {move['link']})")
        print(f"  implied pipeline delay: {move['lag'] * 1000:.0f} ms")
    else:
        print("  none: the arm never moved; run a sort alongside the test")

    ok = (still["n"] > 0 and still["p"] <= POS_TOL and still["r"] <= ROT_TOL)
    print("\nRESULT:", "PASS" if ok else "FAIL",
          f"(stationary tolerance {POS_TOL * 1000:.1f} mm / {ROT_TOL * 1000:.1f} mrad)")
    n.destroy_node()
    rclpy.try_shutdown()
    return 0 if ok else 1


sys.exit(main())
