"""Capture the cell's live streams to a session directory (demo video stage 1).

Recording the demo is split into capture and render because the render is far too
slow to keep up with the cell in real time: composing a 2720x1020 four-panel
frame costs more than the frame interval. So this stage only grabs and stores,
as fast as the sources publish, and cell_demo_render.py builds the video
afterwards from what landed on disk.

Every asset is named after its wall-clock capture time in milliseconds, which is
all the renderer needs to line the streams up: there is no separate index, and a
stream that stalls or starts late simply has no files for that stretch.

    ros2 run ... (not a node entry point; run it directly)
    python3 cell_demo_capture.py --out <session_dir> --seconds 120

The UR pendant is NOT captured here: it comes from URSim over VNC, which needs a
different Python environment. See cell_demo_vnc.py.
"""
import argparse
import json
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import Image
from std_msgs.msg import String
from PIL import Image as PILImage

# Minimum seconds between saved frames per image stream. The iso view is the
# video's moving picture so it gets everything the sensor produces; the RGB-D
# view is a near-static top-down inspection shot, so a low rate is plenty.
ISO_PERIOD = 1.0 / 15.0
RGBD_PERIOD = 1.0 / 5.0


def to_pil(msg):
    mode = {"rgb8": "RGB", "bgr8": "RGB", "mono8": "L"}.get(msg.encoding)
    if mode is None:
        return None
    img = PILImage.frombytes(mode, (msg.width, msg.height), bytes(msg.data))
    if msg.encoding == "bgr8":
        b, g, r = img.split()
        img = PILImage.merge("RGB", (r, g, b))
    return img


class Capture(Node):
    def __init__(self, out):
        super().__init__("cell_demo_capture")
        self.out = out
        self.dirs = {}
        for name in ("iso", "rgbd"):
            d = os.path.join(out, name)
            os.makedirs(d, exist_ok=True)
            self.dirs[name] = d
        self.events = open(os.path.join(out, "events.jsonl"), "a", buffering=1)
        self.last = {"iso": 0.0, "rgbd": 0.0}
        self.counts = {"iso": 0, "rgbd": 0, "events": 0}

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, "/iso_camera",
                                 lambda m: self.on_image("iso", m, ISO_PERIOD), qos)
        self.create_subscription(Image, "/rgbd_camera/image",
                                 lambda m: self.on_image("rgbd", m, RGBD_PERIOD), qos)
        for topic, kind in (("/cell/telemetry", "telemetry"),
                            ("/safety/state", "safety"),
                            # Only present if the detector publishes labelled
                            # detections; the PoseArray below is the fallback.
                            ("/detected_parts_named", "detections")):
            self.create_subscription(
                String, topic, lambda m, k=kind: self.on_event(k, m), 10)
        # detector.py always publishes this, but PoseArray carries no labels, so
        # the panel can only report how many were found and where.
        self.create_subscription(PoseArray, "/detected_parts", self.on_poses, 10)
        self.create_timer(5.0, self.report)

    def on_poses(self, msg):
        self.write("detections_raw", {
            "n": len(msg.poses),
            "xy": [[round(p.position.x, 4), round(p.position.y, 4)] for p in msg.poses],
        })

    def on_image(self, name, msg, period):
        now = time.time()
        if now - self.last[name] < period:
            return
        img = to_pil(msg)
        if img is None:
            return
        self.last[name] = now
        img.save(os.path.join(self.dirs[name], f"{int(now * 1000)}.jpg"),
                 quality=92, optimize=False)
        self.counts[name] += 1

    def on_event(self, kind, msg):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        self.write(kind, payload)

    def write(self, kind, payload):
        self.events.write(json.dumps(
            {"t": time.time(), "kind": kind, "data": payload}) + "\n")
        self.counts["events"] += 1

    def report(self):
        self.get_logger().info(
            "captured " + "  ".join(f"{k}={v}" for k, v in self.counts.items()))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--seconds", type=float, default=0.0,
                    help="stop after this long; 0 runs until interrupted")
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    rclpy.init()
    node = Capture(args.out)
    t0 = time.time()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if args.seconds and time.time() - t0 >= args.seconds:
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.report()
        node.events.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
