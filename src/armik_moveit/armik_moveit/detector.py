"""Perception: detect coloured parts in the bin from the RGB-D camera (Phase 3).

Classical, honest perception (no black box): for each part colour, threshold the
RGB image, take the largest blob's centroid, sample the depth there, deproject
the pixel to a 3D point in the camera's optical frame, and transform it to the
world/floor frame using the known camera pose. Detected part poses are published
on /detected_parts (geometry_msgs/PoseArray) and printed with a ground-truth
comparison so accuracy is visible.

PoseArray carries no labels, so the same detections also go out on
/detected_parts_named as JSON keyed by colour. Anything that needs to know WHICH
colour was seen rather than how many (the demo overlay, an HMI) reads that.

    ros2 launch armik_moveit perception.launch.py
    ros2 run   armik_moveit detector
"""
import json
import math

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, PoseArray
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

# Camera link pose in world (matches perception_cell.sdf): straight down.
CAM_XYZ = np.array([0.47, -0.32, 1.00])
CAM_PITCH = 1.5707963  # rot about y so the sensor +x looks down (-z world)

# Colour thresholds in RGB (0-255). Parts are saturated primaries, so per-channel
# bounds pick them out, but bounds alone are not enough: a grey pixel sits inside
# every one of these boxes if it happens to land in the right brightness band. A
# shadowed grey worktop at (105, 105, 105) satisfies "g > 100, r < 110, b < 110"
# and reads as green. So each colour also has to be SATURATED - its channel has
# to beat the others by a margin - which no grey can do at any brightness.
MARGIN = {"red": 60, "green": 40, "blue": 40, "yellow": 50}


def _dominant(hi, *lo):
    """True where every channel in `lo` is at least MARGIN below `hi`."""
    floor = lo[0] if len(lo) == 1 else np.maximum(*lo)
    return hi - floor


COLORS = {
    "red":    lambda r, g, b: (r > 120) & (g < 90) & (b < 90)
                              & (_dominant(r, g, b) > MARGIN["red"]),
    "green":  lambda r, g, b: (g > 100) & (r < 110) & (b < 110)
                              & (_dominant(g, r, b) > MARGIN["green"]),
    "blue":   lambda r, g, b: (b > 120) & (r < 110) & (g < 120)
                              & (_dominant(b, r, g) > MARGIN["blue"]),
    # Yellow is red+green against a low blue, so the margin is measured from the
    # weaker of the two bright channels.
    "yellow": lambda r, g, b: (r > 130) & (g > 110) & (b < 90)
                              & (np.minimum(r, g) - b > MARGIN["yellow"]),
}
GROUND_TRUTH = {  # world xy of each part (for the accuracy readout)
    "red": (0.41, -0.38), "green": (0.53, -0.38),
    "blue": (0.41, -0.26), "yellow": (0.53, -0.26),
}


def _ry(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def camera_to_world():
    """4x4 transform from the camera optical frame to the world frame."""
    r_wl = _ry(CAM_PITCH)  # world <- camera link
    # optical (x right, y down, z forward) expressed in the link frame:
    # optical_x = -link_y, optical_y = -link_z, optical_z = +link_x
    r_lo = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])
    t = np.eye(4)
    t[:3, :3] = r_wl @ r_lo
    t[:3, 3] = CAM_XYZ
    return t


class Detector(Node):
    def __init__(self):
        super().__init__("detector")
        self.rgb = None
        self.depth = None
        self.k = None
        self.t_wo = camera_to_world()
        self.create_subscription(Image, "/rgbd_camera/image", self._on_rgb, 10)
        self.create_subscription(Image, "/rgbd_camera/depth_image", self._on_depth, 10)
        self.create_subscription(CameraInfo, "/rgbd_camera/camera_info", self._on_info, 10)
        self.pub = self.create_publisher(PoseArray, "/detected_parts", 10)
        self.named = self.create_publisher(String, "/detected_parts_named", 10)
        self.create_timer(1.0, self.detect)
        self.get_logger().info("detector ready")

    def _on_rgb(self, msg):
        self.rgb = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)

    def _on_depth(self, msg):
        self.depth = np.frombuffer(msg.data, np.float32).reshape(msg.height, msg.width)

    def _on_info(self, msg):
        self.k = np.array(msg.k).reshape(3, 3)

    def _deproject(self, u, v, d, fx, fy, cx, cy):
        p_opt = np.array([(u - cx) / fx * d, (v - cy) / fy * d, d, 1.0])
        return self.t_wo @ p_opt

    def detect(self):
        if self.rgb is None or self.depth is None or self.k is None:
            return
        rgb = self.rgb.astype(np.int32)
        r, g, b = rgb[:, :, 0], rgb[:, :, 1], rgb[:, :, 2]
        fx, fy = self.k[0, 0], self.k[1, 1]
        cx, cy = self.k[0, 2], self.k[1, 2]

        arr = PoseArray()
        arr.header.frame_id = "world"
        found = {}
        print("--- detections (world frame) ---")
        for name, pred in COLORS.items():
            mask = pred(r, g, b)
            if mask.sum() < 20:
                print(f"  {name:6}: not found")
                continue
            ys, xs = np.nonzero(mask)
            u, v = xs.mean(), ys.mean()
            # median depth over the blob's valid pixels (robust to edge noise)
            dvals = self.depth[ys, xs]
            dvals = dvals[np.isfinite(dvals) & (dvals > 0)]
            if dvals.size == 0:
                print(f"  {name:6}: no depth")
                continue
            d = float(np.median(dvals))
            p_world = self._deproject(u, v, d, fx, fy, cx, cy)

            # orientation: PCA of the blob's principal axis, mapped to a world
            # yaw by deprojecting the centroid and a point along that axis.
            pts = np.column_stack([xs - u, ys - v]).astype(float)
            cov = pts.T @ pts / max(len(pts), 1)
            evals, evecs = np.linalg.eigh(cov)
            major = evecs[:, int(np.argmax(evals))]
            p_axis = self._deproject(u + major[0] * 10, v + major[1] * 10, d, fx, fy, cx, cy)
            yaw = math.atan2(p_axis[1] - p_world[1], p_axis[0] - p_world[0])

            gx, gy = GROUND_TRUTH.get(name, (float("nan"), float("nan")))
            err = np.hypot(p_world[0] - gx, p_world[1] - gy) * 1000
            print(f"  {name:6}: world ({p_world[0]:.3f}, {p_world[1]:.3f}, {p_world[2]:.3f})"
                  f"  yaw {math.degrees(yaw):+.0f} deg  gt ({gx:.2f}, {gy:.2f})  err {err:.0f} mm")
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = (
                float(p_world[0]), float(p_world[1]), float(p_world[2]))
            pose.orientation.z = math.sin(yaw / 2)
            pose.orientation.w = math.cos(yaw / 2)
            arr.poses.append(pose)
            found[name] = {
                "x": float(p_world[0]), "y": float(p_world[1]), "z": float(p_world[2]),
                "yaw_deg": round(math.degrees(yaw), 1),
                "px": int(mask.sum()),
            }
        self.pub.publish(arr)
        self.named.publish(String(data=json.dumps(
            {"colours": sorted(COLORS), "detected": found})))


def main():
    rclpy.init()
    node = Detector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
