"""RGB-D object detector: perceive the pick target and publish its 3D pose.

Subscribes to the simulated RGB-D camera, segments the green target by colour
(classical, GPU-free), samples its depth, and lifts the detection to a 3D pose
in the robot base frame using the tested perception geometry. Publishes the
result on /detected_object_pose so the pick-and-place uses a perceived target
rather than a hardcoded one.
"""
from __future__ import annotations

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

from ur5_pick_place.perception import (
    CameraIntrinsics,
    camera_optical_transform,
    pixel_to_base,
)
from ur5_pick_place.segmentation import (
    COLOR_HSV_RANGES,
    sample_depth,
    segment_largest_blob,
)

# BGR colours used to annotate each part in the debug image.
_ANNOT_BGR = {"red": (0, 0, 255), "green": (0, 200, 0), "blue": (255, 0, 0)}


class DetectorNode(Node):
    def __init__(self) -> None:
        super().__init__("object_detector")
        # Camera extrinsics (body pose in the base frame), defaults match the world SDF.
        self.declare_parameter("camera_xyz", [0.55, 0.15, 0.85])
        self.declare_parameter("camera_rpy", [0.0, 1.5707963, 0.0])
        self.declare_parameter("min_area", 200.0)
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("debug_image_path", "")
        # Which coloured part to publish as the pick target.
        self.declare_parameter("target_color", "green")

        xyz = list(self.get_parameter("camera_xyz").value)
        rpy = list(self.get_parameter("camera_rpy").value)
        self.min_area = float(self.get_parameter("min_area").value)
        self.base_frame = self.get_parameter("base_frame").value
        self.debug_image_path = self.get_parameter("debug_image_path").value
        self.target_color = self.get_parameter("target_color").value
        if self.target_color not in COLOR_HSV_RANGES:
            raise ValueError(f"unknown target_color {self.target_color!r}")

        self.t_base_optical = camera_optical_transform(xyz, rpy)
        self.bridge = CvBridge()
        self.intr: CameraIntrinsics | None = None
        self.depth: np.ndarray | None = None
        self._logged = False
        self._saved_debug = False

        self.create_subscription(CameraInfo, "/rgbd_camera/camera_info", self.on_info, 10)
        self.create_subscription(Image, "/rgbd_camera/depth_image", self.on_depth, 10)
        self.create_subscription(Image, "/rgbd_camera/image", self.on_image, 10)
        self.pub = self.create_publisher(PoseStamped, "/detected_object_pose", 10)
        self.get_logger().info(
            f"object_detector started; target_color={self.target_color}; waiting for camera"
        )

    def on_info(self, msg: CameraInfo) -> None:
        if self.intr is None:
            self.intr = CameraIntrinsics.from_k(msg.k, msg.width, msg.height)
            self.get_logger().info(
                f"intrinsics: fx={self.intr.fx:.1f} cx={self.intr.cx:.1f} "
                f"cy={self.intr.cy:.1f} {self.intr.width}x{self.intr.height}"
            )

    def on_depth(self, msg: Image) -> None:
        self.depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")

    def on_image(self, msg: Image) -> None:
        if self.intr is None or self.depth is None:
            return
        bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        # Detect every colour so the debug image shows the full scene, but only
        # the selected target colour is published as the pick pose.
        detections = {}
        for color, ranges in COLOR_HSV_RANGES.items():
            det = segment_largest_blob(bgr, ranges, min_area=self.min_area)
            if det is not None:
                detections[color] = det

        target = detections.get(self.target_color)
        if target is not None:
            try:
                z = sample_depth(self.depth, target.u, target.v, patch=5)
            except ValueError:
                z = None
            if z is not None:
                p = pixel_to_base(target.u, target.v, z, self.intr, self.t_base_optical)
                pose = PoseStamped()
                pose.header.frame_id = self.base_frame
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.pose.position.x = float(p[0])
                pose.pose.position.y = float(p[1])
                pose.pose.position.z = float(p[2])
                pose.pose.orientation.w = 1.0
                self.pub.publish(pose)
                if not self._logged:
                    self.get_logger().info(
                        f"target '{self.target_color}' at pixel "
                        f"({target.u:.0f},{target.v:.0f}) depth {z:.3f} m -> "
                        f"base ({p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f})"
                    )
                    self._logged = True

        if self.debug_image_path and not self._saved_debug and detections:
            self._save_debug(bgr, detections)

    def _save_debug(self, bgr, detections) -> None:
        for color, det in detections.items():
            x, y, w, h = det.bbox
            thick = 3 if color == self.target_color else 1
            cv2.rectangle(bgr, (x, y), (x + w, y + h), _ANNOT_BGR[color], thick)
            cv2.circle(bgr, (int(det.u), int(det.v)), 3, _ANNOT_BGR[color], -1)
            label = f"{color}{' *' if color == self.target_color else ''}"
            cv2.putText(bgr, label, (x, y - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.5, _ANNOT_BGR[color], 2)
        cv2.imwrite(self.debug_image_path, bgr)
        self.get_logger().info(f"saved debug image to {self.debug_image_path}")
        self._saved_debug = True


def main() -> None:
    rclpy.init()
    node = DetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
