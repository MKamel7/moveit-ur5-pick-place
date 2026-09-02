#!/usr/bin/env python3
"""Republish the simulator's true part poses as TF, for the benchmark to check.

WHY THIS NODE EXISTS AT ALL

The placement benchmark has to confirm that a part reached the pose a trial
commanded, because an accepted `set_pose` is a request and not a placement. See
`placement_guard.py` for what that cost: two rows of a campaign measured a part
that was sitting at its home pose.

Gazebo already publishes exactly this on `/world/<world>/pose/info`, and two
routes to it are closed:

  * A `gz.transport13` subscription made INSIDE the benchmark process receives
    nothing. `subscribe()` returns True and no message ever arrives, while the
    identical code works from a plain shell, from a `setsid` script and from
    under `ros2 launch`. Service requests from that same process work, which is
    why the placements themselves were landing.
  * `ros_gz_bridge` will map `gz.msgs.Pose_V` to `tf2_msgs/msg/TFMessage`, but
    the bridged transforms arrive with an empty `child_frame_id`, so every pose
    is anonymous and no part can be told from another.

So the poses are read where reading them works, in an ordinary node, and
published under the names the benchmark asks about.

WHAT IT IS NOT

Not an input to anything the cell does. Nothing in perception, planning or the
safety path may subscribe to this: it is an evaluation oracle, and a cell that
consumes simulator truth is measuring a simulator rather than a robot. It exists
so a benchmark can tell "the robot missed" from "the part was never there".
"""
from __future__ import annotations

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_msgs.msg import TFMessage

from ur5_pick_place.placement_guard import GROUND_TRUTH_TOPIC, WORLD_NAME

TOPIC = GROUND_TRUTH_TOPIC
PREFIX = "part_"

# The benchmark polls at 20 Hz and a placement is a step change, so publishing
# faster than this would only add traffic to a run whose timings are measured.
MAX_HZ = 20.0


class GroundTruthPublisher(Node):
    def __init__(self) -> None:
        super().__init__("ground_truth_publisher")
        from gz.msgs10.pose_v_pb2 import Pose_V
        from gz.transport13 import Node as GzNode

        self._pub = self.create_publisher(TFMessage, TOPIC, 10)
        self._last_sent = 0.0
        self._gz = GzNode()
        topic = f"/world/{WORLD_NAME}/pose/info"
        if not self._gz.subscribe(Pose_V, topic, self._on_poses):
            raise RuntimeError(f"could not subscribe to {topic}")
        self.get_logger().info(f"republishing {PREFIX}* poses from {topic} on {TOPIC}")

    def _on_poses(self, msg) -> None:
        now = self.get_clock().now()
        seconds = now.nanoseconds * 1e-9
        if seconds - self._last_sent < 1.0 / MAX_HZ:
            return
        self._last_sent = seconds

        out = TFMessage()
        for pose in msg.pose:
            if not pose.name.startswith(PREFIX):
                continue
            tf = TransformStamped()
            tf.header.stamp = now.to_msg()
            tf.header.frame_id = "world"
            tf.child_frame_id = pose.name
            tf.transform.translation.x = pose.position.x
            tf.transform.translation.y = pose.position.y
            tf.transform.translation.z = pose.position.z
            tf.transform.rotation.x = pose.orientation.x
            tf.transform.rotation.y = pose.orientation.y
            tf.transform.rotation.z = pose.orientation.z
            tf.transform.rotation.w = pose.orientation.w
            out.transforms.append(tf)
        if out.transforms:
            self._pub.publish(out)


def main() -> None:
    rclpy.init()
    node = GroundTruthPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
