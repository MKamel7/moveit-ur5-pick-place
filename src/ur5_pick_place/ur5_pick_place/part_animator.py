"""Kinematic part animation and moving conveyor.

There is no physical gripper in this simulation, so this node makes the parts
move convincingly instead: while the arm carries a part it is set to follow the
tool frame smoothly (via TF at 30 Hz), and once placed it rides the conveyor
belt away from the table at a constant speed. The parts are static models whose
pose is driven through the gz set-pose service, which avoids physics jitter.

Coordinated by the pick-and-place node over /carry_cmd (std_msgs/String):
    "attach:part_green"  -> start following the tool with that part
    "detach:part_green"  -> drop it on the belt and start conveying it
"""
from __future__ import annotations

import threading

import rclpy
import tf2_ros
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.pose_pb2 import Pose as GzPose
from gz.transport13 import Node as GzNode
from rclpy.node import Node
from std_msgs.msg import String

WORLD_NAME = "pick_place"
TOOL_FRAME = "tool0"
BASE_FRAME = "base_link"

# Home positions of the parts on the table (match the world SDF).
PART_HOMES = {
    "part_red": (0.50, 0.02, 0.225),
    "part_green": (0.55, 0.17, 0.225),
    "part_blue": (0.50, 0.32, 0.225),
}

CARRY_DROP = 0.03  # part centre below the tool frame while carried
BELT_TOP_Z = 0.175  # part centre resting on the belt
BELT_X = 0.50
BELT_SPEED = 0.06  # m/s, conveyor speed (parts move toward -y, away from the table)
BELT_END_Y = -0.70  # parts stop here at the far end of the belt
TICK = 1.0 / 30.0


class PartAnimator(Node):
    def __init__(self) -> None:
        super().__init__("part_animator")
        self.gz = GzNode()
        self.set_pose_srv = f"/world/{WORLD_NAME}/set_pose"
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.lock = threading.Lock()
        self.carried: str | None = None
        self.on_belt: dict[str, float] = {}  # part -> current y
        self.create_subscription(String, "/carry_cmd", self.on_cmd, 10)
        self.create_timer(TICK, self.tick)
        self.get_logger().info("part_animator ready")

    def _set_pose(self, name: str, x: float, y: float, z: float) -> None:
        req = GzPose()
        req.name = name
        req.position.x = float(x)
        req.position.y = float(y)
        req.position.z = float(z)
        req.orientation.w = 1.0
        try:
            self.gz.request(self.set_pose_srv, req, GzPose, Boolean, 100)
        except Exception:  # noqa: BLE001
            pass

    def on_cmd(self, msg: String) -> None:
        op, _, part = msg.data.partition(":")
        if op == "reset":
            with self.lock:
                self.carried = None
                self.on_belt.clear()
            for name, home in PART_HOMES.items():
                self._set_pose(name, *home)
            self.get_logger().info("parts reset to the table")
            return
        if not part:
            return
        with self.lock:
            if op == "attach":
                self.carried = part
                self.on_belt.pop(part, None)
                self.get_logger().info(f"carrying {part}")
            elif op == "detach":
                if self.carried == part:
                    self.carried = None
                self.on_belt[part] = self._tool_y_or(part, -0.38)
                self.get_logger().info(f"{part} released onto the conveyor")

    def _tool_y_or(self, part: str, default: float) -> float:
        try:
            t = self.tf_buffer.lookup_transform(BASE_FRAME, TOOL_FRAME, rclpy.time.Time())
            return t.transform.translation.y
        except Exception:  # noqa: BLE001
            return default

    def tick(self) -> None:
        with self.lock:
            carried = self.carried
            belt_items = list(self.on_belt.items())
        if carried:
            try:
                t = self.tf_buffer.lookup_transform(BASE_FRAME, TOOL_FRAME, rclpy.time.Time())
                x = t.transform.translation.x
                y = t.transform.translation.y
                z = t.transform.translation.z - CARRY_DROP
                self._set_pose(carried, x, y, z)
            except Exception:  # noqa: BLE001
                pass
        for part, y in belt_items:
            ny = max(y - BELT_SPEED * TICK, BELT_END_Y)
            with self.lock:
                self.on_belt[part] = ny
            self._set_pose(part, BELT_X, ny, BELT_TOP_Z)


def main() -> None:
    rclpy.init()
    node = PartAnimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
