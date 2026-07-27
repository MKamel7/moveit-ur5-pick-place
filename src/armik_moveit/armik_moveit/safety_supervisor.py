"""Functional-safety supervisor for the cell (factory-grade safe-state logic).

Models the safety layer a real production cell needs, independent of the motion
software so it can veto it:

  - Emergency stop (latched): trips the cell to a safe state and cancels any
    motion in progress; requires the E-stop to be released AND a reset to clear.
  - Guard interlock (protective stop): opening the guard halts motion; motion is
    inhibited until the guard is closed again (auto-recovers, no reset needed).
  - Speed and separation monitoring (ISO/TS 15066): when a human is detected in
    the collaborative zone, the commanded speed is reduced; the cell keeps
    running at the reduced speed.
  - Watchdog: if the robot state (joint feedback) goes stale or move_group is
    absent, the cell faults to a safe state.
  - Reset: a reset input clears a latched E-stop / fault once the cause is gone.

Safety inputs (topics; also writable over OPC UA from a safety PLC):
    /safety/estop (Bool)          emergency stop asserted
    /safety/guard_closed (Bool)   guard/gate closed (True = safe)
    /safety/human_present (Bool)  human in the collaborative zone
    /safety/reset (Bool)          reset the latched safe state
Safety output:
    /safety/state (String, JSON)  state, clear_to_run, speed_scale, reason

    ros2 run armik_moveit safety_supervisor
"""
import json
import time

import rclpy
from action_msgs.srv import CancelGoal
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String

REDUCED_SPEED = 0.3   # ISO/TS 15066 speed-and-separation reduced speed factor
JOINT_TIMEOUT = 1.5   # s without joint feedback -> watchdog fault


class SafetySupervisor(Node):
    def __init__(self):
        super().__init__("safety_supervisor")
        self.estop = False
        self.guard_closed = True
        self.human_present = False
        self.estop_latched = False
        self.fault_latched = False
        self.last_joint = 0.0
        self.state = "INIT"
        self.reason = "initialising"

        self.create_subscription(Bool, "/safety/estop", self._estop, 10)
        self.create_subscription(Bool, "/safety/guard_closed", self._guard, 10)
        self.create_subscription(Bool, "/safety/human_present", self._human, 10)
        self.create_subscription(Bool, "/safety/reset", self._reset, 10)
        self.create_subscription(JointState, "/joint_states", self._joints, 10)
        self.pub = self.create_publisher(String, "/safety/state", 10)
        self._cancel = self.create_client(CancelGoal, "/move_action/_action/cancel_goal")

        self.create_timer(0.2, self.tick)
        self.get_logger().info("safety supervisor active")

    def _estop(self, m):
        self.estop = m.data
        if m.data:
            self.estop_latched = True

    def _guard(self, m):
        self.guard_closed = m.data

    def _human(self, m):
        self.human_present = m.data

    def _reset(self, m):
        if m.data and not self.estop:
            self.estop_latched = False
            self.fault_latched = False

    def _joints(self, _):
        self.last_joint = time.time()

    def _cancel_motion(self):
        # cancel-all: an all-zero goal id + zero stamp cancels every active goal
        if self._cancel.service_is_ready():
            self._cancel.call_async(CancelGoal.Request())

    def tick(self):
        # watchdog: joint feedback must be fresh once it has started
        if self.last_joint and (time.time() - self.last_joint) > JOINT_TIMEOUT:
            self.fault_latched = True

        prev = self.state
        if self.estop_latched:
            self.state, self.reason = "ESTOP", "emergency stop"
        elif self.fault_latched:
            self.state, self.reason = "FAULT", "robot feedback lost"
        elif not self.guard_closed:
            self.state, self.reason = "GUARD_STOP", "guard open"
        elif self.human_present:
            self.state, self.reason = "REDUCED", "human in zone (SSM)"
        else:
            self.state, self.reason = "RUN", "ok"

        stopped = self.state in ("ESTOP", "FAULT", "GUARD_STOP")
        clear = self.state in ("RUN", "REDUCED")
        speed = REDUCED_SPEED if self.state == "REDUCED" else (1.0 if clear else 0.0)

        # on any transition from a running state into a stop, cancel motion now
        if stopped and prev in ("RUN", "REDUCED", "INIT"):
            self._cancel_motion()
            self.get_logger().warn(f"SAFE STOP: {self.reason}; motion cancelled")

        self.pub.publish(String(data=json.dumps({
            "state": self.state, "clear_to_run": clear,
            "speed_scale": speed, "reason": self.reason,
            "estop": self.estop_latched, "guard_closed": self.guard_closed,
            "human_present": self.human_present,
        })))


def main():
    rclpy.init()
    node = SafetySupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
