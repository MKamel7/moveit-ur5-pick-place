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

AN INPUT NEVER HEARD FROM IS NOT A SAFE INPUT. The guard and human-presence
signals used to be initialised to their safe-looking values, guard closed and
nobody in the zone, so a supervisor whose safety source never came up at all
published RUN at full speed forever and looked correct doing it. That is
fail-open, and it is the failure a safety layer exists to not have: the
dangerous case is exactly the one where the safety bus is dead. Both inputs now
start UNKNOWN and unknown is treated as unsafe, so the cell holds a protective
stop until a source affirmatively says it is clear.

WHAT IS DELIBERATELY NOT HERE, and why. There is no staleness timeout on the
guard and human inputs, only on joint feedback. A timeout needs the source to
publish cyclically, and these publish ON CHANGE (`opcua_server.py:160`, which
does that specifically so it does not fight the GUI as a second publisher). A
timeout against on-change publishers would trip on a healthy, unchanging cell.
Making it real means one cyclic safety source with a heartbeat, the way a
PROFIsafe or FSoE F-host works, which is a design change and not this fix. The
gap is named here rather than closed badly: a watchdog that false-trips gets
switched off, and then there is no watchdog.

    ros2 run armik_moveit safety_supervisor
"""
import json
import time

import rclpy
from action_msgs.srv import CancelGoal
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String

from armik_moveit.safety_logic import (
    REDUCED_SPEED,
    STOPPED_STATES,
    SafetyInputs,
    decide,
    reset_clears_latches,
    watchdog_expired,
)

_ = REDUCED_SPEED  # re-exported for callers that imported it from here
JOINT_TIMEOUT = 1.5   # s without joint feedback -> watchdog fault
# Grace after start before feedback that has NEVER arrived counts as lost. The
# supervisor is routinely started before move_group, so a 1.5 s bound would
# latch a fault on every ordinary bringup and teach an operator to reset it
# reflexively, which is worse than the fault it reports. Long enough to cover a
# normal start, short enough that a robot which never reports is not silently
# tolerated for the whole run.
STARTUP_GRACE = 10.0


class SafetySupervisor(Node):
    def __init__(self):
        super().__init__("safety_supervisor")
        self.estop = False
        # None means no source has spoken yet. Unknown is unsafe: the guard is
        # not assumed shut and the zone is not assumed empty.
        self.guard_closed: bool | None = None
        self.human_present: bool | None = None
        self.estop_latched = False
        self.fault_latched = False
        self.last_joint = 0.0
        self.started = time.time()
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
        if reset_clears_latches(m.data, self.estop):
            self.estop_latched = False
            self.fault_latched = False

    def _joints(self, _):
        self.last_joint = time.time()

    def _cancel_motion(self):
        # cancel-all: an all-zero goal id + zero stamp cancels every active goal
        if self._cancel.service_is_ready():
            self._cancel.call_async(CancelGoal.Request())

    def tick(self):
        # Watchdog. `if self.last_joint and ...` meant feedback that never
        # arrived at all was never late, so a robot that reported nothing from
        # the start was indistinguishable from a healthy one. Feedback that has
        # never come is now measured from node start instead, past a grace.
        if watchdog_expired(time.time(), self.last_joint, self.started,
                            JOINT_TIMEOUT, STARTUP_GRACE):
            self.fault_latched = True

        prev = self.state
        decision = decide(SafetyInputs(
            estop_latched=self.estop_latched,
            fault_latched=self.fault_latched,
            guard_closed=self.guard_closed,
            human_present=self.human_present,
        ))
        self.state, self.reason = decision.state, decision.reason

        stopped = self.state in STOPPED_STATES
        clear = decision.clear_to_run
        speed = decision.speed_scale

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
