"""The cell's safe-state decision, with no ROS in it.

WHY THIS IS A SEPARATE MODULE. The supervisor's latching, its reset interlock
and its speed scaling were correct and protected by nothing, because the only
place they existed was inside a `Node` that needs rclpy, a robot and a running
graph to instantiate. Logic that can only be exercised by standing up a cell
does not get exercised. Extracting it follows what `opcua_security.py` already
does for the same reason: the security rules moved out of `opcua_server.py` so
a test could reach them, and this is the same move for the safety rules.

The extracted half is the interesting half. Every rule below is a claim about
when a machine is allowed to move, and each one now has a test that fails if it
changes: an E-stop that unlatches itself, a reset that works while the button
is still pressed, or an unknown input that reads as safe would all be caught
here, in the fast gate, without a simulator.

THE ONE RULE THAT MATTERS MOST: unknown is unsafe. `guard_closed` and
`human_present` are tri-state, and `None` means no source has ever reported.
Treating that as "guard shut, nobody there" is how a supervisor with a dead
safety bus publishes RUN at full speed and looks healthy doing it.
"""

from __future__ import annotations

from dataclasses import dataclass

# ISO/TS 15066 speed-and-separation reduced speed factor.
REDUCED_SPEED = 0.3

STOPPED_STATES = ("ESTOP", "FAULT", "GUARD_STOP")
CLEAR_STATES = ("RUN", "REDUCED")


@dataclass(frozen=True)
class SafetyInputs:
    """What the supervisor knows right now.

    `guard_closed` and `human_present` are tri-state on purpose: True, False,
    or None for "no source has reported". None is not a missing value to be
    filled in with a default, it is a distinct and unsafe condition.
    """

    estop_latched: bool = False
    fault_latched: bool = False
    guard_closed: bool | None = None
    human_present: bool | None = None


@dataclass(frozen=True)
class SafetyDecision:
    state: str
    reason: str
    clear_to_run: bool
    speed_scale: float


def decide(inputs: SafetyInputs) -> SafetyDecision:
    """Resolve the inputs to one state, most severe first.

    Order is the safety argument, not a style choice. A latched E-stop outranks
    a fault, a fault outranks the guard, and an unreported input outranks an
    open guard only in the sense that both stop the cell: what differs is the
    reason, so an operator can tell a dead safety bus from an open gate rather
    than hunting a gate that is already shut.
    """
    if inputs.estop_latched:
        state, reason = "ESTOP", "emergency stop"
    elif inputs.fault_latched:
        state, reason = "FAULT", "robot feedback lost"
    elif inputs.guard_closed is None or inputs.human_present is None:
        state, reason = "GUARD_STOP", "safety inputs not reported yet"
    elif not inputs.guard_closed:
        state, reason = "GUARD_STOP", "guard open"
    elif inputs.human_present:
        state, reason = "REDUCED", "human in zone (SSM)"
    else:
        state, reason = "RUN", "ok"

    clear = state in CLEAR_STATES
    speed = REDUCED_SPEED if state == "REDUCED" else (1.0 if clear else 0.0)
    return SafetyDecision(state=state, reason=reason, clear_to_run=clear, speed_scale=speed)


def reset_clears_latches(reset_asserted: bool, estop_asserted: bool) -> bool:
    """Whether a reset may clear the latched states.

    The interlock is that a reset does nothing while the E-stop is still
    asserted. Releasing a latch under a held button would let one operator
    re-enable a cell another operator stopped, which is the whole reason the
    latch exists. `estop_asserted` is the live input, deliberately not the
    latched flag: the latch is what is being cleared.
    """
    return reset_asserted and not estop_asserted


def watchdog_expired(
    now: float,
    last_joint: float,
    started: float,
    joint_timeout: float,
    startup_grace: float,
) -> bool:
    """Whether robot feedback counts as lost.

    Two clocks, because "late" and "never arrived" are different failures. Once
    feedback has been seen, staleness is measured from the last message against
    `joint_timeout`. Before any has been seen there is nothing to measure from,
    and the original guard read `if last_joint and ...`, so feedback that never
    arrived was never late and a robot reporting nothing since boot looked
    exactly like a healthy one. It is measured from node start instead, against
    a longer grace, because the supervisor is routinely started before
    move_group and a watchdog that trips on every normal bringup gets ignored.
    """
    if last_joint:
        return (now - last_joint) > joint_timeout
    return (now - started) > startup_grace
