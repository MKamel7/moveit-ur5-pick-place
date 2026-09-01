#!/usr/bin/env python3
"""The cell's safe-state rules, and the fail-open one that was there.

The supervisor's latching, reset interlock and speed scaling were correct and
nothing protected any of them, because they lived inside a `Node` that needs
rclpy and a running graph to build. This file exercises the extracted rules
with no ROS, no robot and no simulator, so it runs in the fast gate.

The headline case is `test_unreported_inputs_hold_a_protective_stop`. The guard
and human-presence signals used to be initialised to their safe-looking values,
so a supervisor whose safety source never came up published RUN at full speed
and looked healthy doing it. Every other rule here was already right; that one
was wrong, and none of them could fail out loud before now.
"""

import sys
from pathlib import Path

import pytest

PKG = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG))
from armik_moveit.safety_logic import (  # noqa: E402
    REDUCED_SPEED,
    SafetyInputs,
    decide,
    reset_clears_latches,
    watchdog_expired,
)

SAFE = {"guard_closed": True, "human_present": False}


# --- unknown is unsafe -------------------------------------------------------

def test_unreported_inputs_hold_a_protective_stop() -> None:
    """A safety source that never spoke must not read as "all clear".

    The reason is asserted, not just the stop, and that is the point. Seeding
    the original fail-open back in showed this test passing anyway: with a
    tri-state input `not None` is True, so an unreported guard stopped the cell
    by coincidence and reported "guard open" while the rule that should have
    stopped it was gone. A test that accepts the right answer for the wrong
    reason is not protecting the rule it is named after.
    """
    decision = decide(SafetyInputs())

    assert decision.state == "GUARD_STOP"
    assert decision.clear_to_run is False
    assert decision.speed_scale == 0.0
    assert "not reported" in decision.reason


@pytest.mark.parametrize("missing", ["guard_closed", "human_present"])
def test_either_input_alone_being_unreported_is_enough_to_stop(missing: str) -> None:
    """One known-good input does not license the cell while the other is silent."""
    inputs = SafetyInputs(**{**SAFE, missing: None})
    decision = decide(inputs)

    assert decision.clear_to_run is False
    assert "not reported" in decision.reason


def test_the_reason_separates_a_dead_bus_from_an_open_gate() -> None:
    """Both stop the cell; an operator still has to know which to go and fix."""
    silent = decide(SafetyInputs()).reason
    open_guard = decide(SafetyInputs(guard_closed=False, human_present=False)).reason

    assert silent != open_guard
    assert "not reported" in silent
    assert "guard" in open_guard


# --- ordinary states ---------------------------------------------------------

def test_all_clear_runs_at_full_speed() -> None:
    decision = decide(SafetyInputs(**SAFE))

    assert decision.state == "RUN"
    assert decision.clear_to_run is True
    assert decision.speed_scale == 1.0


def test_a_human_in_the_zone_reduces_speed_without_stopping() -> None:
    """ISO/TS 15066 speed and separation: the cell keeps running, slower."""
    decision = decide(SafetyInputs(guard_closed=True, human_present=True))

    assert decision.state == "REDUCED"
    assert decision.clear_to_run is True
    assert decision.speed_scale == pytest.approx(REDUCED_SPEED)
    assert 0.0 < decision.speed_scale < 1.0


def test_an_open_guard_stops_and_commands_zero_speed() -> None:
    decision = decide(SafetyInputs(guard_closed=False, human_present=False))

    assert decision.state == "GUARD_STOP"
    assert decision.speed_scale == 0.0


# --- precedence, which is the safety argument --------------------------------

def test_estop_outranks_everything_including_an_otherwise_clear_cell() -> None:
    decision = decide(SafetyInputs(estop_latched=True, **SAFE))

    assert decision.state == "ESTOP"
    assert decision.speed_scale == 0.0


def test_a_fault_outranks_a_clear_cell_but_not_an_estop() -> None:
    assert decide(SafetyInputs(fault_latched=True, **SAFE)).state == "FAULT"
    assert decide(
        SafetyInputs(estop_latched=True, fault_latched=True, **SAFE)
    ).state == "ESTOP"


def test_no_stopped_state_ever_leaves_speed_above_zero() -> None:
    """The property behind the table: stopped means stopped, in every path."""
    for estop in (True, False):
        for fault in (True, False):
            for guard in (True, False, None):
                for human in (True, False, None):
                    decision = decide(SafetyInputs(estop, fault, guard, human))
                    if not decision.clear_to_run:
                        assert decision.speed_scale == 0.0, decision


# --- the reset interlock -----------------------------------------------------

def test_reset_does_nothing_while_the_estop_is_still_pressed() -> None:
    """Otherwise one operator can re-enable a cell another operator stopped."""
    assert reset_clears_latches(reset_asserted=True, estop_asserted=True) is False


def test_reset_clears_once_the_estop_is_released() -> None:
    assert reset_clears_latches(reset_asserted=True, estop_asserted=False) is True


def test_no_reset_means_no_clear() -> None:
    assert reset_clears_latches(reset_asserted=False, estop_asserted=False) is False


# --- the watchdog ------------------------------------------------------------

def test_fresh_feedback_does_not_trip_the_watchdog() -> None:
    assert watchdog_expired(100.0, last_joint=99.5, started=0.0,
                            joint_timeout=1.5, startup_grace=10.0) is False


def test_stale_feedback_trips_the_watchdog() -> None:
    assert watchdog_expired(100.0, last_joint=98.0, started=0.0,
                            joint_timeout=1.5, startup_grace=10.0) is True


def test_feedback_that_never_arrived_trips_it_too() -> None:
    """The case the original guard missed.

    `if last_joint and ...` meant a robot that had reported nothing since boot
    was never late, so the watchdog protecting against lost feedback was off
    exactly when feedback was most obviously absent.
    """
    assert watchdog_expired(100.0, last_joint=0.0, started=80.0,
                            joint_timeout=1.5, startup_grace=10.0) is True


def test_a_slow_bringup_is_not_a_fault() -> None:
    """The supervisor starts before move_group; that must not latch a fault."""
    assert watchdog_expired(100.0, last_joint=0.0, started=95.0,
                            joint_timeout=1.5, startup_grace=10.0) is False
