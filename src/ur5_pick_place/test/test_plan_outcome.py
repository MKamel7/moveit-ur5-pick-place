"""A failed plan must name its first cause, not its last symptom."""

import sys
from pathlib import Path

PKG = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG))
from ur5_pick_place.plan_outcome import (  # noqa: E402
    NO_IK_SOLUTION,
    NO_PLAN_FOUND,
    NONE,
    START_STATE_IN_COLLISION,
    TARGET_OUT_OF_WORKSPACE,
    classify,
)

OK = dict(in_workspace=True, start_in_collision=False, ik_found=True, plan_found=True)


def test_a_plan_that_worked_says_so():
    outcome = classify(**OK)

    assert outcome.success is True
    assert outcome.failure == NONE


def test_each_failure_is_reported_as_itself():
    assert classify(**{**OK, "plan_found": False}).failure == NO_PLAN_FOUND
    assert classify(**{**OK, "ik_found": False, "plan_found": False}).failure == NO_IK_SOLUTION
    assert classify(**{**OK, "start_in_collision": True,
                       "plan_found": False}).failure == START_STATE_IN_COLLISION
    assert classify(**{**OK, "in_workspace": False, "ik_found": False,
                       "plan_found": False}).failure == TARGET_OUT_OF_WORKSPACE


def test_the_outermost_cause_wins_when_several_are_true():
    """The whole reason the order is fixed.

    An unreachable target also has no IK solution and also finds no plan. If
    the checks ran the other way round, every diagnosis in this cell would read
    "no plan found", which is true and tells an operator nothing.
    """
    everything_wrong = dict(in_workspace=False, start_in_collision=True,
                            ik_found=False, plan_found=False)

    assert classify(**everything_wrong).failure == TARGET_OUT_OF_WORKSPACE

    reachable_but_stuck = {**everything_wrong, "in_workspace": True}
    assert classify(**reachable_but_stuck).failure == START_STATE_IN_COLLISION


def test_the_start_state_collision_that_cost_a_campaign_is_its_own_code():
    """23 consecutive trials on 2026-09-02 were recorded as motion failures.

    The arm was in a start-state collision it could not plan out of. That is a
    different fault from a grasp the cell cannot reach, and the CSV could not
    say so because the planner returned one boolean.
    """
    outcome = classify(in_workspace=True, start_in_collision=True,
                       ik_found=True, plan_found=False)

    assert outcome.failure == START_STATE_IN_COLLISION
    assert "already in collision" in outcome.message


def test_the_message_says_what_to_do_about_it():
    assert "planning time" in classify(**{**OK, "in_workspace": False}).message
    assert "attempt" in classify(**{**OK, "plan_found": False}, attempts=3).message
