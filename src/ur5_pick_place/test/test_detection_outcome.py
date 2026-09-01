#!/usr/bin/env python3
"""Perception's failure modes, told apart instead of collapsed into None.

`get_perceived_top` returns a pose or None, and None was read as "no object".
The case that motivated this file is the one where that reading is not merely
imprecise but wrong: `/detected/<colour>` is TRANSIENT_LOCAL, so a subscriber
gets the last pose ever published even when nothing is producing them. With
the Gazebo server dead, the topic answered instantly with a pose from before
the run, and the placement benchmark scored trials against it and reported a
126 mm perception error. A stale answer looks exactly like a good one.
"""

import sys
from pathlib import Path

import pytest

PKG = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG))
from ur5_pick_place.detection_outcome import (  # noqa: E402
    INVALID_COLOR,
    NO_DETECTION,
    NONE,
    PERCEPTION_UNAVAILABLE,
    STALE,
    classify,
)

LIVE = {"publisher_count": 1, "max_age_s": 1.0}


def test_a_fresh_pose_from_a_live_publisher_succeeds() -> None:
    outcome = classify("green", pose_received=True, age_s=0.05, **LIVE)

    assert outcome.success is True
    assert outcome.failure == NONE


def test_a_stale_pose_is_not_a_detection() -> None:
    """The headline case: retained, plausible, and describing a scene that is gone."""
    outcome = classify("green", pose_received=True, age_s=180.0, **LIVE)

    assert outcome.success is False
    assert outcome.failure == STALE
    assert "180" in outcome.message


def test_a_pose_with_no_timestamp_cannot_be_trusted() -> None:
    """Unverifiable freshness is treated as stale, not as fresh."""
    outcome = classify("green", pose_received=True, age_s=None, **LIVE)

    assert outcome.success is False
    assert outcome.failure == STALE


def test_a_dead_publisher_outranks_a_retained_pose() -> None:
    """Holding a latched pose from a publisher that no longer exists is the trap."""
    outcome = classify("green", pose_received=True, age_s=0.01, publisher_count=0)

    assert outcome.success is False
    assert outcome.failure == PERCEPTION_UNAVAILABLE


def test_an_empty_scene_is_distinct_from_a_broken_pipeline() -> None:
    """Both used to be None. They send an operator to different places."""
    empty = classify("green", pose_received=False, age_s=None, **LIVE)
    broken = classify("green", pose_received=False, age_s=None, publisher_count=0)

    assert empty.failure == NO_DETECTION
    assert broken.failure == PERCEPTION_UNAVAILABLE
    assert empty.failure != broken.failure


@pytest.mark.parametrize("color", ["red", "green", "blue"])
def test_the_three_real_colours_are_accepted(color: str) -> None:
    assert classify(color, True, 0.1, **LIVE).success is True


def test_an_unknown_colour_is_a_caller_bug_not_an_empty_scene() -> None:
    outcome = classify("purple", pose_received=False, age_s=None, **LIVE)

    assert outcome.failure == INVALID_COLOR


def test_the_age_budget_is_a_boundary_not_a_suggestion() -> None:
    assert classify("green", True, 1.0, publisher_count=1, max_age_s=1.0).success is True
    assert classify("green", True, 1.01, publisher_count=1, max_age_s=1.0).success is False


def test_no_failure_path_ever_reports_success() -> None:
    """The property behind the table above."""
    for received in (True, False):
        for age in (None, 0.0, 0.5, 5.0):
            for pubs in (0, 1):
                for color in ("green", "purple"):
                    outcome = classify(color, received, age, pubs)
                    if outcome.failure != NONE:
                        assert outcome.success is False, outcome
                    if outcome.success:
                        assert outcome.message == ""
