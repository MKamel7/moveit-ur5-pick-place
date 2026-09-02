#!/usr/bin/env python3
"""A placement that did not take must not be measurable.

The defect this guards against is not hypothetical. In a 100-trial grasp
campaign started on 2026-09-01, trials 4 and 10 were scored against
`part_green` sitting at its home pose of (0.55, 0.17) rather than at the pose
the row records, because `part_animator`'s reset landed after the placement.
Both rows looked ordinary: perception found a green cube, the cube was on the
table, and one of the two trials scored SUCCESS with a 161.8 mm "perception
error". Nothing in the run said anything was wrong.

So the interesting test here is not that a correct placement passes. It is that
a part at its home pose FAILS the check while a part at the commanded pose
passes, since those two cases were indistinguishable to the benchmark before.
"""

import sys
import time
from pathlib import Path

PKG = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG))
from ur5_pick_place.placement_guard import (  # noqa: E402
    PLACEMENT_TOL_M,
    at_pose,
    wait_until_at,
)

PART_GREEN_HOME = (0.55, 0.17, 0.225)
COMMANDED = (0.6295, 0.0654)  # trial 4 of the campaign that found this


class FakeTruth:
    """A pose feed a test controls, standing in for Gazebo's."""

    def __init__(self, poses: dict, changes_to: dict | None = None, after_s: float = 0.0):
        self._poses = poses
        self._changes_to = changes_to
        self._at = time.time() + after_s

    def get(self, name):
        if self._changes_to is not None and time.time() >= self._at:
            return self._changes_to.get(name)
        return self._poses.get(name)


def test_a_part_at_the_commanded_pose_passes() -> None:
    assert at_pose((0.6295, 0.0654, 0.225), *COMMANDED) is True


def test_the_home_pose_that_scored_a_success_is_rejected() -> None:
    """Trial 10's part was here, and the CSV called it a 161.8 mm error."""
    assert at_pose(PART_GREEN_HOME, *COMMANDED) is False


def test_a_missing_pose_is_not_a_placement() -> None:
    """No pose feed at all must not read as agreement, the way a dead topic did."""
    assert at_pose(None, *COMMANDED) is False


def test_the_tolerance_is_a_boundary_not_a_suggestion() -> None:
    x, y = COMMANDED
    just_inside = (x + PLACEMENT_TOL_M * 0.9, y, 0.225)
    just_outside = (x + PLACEMENT_TOL_M * 1.1, y, 0.225)

    assert at_pose(just_inside, x, y) is True
    assert at_pose(just_outside, x, y) is False


def test_waiting_gives_up_rather_than_blocking_a_campaign() -> None:
    """A part that never arrives costs a bounded wait, not a hung run."""
    truth = FakeTruth({"part_green": PART_GREEN_HOME})

    started = time.time()
    assert wait_until_at(truth, "part_green", *COMMANDED, timeout_s=0.3) is False
    assert time.time() - started < 2.0


def test_waiting_returns_as_soon_as_the_pose_lands() -> None:
    """The common case: the placement takes a moment, and the wait ends with it."""
    truth = FakeTruth(
        {"part_green": PART_GREEN_HOME},
        changes_to={"part_green": (*COMMANDED, 0.225)},
        after_s=0.1,
    )

    started = time.time()
    assert wait_until_at(truth, "part_green", *COMMANDED, timeout_s=3.0) is True
    assert time.time() - started < 1.0
