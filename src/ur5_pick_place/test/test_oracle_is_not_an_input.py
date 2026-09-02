"""Simulator truth is scored against, never consumed.

`ground_truth_node` publishes where the parts REALLY are. That is legitimate
for a benchmark deciding whether a placement took, and it would be cheating
anywhere else: a cell that reads it is measuring a simulator rather than a
robot, and the failure is invisible because everything works beautifully.

`intralogistics-amr` writes this down as ADR 0006 and enforces nothing. This is
the enforcement, and it greps, so it proves nothing about a determined
reimplementation. What it does catch is the ordinary way it would happen: a
perception or planning module subscribing to the topic because the data was
right there.
"""

import sys
from pathlib import Path

import pytest

PKG = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG))
from ur5_pick_place.placement_guard import GROUND_TRUTH_TOPIC  # noqa: E402

#: Modules that drive the cell. Anything here reading the oracle is the fault.
CONTROL_PATH = (
    "pick_place_node.py",
    "detector_node.py",
    "perception.py",
    "segmentation.py",
    "grasp.py",
    "cell_actions.py",
    "part_animator.py",
)

#: Allowed to read it: the benchmark scores placements with it, the guard owns
#: the subscription, and the node itself publishes it.
EVALUATION = ("benchmark_placements.py", "placement_guard.py", "ground_truth_node.py")


def test_the_control_path_does_not_read_the_oracle():
    offenders = []
    for name in CONTROL_PATH:
        path = PKG / "ur5_pick_place" / name
        if not path.is_file():
            continue
        text = path.read_text(encoding="utf-8")
        if GROUND_TRUTH_TOPIC in text or "ground_truth" in text:
            offenders.append(name)

    assert not offenders, (
        f"{offenders} reads simulator ground truth. It is an evaluation oracle: "
        f"a cell that consumes it is measuring the simulator, and every test "
        f"will pass while it does.")


def test_the_evaluation_side_is_allowed_to_and_actually_does():
    """The other half, or this gate could pass by the topic not existing."""
    guard = (PKG / "ur5_pick_place" / "placement_guard.py").read_text(encoding="utf-8")

    assert GROUND_TRUTH_TOPIC in guard


@pytest.mark.parametrize("name", EVALUATION)
def test_the_allowed_list_names_files_that_exist(name):
    """An allow-list of missing files silently permits everything."""
    assert (PKG / "ur5_pick_place" / name).is_file()
