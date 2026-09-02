"""What an aborted cycle left behind, which a boolean cannot say."""

import sys
from pathlib import Path

PKG = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG))
from ur5_pick_place.pick_stages import (  # noqa: E402
    ORDER,
    STAGE_APPROACH,
    STAGE_COMPLETE,
    STAGE_DESCEND,
    STAGE_GRASP,
    STAGE_LIFT,
    STAGE_NONE,
    STAGE_RELEASE,
    STAGE_RETREAT,
    holds_part,
    name,
    next_stage,
    world_changed,
)


def test_the_stages_run_in_the_order_the_cell_executes_them():
    assert next_stage(STAGE_NONE) == STAGE_APPROACH
    assert next_stage(STAGE_APPROACH) == STAGE_DESCEND
    assert next_stage(STAGE_RETREAT) == STAGE_COMPLETE
    assert next_stage(STAGE_COMPLETE) == STAGE_COMPLETE


def test_walking_the_whole_sequence_ends_once():
    stage = STAGE_NONE
    seen = []
    for _ in range(len(ORDER)):
        stage = next_stage(stage)
        seen.append(stage)

    assert seen == list(ORDER)
    assert seen[-1] == STAGE_COMPLETE


def test_an_abort_before_the_grasp_leaves_the_world_alone():
    for stage in (STAGE_NONE, STAGE_APPROACH, STAGE_DESCEND):
        assert holds_part(stage) is False
        assert world_changed(stage) is False


def test_an_abort_while_carrying_says_the_gripper_is_holding_something():
    for stage in (STAGE_GRASP, STAGE_LIFT):
        assert holds_part(stage) is True
        assert world_changed(stage) is False


def test_after_release_the_part_has_moved_even_if_the_arm_has_not_finished():
    """Retreat failed in six of eight campaign failures, and by then the part
    was already on the conveyor. Recovery is not "try again"."""
    for stage in (STAGE_RELEASE, STAGE_RETREAT, STAGE_COMPLETE):
        assert world_changed(stage) is True
        assert holds_part(stage) is False


def test_every_stage_has_a_name_for_the_log():
    for stage in (STAGE_NONE, *ORDER):
        assert name(stage)
