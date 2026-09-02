"""The stages of a pick, and what reaching each one means.

WHY A CYCLE REPORTS A STAGE AND NOT A BOOLEAN

`ExecutePick.action` says it plainly: a cycle that fails on the descent is a
reachability problem, one that fails on the retreat has already moved the part
and left the cell in a different state than it started in, and one vetoed by
the safety supervisor is not a robot fault at all. Recovery differs for each.
The 100-trial campaign of 2026-09-02 could only say `failed_stage="motion"` for
all eight of its failures, and finding out that every one of them was a `lift`
or a `retreat` meant grepping MoveIt's log afterwards.

This module is the stage list and the arithmetic on it, with no ROS in it, so
the ordering and the "did the part move" question can be tested directly.
"""

from __future__ import annotations

# Mirrors ExecutePick.action.
STAGE_NONE = 0
STAGE_APPROACH = 1
STAGE_DESCEND = 2
STAGE_GRASP = 3
STAGE_LIFT = 4
STAGE_TRANSFER = 5
STAGE_RELEASE = 6
STAGE_RETREAT = 7
STAGE_COMPLETE = 8

ORDER = (STAGE_APPROACH, STAGE_DESCEND, STAGE_GRASP, STAGE_LIFT,
         STAGE_TRANSFER, STAGE_RELEASE, STAGE_RETREAT, STAGE_COMPLETE)

NAMES = {
    STAGE_NONE: "none",
    STAGE_APPROACH: "approach",
    STAGE_DESCEND: "descend",
    STAGE_GRASP: "grasp",
    STAGE_LIFT: "lift",
    STAGE_TRANSFER: "transfer",
    STAGE_RELEASE: "release",
    STAGE_RETREAT: "retreat",
    STAGE_COMPLETE: "complete",
}

#: After this stage the gripper holds the part, so an abort leaves the cell
#: holding something. Before it, an abort leaves the world as it was found.
FIRST_HOLDING_STAGE = STAGE_GRASP

#: After this stage the part has been let go somewhere other than where it
#: started, so the world has changed even though the arm may be mid-motion.
FIRST_MOVED_STAGE = STAGE_RELEASE


def name(stage: int) -> str:
    return NAMES[stage]


def next_stage(stage: int) -> int:
    """The stage that follows, or COMPLETE at the end."""
    if stage == STAGE_NONE:
        return ORDER[0]
    if stage == STAGE_COMPLETE:
        return STAGE_COMPLETE
    return ORDER[ORDER.index(stage) + 1]


def holds_part(stage: int) -> bool:
    """True if a cycle stopped here would leave the gripper holding the part."""
    return FIRST_HOLDING_STAGE <= stage < FIRST_MOVED_STAGE


def world_changed(stage: int) -> bool:
    """True if the part is no longer where the cycle found it.

    The question an operator asks first after an aborted cycle, and the one a
    boolean cannot answer: whether recovery means "try again" or "go and look
    at what the cell is holding".
    """
    return stage >= FIRST_MOVED_STAGE
