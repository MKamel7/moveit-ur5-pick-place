"""Say WHICH way a grasp plan failed, instead of returning False.

WHY THIS IS A SEPARATE, PURE MODULE

`pick_one` returns a boolean, so "the pose is unreachable", "the arm was
already in collision before it started" and "reachable, but no collision-free
path exists" arrive identically. The placement benchmark records all three as
`failed_stage="motion"`, and on 2026-09-02 that cost a campaign 23 consecutive
trials: the arm was in a start-state collision it could not plan out of, every
trial recorded a motion failure, and nothing in the data said the cell had
never been asked a question it could answer. A human read the MoveIt log to
find out. `PlanGrasp.action` already defines the four codes; this decides which
one applies, with no ROS, no MoveIt and no arm, so the decision is testable.

THE ORDER OF THE CHECKS IS THE POINT

They are not independent, and a wrong order produces a true-but-useless answer.
A target outside the workspace also has no IK solution, and an arm in collision
also fails to find a path, so reporting the LAST symptom rather than the FIRST
cause is how a diagnosis becomes "no plan found" for everything. Checks run
outermost cause first: workspace, then start state, then IK, then search.
"""

from __future__ import annotations

from dataclasses import dataclass

# Mirrors PlanGrasp.action. Kept as plain ints so this module imports nowhere.
NONE = 0
NO_IK_SOLUTION = 1
START_STATE_IN_COLLISION = 2
NO_PLAN_FOUND = 3
TARGET_OUT_OF_WORKSPACE = 4

_NAMES = {
    NONE: "none",
    NO_IK_SOLUTION: "no IK solution",
    START_STATE_IN_COLLISION: "start state in collision",
    NO_PLAN_FOUND: "no plan found",
    TARGET_OUT_OF_WORKSPACE: "target out of workspace",
}


@dataclass(frozen=True)
class PlanOutcome:
    """What happened, in the terms the action's caller can act on."""

    success: bool
    failure: int
    message: str

    @property
    def failure_name(self) -> str:
        return _NAMES[self.failure]


def classify(*, in_workspace: bool, start_in_collision: bool, ik_found: bool,
             plan_found: bool, attempts: int = 1) -> PlanOutcome:
    """Turn what the planner observed into one outcome.

    Every argument is something the caller already knows by the time it has
    tried, and none of them is a MoveIt type, so a test can produce any
    combination including the ones a real cell rarely reaches.
    """
    if not in_workspace:
        return PlanOutcome(False, TARGET_OUT_OF_WORKSPACE,
                           "the target is outside the arm's reachable volume, so no "
                           "amount of planning time will help")
    if start_in_collision:
        return PlanOutcome(False, START_STATE_IN_COLLISION,
                           "the arm is already in collision, so planning aborts before "
                           "it searches; clear the scene or move the arm first")
    if not ik_found:
        return PlanOutcome(False, NO_IK_SOLUTION,
                           "the target is reachable in position but no joint solution "
                           "satisfies it, usually orientation or a joint limit")
    if not plan_found:
        return PlanOutcome(False, NO_PLAN_FOUND,
                           f"a joint solution exists but no collision-free path to it "
                           f"was found in {attempts} attempt(s)")
    return PlanOutcome(True, NONE, "planned")
