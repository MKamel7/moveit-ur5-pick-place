# Industrial pick-and-place: engineering plan

How this cell moves the arm from a supply bin to a pallet the way an industrial
robotics engineer would, and the fixes that take it from a first working demo to
a master's-level, CV-worthy result.

## The professional approach

A production palletizing cell does **not** plan every motion with a random
sampling planner. It uses:

1. **Deterministic industrial motion.** The
   [Pilz Industrial Motion Planner](https://moveit.picknik.ai/main/doc/how_to_guides/pilz_industrial_motion_planner/pilz_industrial_motion_planner.html)
   is the industry-standard: `PTP` (point-to-point, synchronised trapezoidal
   joint motion) for transfers and `LIN` (straight-line Cartesian) for approach
   and retreat. Motions are smooth, repeatable, and fast, the same profile a
   real UR/KUKA palletizer runs.
2. **Defined grasps.** A fixed top-down grasp pose relative to the part, with a
   pre-grasp standoff and a `LIN` approach straight down, then a `LIN` retreat
   straight up. The tool never arrives at a random angle.
3. **Consistent kinematics.** One tool orientation and a consistent IK seed, so
   the arm never flips its wrist or swings behind its base between picks.
4. **Attach on contact.** The part is attached to the gripper *by id at its real
   pose* the instant the gripper reaches it, so it rigidly follows the tool.
   Nothing teleports.
5. **Time-optimal parameterization.** Velocity/acceleration scaling gives a fast
   trajectory that still respects joint limits.

## Problems in the first version and their fixes

| Observed | Root cause | Fix |
| --- | --- | --- |
| Slow (~100 s/part) | OMPL sampling + long retries | Pilz `PTP`/`LIN`, higher velocity scaling |
| Wonky angles | position-only goals, unconstrained IK | fixed top-down grasp + `LIN` approach/retreat |
| Part teleports into the gripper | part removed and re-added at a fixed tool offset | attach **by id** at the true grasp pose |
| Flaky success (2-4 of 4) | non-deterministic planner | deterministic Pilz = repeatable |

## Steps

1. Enable the Pilz pipeline (+ `pilz_cartesian_limits.yaml`) alongside OMPL.
2. Rewrite the motion: `PTP` transfers to top-down poses, `LIN` for
   approach/descend/retreat; drop position-only goals and the sampling retries.
3. Attach the part by id at contact (no teleport).
4. Tune velocity for a fast, smooth cycle; verify reliability over repeated runs.
5. Update the README and record the demo.

## Beyond this (roadmap)

- **MoveIt Task Constructor** for task-level pick-and-place planning (grasp
  generation, multiple candidate grasps, stage-based failure handling).
- **Perception-driven grasps** (Phase 3): an RGB-D camera and pose estimation
  feed real grasp poses instead of known bin cells.
