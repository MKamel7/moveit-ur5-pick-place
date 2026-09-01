"""Measure the pick-and-place success rate over randomised part placements.

This exists because the project claimed a success rate it had never measured.
The acceptance criterion was "the arm detects an object with the camera,
estimates its pose, and executes a collision-free pick-and-place with >=8/10
success over randomised object placements", and every other part of that bar was
demonstrable while the number itself was not. A claimed-but-unmeasured figure is
worth less than a measured bad one, so this runs the whole loop N times and
writes down what happened, including the failures.

What one trial does, which is the full criterion and not a shortcut:

  1. Move ``part_green`` to a random pose on the table through the Gazebo
     ``set_pose`` service, the same mechanism ``part_animator`` uses.
  2. Wait for the perception node to publish that part on ``/detected/green``.
     A timeout here is a REAL failure and is recorded as one, not retried away.
  3. Run the existing ``pick_one`` from ``pick_place_node``: OMPL plans a
     collision-aware top-down grasp, the part is attached, carried to the belt,
     released, and the arm retreats.
  4. Record the outcome and the stage it failed at, plus the perception error,
     which is the distance between where the part was commanded and where
     perception said it was. That second number is free here and is the more
     interesting one: it separates "the planner failed" from "the planner was
     aimed at the wrong place".

Run it (the simulation must already be up):

    ros2 launch ur_simulation_gz ur_sim_moveit.launch.py ur_type:=ur5e
    ros2 launch ur5_pick_place perception.launch.py
    ros2 launch ur5_pick_place benchmark_placements.launch.py

Configure with environment variables:

    BENCH_TRIALS   number of trials, default 20
    BENCH_SEED     RNG seed, default 0, so a run is reproducible
    BENCH_COLOR    which part to use, default green
    BENCH_CSV      output path, default docs/benchmark_placements.csv

The seed matters. An unseeded benchmark cannot be re-run to check a fix, and a
success rate you cannot reproduce is an anecdote.
"""
from __future__ import annotations

import csv
import math
import os
import random
import time
from dataclasses import asdict, dataclass
from pathlib import Path

import rclpy
from moveit.planning import MoveItPy
from rclpy.logging import get_logger

from ur5_pick_place.pick_place_node import (
    OBJECT_SIZE,
    PLANNING_GROUP,
    READY_STATE,
    _apply_front_constraint,
    _build_static_scene,
    _go_to_named,
    get_perceived_top,
    pick_one,
)

logger = get_logger("benchmark_placements")

WORLD_NAME = "pick_place"

# Sampling window on the table top, in base_link metres.
#
# The table is centred at (0.55, 0.15) with size (0.40, 0.55), so its top spans
# x 0.35..0.75 and y -0.125..0.425. The part is a 50 mm cube, so its centre must
# stay at least 25 mm inside each edge or it hangs off. MARGIN is deliberately
# larger than that: a part balanced on the lip is not a placement the cell is
# supposed to handle, and including it would make the benchmark measure the
# sampler rather than the robot.
MARGIN = 0.06
X_MIN, X_MAX = 0.35 + MARGIN, 0.75 - MARGIN
Y_MIN, Y_MAX = -0.125 + MARGIN, 0.425 - MARGIN

# The table top itself, which is the loosest true statement about where a
# sampled part can legitimately be. Used to reject a trial whose placement never
# took: a part perceived off the table was not presented at the pose the CSV is
# about to record, whatever the arm then did with it.
TABLE_X_MIN, TABLE_X_MAX = 0.35, 0.75
TABLE_Y_MIN, TABLE_Y_MAX = -0.125, 0.425

# Table top 0.20 + half the 50 mm cube.
PART_CENTRE_Z = 0.20 + OBJECT_SIZE[2] / 2.0

# Where the two unused parts are parked so they cannot occlude the camera view
# of the sampled one or block a plan. Off the table, on the floor, out of reach.
PARKED = {
    "red": (0.05, -0.75, 0.03),
    "green": (0.05, -0.85, 0.03),
    "blue": (0.05, -0.95, 0.03),
}

PERCEPTION_TIMEOUT_S = 8.0


@dataclass
class Trial:
    """One randomised placement and what became of it."""

    trial: int
    commanded_x: float
    commanded_y: float
    detected: bool
    perceived_x: float | None
    perceived_y: float | None
    perception_error_mm: float | None
    success: bool
    failed_stage: str
    seconds: float


def _gz_set_pose(gz_node, name: str, x: float, y: float, z: float) -> bool:
    """Move a Gazebo model, the same way part_animator does. True if accepted."""
    from gz.msgs10.boolean_pb2 import Boolean
    from gz.msgs10.pose_pb2 import Pose as GzPose

    req = GzPose()
    req.name = name
    req.position.x = float(x)
    req.position.y = float(y)
    req.position.z = float(z)
    req.orientation.w = 1.0
    try:
        # The result IS relied on. It used to be discarded, on the reasoning
        # that part_animator ignores it too, and that turned a dead simulator
        # into a clean run: with the Gazebo server gone the service answered
        # nothing, every placement was reported accepted, and the benchmark
        # wrote a CSV claiming 3/3 detections and a 126 mm "perception error"
        # that was really the distance from a random sample to the part's
        # unmoved home pose. A benchmark that cannot tell a stopped simulation
        # from a successful placement is measuring its own sampler.
        result = gz_node.request(f"/world/{WORLD_NAME}/set_pose", req, GzPose, Boolean, 200)
    except Exception as exc:  # noqa: BLE001
        logger.warn(f"set_pose raised for {name}: {exc}")
        return False

    # gz request() answers (ok, reply); on a failed call the reply is absent.
    accepted = bool(result[0]) if isinstance(result, tuple) else bool(result)
    if not accepted:
        logger.warn(f"set_pose refused for {name}: is the Gazebo server running?")
    return accepted


def _reset_animator() -> None:
    """Clear part_animator's belt state before a placement is commanded.

    WHY THIS IS NOT OPTIONAL. part_animator keeps every released part in an
    ``on_belt`` dict and re-poses each one at 30 Hz, so a part put back on the
    table by ``set_pose`` is dragged onto the conveyor within a frame. The
    benchmark already builds a ``/carry_cmd`` publisher for pick_one and simply
    never used the ``reset`` op the animator provides for exactly this.

    The cost of leaving it out was not a failed run, which is why it survived:
    a three-trial run scored 3/3 SUCCESS while only the first trial presented
    the placement it recorded. Trials 2 and 3 were perceived at x 0.5007 and
    0.4847, which is BELT_X, with negative y, which is off the table. The arm
    really did pick the part and really did place it, so every stage reported
    success; it was picking from the conveyor at a near-fixed pose while the
    CSV recorded a random table pose. That is a 100% success rate for a
    randomised-placement criterion measured without randomised placements.
    """
    from std_msgs.msg import String

    import ur5_pick_place.pick_place_node as ppn

    msg = String()
    msg.data = "reset"
    ppn._carry_pub.publish(msg)


def _run_trial(robot: MoveItPy, arm, gz_node, index: int, rng: random.Random, color: str) -> Trial:
    started = time.time()
    x = rng.uniform(X_MIN, X_MAX)
    y = rng.uniform(Y_MIN, Y_MAX)

    # Drop any belt state from the previous trial before touching poses, or the
    # animator will overwrite the placement this trial is about to command.
    _reset_animator()
    time.sleep(0.3)

    # Park every part, then place only the sampled one. Parking first means a
    # trial cannot inherit the previous trial's leftover position.
    for other, home in PARKED.items():
        _gz_set_pose(gz_node, f"part_{other}", *home)
    time.sleep(0.4)
    if not _gz_set_pose(gz_node, f"part_{color}", x, y, PART_CENTRE_Z):
        return Trial(index, x, y, False, None, None, None, False, "set_pose", time.time() - started)

    # Give the camera and detector time to see the moved part before asking.
    time.sleep(1.2)

    top = get_perceived_top(f"/detected/{color}", timeout_s=PERCEPTION_TIMEOUT_S)
    if top is None:
        # Not retried and not replaced with the fallback pose. The criterion says
        # the arm detects the object with the camera, so a miss is a failure of
        # the thing being measured.
        return Trial(index, x, y, False, None, None, None, False, "perception",
                     time.time() - started)

    # A trial whose placement did not take must not be scored, in either
    # direction. The belt regression scored as SUCCESS because every stage
    # genuinely worked on a part that was not where the row says it was, and a
    # 300 mm miss on a 50 mm cube lying on a table is not a perception result,
    # it is a part somewhere else. Recording it as a failure is the conservative
    # reading and the one that cannot flatter the success rate.
    #
    # HONEST LIMIT: this asks perception where the part is, and perception is
    # part of what is under test, so a genuinely broken detector would be
    # attributed here rather than to itself. That is the wrong label but the
    # right verdict, and it fails loudly instead of passing quietly.
    if not (TABLE_X_MIN <= top[0] <= TABLE_X_MAX and TABLE_Y_MIN <= top[1] <= TABLE_Y_MAX):
        logger.warn(
            f"trial {index}: part perceived at ({top[0]:.3f}, {top[1]:.3f}), off the table; "
            f"the placement did not take"
        )
        return Trial(index, x, y, True, round(top[0], 4), round(top[1], 4), None, False,
                     "placement", time.time() - started)

    err_mm = math.hypot(top[0] - x, top[1] - y) * 1000.0
    ok = pick_one(robot, arm, top, f"part_{color}")
    _go_to_named(robot, arm, READY_STATE, "home")

    return Trial(
        trial=index,
        commanded_x=round(x, 4),
        commanded_y=round(y, 4),
        detected=True,
        perceived_x=round(top[0], 4),
        perceived_y=round(top[1], 4),
        perception_error_mm=round(err_mm, 2),
        success=ok,
        failed_stage="" if ok else "motion",
        seconds=round(time.time() - started, 2),
    )


def _write_csv(trials: list[Trial], csv_path: Path) -> None:
    """Rewrite the whole CSV from the trials so far.

    Rewritten rather than appended so the file is always complete and always
    has its header, whatever killed the previous run. At 100 to 500 rows the
    cost is irrelevant next to a 40 s trial.
    """
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    with csv_path.open("w", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(fh, fieldnames=list(asdict(trials[0]).keys()))
        writer.writeheader()
        writer.writerows(asdict(t) for t in trials)


def _summarise(trials: list[Trial], csv_path: Path) -> None:
    n = len(trials)
    if n == 0:
        logger.error("no trials ran")
        return
    wins = sum(t.success for t in trials)
    detected = sum(t.detected for t in trials)
    errs = [t.perception_error_mm for t in trials if t.perception_error_mm is not None]

    logger.info("=" * 62)
    logger.info(f"success           {wins}/{n}  ({100.0 * wins / n:.0f}%)")
    logger.info(f"detected          {detected}/{n}")
    if errs:
        errs_sorted = sorted(errs)
        logger.info(
            f"perception error  median {errs_sorted[len(errs) // 2]:.1f} mm, "
            f"worst {max(errs):.1f} mm, n={len(errs)}"
        )
    for stage in ("set_pose", "perception", "placement", "motion"):
        count = sum(t.failed_stage == stage for t in trials)
        if count:
            logger.info(f"failed at {stage:<10} {count}")
    logger.info(f"written to        {csv_path}")
    logger.info("=" * 62)


def main() -> None:
    trials_n = int(os.environ.get("BENCH_TRIALS", "20"))
    seed = int(os.environ.get("BENCH_SEED", "0"))
    color = os.environ.get("BENCH_COLOR", "green")
    csv_path = Path(os.environ.get("BENCH_CSV", "docs/benchmark_placements.csv"))

    rclpy.init()
    from gz.transport13 import Node as GzNode

    gz_node = GzNode()

    # pick_one() signals the animator over /carry_cmd so a grasped part follows
    # the tool and a placed one rides the belt. That publisher normally gets set
    # up in pick_place_node.main(), which is not running here, so wire it or
    # every trial would move the arm while the part sat still.
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from std_msgs.msg import String

    import ur5_pick_place.pick_place_node as ppn

    _comm = rclpy.create_node("benchmark_comm")
    _qos = QoSProfile(depth=1)
    _qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    ppn._carry_pub = _comm.create_publisher(String, "/carry_cmd", _qos)

    rng = random.Random(seed)
    robot = MoveItPy(node_name="benchmark_placements")
    arm = robot.get_planning_component(PLANNING_GROUP)

    logger.info(f"{trials_n} trials, seed {seed}, part '{color}'")
    results: list[Trial] = []
    try:
        _apply_front_constraint(arm)
        _go_to_named(robot, arm, READY_STATE, "ready")
        _build_static_scene(robot)
        time.sleep(0.5)

        for i in range(1, trials_n + 1):
            t = _run_trial(robot, arm, gz_node, i, rng, color)
            results.append(t)
            # Written now, not at the end. The `finally` below cannot be relied
            # on: this node spends nearly all its time blocked inside MoveIt's
            # C++ execution, where a SIGINT is not delivered until control
            # returns to the interpreter. A campaign that hung on trial 31 of
            # 100 refused SIGINT for the 10 s launch allows, was SIGKILLed, and
            # lost all 30 completed trials, which is exactly what the comment
            # on the `finally` block promised would not happen. Flushing per
            # trial costs one file write per 40 s and bounds the loss at one row.
            _write_csv(results, csv_path)
            logger.info(
                f"trial {i}/{trials_n}: {'ok' if t.success else 'FAIL ' + t.failed_stage} "
                f"at ({t.commanded_x:.3f}, {t.commanded_y:.3f}) in {t.seconds:.1f}s"
            )
    finally:
        # A last write and the summary. The CSV is already current from the
        # per-trial flush above, so this is belt and braces rather than the
        # only chance to keep the data, which is what it used to be.
        if results:
            _write_csv(results, csv_path)
            _summarise(results, csv_path)
        try:
            time.sleep(1.0)
            robot.shutdown()
        except Exception as exc:  # noqa: BLE001
            logger.warn(f"shutdown raised (ignored): {exc}")
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
