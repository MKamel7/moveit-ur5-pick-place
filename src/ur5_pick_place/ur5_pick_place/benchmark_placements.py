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
        # Argument order and types mirror part_animator._set_pose exactly. The
        # return shape of gz request() is not relied on, because part_animator
        # ignores it too and this is not the place to discover it differs.
        gz_node.request(f"/world/{WORLD_NAME}/set_pose", req, GzPose, Boolean, 200)
        return True
    except Exception as exc:  # noqa: BLE001
        logger.warn(f"set_pose failed for {name}: {exc}")
        return False


def _run_trial(robot: MoveItPy, arm, gz_node, index: int, rng: random.Random, color: str) -> Trial:
    started = time.time()
    x = rng.uniform(X_MIN, X_MAX)
    y = rng.uniform(Y_MIN, Y_MAX)

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
        return Trial(index, x, y, False, None, None, None, False, "perception", time.time() - started)

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
    for stage in ("set_pose", "perception", "motion"):
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
            logger.info(
                f"trial {i}/{trials_n}: {'ok' if t.success else 'FAIL ' + t.failed_stage} "
                f"at ({t.commanded_x:.3f}, {t.commanded_y:.3f}) in {t.seconds:.1f}s"
            )
    finally:
        # Write whatever ran. A benchmark interrupted at trial 13 of 20 still has
        # 13 results worth keeping, and losing them to a tidy exit is a bad trade.
        if results:
            csv_path.parent.mkdir(parents=True, exist_ok=True)
            with csv_path.open("w", newline="", encoding="utf-8") as fh:
                writer = csv.DictWriter(fh, fieldnames=list(asdict(results[0]).keys()))
                writer.writeheader()
                writer.writerows(asdict(t) for t in results)
            _summarise(results, csv_path)
        try:
            time.sleep(1.0)
            robot.shutdown()
        except Exception as exc:  # noqa: BLE001
            logger.warn(f"shutdown raised (ignored): {exc}")
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
