#!/usr/bin/env python3
"""Turn the placement benchmark's CSV into the numbers a reader can check.

WHY THIS IS SEPARATE FROM THE BENCHMARK

`benchmark_placements.py` prints a summary at the end of a run, and that summary
dies with the process. The campaign that produces the headline figure takes over
an hour and runs once, so the arithmetic behind the figure has to be
regenerable from the committed CSV alone, by anyone, without ROS, MoveIt or
Gazebo installed. That is also what lets a test assert the README's numbers
against the data instead of against a memory of the run.

WHAT IT REPORTS, AND WHY EACH ONE IS SEPARATE

The roadmap asks for detection success, position error at p50 and p95, grasp
success and end to end cycle success, and those are four different questions:

  detection rate     did perception find the part at all
  perception error   how far its answer was from where the part was commanded,
                     over the trials where it answered
  motion success     of the trials perception handed to the planner, how many
                     the arm actually picked and placed
  end to end rate    of all trials, how many completed, which is the only
                     figure the acceptance criterion is about

Reporting only the last one hides which half is weak; reporting only the first
flatters the cell. A trial that fails at `placement` is a trial whose part never
reached the commanded pose, and it is counted as a failure of the run rather
than of the robot, which is the conservative reading and cannot inflate
anything.

PERCENTILES ON 100 SAMPLES

Nearest rank, no interpolation. With n=100 the p95 is the 95th smallest value
and nothing is invented between samples; interpolating between two measurements
would produce a number that was never measured.

Run it:

    python3 tools/summarise_benchmark.py docs/benchmark_placements.csv
    python3 tools/summarise_benchmark.py docs/benchmark_placements.csv --markdown
"""
from __future__ import annotations

import argparse
import csv
import math
from dataclasses import dataclass
from pathlib import Path

# Every stage a trial can fail at. `recovery` and `timeout` are harness faults
# and the rest are the cell; the README gate depends on telling them apart.
STAGES = ("recovery", "reset", "set_pose", "perception", "placement", "motion", "timeout")


def percentile(values: list[float], q: float) -> float:
    """Nearest-rank percentile, q in 0..100. Empty input is an error, not 0.0."""
    if not values:
        raise ValueError("no values to take a percentile of")
    ordered = sorted(values)
    rank = max(1, math.ceil(q / 100.0 * len(ordered)))
    return ordered[rank - 1]


@dataclass
class Summary:
    trials: int
    detected: int
    handed_to_planner: int
    motion_ok: int
    end_to_end: int
    err_p50: float | None
    err_p95: float | None
    err_max: float | None
    failures: dict[str, int]
    seconds_total: float
    seconds_median: float

    @property
    def detection_rate(self) -> float:
        return self.detected / self.trials

    @property
    def motion_rate(self) -> float | None:
        """Success over the trials the planner was actually given a pose for."""
        if self.handed_to_planner == 0:
            return None
        return self.motion_ok / self.handed_to_planner

    @property
    def end_to_end_rate(self) -> float:
        return self.end_to_end / self.trials


def summarise(rows: list[dict]) -> Summary:
    if not rows:
        raise ValueError("the CSV has no trials in it")

    def truthy(value: str) -> bool:
        return value.strip().lower() == "true"

    detected = [r for r in rows if truthy(r["detected"])]
    # A trial only reaches the planner if perception answered AND the placement
    # was on the table. `perception_error_mm` is written exactly then, so its
    # presence is the honest test of what the planner was asked to do.
    handed = [r for r in detected if r["perception_error_mm"]]
    errs = [float(r["perception_error_mm"]) for r in handed]
    wins = [r for r in rows if truthy(r["success"])]
    times = [float(r["seconds"]) for r in rows]

    return Summary(
        trials=len(rows),
        detected=len(detected),
        handed_to_planner=len(handed),
        motion_ok=sum(1 for r in handed if truthy(r["success"])),
        end_to_end=len(wins),
        err_p50=percentile(errs, 50) if errs else None,
        err_p95=percentile(errs, 95) if errs else None,
        err_max=max(errs) if errs else None,
        failures={s: sum(1 for r in rows if r["failed_stage"] == s) for s in STAGES},
        seconds_total=sum(times),
        seconds_median=percentile(times, 50),
    )


def read_csv(path: Path) -> list[dict]:
    with path.open(newline="", encoding="utf-8") as fh:
        return list(csv.DictReader(fh))


def as_text(s: Summary) -> str:
    lines = [
        f"trials                {s.trials}",
        f"detection             {s.detected}/{s.trials}  ({100 * s.detection_rate:.0f}%)",
        f"reached the planner   {s.handed_to_planner}/{s.trials}",
    ]
    if s.motion_rate is not None:
        lines.append(
            f"pick and place        {s.motion_ok}/{s.handed_to_planner}  "
            f"({100 * s.motion_rate:.0f}% of the trials perception handed over)"
        )
    lines.append(
        f"end to end            {s.end_to_end}/{s.trials}  ({100 * s.end_to_end_rate:.0f}%)"
    )
    if s.err_p50 is not None:
        lines.append(
            f"perception error      p50 {s.err_p50:.1f} mm, p95 {s.err_p95:.1f} mm, "
            f"worst {s.err_max:.1f} mm, n={s.handed_to_planner}"
        )
    for stage, count in s.failures.items():
        if count:
            lines.append(f"failed at {stage:<11} {count}")
    lines.append(
        f"wall clock            {s.seconds_total / 60:.0f} min, median trial "
        f"{s.seconds_median:.0f} s"
    )
    return "\n".join(lines)


def as_markdown(s: Summary) -> str:
    rows = [
        ("trials", f"{s.trials}, seeded and reproducible"),
        ("detection", f"**{s.detected} of {s.trials}**"),
        ("end to end pick and place", f"**{s.end_to_end} of {s.trials}**"),
    ]
    if s.motion_rate is not None:
        rows.append(
            ("pick and place, given a pose", f"{s.motion_ok} of {s.handed_to_planner}")
        )
    if s.err_p50 is not None:
        rows.append(
            (
                "perception error",
                f"p50 **{s.err_p50:.1f} mm**, p95 **{s.err_p95:.1f} mm**, "
                f"worst {s.err_max:.1f} mm",
            )
        )
    for stage, count in s.failures.items():
        if count:
            rows.append((f"failed at {stage}", str(count)))
    rows.append(("median trial", f"{s.seconds_median:.0f} s"))

    out = ["| | result |", "|---|---|"]
    out += [f"| {name} | {value} |" for name, value in rows]
    return "\n".join(out)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("csv", type=Path, nargs="?",
                        default=Path("docs/benchmark_placements.csv"))
    parser.add_argument("--markdown", action="store_true",
                        help="emit the README table instead of the plain summary")
    args = parser.parse_args()

    s = summarise(read_csv(args.csv))
    print(as_markdown(s) if args.markdown else as_text(s))


if __name__ == "__main__":
    main()
