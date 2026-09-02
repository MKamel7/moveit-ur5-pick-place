#!/usr/bin/env python3
"""The campaign figures in the README are the ones in the CSV.

WHY THIS GATE EXISTS

The README is about to quote a success rate, a detection rate and a perception
error. Every one of those is a number a reader cannot check without rerunning an
hour of simulation, which is exactly the kind of claim this project's own rules
say must be regenerable from a checkout. `docs/benchmark_placements.csv` is
committed, so the arithmetic behind each figure is recomputable in milliseconds,
and this fails the build when the prose and the data disagree.

It guards a real failure mode rather than a hypothetical one. This project has
already published, in an earlier campaign, a row claiming a 161.8 mm perception
error against a part that was never at the pose the row recorded, and a 3/3
success rate measured without randomised placements. A number in a README ages
the moment the data under it is regenerated; nothing else here notices.

Verify it by falsification, per the house rule: change one digit in the README
table and this must go red.
"""

import importlib.util
import re
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
README = ROOT / "README.md"
CSV = ROOT / "docs" / "benchmark_placements.csv"


def _summariser():
    """Load tools/summarise_benchmark.py, which is not an installed module."""
    spec = importlib.util.spec_from_file_location(
        "summarise_benchmark", ROOT / "tools" / "summarise_benchmark.py"
    )
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def measured():
    sb = _summariser()
    return sb.summarise(sb.read_csv(CSV))


@pytest.fixture(scope="module")
def quoted():
    text = README.read_text(encoding="utf-8")
    section = re.search(
        r"## Randomised placement campaign(.*?)(?=\n## )", text, re.S
    )
    assert section, "the README has no campaign section for this gate to check"
    return section.group(1)


def _one(pattern: str, text: str) -> re.Match:
    m = re.search(pattern, text)
    assert m, f"the campaign table does not quote {pattern!r}"
    return m


def test_the_trial_count_is_the_csv_length(quoted, measured):
    assert int(_one(r"\| trials \| \*\*(\d+)\*\*", quoted).group(1)) == measured.trials


def test_the_detection_rate_is_measured(quoted, measured):
    m = _one(r"\| detection \| \*\*(\d+) of (\d+)\*\*", quoted)
    assert (int(m.group(1)), int(m.group(2))) == (measured.detected, measured.trials)


def test_the_headline_success_rate_is_measured(quoted, measured):
    m = _one(r"\| end to end pick and place \| \*\*(\d+) of (\d+)\*\*", quoted)
    assert (int(m.group(1)), int(m.group(2))) == (measured.end_to_end, measured.trials)


def test_the_perception_error_percentiles_are_measured(quoted, measured):
    m = _one(
        r"p50 \*\*([\d.]+) mm\*\*, p95 \*\*([\d.]+) mm\*\*, worst \*\*([\d.]+) mm\*\*",
        quoted,
    )
    assert float(m.group(1)) == pytest.approx(measured.err_p50, abs=0.05)
    assert float(m.group(2)) == pytest.approx(measured.err_p95, abs=0.05)
    assert float(m.group(3)) == pytest.approx(measured.err_max, abs=0.05)


def test_every_failure_is_accounted_for_by_stage(quoted, measured):
    """A row that failed must be attributed, and the attribution must add up.

    This is the check that would have caught the harness faults of 2026-09-02:
    a campaign whose failures are `recovery` or `timeout` rows is reporting on
    itself, and burying those in one aggregate number would hide it.
    """
    failures = measured.trials - measured.end_to_end
    assert sum(measured.failures.values()) == failures

    harness = measured.failures["recovery"] + measured.failures["timeout"]
    assert harness == 0, (
        f"{harness} trials failed inside the harness (recovery or timeout), so the "
        f"success rate is not a statement about the cell"
    )
