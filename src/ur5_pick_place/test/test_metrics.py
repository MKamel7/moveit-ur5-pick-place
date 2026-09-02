"""Unit tests for path-length metrics used to quantify planning results."""
import math

import pytest
from ur5_pick_place.metrics import (
    cartesian_path_length,
    joint_path_length,
    segment_aabb_intersection,
)


def test_joint_path_length_empty_or_single_is_zero():
    assert joint_path_length([]) == 0.0
    assert joint_path_length([[0.0, 0.0, 0.0]]) == 0.0


def test_joint_path_length_two_points():
    # L2 distance between the two joint vectors.
    d = joint_path_length([[0.0, 0.0], [3.0, 4.0]])
    assert d == pytest.approx(5.0)


def test_joint_path_length_sums_segments():
    d = joint_path_length([[0.0], [1.0], [1.0], [4.0]])
    assert d == pytest.approx(1.0 + 0.0 + 3.0)


def test_joint_path_length_rejects_ragged():
    with pytest.raises(ValueError):
        joint_path_length([[0.0, 0.0], [1.0]])


WALL_CENTER = (0.40, 0.0, 0.30)
WALL_SIZE = (0.10, 0.02, 0.40)  # thin in y, tall in z


def test_segment_through_wall_intersects():
    # Straight EE move across the wall midline at a height inside the wall.
    assert segment_aabb_intersection(
        (0.40, 0.25, 0.35), (0.40, -0.25, 0.35), WALL_CENTER, WALL_SIZE
    )


def test_segment_over_wall_misses():
    # Same lateral move but well above the top of the wall (z > 0.50).
    assert not segment_aabb_intersection(
        (0.40, 0.25, 0.70), (0.40, -0.25, 0.70), WALL_CENTER, WALL_SIZE
    )


def test_segment_beside_wall_misses():
    # Move that stays on one side of the thin wall in x.
    assert not segment_aabb_intersection(
        (0.60, 0.25, 0.35), (0.60, -0.25, 0.35), WALL_CENTER, WALL_SIZE
    )


def test_segment_fully_inside_box_intersects():
    assert segment_aabb_intersection((0.40, 0.0, 0.30), (0.41, 0.0, 0.31), WALL_CENTER, WALL_SIZE)


def test_cartesian_path_length_two_points():
    assert cartesian_path_length([[0, 0, 0], [1, 2, 2]]) == pytest.approx(3.0)


def test_cartesian_path_length_arc_is_longer_than_chord():
    # A semicircle of radius 1 sampled finely approaches pi in length, which is
    # longer than the straight chord of length 2.
    pts = [[math.cos(t), math.sin(t), 0.0] for t in [i * math.pi / 50 for i in range(51)]]
    length = cartesian_path_length(pts)
    assert length > 2.0
    assert length == pytest.approx(math.pi, abs=1e-2)


def test_percentile_is_nearest_rank_and_invents_nothing():
    """One implementation, because there were two that disagreed.

    The benchmark node printed its median as sorted(errs)[len // 2] while the
    summariser that regenerates the published table used nearest rank. On an
    even sample those differ, so the live log and the committed report could
    disagree about the same run.
    """
    from ur5_pick_place.metrics import percentile

    values = [1.0, 2.0, 3.0, 4.0]

    assert percentile(values, 50) == 2.0          # nearest rank, not the mean of 2 and 3
    assert percentile(values, 100) == 4.0
    assert percentile(values, 1) == 1.0
    assert percentile([5.0], 95) == 5.0


def test_percentile_refuses_an_empty_sample():
    """Returning 0.0 for "no data" is how an empty run reports a perfect score."""
    import pytest
    from ur5_pick_place.metrics import percentile

    with pytest.raises(ValueError):
        percentile([], 50)


def test_the_published_p95_is_the_95th_of_a_hundred():
    from ur5_pick_place.metrics import percentile

    values = [float(i) for i in range(1, 101)]

    assert percentile(values, 95) == 95.0
