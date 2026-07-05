"""Path-length metrics for quantifying motion plans.

Pure functions so they can be unit tested and reused by the obstacle-avoidance
evaluation without a running robot.
"""
from __future__ import annotations

from collections.abc import Sequence

import numpy as np


def joint_path_length(waypoints: Sequence[Sequence[float]]) -> float:
    """Sum of Euclidean distances between consecutive joint-space waypoints.

    Args:
        waypoints: list of joint position vectors (radians), all the same length.

    Returns:
        Total joint-space path length. 0.0 for fewer than two waypoints.
    """
    if len(waypoints) < 2:
        return 0.0
    arr = [np.asarray(w, dtype=float) for w in waypoints]
    n = arr[0].shape[0]
    if any(w.shape[0] != n for w in arr):
        raise ValueError("all waypoints must have the same dimension")
    return float(sum(np.linalg.norm(arr[i + 1] - arr[i]) for i in range(len(arr) - 1)))


def segment_aabb_intersection(
    p0: Sequence[float],
    p1: Sequence[float],
    box_center: Sequence[float],
    box_size: Sequence[float],
) -> bool:
    """Whether the segment p0->p1 intersects an axis-aligned box.

    Used to prove that a naive straight-line end-effector move between two poses
    would pass through an obstacle (which OMPL must instead route around). Uses
    the slab method.

    Args:
        p0, p1: segment endpoints [x, y, z].
        box_center: box centre [x, y, z].
        box_size: full box extents [sx, sy, sz].

    Returns:
        True if the closed segment intersects the closed box.
    """
    p0 = np.asarray(p0, dtype=float)
    p1 = np.asarray(p1, dtype=float)
    c = np.asarray(box_center, dtype=float)
    half = np.asarray(box_size, dtype=float) / 2.0
    lo, hi = c - half, c + half
    d = p1 - p0

    t_enter, t_exit = 0.0, 1.0
    for i in range(3):
        if abs(d[i]) < 1e-12:
            # Segment parallel to this slab: reject if origin is outside it.
            if p0[i] < lo[i] or p0[i] > hi[i]:
                return False
        else:
            t1 = (lo[i] - p0[i]) / d[i]
            t2 = (hi[i] - p0[i]) / d[i]
            if t1 > t2:
                t1, t2 = t2, t1
            t_enter = max(t_enter, t1)
            t_exit = min(t_exit, t2)
            if t_enter > t_exit:
                return False
    return True


def cartesian_path_length(points: Sequence[Sequence[float]]) -> float:
    """Sum of Euclidean distances between consecutive 3D points.

    Args:
        points: list of [x, y, z] positions.

    Returns:
        Total cartesian path length. 0.0 for fewer than two points.
    """
    if len(points) < 2:
        return 0.0
    arr = [np.asarray(p, dtype=float) for p in points]
    return float(sum(np.linalg.norm(arr[i + 1] - arr[i]) for i in range(len(arr) - 1)))
