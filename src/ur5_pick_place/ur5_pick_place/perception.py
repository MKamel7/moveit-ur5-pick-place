"""Perception-to-pose geometry.

Pure, dependency-light functions that turn a 2D pixel detection plus a depth
reading into a 3D point in the robot base frame. Keeping this free of ROS
imports lets it be unit tested anywhere and reused from both the detector node
and the offline evaluation scripts.

Optical frame convention (REP 103 / ROS): +x right, +y down, +z forward along
the view direction.
"""
from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class CameraIntrinsics:
    """Pinhole intrinsics for a rectified depth/RGB camera."""

    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int

    @classmethod
    def from_k(cls, k: Sequence[float], width: int, height: int) -> CameraIntrinsics:
        """Build from a row-major 3x3 camera matrix K (as in sensor_msgs/CameraInfo)."""
        k = list(k)
        if len(k) != 9:
            raise ValueError(f"K must have 9 elements, got {len(k)}")
        return cls(fx=k[0], fy=k[4], cx=k[2], cy=k[5], width=width, height=height)


def deproject_pixel_to_camera(
    u: float, v: float, depth: float, intr: CameraIntrinsics
) -> np.ndarray:
    """Back-project a pixel (u, v) at a given depth into the camera optical frame.

    Args:
        u: pixel column.
        v: pixel row.
        depth: range along the optical +z axis, in metres. Must be positive.
        intr: camera intrinsics.

    Returns:
        A (3,) array [x, y, z] in the optical frame.
    """
    if not depth > 0.0:
        raise ValueError(f"depth must be positive, got {depth}")
    x = (u - intr.cx) * depth / intr.fx
    y = (v - intr.cy) * depth / intr.fy
    z = float(depth)
    return np.array([x, y, z], dtype=float)


def transform_matrix_from_quaternion(
    translation: Sequence[float], quaternion: Sequence[float]
) -> np.ndarray:
    """Build a 4x4 homogeneous transform from a translation and a quaternion.

    Args:
        translation: [tx, ty, tz].
        quaternion: [qx, qy, qz, qw] (ROS/geometry_msgs order). Normalised
            internally.

    Returns:
        A 4x4 homogeneous transform matrix.
    """
    qx, qy, qz, qw = (float(c) for c in quaternion)
    n = np.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if n == 0.0:
        raise ValueError("quaternion must be non-zero")
    qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
    R = np.array(
        [
            [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
            [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
            [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
        ]
    )
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [float(c) for c in translation]
    return T


def transform_point(T: np.ndarray, p: Sequence[float]) -> np.ndarray:
    """Apply a 4x4 homogeneous transform to a 3D point."""
    T = np.asarray(T, dtype=float)
    if T.shape != (4, 4):
        raise ValueError(f"T must be 4x4, got {T.shape}")
    ph = np.array([p[0], p[1], p[2], 1.0], dtype=float)
    return (T @ ph)[:3]


def pixel_to_base(
    u: float,
    v: float,
    depth: float,
    intr: CameraIntrinsics,
    T_base_optical: np.ndarray,
) -> np.ndarray:
    """Lift a pixel+depth detection to a 3D point in the base frame.

    Args:
        u, v: pixel coordinates of the detection.
        depth: depth reading at that pixel, in metres.
        intr: camera intrinsics.
        T_base_optical: transform from the camera optical frame to the base
            frame (i.e. base_T_optical).

    Returns:
        A (3,) array [x, y, z] in the base frame.
    """
    p_cam = deproject_pixel_to_camera(u, v, depth, intr)
    return transform_point(T_base_optical, p_cam)
