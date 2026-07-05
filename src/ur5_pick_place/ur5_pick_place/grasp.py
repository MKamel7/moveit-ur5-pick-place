"""Grasp-pose planning for a top-down parallel-jaw pick.

Given a 3D object position in the robot base frame, produce a grasp pose whose
tool z-axis (the approach direction) points straight down, plus pre-grasp and
retreat poses that stand off along that approach axis. Keeping this pure makes
the grasp geometry unit-testable and independent of MoveIt.
"""
from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class GraspPose:
    """A grasp pose: position (3,) and orientation quaternion (4,) in xyzw order."""

    position: np.ndarray
    orientation: np.ndarray


def _matrix_to_quaternion(R: np.ndarray) -> np.ndarray:
    """Convert a 3x3 rotation matrix to a unit quaternion [x, y, z, w]."""
    m = np.asarray(R, dtype=float)
    trace = m[0, 0] + m[1, 1] + m[2, 2]
    if trace > 0.0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (m[2, 1] - m[1, 2]) * s
        y = (m[0, 2] - m[2, 0]) * s
        z = (m[1, 0] - m[0, 1]) * s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = 2.0 * np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2])
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = 2.0 * np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2])
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1])
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    q = np.array([x, y, z, w], dtype=float)
    return q / np.linalg.norm(q)


def top_down_grasp(
    object_position: Sequence[float], yaw: float = 0.0, z_offset: float = 0.0
) -> GraspPose:
    """Build a top-down grasp pose above an object.

    Args:
        object_position: [x, y, z] of the object in the base frame.
        yaw: rotation of the gripper about the vertical axis, radians. Use this
            to align the jaws with the object's principal axis.
        z_offset: vertical offset added to the object z (e.g. grasp slightly
            above the object centre), metres.

    Returns:
        A GraspPose with the tool z-axis pointing down.
    """
    # Rx(pi) points the tool z-axis down; Rz(yaw) spins the jaws about vertical.
    c, s = np.cos(yaw), np.sin(yaw)
    rz = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    rx_pi = np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]])
    R = rz @ rx_pi
    position = np.array(
        [object_position[0], object_position[1], object_position[2] + z_offset], dtype=float
    )
    return GraspPose(position=position, orientation=_matrix_to_quaternion(R))


def _standoff_along_approach(grasp: GraspPose, standoff: float) -> GraspPose:
    if standoff < 0.0:
        raise ValueError(f"standoff must be non-negative, got {standoff}")
    from ur5_pick_place.perception import transform_matrix_from_quaternion

    R = transform_matrix_from_quaternion([0, 0, 0], grasp.orientation)[:3, :3]
    approach = R[:, 2]  # tool z-axis, the direction of approach toward the object
    position = grasp.position - standoff * approach
    return GraspPose(position=position, orientation=grasp.orientation.copy())


def pregrasp_pose(grasp: GraspPose, standoff: float) -> GraspPose:
    """Pose standing off from the grasp along the approach axis (pre-grasp/approach)."""
    return _standoff_along_approach(grasp, standoff)


def retreat_pose(grasp: GraspPose, standoff: float) -> GraspPose:
    """Pose standing off from the grasp along the approach axis (post-grasp lift)."""
    return _standoff_along_approach(grasp, standoff)
