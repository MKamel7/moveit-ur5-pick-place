"""Unit tests for grasp-pose planning.

Given a 3D object position in the base frame, we generate a top-down grasp
pose (end-effector pointing straight down) plus collision-safe pre-grasp and
retreat poses standing off along the approach axis. All geometry is checked
against the resulting rotation matrix so the tests do not depend on a specific
quaternion sign convention.
"""
import math

import numpy as np
import pytest
from ur5_pick_place.grasp import GraspPose, pregrasp_pose, retreat_pose, top_down_grasp
from ur5_pick_place.perception import transform_matrix_from_quaternion


def _tool_axes(pose: GraspPose):
    """Return (x, y, z) tool axes expressed in the base frame."""
    R = transform_matrix_from_quaternion([0, 0, 0], pose.orientation)[:3, :3]
    return R[:, 0], R[:, 1], R[:, 2]


def test_top_down_grasp_points_down():
    g = top_down_grasp([0.5, 0.1, 0.05])
    _, _, tool_z = _tool_axes(g)
    # A top-down grasp has the tool z-axis (approach direction) pointing down.
    np.testing.assert_allclose(tool_z, [0.0, 0.0, -1.0], atol=1e-9)


def test_top_down_grasp_position_with_offset():
    g = top_down_grasp([0.5, 0.1, 0.05], z_offset=0.02)
    np.testing.assert_allclose(g.position, [0.5, 0.1, 0.07], atol=1e-12)


def test_yaw_rotates_tool_x_in_horizontal_plane():
    g = top_down_grasp([0.4, 0.0, 0.0], yaw=math.pi / 2)
    tool_x, _, tool_z = _tool_axes(g)
    # z still points down; x has rotated 90 deg about vertical.
    np.testing.assert_allclose(tool_z, [0.0, 0.0, -1.0], atol=1e-9)
    np.testing.assert_allclose(tool_x, [0.0, 1.0, 0.0], atol=1e-9)


def test_pregrasp_stands_off_above_the_grasp():
    g = top_down_grasp([0.5, 0.1, 0.05])
    pre = pregrasp_pose(g, standoff=0.10)
    # Pre-grasp is 0.10 m above the grasp along the approach axis (world +z here).
    np.testing.assert_allclose(pre.position, [0.5, 0.1, 0.15], atol=1e-9)
    # Orientation is unchanged so the approach is a straight cartesian descent.
    np.testing.assert_allclose(pre.orientation, g.orientation, atol=1e-12)


def test_retreat_stands_off_above_the_grasp():
    g = top_down_grasp([0.2, -0.3, 0.04])
    ret = retreat_pose(g, standoff=0.12)
    np.testing.assert_allclose(ret.position, [0.2, -0.3, 0.16], atol=1e-9)


def test_standoff_must_be_non_negative():
    g = top_down_grasp([0.5, 0.0, 0.0])
    with pytest.raises(ValueError):
        pregrasp_pose(g, standoff=-0.01)


def test_grasp_orientation_is_unit_quaternion():
    g = top_down_grasp([0.5, 0.1, 0.05], yaw=0.7)
    assert g.orientation.shape == (4,)
    np.testing.assert_allclose(np.linalg.norm(g.orientation), 1.0, atol=1e-9)
