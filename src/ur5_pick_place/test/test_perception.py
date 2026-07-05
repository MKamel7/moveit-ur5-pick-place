"""Unit tests for the perception-to-pose geometry.

These cover the core differentiator of the project: turning a 2D pixel
detection plus a depth reading into a 3D grasp point expressed in the robot
base frame. The math is pure and deterministic, so it is tested without any
ROS runtime.
"""
import math

import numpy as np
import pytest
from ur5_pick_place.perception import (
    CameraIntrinsics,
    camera_optical_transform,
    deproject_pixel_to_camera,
    euler_to_quaternion,
    pixel_to_base,
    transform_matrix_from_quaternion,
    transform_point,
)


@pytest.fixture
def intr():
    # A plausible 640x480 depth camera.
    return CameraIntrinsics(fx=600.0, fy=600.0, cx=320.0, cy=240.0, width=640, height=480)


def test_intrinsics_from_camera_info_k(intr):
    k = [600.0, 0.0, 320.0, 0.0, 600.0, 240.0, 0.0, 0.0, 1.0]
    got = CameraIntrinsics.from_k(k, width=640, height=480)
    assert got == intr


def test_deproject_center_pixel_is_on_optical_axis(intr):
    # The principal point at depth d maps to (0, 0, d) in the optical frame.
    p = deproject_pixel_to_camera(320.0, 240.0, 1.5, intr)
    np.testing.assert_allclose(p, [0.0, 0.0, 1.5], atol=1e-9)


def test_deproject_offset_pixel(intr):
    # One focal length to the right of the principal point is 1 rad-ish; at
    # depth 1.0 that is exactly x = (u - cx) * z / fx = 1.0 metre.
    p = deproject_pixel_to_camera(920.0, 240.0, 1.0, intr)
    np.testing.assert_allclose(p, [1.0, 0.0, 1.0], atol=1e-9)
    # Below the principal point => positive y in optical convention.
    p2 = deproject_pixel_to_camera(320.0, 840.0, 2.0, intr)
    np.testing.assert_allclose(p2, [0.0, 2.0, 2.0], atol=1e-9)


def test_deproject_rejects_nonpositive_depth(intr):
    with pytest.raises(ValueError):
        deproject_pixel_to_camera(320.0, 240.0, 0.0, intr)
    with pytest.raises(ValueError):
        deproject_pixel_to_camera(320.0, 240.0, -0.3, intr)


def test_transform_identity_is_noop():
    p = np.array([0.3, -0.2, 1.1])
    np.testing.assert_allclose(transform_point(np.eye(4), p), p, atol=1e-12)


def test_transform_translation_only():
    T = np.eye(4)
    T[:3, 3] = [1.0, 2.0, 3.0]
    np.testing.assert_allclose(transform_point(T, [0.1, 0.0, -0.5]), [1.1, 2.0, 2.5], atol=1e-12)


def test_transform_matrix_from_quaternion_identity():
    T = transform_matrix_from_quaternion([1.0, 2.0, 3.0], [0.0, 0.0, 0.0, 1.0])
    np.testing.assert_allclose(T[:3, :3], np.eye(3), atol=1e-12)
    np.testing.assert_allclose(T[:3, 3], [1.0, 2.0, 3.0], atol=1e-12)


def test_transform_matrix_from_quaternion_90deg_z():
    # +90 deg about z: x-axis rotates onto y-axis.
    q = [0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4)]
    T = transform_matrix_from_quaternion([0.0, 0.0, 0.0], q)
    np.testing.assert_allclose(transform_point(T, [1.0, 0.0, 0.0]), [0.0, 1.0, 0.0], atol=1e-9)


def test_transform_matrix_is_orthonormal():
    q = [0.1, -0.3, 0.2, 0.9]
    T = transform_matrix_from_quaternion([0, 0, 0], q)
    R = T[:3, :3]
    np.testing.assert_allclose(R @ R.T, np.eye(3), atol=1e-9)
    np.testing.assert_allclose(np.linalg.det(R), 1.0, atol=1e-9)


def test_pixel_to_base_downward_camera(intr):
    # Camera mounted 1.2 m above the base origin, looking straight down.
    # Optical frame convention: +z_optical points along the view direction.
    # Looking down means z_optical = -z_base. A rotation of pi about the base
    # x-axis maps optical (0,0,d) -> base (0,0,-d), i.e. d metres below camera.
    q = [1.0, 0.0, 0.0, 0.0]  # 180 deg about x
    T_base_optical = transform_matrix_from_quaternion([0.4, 0.0, 1.2], q)
    # An object seen at the principal point, 0.9 m away, sits directly under
    # the camera at height 1.2 - 0.9 = 0.3 m.
    p_base = pixel_to_base(320.0, 240.0, 0.9, intr, T_base_optical)
    np.testing.assert_allclose(p_base, [0.4, 0.0, 0.3], atol=1e-9)


def test_euler_to_quaternion_pitch_90_matches_matrix():
    # Pitch +90 deg about Y maps +x -> -z.
    q = euler_to_quaternion(0.0, math.pi / 2, 0.0)
    T = transform_matrix_from_quaternion([0, 0, 0], q)
    np.testing.assert_allclose(transform_point(T, [1.0, 0.0, 0.0]), [0.0, 0.0, -1.0], atol=1e-9)


def test_camera_optical_transform_downward_camera_looks_down():
    # Camera body pitched 90 deg (looking down) at height 0.85. The optical
    # z-axis (view direction) must point straight down in the base frame.
    T = camera_optical_transform([0.55, 0.0, 0.85], [0.0, math.pi / 2, 0.0])
    optical_z_in_base = T[:3, :3] @ np.array([0.0, 0.0, 1.0])
    np.testing.assert_allclose(optical_z_in_base, [0.0, 0.0, -1.0], atol=1e-9)
    np.testing.assert_allclose(T[:3, 3], [0.55, 0.0, 0.85], atol=1e-9)


def test_camera_optical_transform_recovers_known_object(intr):
    # A downward camera at (0.55, 0, 0.85). An object directly below it appears
    # at the image centre; at range r its base position is (0.55, 0, 0.85 - r).
    T = camera_optical_transform([0.55, 0.0, 0.85], [0.0, math.pi / 2, 0.0])
    r = 0.60
    p_base = pixel_to_base(intr.cx, intr.cy, r, intr, T)
    np.testing.assert_allclose(p_base, [0.55, 0.0, 0.25], atol=1e-9)


def test_pixel_to_base_matches_manual_composition(intr):
    q = [0.1, -0.2, 0.3, 0.9]
    n = 1.0 / math.sqrt(sum(c * c for c in q))
    q = [c * n for c in q]
    T = transform_matrix_from_quaternion([0.5, 0.1, 1.0], q)
    expected = transform_point(T, deproject_pixel_to_camera(500.0, 300.0, 0.8, intr))
    got = pixel_to_base(500.0, 300.0, 0.8, intr, T)
    np.testing.assert_allclose(got, expected, atol=1e-12)
