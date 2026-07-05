"""Unit tests for classical colour+shape segmentation.

Because there is no GPU on the target machine, the detector uses HSV colour
segmentation plus a largest-blob / shape filter instead of a learned model.
These tests build synthetic images so the behaviour is deterministic.
"""
import cv2
import numpy as np
import pytest
from ur5_pick_place.segmentation import sample_depth, segment_largest_blob

# HSV range for a saturated green object (OpenCV hue is 0..179).
GREEN = [((40, 80, 80), (80, 255, 255))]


def _green_box_image(x1, y1, x2, y2, size=(480, 640)):
    img = np.zeros((size[0], size[1], 3), dtype=np.uint8)
    cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), thickness=-1)
    return img


def test_segment_finds_green_box_centroid():
    img = _green_box_image(100, 150, 300, 350)
    det = segment_largest_blob(img, GREEN, min_area=500)
    assert det is not None
    assert det.u == pytest.approx(200.0, abs=2.0)
    assert det.v == pytest.approx(250.0, abs=2.0)
    # Area of a 200x200 filled box, within a couple percent.
    assert det.area == pytest.approx(200 * 200, rel=0.05)


def test_segment_returns_bbox_covering_the_box():
    det = segment_largest_blob(_green_box_image(100, 150, 300, 350), GREEN, min_area=500)
    x, y, w, h = det.bbox
    assert x == pytest.approx(100, abs=2)
    assert y == pytest.approx(150, abs=2)
    assert w == pytest.approx(200, abs=3)
    assert h == pytest.approx(200, abs=3)


def test_segment_picks_the_larger_of_two_blobs():
    img = _green_box_image(20, 20, 60, 60)  # small blob (40x40)
    cv2.rectangle(img, (300, 200), (460, 360), (0, 255, 0), -1)  # large blob (160x160)
    det = segment_largest_blob(img, GREEN, min_area=500)
    # Centroid should be near the large blob's centre (380, 280).
    assert det.u == pytest.approx(380.0, abs=3.0)
    assert det.v == pytest.approx(280.0, abs=3.0)


def test_segment_rejects_when_blob_below_min_area():
    img = _green_box_image(10, 10, 20, 20)  # ~10x10, tiny
    assert segment_largest_blob(img, GREEN, min_area=500) is None


def test_segment_returns_none_when_colour_absent():
    img = np.zeros((480, 640, 3), dtype=np.uint8)  # all black
    assert segment_largest_blob(img, GREEN, min_area=500) is None


def test_sample_depth_median_over_patch():
    depth = np.full((480, 640), 1.234, dtype=np.float32)
    assert sample_depth(depth, 320, 240, patch=5) == pytest.approx(1.234, abs=1e-6)


def test_sample_depth_ignores_zeros_and_nans():
    depth = np.zeros((480, 640), dtype=np.float32)
    depth[:] = np.nan
    # A single valid pixel in the patch should be recovered.
    depth[240, 320] = 0.9
    assert sample_depth(depth, 320, 240, patch=5) == pytest.approx(0.9, abs=1e-6)


def test_sample_depth_raises_when_no_valid_depth():
    depth = np.zeros((480, 640), dtype=np.float32)
    with pytest.raises(ValueError):
        sample_depth(depth, 320, 240, patch=5)
