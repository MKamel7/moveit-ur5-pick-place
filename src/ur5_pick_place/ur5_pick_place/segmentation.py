"""Classical colour + shape segmentation for the pick target.

The target machine has no NVIDIA GPU, so instead of a learned detector this
module isolates the object by HSV colour, keeps the largest blob above a
minimum area, and reports its image-space centroid and bounding box. That 2D
detection is later lifted to 3D by ``perception.pixel_to_base`` using the depth
image and the camera-to-base transform.
"""
from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass

import cv2
import numpy as np

HsvRange = tuple[Sequence[int], Sequence[int]]

# HSV colour presets for the industrial parts (OpenCV hue 0..179). Red wraps the
# hue axis so it needs two ranges.
COLOR_HSV_RANGES: dict[str, list[HsvRange]] = {
    "red": [((0, 90, 60), (10, 255, 255)), ((170, 90, 60), (179, 255, 255))],
    "green": [((35, 70, 40), (85, 255, 255))],
    "blue": [((100, 120, 40), (130, 255, 255))],
}


@dataclass(frozen=True)
class Detection:
    """A 2D detection in image space."""

    u: float  # centroid column (x)
    v: float  # centroid row (y)
    area: float  # blob area in pixels
    bbox: tuple[int, int, int, int]  # (x, y, w, h)

    @property
    def aspect_ratio(self) -> float:
        _, _, w, h = self.bbox
        return w / h if h else 0.0


def segment_largest_blob(
    bgr: np.ndarray,
    hsv_ranges: list[HsvRange],
    min_area: float = 500.0,
) -> Detection | None:
    """Segment the largest colour blob matching any of the given HSV ranges.

    Args:
        bgr: BGR image (as delivered by cv_bridge).
        hsv_ranges: list of (lower, upper) HSV triples; masks are OR-combined so
            colours that wrap the hue axis (e.g. red) can be passed as two ranges.
        min_area: reject blobs smaller than this many pixels.

    Returns:
        The largest qualifying Detection, or None if nothing qualifies.
    """
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for lower, upper in hsv_ranges:
        mask |= cv2.inRange(hsv, np.array(lower, dtype=np.uint8), np.array(upper, dtype=np.uint8))

    # Clean up speckle so the largest-contour choice is stable.
    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None

    largest = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(largest)
    if area < min_area:
        return None

    m = cv2.moments(largest)
    if m["m00"] == 0:
        return None
    u = m["m10"] / m["m00"]
    v = m["m01"] / m["m00"]
    x, y, w, h = cv2.boundingRect(largest)
    return Detection(
        u=float(u), v=float(v), area=float(area), bbox=(int(x), int(y), int(w), int(h))
    )


def sample_depth(depth: np.ndarray, u: float, v: float, patch: int = 5) -> float:
    """Robustly sample depth at a pixel by taking the median over a small patch.

    Invalid readings (0 or NaN, the usual "no return" sentinels for depth
    cameras) are discarded before taking the median.

    Args:
        depth: 2D depth image in metres.
        u, v: pixel coordinates (column, row).
        patch: half-window is patch // 2 on each side.

    Returns:
        The median valid depth in metres.

    Raises:
        ValueError: if there is no valid depth in the patch.
    """
    h, w = depth.shape[:2]
    r = max(patch // 2, 0)
    ui, vi = int(round(u)), int(round(v))
    u0, u1 = max(ui - r, 0), min(ui + r + 1, w)
    v0, v1 = max(vi - r, 0), min(vi + r + 1, h)
    window = depth[v0:v1, u0:u1].astype(np.float64)
    valid = window[np.isfinite(window) & (window > 0.0)]
    if valid.size == 0:
        raise ValueError(f"no valid depth in patch around ({u}, {v})")
    return float(np.median(valid))
