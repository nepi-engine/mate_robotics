"""Geometry for the user-drawn optical-flow feature mask."""

import math

import cv2
import numpy as np


# The old two rectangular exclusions form this single L-shaped polygon.
# Points are fractions of the corrected image width and height.
DEFAULT_MASK_POLYGON = [
    [0.0, 170 / 480], [280 / 640, 170 / 480],
    [280 / 640, 290 / 480], [1.0, 290 / 480],
    [1.0, 1.0], [0.0, 1.0],
]


def validate_mask_polygon(points):
    """Return a safe, JSON-compatible polygon in normalized image coordinates."""
    if not isinstance(points, (list, tuple)) or not 3 <= len(points) <= 100:
        raise ValueError("Mask polygon must contain 3 to 100 points")
    polygon = []
    for point in points:
        if not isinstance(point, (list, tuple)) or len(point) != 2:
            raise ValueError("Each mask point must be an [x, y] pair")
        if any(isinstance(value, bool) for value in point):
            raise ValueError("Mask coordinates must be numbers")
        try:
            x, y = float(point[0]), float(point[1])
        except (TypeError, ValueError):
            raise ValueError("Mask coordinates must be numbers")
        if not math.isfinite(x) or not math.isfinite(y) or not (0 <= x <= 1 and 0 <= y <= 1):
            raise ValueError("Mask coordinates must be between 0 and 1")
        polygon.append([x, y])
    twice_area = abs(sum(
        x0 * y1 - x1 * y0
        for (x0, y0), (x1, y1) in zip(polygon, polygon[1:] + polygon[:1])
    ))
    if twice_area < 0.0002:
        raise ValueError("Mask polygon has no usable area")
    return polygon


def foreground_feature_mask(shape, polygon):
    """White tracks features; the user polygon is black and ignored."""
    height, width = shape[:2]
    mask = np.full((height, width), 255, dtype=np.uint8)
    vertices = np.array([
        [round(x * (width - 1)), round(y * (height - 1))]
        for x, y in polygon
    ], dtype=np.int32)
    cv2.fillPoly(mask, [vertices], 0)
    return mask
