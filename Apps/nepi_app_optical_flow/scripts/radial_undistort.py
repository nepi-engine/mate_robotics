"""One-parameter radial correction shared by the tuner, tracker, and viewer.

For each output pixel (x, y), sample the input at
    (cx, cy) + (1 + k1 * r**2) * ((x, y) - (cx, cy))
where r is measured in units of the image's half diagonal. Negative k1
counteracts barrel distortion; zero leaves the image unchanged. This simple
model is a visual tuning aid, not a calibrated fisheye camera model.
"""

import json
import math
from pathlib import Path

import cv2
import numpy as np


MODEL = "radial_k1_half_diagonal_v1"
DEFAULT_K1 = -0.2100
MIN_K1 = -0.30
MAX_K1 = 0.30


def validate_k1(k1):
    try:
        k1 = float(k1)
    except (TypeError, ValueError) as exc:
        raise ValueError("k1 must be a number") from exc
    if not math.isfinite(k1) or not MIN_K1 <= k1 <= MAX_K1:
        raise ValueError(f"k1 must be between {MIN_K1} and {MAX_K1}")
    return k1


def build_maps(width, height, k1):
    """Return OpenCV remap arrays for a frame of the given dimensions."""
    if width < 2 or height < 2:
        raise ValueError("Image must be at least 2 by 2 pixels")
    k1 = validate_k1(k1)
    cx = (width - 1) / 2.0
    cy = (height - 1) / 2.0
    radius = math.hypot(cx, cy)
    ys, xs = np.indices((height, width), dtype=np.float32)
    dx = xs - cx
    dy = ys - cy
    r2 = (dx * dx + dy * dy) / (radius * radius)
    scale = 1.0 + k1 * r2
    map_x = (cx + dx * scale).astype(np.float32)
    map_y = (cy + dy * scale).astype(np.float32)
    return map_x, map_y


def correct(frame, k1, maps=None):
    """Apply the same correction used by the live optical-flow processor."""
    k1 = validate_k1(k1)
    if k1 == 0.0:
        return frame.copy()
    if maps is None:
        maps = build_maps(frame.shape[1], frame.shape[0], k1)
    return cv2.remap(frame, maps[0], maps[1], cv2.INTER_LINEAR,
                     borderMode=cv2.BORDER_CONSTANT, borderValue=(0, 0, 0))


def preset(width, height, k1):
    if width < 2 or height < 2:
        raise ValueError("Image must be at least 2 by 2 pixels")
    return {"model": MODEL, "width": int(width), "height": int(height),
            "k1": validate_k1(k1)}


def load_preset(path):
    with Path(path).open(encoding="utf-8") as handle:
        data = json.load(handle)
    if not isinstance(data, dict) or data.get("model") != MODEL:
        raise ValueError(f"Preset must use model {MODEL}")
    for name in ("width", "height"):
        if isinstance(data.get(name), bool) or not isinstance(data.get(name), int):
            raise ValueError(f"Preset {name} must be an integer")
    if isinstance(data.get("k1"), bool):
        raise ValueError("Preset k1 must be a number")
    return preset(data["width"], data["height"], data["k1"])


def save_preset(path, width, height, k1):
    data = preset(width, height, k1)
    with Path(path).open("w", encoding="utf-8") as handle:
        json.dump(data, handle, indent=2)
        handle.write("\n")
    return data
