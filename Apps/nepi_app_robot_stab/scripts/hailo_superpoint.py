"""Extract SuperPoint features with a Hailo-8 HEF on the NEPI Pi.

The HEF must expose the standard SuperPoint raw heads: a 65-channel keypoint
logit tensor and a 256-channel coarse descriptor tensor. Other models,
including YOLO HEFs, are rejected. Matching and motion fitting remain on CPU.
"""

from pathlib import Path

import cv2
import numpy as np


def _head_hwc(value, channels):
    """Convert one dequantized Hailo tensor to HWC layout."""
    array = np.asarray(value)
    if array.ndim == 4 and array.shape[0] == 1:
        array = array[0]
    if array.ndim != 3:
        raise ValueError(f"Expected a 3D SuperPoint head, got {array.shape}")
    if array.shape[-1] == channels:
        return np.asarray(array, dtype=np.float32)
    if array.shape[0] == channels:
        return np.asarray(array.transpose(1, 2, 0), dtype=np.float32)
    raise ValueError(f"Expected a {channels}-channel SuperPoint head, got {array.shape}")


def decode_superpoint(logits, coarse_descriptors, threshold=0.015,
                      nms_radius=4, border=8, max_keypoints=1000):
    """Return Nx2 keypoints and Nx256 L2-normalized descriptors in model pixels."""
    scores = _head_hwc(logits, 65)
    descriptors = _head_hwc(coarse_descriptors, 256)
    if scores.shape[:2] != descriptors.shape[:2]:
        raise ValueError("SuperPoint output heads have different spatial dimensions")
    hc, wc = scores.shape[:2]

    # The final detector channel is the dustbin. The other 64 channels form
    # the 8x8 positions in each coarse feature cell.
    scores = scores - scores.max(axis=2, keepdims=True)
    probabilities = np.exp(scores)
    probabilities /= probabilities.sum(axis=2, keepdims=True)
    heatmap = probabilities[:, :, :64].reshape(hc, wc, 8, 8)
    heatmap = heatmap.transpose(0, 2, 1, 3).reshape(hc * 8, wc * 8)
    height, width = heatmap.shape
    valid = heatmap >= threshold
    if border:
        valid[:border, :] = False
        valid[-border:, :] = False
        valid[:, :border] = False
        valid[:, -border:] = False

    neighborhood = 2 * nms_radius + 1
    peaks = cv2.dilate(heatmap, np.ones((neighborhood, neighborhood), np.uint8))
    ys, xs = np.where(valid & (heatmap >= peaks))
    if xs.size == 0:
        return np.empty((0, 2), dtype=np.float32), np.empty((0, 256), dtype=np.float32)
    order = np.argsort(-heatmap[ys, xs])[:max_keypoints]
    xs, ys = xs[order].astype(np.float32), ys[order].astype(np.float32)

    # Equivalent to grid_sample with align_corners=False at SuperPoint's
    # 1/8-resolution descriptor map. Interpolate only selected keypoints.
    cx = np.clip(xs / 8.0 - 0.5, 0, wc - 1)
    cy = np.clip(ys / 8.0 - 0.5, 0, hc - 1)
    x0, y0 = np.floor(cx).astype(int), np.floor(cy).astype(int)
    x1, y1 = np.minimum(x0 + 1, wc - 1), np.minimum(y0 + 1, hc - 1)
    wx, wy = (cx - x0)[:, None], (cy - y0)[:, None]
    sampled = ((1 - wx) * (1 - wy) * descriptors[y0, x0]
               + wx * (1 - wy) * descriptors[y0, x1]
               + (1 - wx) * wy * descriptors[y1, x0]
               + wx * wy * descriptors[y1, x1])
    sampled /= np.maximum(np.linalg.norm(sampled, axis=1, keepdims=True), 1e-12)
    return np.column_stack((xs, ys)), sampled.astype(np.float32)


class HailoSuperPoint:
    """Keep HailoRT virtual streams open for live frame-by-frame inference."""

    def __init__(self, hef_path, input_scale=1.0 / 255.0):
        path = Path(hef_path).expanduser()
        if not path.is_file():
            raise FileNotFoundError(f"SuperPoint HEF not found: {path}")
        try:
            from hailo_platform import (HEF, VDevice, HailoStreamInterface,
                                        InferVStreams, ConfigureParams,
                                        InputVStreamParams, OutputVStreamParams,
                                        FormatType)
        except ImportError as exc:
            raise RuntimeError("NEPI Python needs the hailo_platform package") from exc

        self.hef = HEF(str(path))
        inputs = self.hef.get_input_vstream_infos()
        outputs = self.hef.get_output_vstream_infos()
        if len(inputs) != 1 or len(outputs) != 2:
            raise ValueError("HEF must have one image input and two raw SuperPoint outputs")
        shape = tuple(inputs[0].shape)
        if len(shape) != 3 or shape[2] not in (1, 3):
            raise ValueError(f"Expected HWC grayscale/RGB SuperPoint input, got {shape}")
        if shape[0] % 8 or shape[1] % 8:
            raise ValueError(f"SuperPoint input dimensions must be multiples of 8: {shape}")
        self.height, self.width, self.channels = shape
        self.input_name = inputs[0].name
        self.input_scale = float(input_scale)
        if self.input_scale <= 0:
            raise ValueError("input_scale must be positive")

        expected = (self.height // 8, self.width // 8)
        output_names = {}
        for output in outputs:
            output_shape = tuple(output.shape)
            for channels in (65, 256):
                if (output_shape == (*expected, channels)
                        or output_shape == (channels, *expected)):
                    output_names[channels] = output.name
        if set(output_names) != {65, 256}:
            raise ValueError(
                "HEF must expose raw SuperPoint heads with shapes "
                f"{(*expected, 65)} and {(*expected, 256)}; got "
                f"{[tuple(info.shape) for info in outputs]}"
            )
        self.score_name = output_names[65]
        self.descriptor_name = output_names[256]

        self.device = VDevice()
        params = ConfigureParams.create_from_hef(self.hef, interface=HailoStreamInterface.PCIe)
        self.network_group = self.device.configure(self.hef, params)[0]
        self.group_params = self.network_group.create_params()
        input_params = InputVStreamParams.make_from_network_group(
            self.network_group, quantized=False, format_type=FormatType.FLOAT32)
        output_params = OutputVStreamParams.make_from_network_group(
            self.network_group, quantized=False, format_type=FormatType.FLOAT32)
        self.pipeline_context = InferVStreams(self.network_group, input_params, output_params)
        self.pipeline = self.pipeline_context.__enter__()
        self.active_context = self.network_group.activate(self.group_params)
        try:
            self.active_context.__enter__()
        except BaseException:
            self.pipeline_context.__exit__(None, None, None)
            raise

    def extract(self, gray):
        height, width = gray.shape
        resized = cv2.resize(gray, (self.width, self.height), interpolation=cv2.INTER_AREA)
        if self.channels == 3:
            resized = cv2.cvtColor(resized, cv2.COLOR_GRAY2RGB)
        else:
            resized = resized[:, :, None]
        image = np.ascontiguousarray(resized[None].astype(np.float32) * self.input_scale)
        result = self.pipeline.infer({self.input_name: image})
        points, descriptors = decode_superpoint(
            result[self.score_name], result[self.descriptor_name])
        keypoints = [cv2.KeyPoint(float(x * width / self.width),
                                 float(y * height / self.height), 1.0)
                     for x, y in points]
        return keypoints, descriptors if len(keypoints) else None

    def close(self):
        self.active_context.__exit__(None, None, None)
        self.pipeline_context.__exit__(None, None, None)
