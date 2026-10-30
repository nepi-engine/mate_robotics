"""Robust, stateful image-motion estimation for robot stabilization."""

from pathlib import Path

import cv2
import numpy as np


class OpticalFlow:
    """Estimate displacement from a fixed reference image.

    ORB descriptors work across much larger frame-to-frame motion than a
    small Lucas-Kanade search window.  RANSAC only retains the coherent camera
    motion, and exponential smoothing mirrors ``rov-25``'s ``StableUtils``.
    """

    def __init__(
        self,
        ref_frame,
        nfeatures=1000,
        smooth=0.1,
        min_inliers=8,
        min_inlier_ratio=0.35,
        max_jump_px=75.0,
        track_incrementally=False,
        feature_extractor=None,
    ):
        """Create a tracker.

        ``smooth`` is the contribution from each new estimate.  The default
        0.1 is the value used by the proven ROV stabilization utility.
        ``max_jump_px`` rejects a sudden raw-estimate change caused by blur,
        shadows, or a bad feature match; set it to ``None`` to disable it.
        With ``track_incrementally=True``, each accepted frame becomes the
        next reference and the returned motion is accumulated from the first
        frame. This prevents long-term feature loss in shadowed video.
        """
        if not 0.0 < smooth <= 1.0:
            raise ValueError("smooth must be greater than 0 and at most 1.")
        if min_inliers < 4:
            raise ValueError("min_inliers must be at least 4.")

        self.smooth = float(smooth)
        self.min_inliers = int(min_inliers)
        self.min_inlier_ratio = float(min_inlier_ratio)
        self.max_jump_px = max_jump_px
        self.track_incrementally = track_incrementally
        self.feature_extractor = feature_extractor
        self.orb = None if feature_extractor is not None else cv2.ORB.create(
            nfeatures=nfeatures, fastThreshold=12
        )
        norm = cv2.NORM_L2 if feature_extractor is not None else cv2.NORM_HAMMING
        self.matcher = cv2.BFMatcher(norm, crossCheck=True)
        self.clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))

        self.dx = 0.0
        self.dy = 0.0
        self._position = np.zeros(2, dtype=float)
        self._last_raw = None
        self.last_inlier_count = 0
        self.last_feature_count = 0
        self.set_reference(ref_frame)

    @staticmethod
    def get_displacement_px(img1, img2):
        """Return one unsmoothed, quality-gated estimate for two images."""
        tracker = OpticalFlow(img1, smooth=1.0, max_jump_px=None)
        return tracker.compute_displacement_px(img2)

    def set_reference(self, frame):
        """Use *frame* as the fixed reference without resetting smoothing."""
        reference = self.preprocess(frame)
        self._set_reference_processed(reference)

    def _set_reference_processed(self, reference, features=None):
        """Set a reference that has already been preprocessed."""
        self.reference_shape = reference.shape
        if features is None:
            features = self._extract(reference)
        self.ref_keypoints, self.ref_descriptors = features

    def _extract(self, image):
        if self.feature_extractor is not None:
            return self.feature_extractor.extract(image)
        return self.orb.detectAndCompute(image, None)

    def compute_displacement_px(self, current_frame):
        """Estimate and smooth the current displacement from the reference.

        When the image is too blurred or shadowed to provide enough mutually
        consistent matches, the previous stable value is returned unchanged.
        """
        current = self.preprocess(current_frame)
        self.last_inlier_count = 0
        if current.shape != self.reference_shape:
            raise ValueError(
                "Images must have the same dimensions; got "
                f"{self.reference_shape} and {current.shape}."
            )

        cur_keypoints, cur_descriptors = self._extract(current)
        self.last_feature_count = len(cur_keypoints)
        if self.ref_descriptors is None or cur_descriptors is None:
            return self.dx, self.dy

        matches = sorted(
            self.matcher.match(self.ref_descriptors, cur_descriptors),
            key=lambda match: match.distance,
        )
        good_matches = matches[: min(80, len(matches))]
        if len(good_matches) < self.min_inliers:
            return self.dx, self.dy

        source = np.array(
            [self.ref_keypoints[match.queryIdx].pt for match in good_matches],
            dtype=np.float32,
        ).reshape(-1, 1, 2)
        destination = np.array(
            [cur_keypoints[match.trainIdx].pt for match in good_matches],
            dtype=np.float32,
        ).reshape(-1, 1, 2)
        transform, inlier_mask = cv2.estimateAffinePartial2D(
            source,
            destination,
            method=cv2.RANSAC,
            ransacReprojThreshold=3.0,
        )
        if transform is None or inlier_mask is None:
            return self.dx, self.dy

        self.last_inlier_count = int(inlier_mask.ravel().sum())
        inlier_ratio = self.last_inlier_count / len(good_matches)
        if (
            self.last_inlier_count < self.min_inliers
            or inlier_ratio < self.min_inlier_ratio
        ):
            return self.dx, self.dy

        raw = np.array([transform[0, 2], transform[1, 2]], dtype=float)
        if (
            self.max_jump_px is not None
            and self._last_raw is not None
            and np.linalg.norm(raw - self._last_raw) > self.max_jump_px
        ):
            return self.dx, self.dy

        self._last_raw = raw
        if self.track_incrementally:
            self._position += raw
            target = self._position
            self._set_reference_processed(current, (cur_keypoints, cur_descriptors))
        else:
            target = raw

        self.dx = self.smooth * target[0] + (1.0 - self.smooth) * self.dx
        self.dy = self.smooth * target[1] + (1.0 - self.smooth) * self.dy
        return self.dx, self.dy

    def preprocess(self, img):
        """Normalize lighting and suppress high-frequency blur/noise artifacts."""
        grayscale = self.convert_to_grayscale(img)
        normalized = self.clahe.apply(grayscale)
        return cv2.GaussianBlur(normalized, (3, 3), 0)

    @staticmethod
    def convert_to_grayscale(img):
        """Load *img*, if necessary, and return a single-channel image."""
        if isinstance(img, (str, Path)):
            image_path = Path(img)
            grayscale = cv2.imread(str(image_path), cv2.IMREAD_GRAYSCALE)
            if grayscale is None:
                raise ValueError(f"Could not read image: {image_path}")
            return grayscale

        if not isinstance(img, np.ndarray):
            raise TypeError("Image must be a file path or NumPy array.")
        if img.size == 0:
            raise ValueError("Image array must not be empty.")
        if img.ndim == 2:
            return img
        if img.ndim == 3 and img.shape[2] == 3:
            return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        if img.ndim == 3 and img.shape[2] == 4:
            return cv2.cvtColor(img, cv2.COLOR_BGRA2GRAY)
        raise ValueError("Image array must be grayscale, BGR, or BGRA.")
