"""Manually inspect optical-flow displacement across ``vid.mov``.

Place the video at ``Apps/nepi_app_robot_stab/tests/data/vid.mov``
"""

from pathlib import Path
import sys

import cv2
import matplotlib.pyplot as plt
import numpy as np

SCRIPTS_DIR = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))

from optical_flow import OpticalFlow


TESTS_DIR = Path(__file__).resolve().parent
NUM_FRAMES = 100


def default_video_path():
    """Find the checked-in test video regardless of its location or extension case."""
    for candidate in (
        TESTS_DIR / "vid.mov",
        TESTS_DIR / "vid.MOV",
        TESTS_DIR / "data" / "vid.mov",
        TESTS_DIR / "data" / "vid.MOV",
    ):
        if candidate.is_file():
            return candidate
    return TESTS_DIR / "data" / "vid.MOV"


def main(video_path=None):
    """Track video motion from its first frame and plot the displacement."""
    video_path = Path(video_path) if video_path is not None else default_video_path()
    cap = cv2.VideoCapture(str(video_path))
    if not cap.isOpened():
        print(f"Could not open video: {video_path}")
        return

    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    if total_frames < 1:
        print(f"Video contains no frames: {video_path}")
        cap.release()
        return

    indices = np.unique(
        np.linspace(0, total_frames - 1, min(NUM_FRAMES, total_frames), dtype=int)
    )
    cap.set(cv2.CAP_PROP_POS_FRAMES, int(indices[0]))
    ok, ref_frame = cap.read()
    if not ok:
        print("Could not read first frame.")
        cap.release()
        return

    tracker = OpticalFlow(ref_frame, smooth=0.1, track_incrementally=True)

    dx_values = [0.0]
    dy_values = [0.0]
    print(f"{'Frame':>6}  {'dx (px)':>10}  {'dy (px)':>10}")
    print(f"{indices[0]:>6}  {0.0:>10.2f}  {0.0:>10.2f}")
    for index in indices[1:]:
        cap.set(cv2.CAP_PROP_POS_FRAMES, int(index))
        ok, frame = cap.read()
        if not ok:
            print(f"Could not read frame {index}.")
            continue

        dx, dy = tracker.compute_displacement_px(frame)
        # Preserve the ROV stabilization test's camera-coordinate convention.
        dx_values.append(-dx)
        dy_values.append(dy)
        print(f"{index:>6}  {-dx:>10.2f}  {dy:>10.2f}")

    cap.release()
    plot_displacements(dx_values, dy_values)


def plot_displacements(dx_values, dy_values):
    """Render the time-series and XY path used by the original ROV test."""
    frames = range(len(dx_values))
    _, axes = plt.subplots(1, 3, figsize=(15, 4))

    axes[0].plot(frames, dx_values, color="steelblue")
    axes[0].set(title="X Displacement over Time", xlabel="Frame", ylabel="dx (px)")

    axes[1].plot(frames, dy_values, color="tomato")
    axes[1].set(title="Y Displacement over Time", xlabel="Frame", ylabel="dy (px)")

    axes[2].plot(dx_values, dy_values, color="seagreen")
    axes[2].plot(dx_values[0], dy_values[0], "go", markersize=8, label="start")
    axes[2].plot(dx_values[-1], dy_values[-1], "rs", markersize=8, label="end")
    axes[2].set(title="Parametric XY Path", xlabel="dx (px)", ylabel="dy (px)")
    axes[2].set_aspect("equal")
    axes[2].legend()

    for axis in axes:
        axis.grid(True)
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else None)
