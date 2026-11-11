"""Live camera with a displacement path overlay and x/y history plots."""

import argparse
from collections import deque
import math
import os
import queue
import socket
import struct
import threading
import tkinter as tk
import time

import cv2
import numpy as np
from PIL import Image, ImageTk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

from live_udp_stabilization import (
    FrameAssembler,
    camera_motion_from_image_motion,
    foreground_feature_mask,
)
from optical_flow import OpticalFlow
from radial_undistort import build_maps, correct, validate_k1


PACKET = struct.Struct("!dff")  # elapsed seconds, x px, y px
WINDOW_SECONDS = 10.0


def padded_limits(values):
    """Use the same autoscaling rule as the ROV PID plots."""
    if not values:
        return -1.0, 1.0
    low, high = min(values), max(values)
    padding = max((high - low) * 0.1, 0.1)
    return low - padding, high + padding


def draw_path_overlay(frame, samples):
    """Keep the current position centered and draw the trail over the full video."""
    height, width = frame.shape[:2]
    center = (width // 2, height // 2)
    if samples:
        current_x, current_y = samples[-1][1:]
        # NEPI publishes camera_x = image_dy, camera_y = -image_dx.
        # A past camera center appears at image_motion_now - image_motion_past
        # in the current, unrotated frame.
        points = np.array([
            (center[0] + round(y - current_y),
             center[1] + round(current_x - x))
            for _, x, y in samples
        ], dtype=np.int32)
        if len(points) > 1:
            cv2.polylines(frame, [points], False, (0, 0, 0), 5, cv2.LINE_AA)
            cv2.polylines(frame, [points], False, (90, 225, 110), 2, cv2.LINE_AA)
    cv2.circle(frame, center, 8, (0, 0, 0), -1, cv2.LINE_AA)
    cv2.circle(frame, center, 6, (0, 140, 255), -1, cv2.LINE_AA)
    return frame


class CameraPathTracker:
    """Track the same frames shown in the viewer, so its trail stays in sync."""

    def __init__(self, undistort_k1):
        self.undistort_k1 = undistort_k1
        self.maps = None
        self.tracker = None
        self.started = None
        self.samples = deque()

    def process(self, frame):
        if self.maps is None:
            self.maps = build_maps(frame.shape[1], frame.shape[0], self.undistort_k1)
        corrected = correct(frame, self.undistort_k1, self.maps)
        if self.tracker is None:
            mask = foreground_feature_mask(frame.shape)
            mask = cv2.remap(mask, *self.maps, interpolation=cv2.INTER_NEAREST,
                             borderMode=cv2.BORDER_CONSTANT, borderValue=0)
            self.tracker = OpticalFlow(corrected, nfeatures=700, smooth=1.0,
                                       track_incrementally=True, feature_mask=mask)
            self.started = time.monotonic()
            x, y = 0.0, 0.0
        else:
            dx, dy = self.tracker.compute_displacement_px(corrected)
            x, y = camera_motion_from_image_motion(dx, dy)
        elapsed = time.monotonic() - self.started
        self.samples.append((elapsed, x, y))
        while self.samples and elapsed - self.samples[0][0] > WINDOW_SECONDS:
            self.samples.popleft()
        return corrected, tuple(self.samples)


class DisplacementPlot:
    def __init__(self, root, bind_ip, port, source_ip, camera_port, undistort_k1):
        self.root = root
        self.source_ip = source_ip
        self.undistort_k1 = undistort_k1
        self.samples = deque()
        self.last_received = None
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((bind_ip, port))
        self.sock.setblocking(False)
        self.camera_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.camera_sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4_194_304)
        self.camera_sock.bind((bind_ip, camera_port))
        self.camera_sock.settimeout(0.2)
        self.frames = queue.Queue(maxsize=1)
        self.camera_error = None
        self.stop = threading.Event()
        self.camera_worker = threading.Thread(target=self.receive_video, daemon=True)
        self.camera_worker.start()

        root.title("ROV Camera Displacement Visualization")
        screen_width = root.winfo_screenwidth()
        screen_height = root.winfo_screenheight()
        x = min(int(os.environ.get("ROV_LEFT_TASKBAR_WIDTH", "100")) +
                int(os.environ.get("ROV_STATUS_PANEL_WIDTH", "635")) + 15,
                max(screen_width - 1100, 0))
        width = min(1250, max(screen_width - x - 20, 540))
        height = min(650, max(screen_height - 70, 400))
        root.geometry(f"{width}x{height}+{x}+30")

        layout = tk.PanedWindow(root, orient=tk.HORIZONTAL)
        layout.pack(fill=tk.BOTH, expand=True)
        plots = tk.Frame(layout)
        video = tk.Frame(layout)
        layout.add(plots, minsize=240)
        layout.add(video, minsize=300)

        figure = Figure(figsize=(5, 6), dpi=100, layout="constrained")
        self.x_axis, self.y_axis = figure.subplots(2, 1)

        self.x_line, = self.x_axis.plot([], [], label="X displacement")
        self.y_line, = self.y_axis.plot([], [], label="Y displacement")

        for axis, title, ylabel in (
            (self.x_axis, "NEPI Horizontal Displacement", "X (px)"),
            (self.y_axis, "NEPI Vertical Displacement", "Y (px)"),
        ):
            axis.set_title(title)
            axis.set_xlabel("Time (s)")
            axis.set_ylabel(ylabel)
            axis.grid(True)
            axis.legend(loc="upper right")
            axis.set_xlim(0, WINDOW_SECONDS)
            axis.set_ylim(-1, 1)

        self.canvas = FigureCanvasTkAgg(figure, master=plots)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        self.canvas.draw_idle()
        self.video_label = tk.Label(video, text="Waiting for camera video...",
                                    bg="black", fg="white")
        self.video_label.pack(fill=tk.BOTH, expand=True)
        root.bind("<Escape>", lambda _event: root.destroy())
        root.after(40, self.poll)

    def receive_video(self):
        assembler = FrameAssembler()
        path_tracker = CameraPathTracker(self.undistort_k1)
        while not self.stop.is_set():
            try:
                packet, _address = self.camera_sock.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                break
            jpeg = assembler.add(packet)
            if jpeg is not None:
                try:
                    frame = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
                    if frame is None:
                        continue
                    frame_and_path = path_tracker.process(frame)
                except (cv2.error, ValueError) as exc:
                    self.camera_error = f"Camera tracking stopped: {exc}"
                    print(self.camera_error, flush=True)
                    break
                try:
                    self.frames.put_nowait(frame_and_path)
                except queue.Full:
                    try:
                        self.frames.get_nowait()
                    except queue.Empty:
                        pass
                    self.frames.put_nowait(frame_and_path)

    def poll(self):
        changed = False
        while True:
            try:
                packet, address = self.sock.recvfrom(1024)
            except BlockingIOError:
                break
            if self.source_ip and address[0] != self.source_ip:
                continue
            if len(packet) != PACKET.size:
                continue
            elapsed, x, y = PACKET.unpack(packet)
            if not all(math.isfinite(value) for value in (elapsed, x, y)):
                continue
            if elapsed < 0 or (self.samples and elapsed <= self.samples[-1][0]):
                continue
            received_at = time.monotonic()
            if self.last_received is not None and received_at - self.last_received > 2.0:
                self.samples.clear()
            self.last_received = received_at
            self.samples.append((elapsed, x, y))
            while self.samples and elapsed - self.samples[0][0] > WINDOW_SECONDS:
                self.samples.popleft()
            changed = True
        if changed:
            self.refresh()
        if self.camera_error is not None:
            self.video_label.configure(text=self.camera_error, image="")
            self.video_label.image = None
            self.root.after(40, self.poll)
            return
        try:
            frame, path = self.frames.get_nowait()
        except queue.Empty:
            pass
        else:
            draw_path_overlay(frame, path)
            available_width = self.video_label.winfo_width()
            available_height = self.video_label.winfo_height()
            if available_width > 1 and available_height > 1:
                scale = min(1.0, available_width / frame.shape[1],
                            available_height / frame.shape[0])
                if scale < 1.0:
                    frame = cv2.resize(frame, None, fx=scale, fy=scale,
                                       interpolation=cv2.INTER_AREA)
            photo = ImageTk.PhotoImage(Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)))
            self.video_label.configure(image=photo, text="")
            self.video_label.image = photo
        self.root.after(40, self.poll)

    def refresh(self):
        times = [sample[0] for sample in self.samples]
        xs = [sample[1] for sample in self.samples]
        ys = [sample[2] for sample in self.samples]
        end = max(times[-1], WINDOW_SECONDS)
        start = max(0.0, times[-1] - WINDOW_SECONDS)

        for axis, line, values in (
            (self.x_axis, self.x_line, xs),
            (self.y_axis, self.y_line, ys),
        ):
            line.set_data(times, values)
            axis.set_xlim(start, end)
            axis.set_ylim(*padded_limits(values))

        self.canvas.draw_idle()

    def close(self):
        self.stop.set()
        self.camera_sock.close()
        self.sock.close()
        self.camera_worker.join(timeout=1)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bind-ip", default="192.168.68.12")
    parser.add_argument("--port", type=int, default=5601)
    parser.add_argument("--source-ip", default="192.168.68.103")
    parser.add_argument("--camera-port", type=int, default=5600)
    parser.add_argument("--undistort-k1", type=validate_k1, default=0.0)
    args = parser.parse_args()
    root = tk.Tk()
    plot = DisplacementPlot(root, args.bind_ip, args.port, args.source_ip,
                            args.camera_port, args.undistort_k1)
    try:
        root.mainloop()
    finally:
        plot.close()


if __name__ == "__main__":
    main()
