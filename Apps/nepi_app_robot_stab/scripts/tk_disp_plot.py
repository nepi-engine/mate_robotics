"""PID-style live camera displacement plots in a Tkinter window."""

import argparse
from collections import deque
import math
import os
import socket
import struct
import tkinter as tk
import time

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure


PACKET = struct.Struct("!dff")  # elapsed seconds, x px, y px
WINDOW_SECONDS = 10.0


def padded_limits(values):
    """Use the same autoscaling rule as the ROV PID plots."""
    if not values:
        return -1.0, 1.0
    low, high = min(values), max(values)
    padding = max((high - low) * 0.1, 0.1)
    return low - padding, high + padding


class DisplacementPlot:
    def __init__(self, root, bind_ip, port, source_ip):
        self.root = root
        self.source_ip = source_ip
        self.samples = deque()
        self.last_received = None
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((bind_ip, port))
        self.sock.setblocking(False)

        root.title("ROV Camera Displacement Visualization")
        screen_width = root.winfo_screenwidth()
        screen_height = root.winfo_screenheight()
        x = min(int(os.environ.get("ROV_LEFT_TASKBAR_WIDTH", "100")) +
                int(os.environ.get("ROV_STATUS_PANEL_WIDTH", "635")) + 15,
                max(screen_width - 400, 0))
        width = min(1400, max(screen_width - x - 20, 400))
        height = min(1000, max(screen_height - 70, 400))
        root.geometry(f"{width}x{height}+{x}+30")

        figure = Figure(figsize=(12, 8), dpi=100, layout="constrained")
        axes = figure.subplot_mosaic([["x", "path"], ["y", "path"]])
        self.x_axis = axes["x"]
        self.y_axis = axes["y"]
        self.path_axis = axes["path"]

        self.x_line, = self.x_axis.plot([], [], label="X displacement")
        self.y_line, = self.y_axis.plot([], [], label="Y displacement")
        self.path_line, = self.path_axis.plot([], [], label="Camera path")
        self.current_point, = self.path_axis.plot([], [], "o", label="Current")

        for axis, title, ylabel in (
            (self.x_axis, "Horizontal Displacement", "X (px)"),
            (self.y_axis, "Vertical Displacement", "Y (px)"),
        ):
            axis.set_title(title)
            axis.set_xlabel("Time (s)")
            axis.set_ylabel(ylabel)
            axis.grid(True)
            axis.legend(loc="upper right")
            axis.set_xlim(0, WINDOW_SECONDS)
            axis.set_ylim(-1, 1)

        self.path_axis.set_title("Camera Motion Path")
        self.path_axis.set_xlabel("X (px)")
        self.path_axis.set_ylabel("Y (px, down positive)")
        self.path_axis.grid(True)
        self.path_axis.legend(loc="upper right")
        self.path_axis.set_aspect("equal", adjustable="box")
        self.path_axis.set_xlim(-1, 1)
        self.path_axis.set_ylim(1, -1)

        self.canvas = FigureCanvasTkAgg(figure, master=root)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        self.canvas.draw_idle()
        root.bind("<Escape>", lambda _event: root.destroy())
        root.after(40, self.poll)

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

        self.path_line.set_data(xs, ys)
        self.current_point.set_data([xs[-1]], [ys[-1]])
        self.path_axis.set_xlim(*padded_limits(xs))
        low_y, high_y = padded_limits(ys)
        self.path_axis.set_ylim(high_y, low_y)
        self.canvas.draw_idle()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bind-ip", default="192.168.68.12")
    parser.add_argument("--port", type=int, default=5601)
    parser.add_argument("--source-ip", default="192.168.68.103")
    args = parser.parse_args()
    root = tk.Tk()
    plot = DisplacementPlot(root, args.bind_ip, args.port, args.source_ip)
    try:
        root.mainloop()
    finally:
        plot.sock.close()


if __name__ == "__main__":
    main()
