#!/usr/bin/env python3
"""Click a new exclusion polygon on an Optical Flow camera image.

Run: python3 draw_rov_mask.py [--image path/to/frame.png] [--k1 -0.21]

Left click to add vertices. Enter closes the polygon. Backspace removes the
last vertex, R starts over, and S saves coordinates and an overlay preview.
The default image is the ROV frame supplied for this mask redraw.
"""

import argparse
import json
import math
import tkinter as tk
from pathlib import Path
from tkinter import ttk

from PIL import Image, ImageDraw, ImageTk


HERE = Path(__file__).resolve().parent
REFERENCE_WIDTH = 640
REFERENCE_HEIGHT = 480
DEFAULT_K1 = -0.21


def display_to_mask_point(x, y, width, height, k1):
    """Map a point on the corrected display back to the app's raw mask grid."""
    displayed_x = x * REFERENCE_WIDTH / width
    displayed_y = y * REFERENCE_HEIGHT / height
    cx, cy = (REFERENCE_WIDTH - 1) / 2, (REFERENCE_HEIGHT - 1) / 2
    dx, dy = displayed_x - cx, displayed_y - cy
    radius_squared = cx * cx + cy * cy
    radial_scale = 1 + k1 * (dx * dx + dy * dy) / radius_squared
    raw_x, raw_y = cx + dx * radial_scale, cy + dy * radial_scale
    return [
        max(0, min(REFERENCE_WIDTH - 1, round(raw_x))),
        max(0, min(REFERENCE_HEIGHT - 1, round(raw_y))),
    ]


def render_overlay(image, points, closed):
    result = image.convert("RGBA")
    overlay = Image.new("RGBA", image.size, (0, 0, 0, 0))
    draw = ImageDraw.Draw(overlay)
    if closed and len(points) >= 3:
        draw.polygon(points, fill=(255, 0, 0, 105), outline=(255, 0, 0, 255), width=3)
    elif len(points) >= 2:
        draw.line(points, fill=(255, 220, 0, 255), width=3, joint="curve")
    for index, (x, y) in enumerate(points):
        draw.ellipse((x - 5, y - 5, x + 5, y + 5), fill=(255, 220, 0, 255),
                     outline=(0, 0, 0, 255), width=1)
        draw.text((x + 7, y + 4), str(index + 1), fill=(255, 255, 255, 255),
                  stroke_width=2, stroke_fill=(0, 0, 0, 255))
    return Image.alpha_composite(result, overlay).convert("RGB")


class MaskEditor:
    def __init__(self, image_path, output_path, k1):
        self.image_path = image_path
        self.output_path = output_path
        self.k1 = k1
        self.image = Image.open(image_path).convert("RGB")
        self.width, self.height = self.image.size
        self.scale = min(1200 / self.width, 900 / self.height, 1.0)
        self.points = []
        self.closed = False

        self.root = tk.Tk()
        self.root.title("Draw ROV claw and cable mask")
        self.canvas = tk.Canvas(self.root, width=round(self.width * self.scale),
                                height=round(self.height * self.scale), cursor="crosshair",
                                highlightthickness=0)
        self.canvas.pack()
        self.photo = None
        self.canvas_image = self.canvas.create_image(0, 0, anchor=tk.NW)
        self.canvas.bind("<Button-1>", self.add_point)

        controls = ttk.Frame(self.root, padding=8)
        controls.pack(fill="x")
        ttk.Button(controls, text="Close polygon (Enter)", command=self.close_polygon).pack(side="left")
        ttk.Button(controls, text="Undo (Backspace)", command=self.undo).pack(side="left", padx=5)
        ttk.Button(controls, text="Start over (R)", command=self.reset).pack(side="left")
        ttk.Button(controls, text="Save coordinates (S)", command=self.save).pack(side="right")
        self.status = tk.StringVar(value="Click around the area to exclude, then press Enter and S.")
        ttk.Label(self.root, textvariable=self.status, padding=(8, 0, 8, 8)).pack(anchor="w")
        self.root.bind("<Return>", lambda _event: self.close_polygon())
        self.root.bind("<BackSpace>", lambda _event: self.undo())
        self.root.bind("<KeyPress-r>", lambda _event: self.reset())
        self.root.bind("<KeyPress-s>", lambda _event: self.save())
        self.redraw()

    def redraw(self):
        displayed = render_overlay(self.image, self.points, self.closed)
        if self.scale != 1:
            displayed = displayed.resize((round(self.width * self.scale),
                                          round(self.height * self.scale)), Image.Resampling.LANCZOS)
        self.photo = ImageTk.PhotoImage(displayed)
        self.canvas.itemconfigure(self.canvas_image, image=self.photo)

    def add_point(self, event):
        if self.closed:
            self.status.set("Polygon closed. Press R to redraw, or S to save.")
            return
        x = max(0, min(self.width - 1, round(event.x / self.scale)))
        y = max(0, min(self.height - 1, round(event.y / self.scale)))
        self.points.append((x, y))
        self.status.set(f"{len(self.points)} point(s). Press Enter when the boundary is complete.")
        self.redraw()

    def close_polygon(self):
        if len(self.points) < 3:
            self.status.set("Click at least three points before closing the polygon.")
            return
        self.closed = True
        self.status.set("Red is excluded from tracking. Press S to save or R to redraw.")
        self.redraw()

    def undo(self):
        if self.closed:
            self.closed = False
        if self.points:
            self.points.pop()
        self.status.set(f"{len(self.points)} point(s).")
        self.redraw()

    def reset(self):
        self.points.clear()
        self.closed = False
        self.status.set("Click around the area to exclude.")
        self.redraw()

    def save(self):
        if len(self.points) < 3:
            self.status.set("Click at least three points before saving.")
            return
        self.closed = True
        mask_points = [display_to_mask_point(x, y, self.width, self.height, self.k1)
                       for x, y in self.points]
        data = {
            "image": str(self.image_path),
            "image_size": [self.width, self.height],
            "k1": self.k1,
            "display_polygon_px": [list(point) for point in self.points],
            "mask_polygon_640x480": mask_points,
        }
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        self.output_path.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")
        preview_path = self.output_path.with_name(self.output_path.stem + "_preview.png")
        render_overlay(self.image, self.points, True).save(preview_path)
        print("Mask polygon (640x480 reference coordinates):", flush=True)
        print(json.dumps(mask_points), flush=True)
        print(f"Saved {self.output_path} and {preview_path}", flush=True)
        self.status.set(f"Saved {self.output_path.name} and {preview_path.name}")
        self.redraw()

    def run(self):
        self.root.mainloop()


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--image", type=Path, default=HERE / "rov_mask_reference.png",
                        help="camera image to click on (default: bundled ROV frame)")
    parser.add_argument("--output", type=Path, default=HERE / "rov_mask_polygon.json",
                        help="JSON output path")
    parser.add_argument("--k1", type=float, default=DEFAULT_K1,
                        help="radial correction used by the displayed image (default: -0.21)")
    args = parser.parse_args()
    if not args.image.is_file():
        parser.error(f"Image not found: {args.image}")
    if not math.isfinite(args.k1) or not -0.3 <= args.k1 <= 0.3:
        parser.error("--k1 must be between -0.3 and 0.3")
    MaskEditor(args.image.resolve(), args.output.resolve(), args.k1).run()


if __name__ == "__main__":
    main()
