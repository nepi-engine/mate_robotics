#!/usr/bin/env python3
"""Open a camera still, tune radial undistortion, and save a live-tracker preset.

Usage: python3 undistort_tuner.py [camera_photo.png]
Without an image argument, a file picker opens after the window appears.
"""

import argparse
from pathlib import Path
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

import cv2
from PIL import Image, ImageTk

from radial_undistort import (MAX_K1, MIN_K1, correct, load_preset,
                              save_preset, validate_k1)


IMAGE_TYPES = [("Images", "*.png *.jpg *.jpeg *.bmp *.tif *.tiff *.webp"),
               ("All files", "*")]
LIVE_WIDTH = 640
LIVE_HEIGHT = 480


class UndistortTuner:
    def __init__(self, root, initial_image=None):
        self.root = root
        self.frame = None
        self.source_frame = None
        self.image_path = None
        self.preview_job = None
        self.original_photo = None
        self.corrected_photo = None
        self.k1 = tk.DoubleVar(value=0.0)
        self.k1_text = tk.StringVar(value="0.0000")
        self.status = tk.StringVar(value="Open a still image from the camera to begin.")

        root.title("ROV Camera Undistortion Tuner")
        root.geometry("1360x830")
        root.minsize(850, 580)

        toolbar = ttk.Frame(root, padding=(12, 12, 12, 6))
        toolbar.pack(fill="x")
        ttk.Button(toolbar, text="Open camera image…", command=self.open_image).pack(side="left")
        ttk.Button(toolbar, text="Crop screenshot…", command=self.crop_image).pack(side="left", padx=(8, 0))
        ttk.Button(toolbar, text="Load preset…", command=self.load_preset).pack(side="left", padx=(8, 0))
        ttk.Button(toolbar, text="Save preset…", command=self.save_preset).pack(side="left", padx=(8, 0))
        ttk.Button(toolbar, text="Export corrected image…", command=self.export_image).pack(side="left", padx=(8, 0))

        controls = ttk.Frame(root, padding=(12, 5, 12, 5))
        controls.pack(fill="x")
        ttk.Label(controls, text="Radial k1").pack(side="left")
        self.slider = ttk.Scale(controls, from_=MIN_K1, to=MAX_K1,
                                variable=self.k1, command=self.on_slider)
        self.slider.pack(side="left", fill="x", expand=True, padx=12)
        entry = ttk.Entry(controls, textvariable=self.k1_text, width=9)
        entry.pack(side="left")
        entry.bind("<Return>", self.on_entry)
        entry.bind("<FocusOut>", self.on_entry)
        ttk.Button(controls, text="Reset to 0", command=lambda: self.set_k1(0.0)).pack(side="left", padx=(8, 0))

        ttk.Label(root, text="Negative values correct barrel distortion. Zero leaves the photo unchanged. "
                  "Load the full 4:3 camera image (crop out window borders). Use straight scene edges to judge the result; "
                  "depth differences need a separate motion model.",
                  padding=(12, 2, 12, 8), wraplength=1300).pack(fill="x")

        previews = ttk.Frame(root, padding=(12, 0, 12, 8))
        previews.pack(fill="both", expand=True)
        previews.columnconfigure(0, weight=1, uniform="preview")
        previews.columnconfigure(1, weight=1, uniform="preview")
        previews.rowconfigure(1, weight=1)
        ttk.Label(previews, text="Original", anchor="center").grid(row=0, column=0, sticky="ew")
        ttk.Label(previews, text="Corrected", anchor="center").grid(row=0, column=1, sticky="ew")
        self.original_label = ttk.Label(previews, text="Open a camera image", anchor="center")
        self.corrected_label = ttk.Label(previews, text="Move the slider to preview", anchor="center")
        self.original_label.grid(row=1, column=0, sticky="nsew", padx=(0, 6))
        self.corrected_label.grid(row=1, column=1, sticky="nsew", padx=(6, 0))
        previews.bind("<Configure>", lambda _event: self.schedule_preview())

        ttk.Label(root, textvariable=self.status, padding=(12, 4, 12, 10),
                  wraplength=1300).pack(fill="x")

        if initial_image is not None:
            root.after(100, lambda: self.open_image(initial_image))
        else:
            root.after(150, self.open_image)

    def set_k1(self, value):
        value = validate_k1(value)
        self.k1.set(value)
        self.k1_text.set(f"{value:.4f}")
        self.schedule_preview()

    def on_slider(self, _value):
        self.k1_text.set(f"{self.k1.get():.4f}")
        self.schedule_preview()

    def on_entry(self, _event):
        try:
            self.set_k1(self.k1_text.get())
        except ValueError as exc:
            self.k1_text.set(f"{self.k1.get():.4f}")
            messagebox.showerror("Invalid k1", str(exc), parent=self.root)

    def open_image(self, path=None):
        if path is None:
            path = filedialog.askopenfilename(title="Choose a still from the camera",
                                              filetypes=IMAGE_TYPES, parent=self.root)
        if not path:
            return
        frame = cv2.imread(str(path), cv2.IMREAD_COLOR)
        if frame is None:
            messagebox.showerror("Image could not be opened", str(path), parent=self.root)
            return
        self.frame = frame
        self.source_frame = frame
        self.image_path = Path(path)
        self.status.set(f"{self.image_path.name} • {frame.shape[1]} × {frame.shape[0]} px • "
                        f"k1 = {self.k1.get():+.4f}")
        self.schedule_preview()

    def crop_image(self):
        if self.source_frame is None:
            messagebox.showinfo("Open an image first", "Choose a camera screenshot first.", parent=self.root)
            return
        height, width = self.source_frame.shape[:2]
        image = Image.fromarray(cv2.cvtColor(self.source_frame, cv2.COLOR_BGR2RGB))
        image.thumbnail((min(self.root.winfo_screenwidth() - 100, 1100),
                         min(self.root.winfo_screenheight() - 170, 750)),
                        Image.Resampling.LANCZOS)
        scale_x = width / image.width
        scale_y = height / image.height
        popup = tk.Toplevel(self.root)
        popup.title("Crop to the full camera image")
        popup.transient(self.root)
        popup.grab_set()
        ttk.Label(popup, text="Drag around the camera picture, excluding window borders and controls.",
                  padding=8).pack()
        canvas = tk.Canvas(popup, width=image.width, height=image.height, cursor="crosshair")
        canvas.pack(padx=8)
        photo = ImageTk.PhotoImage(image)
        canvas.photo = photo
        canvas.create_image(0, 0, image=photo, anchor="nw")
        selection = [None, None]
        rectangle = [None]

        def press(event):
            selection[0] = (event.x, event.y)
            selection[1] = selection[0]
            if rectangle[0] is not None:
                canvas.delete(rectangle[0])
            rectangle[0] = canvas.create_rectangle(event.x, event.y, event.x, event.y,
                                                    outline="#ff7f0e", width=3)

        def drag(event):
            if selection[0] is None:
                return
            x = max(0, min(event.x, image.width))
            y = max(0, min(event.y, image.height))
            selection[1] = (x, y)
            canvas.coords(rectangle[0], *selection[0], x, y)

        def apply_crop():
            if selection[0] is None or selection[1] is None:
                messagebox.showinfo("Select the camera image", "Drag a rectangle first.", parent=popup)
                return
            x0, x1 = sorted((selection[0][0], selection[1][0]))
            y0, y1 = sorted((selection[0][1], selection[1][1]))
            left, right = round(x0 * scale_x), round(x1 * scale_x)
            top, bottom = round(y0 * scale_y), round(y1 * scale_y)
            if right - left < 20 or bottom - top < 20:
                messagebox.showerror("Crop is too small", "Select the full camera image.", parent=popup)
                return
            self.frame = self.source_frame[top:bottom, left:right].copy()
            popup.destroy()
            self.schedule_preview()

        canvas.bind("<ButtonPress-1>", press)
        canvas.bind("<B1-Motion>", drag)
        canvas.bind("<ButtonRelease-1>", drag)
        buttons = ttk.Frame(popup, padding=8)
        buttons.pack(fill="x")
        ttk.Button(buttons, text="Use crop", command=apply_crop).pack(side="right")
        ttk.Button(buttons, text="Cancel", command=popup.destroy).pack(side="right", padx=8)

    def schedule_preview(self):
        if self.frame is None:
            return
        if self.preview_job is not None:
            self.root.after_cancel(self.preview_job)
        self.preview_job = self.root.after(75, self.refresh_preview)

    @staticmethod
    def make_photo(frame, width, height):
        image = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        image.thumbnail((max(width, 100), max(height, 100)), Image.Resampling.LANCZOS)
        return ImageTk.PhotoImage(image)

    def refresh_preview(self):
        self.preview_job = None
        if self.frame is None:
            return
        width = max(self.original_label.winfo_width() - 12, 250)
        height = max(self.original_label.winfo_height() - 12, 250)
        corrected = correct(self.frame, self.k1.get())
        self.original_photo = self.make_photo(self.frame, width, height)
        self.corrected_photo = self.make_photo(corrected, width, height)
        self.original_label.configure(image=self.original_photo, text="")
        self.corrected_label.configure(image=self.corrected_photo, text="")
        self.status.set(f"{self.image_path.name} • {self.frame.shape[1]} × {self.frame.shape[0]} px • "
                        f"k1 = {self.k1.get():+.4f}")

    def load_preset(self):
        path = filedialog.askopenfilename(title="Load an undistortion preset",
                                          filetypes=[("JSON", "*.json")], parent=self.root)
        if not path:
            return
        try:
            data = load_preset(path)
        except (OSError, ValueError) as exc:
            messagebox.showerror("Preset could not be loaded", str(exc), parent=self.root)
            return
        if (data["width"], data["height"]) != (LIVE_WIDTH, LIVE_HEIGHT):
            messagebox.showerror("Camera size differs",
                                 "This tuner expects a 640 × 480 live camera preset.", parent=self.root)
            return
        self.set_k1(data["k1"])
        self.status.set(f"Loaded {Path(path).name} • k1 = {data['k1']:+.4f}")

    def save_preset(self):
        if self.frame is None:
            messagebox.showinfo("Open an image first", "Choose a camera image before saving.", parent=self.root)
            return
        width, height = self.frame.shape[1], self.frame.shape[0]
        if abs(width / height - LIVE_WIDTH / LIVE_HEIGHT) > 0.01:
            messagebox.showerror("Crop to the camera image",
                                 "The live stream is 4:3. Crop away any screenshot borders "
                                 "and load the full camera image before saving a preset.",
                                 parent=self.root)
            return
        path = filedialog.asksaveasfilename(title="Save undistortion preset",
                                            defaultextension=".json",
                                            initialfile="camera_undistort.json",
                                            filetypes=[("JSON", "*.json")], parent=self.root)
        if not path:
            return
        try:
            save_preset(path, LIVE_WIDTH, LIVE_HEIGHT, self.k1.get())
        except OSError as exc:
            messagebox.showerror("Preset could not be saved", str(exc), parent=self.root)
            return
        self.status.set(f"Saved {path}. Use: ./nepi_disp.sh --undistort-config {path}")

    def export_image(self):
        if self.frame is None:
            messagebox.showinfo("Open an image first", "Choose a camera image before exporting.", parent=self.root)
            return
        path = filedialog.asksaveasfilename(title="Export corrected image",
                                            defaultextension=".png",
                                            initialfile=f"{self.image_path.stem}_corrected.png",
                                            filetypes=[("PNG", "*.png"), ("JPEG", "*.jpg")],
                                            parent=self.root)
        if not path:
            return
        if not cv2.imwrite(path, correct(self.frame, self.k1.get())):
            messagebox.showerror("Export failed", path, parent=self.root)
            return
        self.status.set(f"Saved corrected image to {path}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("image", nargs="?", help="optional screenshot or camera still")
    args = parser.parse_args()
    root = tk.Tk()
    UndistortTuner(root, args.image)
    root.mainloop()


if __name__ == "__main__":
    main()
