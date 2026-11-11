#!/usr/bin/env python3
"""Track any NEPI ROS image stream and publish an annotated view plus telemetry."""

import json
import threading
import time
from collections import deque

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32, String

from optical_flow import OpticalFlow
from radial_undistort import DEFAULT_K1, build_maps, correct, validate_k1


MAX_TRAIL_POINTS = 4000
MIN_TRAIL_STEP_PX = 1.0


def foreground_feature_mask(shape):
    """Optional mask for the original ROV claw and cable (scaled from 640x480)."""
    height, width = shape[:2]
    mask = np.full((height, width), 255, dtype=np.uint8)
    for x0, y0, x1, y1 in ((0, 290, 640, 480), (0, 170, 280, 290)):
        mask[round(y0 * height / 480):round(y1 * height / 480),
             round(x0 * width / 640):round(x1 * width / 640)] = 0
    return mask


def decode_image(message):
    """Decode common 8-bit sensor_msgs/Image layouts, respecting row stride."""
    channels = {"bgr8": 3, "rgb8": 3, "mono8": 1, "bgra8": 4, "rgba8": 4}
    count = channels.get(message.encoding.lower())
    if count is None or message.step < message.width * count:
        raise ValueError("Unsupported image encoding: %s" % message.encoding)
    rows = np.frombuffer(message.data, dtype=np.uint8).reshape(message.height, message.step)
    pixels = rows[:, :message.width * count].reshape(message.height, message.width, count)
    encoding = message.encoding.lower()
    if encoding == "bgr8":
        return pixels.copy()
    if encoding == "rgb8":
        return cv2.cvtColor(pixels, cv2.COLOR_RGB2BGR)
    if encoding == "mono8":
        return cv2.cvtColor(pixels[:, :, 0], cv2.COLOR_GRAY2BGR)
    if encoding == "bgra8":
        return cv2.cvtColor(pixels, cv2.COLOR_BGRA2BGR)
    return cv2.cvtColor(pixels, cv2.COLOR_RGBA2BGR)


def draw_path(frame, samples, current):
    height, width = frame.shape[:2]
    center = (width // 2, height // 2)
    if samples:
        current_x, current_y = current
        positions = list(samples)
        if positions[-1] != current:
            positions.append(current)
        points = np.array([
            (center[0] + round(y - current_y), center[1] + round(current_x - x))
            for x, y in positions
        ], dtype=np.int32)
        if len(points) > 1:
            cv2.polylines(frame, [points], False, (0, 0, 0), 5, cv2.LINE_AA)
            cv2.polylines(frame, [points], False, (90, 225, 110), 2, cv2.LINE_AA)
    cv2.circle(frame, center, 8, (0, 0, 0), -1, cv2.LINE_AA)
    cv2.circle(frame, center, 6, (0, 140, 255), -1, cv2.LINE_AA)
    return frame


class OpticalFlowApp:
    def __init__(self):
        self.lock = threading.Lock()
        self.new_frame = threading.Event()
        self.pending_frame = None
        self.source_sub = None
        self.selected_topic = rospy.get_param("~image_topic", "")
        self.available_topics = []
        self.k1 = validate_k1(rospy.get_param("~k1", DEFAULT_K1))
        self.mask_foreground = bool(rospy.get_param("~mask_foreground", False))
        self.reset_requested = True
        self.tracker = None
        self.maps = None
        self.samples = deque(maxlen=MAX_TRAIL_POINTS)
        self.last_process = None
        self.fps = 0.0
        self.status = "Waiting for camera"
        self.telemetry_pub = rospy.Publisher("~telemetry", String, queue_size=2, latch=True)
        self.image_pub = rospy.Publisher("~annotated_image", Image, queue_size=1)
        rospy.Subscriber("~set_image_topic", String, self.set_image_topic, queue_size=1)
        rospy.Subscriber("~set_k1", Float32, self.set_k1, queue_size=1)
        rospy.Subscriber("~set_mask_foreground", Bool, self.set_mask, queue_size=1)
        rospy.Subscriber("~reset_reference", Bool, self.reset_reference, queue_size=1)
        rospy.Timer(rospy.Duration(2.0), self.refresh_topics)
        self.worker = threading.Thread(target=self.process_frames, daemon=True)
        self.worker.start()
        self.refresh_topics(None)

    def set_image_topic(self, message):
        with self.lock:
            self.selected_topic = message.data
        self.refresh_topics(None)

    def set_k1(self, message):
        try:
            k1 = validate_k1(message.data)
        except ValueError as error:
            rospy.logwarn(str(error))
            return
        with self.lock:
            self.k1 = k1
            self.reset_requested = True
        rospy.set_param("~k1", k1)

    def set_mask(self, message):
        with self.lock:
            self.mask_foreground = message.data
            self.reset_requested = True
        rospy.set_param("~mask_foreground", message.data)

    def reset_reference(self, message):
        if message.data:
            with self.lock:
                self.reset_requested = True

    def refresh_topics(self, _event):
        try:
            topics = sorted(name for name, kind in rospy.get_published_topics()
                            if kind == "sensor_msgs/Image" and
                            name != rospy.resolve_name("~annotated_image"))
        except rospy.ROSException:
            topics = []
        with self.lock:
            self.available_topics = topics
            requested = self.selected_topic
            if not requested and topics:
                # Prefer the established ROV stream when present; all other cameras remain selectable.
                requested = next((name for name in topics if "netstream_192_168_68_22_5604" in name), topics[0])
                self.selected_topic = requested
            subscribed = self.source_sub.resolved_name if self.source_sub else None
            if requested != subscribed:
                if self.source_sub:
                    self.source_sub.unregister()
                    self.source_sub = None
                self.pending_frame = None
                self.reset_requested = True
                if requested in topics:
                    self.source_sub = rospy.Subscriber(requested, Image, self.receive_frame,
                                                       queue_size=1, buff_size=8 * 1024 * 1024)
                    self.status = "Waiting for frames"
                else:
                    self.status = "Waiting for camera"
            elif self.last_process is not None and time.monotonic() - self.last_process > 3.0:
                self.status = "Waiting for frames"
                self.fps = 0.0
        self.publish_telemetry(None)

    def receive_frame(self, message):
        with self.lock:
            self.pending_frame = message
        self.new_frame.set()

    def publish_telemetry(self, sample):
        with self.lock:
            data = dict(status=self.status, camera_topic=self.selected_topic,
                        available_topics=self.available_topics, k1=self.k1,
                        mask_foreground=self.mask_foreground, fps=round(self.fps, 1))
            if sample is not None:
                data.update(sample)
        self.telemetry_pub.publish(String(data=json.dumps(data)))

    def process_frames(self):
        while not rospy.is_shutdown():
            if not self.new_frame.wait(0.5):
                continue
            self.new_frame.clear()
            with self.lock:
                message = self.pending_frame
                self.pending_frame = None
                k1 = self.k1
                mask_foreground = self.mask_foreground
                reset = self.reset_requested
                self.reset_requested = False
            if message is None:
                continue
            try:
                started = time.monotonic()
                raw = decode_image(message)
                if reset or self.maps is None or raw.shape[:2] != self.maps[2]:
                    maps = build_maps(raw.shape[1], raw.shape[0], k1) if k1 else None
                    self.maps = (maps, k1, raw.shape[:2])
                    self.tracker = None
                    self.samples.clear()
                maps = self.maps[0]
                frame = correct(raw, k1, maps)
                if self.tracker is None:
                    mask = foreground_feature_mask(raw.shape) if mask_foreground else None
                    if mask is not None and maps is not None:
                        mask = cv2.remap(mask, *maps, interpolation=cv2.INTER_NEAREST,
                                         borderMode=cv2.BORDER_CONSTANT, borderValue=0)
                    self.tracker = OpticalFlow(frame, nfeatures=700, smooth=1.0,
                                               track_incrementally=True, feature_mask=mask)
                    x = y = 0.0
                else:
                    dx, dy = self.tracker.compute_displacement_px(frame)
                    x, y = float(dy), -float(dx)
                now = time.monotonic()
                if (not self.samples or
                        np.hypot(x - self.samples[-1][0], y - self.samples[-1][1]) >= MIN_TRAIL_STEP_PX):
                    self.samples.append((x, y))
                annotated = draw_path(frame, self.samples, (x, y))
                outgoing = Image()
                outgoing.header = message.header
                outgoing.height, outgoing.width = annotated.shape[:2]
                outgoing.encoding = "bgr8"
                outgoing.is_bigendian = 0
                outgoing.step = outgoing.width * 3
                outgoing.data = annotated.tobytes()
                self.image_pub.publish(outgoing)
                with self.lock:
                    self.status = "Tracking"
                    if self.last_process is not None:
                        period = max(now - self.last_process, 0.001)
                        self.fps = 0.7 * self.fps + 0.3 / period if self.fps else 1 / period
                    self.last_process = now
                self.publish_telemetry(dict(t=time.time(), x=round(x, 2), y=round(y, 2),
                                            features=self.tracker.last_feature_count,
                                            inliers=self.tracker.last_inlier_count,
                                            processing_ms=round((time.monotonic() - started) * 1000, 1),
                                            width=outgoing.width, height=outgoing.height))
            except Exception as error:
                rospy.logwarn_throttle(5, "Optical flow frame error: %s" % error)
                with self.lock:
                    self.status = "Frame error: %s" % error
                self.publish_telemetry(None)


if __name__ == "__main__":
    rospy.init_node("app_optical_flow")
    OpticalFlowApp()
    rospy.spin()
