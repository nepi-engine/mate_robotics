"""Run OpticalFlow on a live ROV FastMJPG camera stream.

This is a standalone NEPI test runner, not a registered NEPI/ROS app.
It can forward the original UDP packets to the topside FastMJPG viewer.
"""

import argparse
import queue
import socket
import struct
import threading
import time

import cv2
import numpy as np

from optical_flow import OpticalFlow
from radial_undistort import DEFAULT_K1, build_maps, correct, load_preset, preset


HEADER = struct.Struct("!QIII")  # timestamp, packet index, count, body length
MAX_PACKET_LENGTH = 1400
MAX_JPEG_LENGTH = 8_000_000
BODY_LENGTH = MAX_PACKET_LENGTH - HEADER.size
DISPLACEMENT_PACKET = struct.Struct("!dff")  # elapsed seconds, x px, y px


def foreground_feature_mask(frame_shape):
    """Exclude the camera-mounted claw and cable from raw 640x480 frames."""
    height, width = frame_shape[:2]
    mask = np.full((height, width), 255, dtype=np.uint8)
    # This camera's claw fills the bottom of the native frame; its cable
    # crosses the lower-left. Both move with the camera and can win ORB matches.
    for x0, y0, x1, y1 in ((0, 290, 640, 480), (0, 170, 280, 290)):
        left, right = round(x0 * width / 640), round(x1 * width / 640)
        top, bottom = round(y0 * height / 480), round(y1 * height / 480)
        mask[top:bottom, left:right] = 0
    return mask


def camera_motion_from_image_motion(dx, dy):
    """Map this camera's rotated sensor axes to right/down-positive motion."""
    # A leftward camera move shifts the background upward in the raw frame.
    return float(dy), -float(dx)


class FrameAssembler:
    """Reassemble one FastMJPG JPEG, accepting duplicate send rounds."""

    def __init__(self):
        self.last_complete_timestamp = -1
        self.timestamp = None
        self.packet_count = 0
        self.parts = []
        self.received = 0

    def add(self, packet):
        if not HEADER.size <= len(packet) <= MAX_PACKET_LENGTH:
            return None
        timestamp, index, count, body_length = HEADER.unpack_from(packet)
        if timestamp <= self.last_complete_timestamp:
            return None
        if (
            count == 0
            or count > (MAX_JPEG_LENGTH + BODY_LENGTH - 1) // BODY_LENGTH
            or index >= count
            or body_length != len(packet) - HEADER.size
            or body_length == 0
            or body_length > BODY_LENGTH
            or (index < count - 1 and body_length != BODY_LENGTH)
        ):
            return None

        if timestamp != self.timestamp:
            self.timestamp = timestamp
            self.packet_count = count
            self.parts = [None] * count
            self.received = 0
        elif count != self.packet_count:
            return None

        if self.parts[index] is None:
            self.parts[index] = packet[HEADER.size:]
            self.received += 1
        if self.received != self.packet_count:
            return None

        frame = b"".join(self.parts)
        self.last_complete_timestamp = timestamp
        self.timestamp = None
        self.parts = []
        self.received = 0
        return frame if len(frame) <= MAX_JPEG_LENGTH else None


def receive_frames(args, frames, stop):
    incoming = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    incoming.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4_194_304)
    incoming.bind((args.bind_ip, args.port))
    incoming.settimeout(0.5)
    outgoing = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) if args.forward_ip else None
    assembler = FrameAssembler()
    try:
        while not stop.is_set():
            try:
                packet, source = incoming.recvfrom(65535)
            except socket.timeout:
                continue
            if args.source_ip and source[0] != args.source_ip:
                continue
            if outgoing is not None:
                outgoing.sendto(packet, (args.forward_ip, args.forward_port))
            jpeg = assembler.add(packet)
            if jpeg is not None:
                if frames.full():
                    try:
                        frames.get_nowait()
                    except queue.Empty:
                        pass
                frames.put_nowait(jpeg)
    finally:
        incoming.close()
        if outgoing is not None:
            outgoing.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bind-ip", default="192.168.68.103", help="NEPI Ethernet address")
    parser.add_argument("--port", type=int, default=5600, help="incoming camera UDP port")
    parser.add_argument("--source-ip", default="192.168.68.22", help="onboard Pi address")
    parser.add_argument("--forward-ip", help="optional computer address for video relay")
    parser.add_argument("--forward-port", type=int, default=5600)
    parser.add_argument("--print-interval", type=float, default=0.5)
    parser.add_argument("--telemetry-ip", help="computer address for displacement plot")
    parser.add_argument("--telemetry-port", type=int, default=5601)
    parser.add_argument("--backend", choices=("orb", "hailo"), default="orb")
    parser.add_argument("--mask-foreground", action="store_true",
                        help="exclude the camera-mounted claw and cable from feature matching")
    parser.add_argument("--undistort-config", help="override the default lens correction with a JSON preset saved by undistort_tuner.py")
    parser.add_argument("--hef", help="Hailo-8 raw SuperPoint HEF, required for --backend hailo")
    parser.add_argument("--hailo-input-scale", type=float, default=1.0 / 255.0,
                        help="scale applied to input grayscale pixels before HailoRT quantization")
    args = parser.parse_args()
    if args.backend == "hailo" and not args.hef:
        parser.error("--backend hailo requires --hef")
    try:
        undistort_preset = (load_preset(args.undistort_config) if args.undistort_config
                            else preset(640, 480, DEFAULT_K1))
    except (OSError, ValueError) as exc:
        parser.error(f"invalid undistortion preset: {exc}")
    undistort_maps = None

    feature_extractor = None
    if args.backend == "hailo":
        from hailo_superpoint import HailoSuperPoint
        feature_extractor = HailoSuperPoint(args.hef, args.hailo_input_scale)

    telemetry = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) if args.telemetry_ip else None
    telemetry_target = (args.telemetry_ip, args.telemetry_port)
    telemetry_error_reported = False

    frames = queue.Queue(maxsize=1)
    stop = threading.Event()
    worker = threading.Thread(target=receive_frames, args=(args, frames, stop), daemon=True)
    worker.start()
    tracker = None
    processed = 0
    processing_seconds = 0.0
    last_print = time.monotonic()
    tracking_start = None
    print(f"Listening on {args.bind_ip}:{args.port}; backend={args.backend}; "
          "waiting for camera frames", flush=True)
    try:
        while True:
            try:
                jpeg = frames.get(timeout=1.0)
            except queue.Empty:
                if not worker.is_alive():
                    raise RuntimeError("Camera receiver stopped")
                continue
            frame = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
            if frame is None:
                continue
            raw_shape = frame.shape
            width, height = frame.shape[1], frame.shape[0]
            if (width, height) != (undistort_preset["width"], undistort_preset["height"]):
                raise ValueError("Undistortion preset size does not match camera frames: "
                                 f"{undistort_preset['width']}x{undistort_preset['height']} "
                                 f"versus {width}x{height}")
            if undistort_maps is None:
                undistort_maps = build_maps(width, height, undistort_preset["k1"])
                print(f"Applying radial undistortion k1={undistort_preset['k1']:+.4f}", flush=True)
            frame = correct(frame, undistort_preset["k1"], undistort_maps)
            if tracker is None:
                feature_mask = foreground_feature_mask(raw_shape) if args.mask_foreground else None
                if feature_mask is not None and undistort_maps is not None:
                    feature_mask = cv2.remap(feature_mask, *undistort_maps,
                                             interpolation=cv2.INTER_NEAREST,
                                             borderMode=cv2.BORDER_CONSTANT, borderValue=0)
                tracker = OpticalFlow(frame, smooth=1.0, track_incrementally=True,
                                      feature_extractor=feature_extractor,
                                      feature_mask=feature_mask)
                tracking_start = time.monotonic()
                print(f"Reference frame received: {frame.shape[1]}x{frame.shape[0]}; "
                      f"{len(tracker.ref_keypoints)} features; "
                      f"foreground mask={'on' if args.mask_foreground else 'off'}", flush=True)
                if telemetry is not None:
                    try:
                        telemetry.sendto(DISPLACEMENT_PACKET.pack(0.0, 0.0, 0.0), telemetry_target)
                    except OSError as exc:
                        if not telemetry_error_reported:
                            print(f"Displacement telemetry send failed: {exc}", flush=True)
                            telemetry_error_reported = True
                continue
            process_start = time.monotonic()
            dx, dy = tracker.compute_displacement_px(frame)
            processing_seconds += time.monotonic() - process_start
            processed += 1
            now = time.monotonic()
            plot_x, plot_y = camera_motion_from_image_motion(dx, dy)
            if telemetry is not None:
                try:
                    telemetry.sendto(
                        DISPLACEMENT_PACKET.pack(now - tracking_start, plot_x, plot_y),
                        telemetry_target,
                    )
                except OSError as exc:
                    if not telemetry_error_reported:
                        print(f"Displacement telemetry send failed: {exc}", flush=True)
                        telemetry_error_reported = True
            if now - last_print >= args.print_interval:
                print(
                    f"camera displacement: x={plot_x:+.1f} px, y={plot_y:+.1f} px "
                    f"({processed / (now - last_print):.1f} processed fps; "
                    f"{1000 * processing_seconds / processed:.1f} ms/frame; "
                    f"{tracker.last_feature_count} features; "
                    f"{tracker.last_inlier_count} inliers; {args.backend})",
                    flush=True,
                )
                processed = 0
                processing_seconds = 0.0
                last_print = now
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        worker.join(timeout=1.0)
        if telemetry is not None:
            telemetry.close()
        if feature_extractor is not None:
            feature_extractor.close()


if __name__ == "__main__":
    main()
