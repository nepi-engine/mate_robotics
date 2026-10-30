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


HEADER = struct.Struct("!QIII")  # timestamp, packet index, count, body length
MAX_PACKET_LENGTH = 1400
MAX_JPEG_LENGTH = 8_000_000
BODY_LENGTH = MAX_PACKET_LENGTH - HEADER.size
DISPLACEMENT_PACKET = struct.Struct("!dff")  # elapsed seconds, x px, y px


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
    parser.add_argument("--hef", help="Hailo-8 raw SuperPoint HEF, required for --backend hailo")
    parser.add_argument("--hailo-input-scale", type=float, default=1.0 / 255.0,
                        help="scale applied to input grayscale pixels before HailoRT quantization")
    args = parser.parse_args()
    if args.backend == "hailo" and not args.hef:
        parser.error("--backend hailo requires --hef")

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
            if tracker is None:
                tracker = OpticalFlow(frame, smooth=0.1, track_incrementally=True,
                                      feature_extractor=feature_extractor)
                tracking_start = time.monotonic()
                print(f"Reference frame received: {frame.shape[1]}x{frame.shape[0]}", flush=True)
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
            plot_x, plot_y = -float(dx), float(dy)
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
