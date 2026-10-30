#!/usr/bin/env python3
##
## Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
##
## This file is part of nepi-engine
## (see https://github.com/nepi-engine).
##
## License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
##

# Shows a FastMJPG video stream on this computer, without FastMJPG installed.
#
# FastMJPG, the Coral Crusaders' video tool (rov-26/FastMJPG), sends each
# camera frame as one JPEG cut into UDP packets, in its own format that no
# standard player reads. This script receives that format, puts each frame back
# together and shows the newest one in a window. It stands in for the pilot's
# `FastMJPG receive ... render` (rov-26 quick_render.sh) and needs only
# Python 3, OpenCV and numpy.
#
# Start it, then point a FastMJPG `send` output at this computer's address and
# port. For the team's 640x480 stream from a Pi:
#   python3 fastmjpg_viewer.py --port 5600
#   ./bin/FastMJPG capture /dev/video2 640 480 1 30 send <PI_IP> 5600 <THIS_IP> 5600 1400 1000000 2
#
# --max-packet must equal the sender's MAX_PACKET_LENGTH (1400 in the team's
# command). Packets do not carry their position in the frame, so the receiver
# works it out from that number. A mismatch shows up as bad packets, with the
# reason printed.
#
# Press q or Esc in the window, or Ctrl+C in the terminal, to quit. Once a
# second the terminal shows the frame rate, lost frames, bad packets and frame
# age. Age is this computer's clock minus the sender's capture time, so it
# means something only when both clocks are synced (NTP).
#
# The wire format and the receiver rules are in Nepi_Connect/LINK_PROTOCOL.md
# ("Video to NEPI"). NEPI's camera driver for these streams,
# Drivers/idx_drivers/idx_net_stream_driver.py, carries an identical copy of
# FastMJPGReceiver, because NEPI loads each driver from a single file. Change
# both copies together; this script is the way to test the class outside NEPI.

import argparse
import socket
import struct
import sys
import threading
import time

import cv2
import numpy as np


# Every FastMJPG packet starts with this header, all fields big-endian
# (FastMJPG/include/VideoUDPShared.h): capture time in microseconds since 1970
# on the sender's clock, packet index, packet count, body length. The body is
# the slice of the JPEG at index * (MAX_PACKET_LENGTH - 20).
HEADER = struct.Struct('>QIII')


class FastMJPGReceiver(object):
    """Receive one FastMJPG UDP stream and keep its newest complete frame.

    A background thread reads the UDP port and reassembles frames. Callers take
    the newest complete frame with get_jpeg(), so a slow caller skips frames
    rather than falling behind. Bad packets are counted and dropped, never
    raised, so a stray datagram on the port cannot stop the stream.
    """

    MAX_PARTIAL_FRAMES = 4 # Incomplete frames held at once, waiting for missing packets
    RECV_BUFFER_BYTES = 4 * 1024 * 1024 # Asked of the kernel, which may grant less
    SOCKET_TIMEOUT_SEC = 0.5 # How often the receive thread checks for stop
    # A frame this much older than the newest one delivered means the sender's
    # clock stepped back (an NTP correction, or a restart before its clock
    # synced), not a late packet. Repeat send rounds arrive within milliseconds.
    CLOCK_STEP_US = 2000000
    FPS_SMOOTHING = 0.1

    def __init__(self, port, bind_ip = '0.0.0.0', max_packet_length = 1400, max_jpeg_length = 1000000,
                 sender_ip = None):
        self.port = int(port)
        self.bind_ip = str(bind_ip)
        self.max_packet_length = int(max_packet_length)
        self.max_jpeg_length = int(max_jpeg_length)
        self.max_body_length = self.max_packet_length - HEADER.size
        if self.max_body_length <= 0:
            raise ValueError('max_packet_length must be more than the ' + str(HEADER.size) + '-byte header')
        # The most packets a frame of max_jpeg_length bytes can take
        self.max_packet_count = (self.max_jpeg_length + self.max_body_length - 1) // self.max_body_length
        # Packets from any other address are dropped. None or 0.0.0.0 takes any sender.
        if sender_ip in [None, '', '0.0.0.0']:
            sender_ip = None
        self.sender_ip = sender_ip

        # frame_lock guards the newest-frame slot and the health fields
        self.frame_lock = threading.Lock()
        self.frame_cond = threading.Condition(self.frame_lock)
        self.jpeg = None
        self.capture_us = None
        self.arrival_time = None
        self.frame_seq = 0
        self.returned_seq = 0
        self.sender = None
        self.start_time = None
        self.frame_count = 0
        self.lost_frames = 0
        self.packet_count = 0
        self.bad_packets = 0
        self.current_fps = 0.0
        self.last_error = ''

        self.sock = None
        self.recv_thread = None
        self.stop_event = None


    def start(self):
        """Open the UDP port and start receiving in a background thread.

        Returns:
            list: [success (bool), msg (str)].
        """
        if self.recv_thread is not None:
            return True, 'Already receiving on UDP port ' + str(self.port)
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, self.RECV_BUFFER_BYTES)
            sock.bind((self.bind_ip, self.port))
        except OSError as e:
            sock.close()
            return False, 'Cannot receive on ' + self.bind_ip + ' UDP port ' + str(self.port) + ': ' + str(e)
        sock.settimeout(self.SOCKET_TIMEOUT_SEC)
        granted_kb = sock.getsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF) // 1024
        with self.frame_lock:
            # Rate and stream age count from this run's frames only
            self.start_time = time.time()
            self.arrival_time = None
            self.current_fps = 0.0
        self.sock = sock
        self.stop_event = threading.Event()
        self.recv_thread = threading.Thread(target = self.runRecvThread, args = [sock, self.stop_event])
        self.recv_thread.daemon = True
        self.recv_thread.start()
        return True, ('Receiving on ' + self.bind_ip + ' UDP port ' + str(self.port) +
                      ' (receive buffer ' + str(granted_kb) + ' KB)')

    def stop(self):
        """Stop receiving and close the UDP port.

        Returns:
            list: [success (bool), msg (str)].
        """
        if self.recv_thread is None:
            return True, 'Not receiving'
        self.stop_event.set()
        self.recv_thread.join(self.SOCKET_TIMEOUT_SEC * 2 + 1.0)
        self.recv_thread = None
        # A thread still inside recvfrom() gets an error from the closed socket,
        # sees the stop event and exits.
        self.sock.close()
        self.sock = None
        with self.frame_cond:
            self.frame_cond.notify_all()
        return True, 'Stopped receiving on UDP port ' + str(self.port)

    def is_running(self):
        """Report whether the receive thread is started.

        Returns:
            bool: True between start() and stop().
        """
        return self.recv_thread is not None

    def get_jpeg(self, wait_sec = 0.1):
        """Return the newest complete frame not returned before.

        Waits up to wait_sec for one to arrive. Frames that complete between
        calls are skipped, so each call gets the newest frame.

        Args:
            wait_sec (float): Longest wait for a new frame, in seconds.

        Returns:
            list: [jpeg (bytes), capture_us (int), arrival_time (float),
                success (bool), msg (str)]. capture_us is the sender's capture
                time in microseconds since 1970, on the sender's clock.
                arrival_time is time.time() on this computer when the frame's
                last packet arrived. The first three are None when success is
                False.
        """
        with self.frame_cond:
            if self.frame_seq == self.returned_seq and self.recv_thread is not None:
                self.frame_cond.wait(wait_sec)
            if self.frame_seq == self.returned_seq:
                return None, None, None, False, 'No new frame'
            self.returned_seq = self.frame_seq
            return self.jpeg, self.capture_us, self.arrival_time, True, 'Success'

    def get_health(self):
        """Report the stream's state, for status displays and watchdogs.

        Returns:
            dict: running (bool) -- the receive thread is started;
                sender (tuple or None) -- (ip, port) of the newest frame's sender;
                frame_count (int) -- complete frames received;
                lost_frames (int) -- frames given up on with packets missing;
                packet_count (int) -- datagrams received, good or bad;
                bad_packets (int) -- datagrams dropped as malformed or from
                    the wrong sender;
                fps (float) -- smoothed rate of complete frames in this run;
                seconds_since_frame (float or None) -- seconds since this run's
                    newest frame, or since start() if none has arrived; None
                    when not running;
                last_error (str) -- why the most recent bad packet was dropped.
        """
        now = time.time()
        with self.frame_lock:
            seconds_since_frame = None
            if self.recv_thread is not None and self.start_time is not None:
                reference_time = self.start_time
                if self.arrival_time is not None:
                    reference_time = self.arrival_time
                seconds_since_frame = now - reference_time
            return dict(running = self.recv_thread is not None,
                        sender = self.sender,
                        frame_count = self.frame_count,
                        lost_frames = self.lost_frames,
                        packet_count = self.packet_count,
                        bad_packets = self.bad_packets,
                        fps = self.current_fps,
                        seconds_since_frame = seconds_since_frame,
                        last_error = self.last_error)


    def runRecvThread(self, sock, stop_event):
        # Reassembly state belongs to this thread alone. partial_frames maps a
        # capture time to [packet count, bodies by index, bodies held].
        state = dict(partial_frames = dict(), last_delivered_us = 0)
        while not stop_event.is_set():
            try:
                packet, sender = sock.recvfrom(65536)
            except socket.timeout:
                continue
            except OSError as e:
                if stop_event.is_set():
                    break
                self.recordBadPacket('Receive error: ' + str(e))
                stop_event.wait(self.SOCKET_TIMEOUT_SEC)
                continue
            with self.frame_lock:
                self.packet_count += 1
            self.handlePacket(state, packet, sender)

    def handlePacket(self, state, packet, sender):
        source = ' (from ' + sender[0] + ':' + str(sender[1]) + ')'
        if self.sender_ip is not None and sender[0] != self.sender_ip:
            self.recordBadPacket('Packet from ' + sender[0] + ' ignored: expecting the stream from ' + self.sender_ip)
            return
        if len(packet) < HEADER.size:
            self.recordBadPacket('Packet shorter than the ' + str(HEADER.size) + '-byte FastMJPG header' + source)
            return
        [capture_us, index, count, body_length] = HEADER.unpack_from(packet)
        if len(packet) != HEADER.size + body_length:
            self.recordBadPacket('Packet length does not match its header, so it is not FastMJPG' + source)
            return
        if body_length == 0 or count == 0 or index >= count or count > self.max_packet_count:
            self.recordBadPacket('Packet index or count out of range for max JPEG length ' +
                                 str(self.max_jpeg_length) + source)
            return
        if body_length > self.max_body_length or (index < count - 1 and body_length != self.max_body_length):
            self.recordBadPacket('Packet size does not fit max packet length ' + str(self.max_packet_length) +
                                 ', which must equal the sender\'s MAX_PACKET_LENGTH' + source)
            return
        if index == count - 1 and (count - 1) * self.max_body_length + body_length > self.max_jpeg_length:
            self.recordBadPacket('Frame larger than max JPEG length ' + str(self.max_jpeg_length) + source)
            return

        partial_frames = state['partial_frames']
        if capture_us <= state['last_delivered_us']:
            if state['last_delivered_us'] - capture_us < self.CLOCK_STEP_US:
                return # A repeat send round of a frame already delivered, or a late packet
            partial_frames.clear()
            state['last_delivered_us'] = 0

        frame = partial_frames.get(capture_us)
        if frame is None:
            if len(partial_frames) >= self.MAX_PARTIAL_FRAMES:
                oldest_us = min(partial_frames)
                if capture_us < oldest_us:
                    return # A late packet of a frame already given up on
                del partial_frames[oldest_us]
                self.recordLostFrames(1)
            frame = [count, [None] * count, 0]
            partial_frames[capture_us] = frame
        elif frame[0] != count:
            self.recordBadPacket('Packet count disagrees with earlier packets of the same frame' + source)
            return
        bodies = frame[1]
        if bodies[index] is not None:
            return # Already held, from an earlier send round
        bodies[index] = packet[HEADER.size:]
        frame[2] += 1
        if frame[2] < count:
            return

        # Complete. Older frames still missing packets can never be shown now.
        done_us = [t for t in partial_frames if t <= capture_us]
        for t in done_us:
            del partial_frames[t]
        state['last_delivered_us'] = capture_us
        self.deliverFrame(b''.join(bodies), capture_us, sender, len(done_us) - 1)

    def deliverFrame(self, jpeg, capture_us, sender, lost_frames):
        now = time.time()
        with self.frame_lock:
            if self.arrival_time is not None and now > self.arrival_time:
                fps = 1.0 / (now - self.arrival_time)
                if self.current_fps == 0.0:
                    self.current_fps = fps
                else:
                    self.current_fps += self.FPS_SMOOTHING * (fps - self.current_fps)
            self.jpeg = jpeg
            self.capture_us = capture_us
            self.arrival_time = now
            self.sender = sender
            self.frame_seq += 1
            self.frame_count += 1
            self.lost_frames += lost_frames
            self.frame_cond.notify_all()

    def recordLostFrames(self, count):
        with self.frame_lock:
            self.lost_frames += count

    def recordBadPacket(self, msg):
        with self.frame_lock:
            self.bad_packets += 1
            self.last_error = msg


#**********************
# Viewer

def waitingImage(port):
    img = np.zeros((480, 640, 3), np.uint8)
    cv2.putText(img, 'Waiting for FastMJPG video', (40, 220), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
    cv2.putText(img, 'on UDP port ' + str(port), (40, 270), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
    return img

def windowClosed(window):
    # getWindowProperty returns -1 once the window is gone, e.g. closed with
    # its X button. OpenCV would otherwise reopen it on the next imshow().
    try:
        return cv2.getWindowProperty(window, cv2.WND_PROP_AUTOSIZE) < 0
    except cv2.error:
        return True

def formatStats(health, frame_size, shown, ages):
    sender = health['sender']
    if health['frame_count'] == 0:
        return 'Waiting for FastMJPG video (datagrams so far: ' + str(health['packet_count']) + ')'
    source = 'from ' + sender[0] + ':' + str(sender[1])
    if health['seconds_since_frame'] > 2.0:
        return 'No frames for ' + str(int(health['seconds_since_frame'])) + ' s (last ' + source + ')'
    stats = (str(frame_size[0]) + 'x' + str(frame_size[1]) +
             '  in ' + '{:.1f}'.format(health['fps']) + ' fps, shown ' + str(shown) +
             ' | lost ' + str(health['lost_frames']) +
             ' | bad packets ' + str(health['bad_packets']))
    if len(ages) > 0:
        age_ms = 1000.0 * sum(ages) / len(ages)
        stats += ' | age ' + str(int(round(age_ms))) + ' ms'
        if age_ms < 0 or age_ms > 10000:
            stats += ' (clocks not synced?)'
    return stats + ' | ' + source

def main():
    parser = argparse.ArgumentParser(description = 'Show a FastMJPG UDP video stream.')
    parser.add_argument('--port', type = int, default = 5600,
                        help = "UDP port to receive on: the sender's REMOTE_PORT (default 5600)")
    parser.add_argument('--bind', default = '0.0.0.0',
                        help = 'Local address to receive on (default: all addresses)')
    parser.add_argument('--from', dest = 'sender_ip', default = None,
                        help = 'Show only the stream from this sender IP address (default: any sender)')
    parser.add_argument('--max-packet', type = int, default = 1400,
                        help = "The sender's MAX_PACKET_LENGTH, which must match exactly (default 1400)")
    parser.add_argument('--max-jpeg', type = int, default = 1000000,
                        help = "The sender's MAX_JPEG_LENGTH (default 1000000)")
    parser.add_argument('--no-window', action = 'store_true',
                        help = 'Print the stats only, with no video window (e.g. over SSH)')
    parser.add_argument('--seconds', type = float, default = 0,
                        help = 'Quit after this many seconds (default: run until stopped)')
    args = parser.parse_args()

    receiver = FastMJPGReceiver(args.port, args.bind, args.max_packet, args.max_jpeg, args.sender_ip)
    [success, msg] = receiver.start()
    print(msg)
    if not success:
        return 1
    if receiver.sender_ip is not None:
        print('Showing only the stream from ' + receiver.sender_ip)
    print('Quit with q or Esc in the window, or Ctrl+C here.')

    window = None
    if not args.no_window:
        window = 'FastMJPG UDP port ' + str(args.port)
        cv2.namedWindow(window, cv2.WINDOW_NORMAL) # Resizable
        cv2.imshow(window, waitingImage(args.port))

    start_time = time.time()
    next_report = start_time + 1.0
    frame_size = (0, 0)
    shown = 0
    ages = []
    last_error = ''
    decode_failures = 0
    try:
        while True:
            [jpeg, capture_us, arrival_time, success, msg] = receiver.get_jpeg(wait_sec = 0.02)
            if success:
                img = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
                if img is None:
                    decode_failures += 1
                else:
                    shown += 1
                    frame_size = (img.shape[1], img.shape[0])
                    if window is not None:
                        cv2.imshow(window, img)
                    ages.append(time.time() - capture_us / 1000000.0)
            if window is not None:
                key = cv2.waitKey(1) & 0xFF
                if key in (ord('q'), 27) or windowClosed(window):
                    break
            now = time.time()
            if now >= next_report:
                health = receiver.get_health()
                if health['last_error'] != '' and health['last_error'] != last_error:
                    print('Bad packet: ' + health['last_error'])
                last_error = health['last_error']
                stats = formatStats(health, frame_size, shown, ages)
                if decode_failures > 0:
                    stats += ' | JPEG decode failures ' + str(decode_failures)
                print(stats)
                shown = 0
                ages = []
                next_report = now + 1.0
            if args.seconds > 0 and now - start_time >= args.seconds:
                break
    except KeyboardInterrupt:
        pass
    finally:
        receiver.stop()
        if window is not None:
            cv2.destroyAllWindows()
    return 0


if __name__ == '__main__':
    sys.exit(main())
