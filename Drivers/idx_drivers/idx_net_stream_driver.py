#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus <https://www.numurus.com>.
#
# This file is part of nepi applications (nepi_drivers) repo
# (see https://https://github.com/nepi-engine/nepi_drivers)
#
# License: nepi applications are licensed under the "Numurus Software License",
# which can be found at: <https://numurus.com/wp-content/uploads/Numurus-Software-License-Terms.pdf>
#
# Redistributions in source code must retain this top-level comment block.
# Plagiarizing this software to sidestep the license obligations is illegal.
#
# Contact Information:
# ====================
# - mailto:nepi@numurus.com

# Network stream reader for the IDX_NET_STREAM driver. One instance reads one
# stream URL. RTSP and HTTP streams go through OpenCV's FFmpeg backend, the only
# video backend in the NEPI container's OpenCV build (GStreamer is not compiled
# in). FastMJPG streams (fastmjpg://) go through FastMJPGReceiver, further down,
# because FFmpeg cannot read FastMJPG's UDP format.
#
# A FastMJPG stream is pushed to NEPI rather than pulled: the sender's FastMJPG
# `send` output targets NEPI's address and a UDP port here. So in a fastmjpg://
# URL the host is the sender to accept packets from (0.0.0.0 accepts any), and
# the port is NEPI's own UDP port. Each frame arrives as one whole JPEG and is
# decoded only in getImage().
#
# The FFmpeg reader logic is adapted from idx_onvif_generic_driver.py (FFmpeg backend,
# RTSP over TCP, background grab thread, retrieve on demand, consecutive-failure
# counting). It is copied rather than imported because that module imports
# python-onvif when it loads. Differences from that reader:
#  - The grab thread opens the capture, and re-opens it after repeated read
#    failures, so no caller ever blocks on the network.
#  - Open and read are bounded by timeouts.
#  - Each grabbed frame gets a sequence number, so getImage() hands out only the
#    newest frame and never the same frame twice.

import os
import re
import time
import socket
import struct
import ipaddress
import threading
import cv2
import numpy as np
from urllib.parse import urlsplit, parse_qs

from nepi_sdk import nepi_utils


PKG_NAME = 'IDX_NET_STREAM' # Use in display menus
FILE_TYPE = 'DRIVER'


class NetStreamCamDriver(object):
    SUPPORTED_SCHEMES = ['rtsp', 'rtsps', 'http', 'https', 'fastmjpg']
    DEFAULT_PORTS = dict(rtsp = 554, rtsps = 322, http = 80, https = 443, fastmjpg = 5600)
    # fastmjpg:// URL options and their defaults, which are the values in the
    # Coral Crusaders' FastMJPG send command. Both must equal the sender's.
    FASTMJPG_OPTIONS = dict(max_packet = 1400, max_jpeg = 1000000)

    OPEN_TIMEOUT_SEC = 5.0
    READ_TIMEOUT_SEC = 3.0
    MAX_CONSEC_FRAME_FAIL_COUNT = 3 # Grab failures on one open capture before it is re-opened
    REOPEN_DELAY_SEC = 1.0
    FPS_SMOOTHING = 0.1
    FPS_HOLD_SEC = 1.0 # FastMJPG frame rate reads 0 after this long without a frame

    def __init__(self, url):
        [supported, msg] = self.is_supported_url(url)
        if supported == False:
            raise ValueError(msg)
        self.url = str(url).strip()
        self.safe_url = self.get_safe_url(self.url)

        # Set for fastmjpg:// URLs, which use it in place of the FFmpeg capture
        self.fastmjpg_receiver = None
        fastmjpg_config = self.parseFastMJPGUrl(self.url)
        if fastmjpg_config is not None:
            self.fastmjpg_receiver = FastMJPGReceiver(fastmjpg_config['port'], '0.0.0.0',
                                                      fastmjpg_config['max_packet'], fastmjpg_config['max_jpeg'],
                                                      fastmjpg_config['sender_ip'])

        # cap_lock guards the capture object, which grab() in the acquisition
        # thread and retrieve() in getImage() both use, plus the newest-frame slot.
        self.cap_lock = threading.Lock()
        self.frame_cond = threading.Condition(self.cap_lock)
        self.cap = None
        self.frame_seq = 0
        self.frame_timestamp = None
        self.returned_seq = 0

        # health_lock guards the health fields. It is never held across network
        # I/O, so get_health() returns immediately even while a read is stalled.
        self.health_lock = threading.Lock()
        self.connected = False
        self.frame_count = 0
        self.consec_failures = 0
        self.last_frame_time = None
        self.acq_start_time = None
        self.current_fps = 0.0
        self.last_error = ''

        self.acq_thread = None
        self.acq_stop_event = None


    #**********************
    # URL helpers, usable without an instance (discovery uses them)

    @staticmethod
    def get_safe_url(url):
        """Return a stream URL with any credentials removed.

        Strips a 'user:password@' or 'user@' prefix from the authority, so the
        result is safe to log and publish. Works on malformed input as well.

        Args:
            url (str): Stream URL, possibly carrying credentials.

        Returns:
            str: The URL without its user info.
        """
        return re.sub(r'^([A-Za-z][A-Za-z0-9+.\-]*://)?[^/?#]*@', r'\1', str(url).strip(), count = 1)

    @classmethod
    def is_supported_url(cls, url):
        """Check that a URL uses a supported scheme and names a host.

        Args:
            url (str): Stream URL.

        Returns:
            list: [supported (bool), msg (str)]. msg is credential-free.
        """
        safe_url = cls.get_safe_url(url)
        try:
            parts = urlsplit(str(url).strip())
            hostname = parts.hostname
            parts.port # Raises ValueError for a non-numeric or out-of-range port
        except ValueError as e:
            return False, "Invalid stream URL " + safe_url + ": " + str(e)
        scheme = parts.scheme.lower()
        if scheme not in cls.SUPPORTED_SCHEMES:
            return False, ("Unsupported stream URL scheme '" + scheme + "' in " + safe_url +
                           " (supported: " + ", ".join(cls.SUPPORTED_SCHEMES) + ")")
        if scheme == 'fastmjpg':
            try:
                cls.parseFastMJPGUrl(url)
            except ValueError as e:
                return False, "Invalid stream URL " + safe_url + ": " + str(e)
            return True, "Supported"
        if hostname is None or hostname == '':
            return False, "No host in stream URL " + safe_url
        return True, "Supported"

    @classmethod
    def get_local_port(cls, url):
        """Return the local UDP port a stream URL makes this driver listen on.

        Only fastmjpg:// streams are pushed to NEPI, so only they hold a local
        port, and two of them cannot share one. NEPI connects out to every
        other kind of stream.

        Args:
            url (str): Stream URL.

        Returns:
            int or None: The UDP port, or None for a URL that holds no local
                port (or is not a valid fastmjpg:// URL).
        """
        try:
            config = cls.parseFastMJPGUrl(url)
        except ValueError:
            return None
        if config is None:
            return None
        return config['port']

    @classmethod
    def parseFastMJPGUrl(cls, url):
        # Returns None for a URL of another scheme, else dict(sender_ip, port,
        # max_packet, max_jpeg). Raises ValueError, with a credential-free
        # message, for a fastmjpg:// URL that cannot be used.
        parts = urlsplit(str(url).strip())
        if parts.scheme.lower() != 'fastmjpg':
            return None
        host_rule = "the host must be the sender's IPv4 address, or 0.0.0.0 to accept any sender"
        if parts.hostname is None or parts.hostname == '':
            raise ValueError(host_rule)
        try:
            sender_ip = str(ipaddress.IPv4Address(parts.hostname))
        except ValueError:
            raise ValueError(host_rule + ", not '" + parts.hostname + "'") from None
        port = parts.port # Raises ValueError for a non-numeric or out-of-range port
        if port is None:
            port = cls.DEFAULT_PORTS['fastmjpg']
        if port == 0:
            raise ValueError("port 0 is not a usable UDP port")
        if parts.path not in ['', '/']:
            raise ValueError("a fastmjpg:// URL has no path")
        options = dict(cls.FASTMJPG_OPTIONS)
        for [key, values] in parse_qs(parts.query, keep_blank_values = True).items():
            if key not in options:
                raise ValueError("unknown option '" + key + "' (options: " + ", ".join(sorted(options)) + ")")
            try:
                options[key] = int(values[-1])
            except ValueError:
                raise ValueError("option " + key + " must be a whole number") from None
        if options['max_packet'] <= HEADER.size or options['max_packet'] > 65507:
            raise ValueError("max_packet must be from " + str(HEADER.size + 1) + " to 65507")
        if options['max_jpeg'] < 1:
            raise ValueError("max_jpeg must be at least 1")
        return dict(sender_ip = sender_ip, port = port,
                    max_packet = options['max_packet'], max_jpeg = options['max_jpeg'])

    @classmethod
    def probe_stream(cls, url):
        """Open a stream URL, read one frame, and close it again.

        Blocks for at most the open timeout plus the read timeout when the
        installed OpenCV supports its timeout properties; otherwise FFmpeg's own
        socket timeouts and OpenCV's internal FFmpeg interrupt bound the call.
        A fastmjpg:// URL instead listens on its UDP port for at most the read
        timeout.

        Args:
            url (str): Stream URL.

        Returns:
            list: [success (bool), msg (str)]. msg is credential-free.
        """
        [supported, msg] = cls.is_supported_url(url)
        if supported == False:
            return False, msg
        fastmjpg_config = cls.parseFastMJPGUrl(url)
        if fastmjpg_config is not None:
            return cls.probeFastMJPG(fastmjpg_config)
        safe_url = cls.get_safe_url(url)
        [cap, msg] = cls.openCapture(url)
        if cap is None:
            return False, msg
        try:
            success, cv2_img = cap.read()
        except Exception:
            success = False
            cv2_img = None
        finally:
            cap.release()
        if success == False or cv2_img is None:
            return False, "Opened but got no frame from " + safe_url
        return True, "Got " + str(cv2_img.shape[1]) + "x" + str(cv2_img.shape[0]) + " frame from " + safe_url

    @classmethod
    def openCapture(cls, url):
        os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = FFMPEG_CAPTURE_OPTIONS
        cap = None
        # OpenCV 4.5.2+ takes open and read timeouts as capture properties at
        # construction. Older builds only have FFmpeg's own timeouts (set in
        # FFMPEG_CAPTURE_OPTIONS) plus OpenCV's built-in FFmpeg interrupt.
        if hasattr(cv2, 'CAP_PROP_OPEN_TIMEOUT_MSEC') and hasattr(cv2, 'CAP_PROP_READ_TIMEOUT_MSEC'):
            params = [cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, int(cls.OPEN_TIMEOUT_SEC * 1000),
                      cv2.CAP_PROP_READ_TIMEOUT_MSEC, int(cls.READ_TIMEOUT_SEC * 1000)]
            try:
                cap = cv2.VideoCapture(url, cv2.CAP_FFMPEG, params)
            except Exception:
                cap = None # Properties exist but this build lacks the params constructor
        if cap is None:
            cap = cv2.VideoCapture(url, cv2.CAP_FFMPEG)
        if not cap.isOpened():
            cap.release()
            return None, "Failed to open stream " + cls.get_safe_url(url)
        return cap, "Success"

    @classmethod
    def probeFastMJPG(cls, config):
        receiver = FastMJPGReceiver(config['port'], '0.0.0.0', config['max_packet'], config['max_jpeg'],
                                    config['sender_ip'])
        [success, msg] = receiver.start()
        if success == False:
            return False, msg
        try:
            [jpeg, capture_us, arrival_time, success, msg] = receiver.get_jpeg(wait_sec = cls.READ_TIMEOUT_SEC)
            health = receiver.get_health()
        finally:
            receiver.stop()
        port_desc = "UDP port " + str(config['port'])
        if success == False:
            if health['last_error'] != '':
                reason = health['last_error']
            elif health['packet_count'] > 0:
                reason = str(health['packet_count']) + " packets but no complete frame"
            else:
                reason = "nothing received"
            return False, ("No FastMJPG frame on " + port_desc + " within " +
                           str(int(cls.READ_TIMEOUT_SEC)) + " s: " + reason)
        cv2_img = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
        if cv2_img is None:
            return False, "FastMJPG frame on " + port_desc + " is not a valid JPEG"
        return True, ("Got " + str(cv2_img.shape[1]) + "x" + str(cv2_img.shape[0]) + " FastMJPG frame from " +
                      health['sender'][0] + " on " + port_desc)


    #**********************
    # Image acquisition

    def imageAcquisitionRunning(self):
        if self.fastmjpg_receiver is not None:
            return self.fastmjpg_receiver.is_running()
        return self.acq_thread is not None and not self.acq_stop_event.is_set()

    def startImageAcquisition(self):
        if self.imageAcquisitionRunning():
            return True, "Already capturing from " + self.safe_url
        if self.fastmjpg_receiver is not None:
            return self.startFastMJPG()
        with self.health_lock:
            self.acq_start_time = nepi_utils.get_time()
            self.consec_failures = 0
            self.current_fps = 0.0
        self.acq_stop_event = threading.Event()
        self.acq_thread = threading.Thread(target=self.runImgAcqThread, args=[self.acq_stop_event])
        self.acq_thread.daemon = True
        self.acq_thread.start()
        return True, "Started capture from " + self.safe_url

    def stopImageAcquisition(self):
        if self.fastmjpg_receiver is not None:
            return self.stopFastMJPG()
        if self.acq_thread is None:
            return True, "No current capture from " + self.safe_url
        thread = self.acq_thread
        self.acq_stop_event.set()
        self.acq_thread = None
        # The thread releases its own capture once its current open or read
        # returns, which the timeouts bound. A thread still finishing after this
        # join cannot touch a later capture: it only clears self.cap if it is
        # still its own.
        thread.join(self.READ_TIMEOUT_SEC + 1.0)
        with self.health_lock:
            self.connected = False
            self.acq_start_time = None
            self.current_fps = 0.0
        if thread.is_alive():
            return True, "Stopping capture from " + self.safe_url + " (reader still finishing a read)"
        return True, "Success"

    def runImgAcqThread(self, stop_event):
        cap = None
        grab_failures = 0
        while not stop_event.is_set():
            if cap is None:
                [cap, msg] = self.openCapture(self.url)
                if cap is None:
                    self.recordFailure(msg)
                    stop_event.wait(self.REOPEN_DELAY_SEC)
                    continue
                with self.cap_lock:
                    if stop_event.is_set():
                        break
                    self.cap = cap
                    self.returned_seq = self.frame_seq
                    with self.health_lock:
                        self.connected = True
                grab_failures = 0

            drop_cap = False
            with self.cap_lock:
                if stop_event.is_set():
                    break
                # grab() waits for the next frame and, in OpenCV's FFmpeg backend,
                # also runs the codec decode. Every frame has to pass through the
                # decoder anyway (H.264 frames reference earlier ones); grabbing
                # continuously keeps the socket drained so frames never queue.
                # retrieve(), the BGR conversion and copy, runs only in getImage().
                success = cap.grab()
                timestamp = nepi_utils.get_time() # NEPI clock; the stream source's clock is not shared
                if stop_event.is_set():
                    break
                if success:
                    self.frame_seq += 1
                    self.frame_timestamp = timestamp
                    self.frame_cond.notify_all()
                else:
                    grab_failures += 1
                    if grab_failures >= self.MAX_CONSEC_FRAME_FAIL_COUNT:
                        drop_cap = True
                        self.cap = None
                        self.returned_seq = self.frame_seq

            if success:
                grab_failures = 0
                self.recordFrame(timestamp)
                time.sleep(0.001) # Yield so a waiting getImage() can take the lock
            else:
                self.recordFailure("Failed to read frame from " + self.safe_url)
                if drop_cap:
                    cap.release()
                    cap = None
                    with self.health_lock:
                        self.connected = False
                    stop_event.wait(self.REOPEN_DELAY_SEC)

        with self.cap_lock:
            if cap is not None and self.cap is cap:
                self.cap = None
                self.returned_seq = self.frame_seq
            self.frame_cond.notify_all()
        if cap is not None:
            cap.release()

    def recordFrame(self, timestamp):
        with self.health_lock:
            # Measure only between frames of the current acquisition run
            if (self.last_frame_time is not None and self.acq_start_time is not None and
                    self.last_frame_time >= self.acq_start_time and timestamp > self.last_frame_time):
                fps = 1.0 / (timestamp - self.last_frame_time)
                if self.current_fps == 0.0:
                    self.current_fps = fps
                else:
                    self.current_fps += self.FPS_SMOOTHING * (fps - self.current_fps)
            self.last_frame_time = timestamp
            self.frame_count += 1
            self.consec_failures = 0
            self.last_error = ''

    def recordFailure(self, msg):
        with self.health_lock:
            self.consec_failures += 1
            self.last_error = msg
            self.current_fps = 0.0

    def getImage(self, wait_sec = 0.1):
        # Returns [cv2_img, timestamp, success, msg]. Decodes and returns only
        # the newest grabbed frame, each frame at most once, waiting up to about
        # wait_sec for one to arrive. timestamp is the NEPI time the frame arrived.
        if self.fastmjpg_receiver is not None:
            return self.getFastMJPGImage(wait_sec)
        deadline = nepi_utils.get_time() + wait_sec
        if not self.cap_lock.acquire(timeout = wait_sec):
            return None, None, False, "Waiting for frame from " + self.safe_url
        try:
            if not self.imageAcquisitionRunning():
                return None, None, False, "Capture for " + self.safe_url + " not running"
            if self.frame_seq == self.returned_seq:
                remaining = deadline - nepi_utils.get_time()
                if remaining > 0:
                    self.frame_cond.wait(remaining) # Releases the lock while waiting
            if self.cap is None or self.frame_seq == self.returned_seq:
                return None, None, False, "No new frame from " + self.safe_url
            self.returned_seq = self.frame_seq
            timestamp = self.frame_timestamp
            success, cv2_img = self.cap.retrieve()
        finally:
            self.cap_lock.release()
        if success == False or cv2_img is None:
            return None, None, False, "Failed to decode frame from " + self.safe_url
        return cv2_img, timestamp, True, "Success"

    def getFramerate(self):
        if self.fastmjpg_receiver is not None:
            return True, self.getFastMJPGFps(self.fastmjpg_receiver.get_health())
        with self.health_lock:
            fps = self.current_fps
        return True, fps

    def get_health(self):
        """Report stream health for the node's watchdog and logs.

        Returns:
            dict: running (bool) -- acquisition started and not stopped;
                connected (bool) -- a capture is currently open (for a
                    fastmjpg:// URL, its UDP port is open);
                frame_count (int) -- frames received since construction;
                consecutive_failures (int) -- failed opens and reads since the
                    last good frame;
                seconds_since_frame (float or None) -- seconds since the newest
                    frame, or since acquisition started if no frame has arrived
                    since; None when acquisition is not running;
                fps (float) -- smoothed incoming frame rate;
                last_error (str) -- most recent failure message, credential-free.
        """
        if self.fastmjpg_receiver is not None:
            return self.getFastMJPGHealth()
        running = self.imageAcquisitionRunning()
        now = nepi_utils.get_time()
        with self.health_lock:
            seconds_since_frame = None
            if running and self.acq_start_time is not None:
                reference_time = self.acq_start_time
                if self.last_frame_time is not None and self.last_frame_time > reference_time:
                    reference_time = self.last_frame_time
                seconds_since_frame = now - reference_time
            return dict(running = running,
                        connected = self.connected,
                        frame_count = self.frame_count,
                        consecutive_failures = self.consec_failures,
                        seconds_since_frame = seconds_since_frame,
                        fps = self.current_fps,
                        last_error = self.last_error)


    #**********************
    # FastMJPG acquisition, for fastmjpg:// URLs

    def startFastMJPG(self):
        with self.health_lock:
            # Kept from the first start request until stop, so the node's stream
            # watchdog also covers a UDP port that cannot be opened
            if self.acq_start_time is None:
                self.acq_start_time = nepi_utils.get_time()
        [success, msg] = self.fastmjpg_receiver.start()
        with self.health_lock:
            self.connected = success
            if success == False:
                self.last_error = msg
        if success == False:
            return False, msg
        return True, "Started receiving " + self.safe_url

    def stopFastMJPG(self):
        self.fastmjpg_receiver.stop()
        with self.health_lock:
            self.connected = False
            self.acq_start_time = None
            self.last_error = ''
        return True, "Success"

    def getFastMJPGImage(self, wait_sec):
        if not self.fastmjpg_receiver.is_running():
            return None, None, False, "Capture for " + self.safe_url + " not running"
        [jpeg, capture_us, arrival_time, success, msg] = self.fastmjpg_receiver.get_jpeg(wait_sec = wait_sec)
        if success == False:
            return None, None, False, "No new frame from " + self.safe_url
        cv2_img = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
        if cv2_img is None:
            return None, None, False, "Failed to decode frame from " + self.safe_url
        # arrival_time comes from time.time(), the clock nepi_utils.get_time()
        # reads. capture_us is on the sender's clock, which NEPI does not share.
        return cv2_img, arrival_time, True, "Success"

    def getFastMJPGFps(self, receiver_health):
        seconds_since_frame = receiver_health['seconds_since_frame']
        if seconds_since_frame is None or seconds_since_frame > self.FPS_HOLD_SEC:
            return 0.0
        return receiver_health['fps']

    def getFastMJPGHealth(self):
        receiver_health = self.fastmjpg_receiver.get_health()
        with self.health_lock:
            acq_start_time = self.acq_start_time
            start_error = self.last_error
        seconds_since_frame = receiver_health['seconds_since_frame']
        if seconds_since_frame is None and acq_start_time is not None:
            # Started, but the UDP port could not be opened
            seconds_since_frame = nepi_utils.get_time() - acq_start_time
        last_error = receiver_health['last_error']
        if receiver_health['running'] == False:
            last_error = start_error
        return dict(running = acq_start_time is not None,
                    connected = receiver_health['running'],
                    frame_count = receiver_health['frame_count'],
                    consecutive_failures = 0,
                    seconds_since_frame = seconds_since_frame,
                    fps = self.getFastMJPGFps(receiver_health),
                    last_error = last_error)


#**********************
# FFmpeg capture options

def getAvformatMajorVersion():
    match = re.search(r'avformat:\s*YES\s*\((\d+)\.', cv2.getBuildInformation())
    if match is None:
        return 0
    return int(match.group(1))

def getFfmpegCaptureOptions():
    timeout_us = str(int(NetStreamCamDriver.READ_TIMEOUT_SEC * 1000000))
    options = [
        # Pull RTSP media interleaved on the RTSP TCP connection rather than over
        # UDP, as the ONVIF reader does: fewer decode errors, and a lost UDP packet
        # on a tether would otherwise smear a frame. Ignored for http(s).
        'rtsp_transport;tcp',
        # Demuxer flag: do not hold packets in FFmpeg's probe buffer on the way in.
        'fflags;nobuffer',
        # Decoder flag AV_CODEC_FLAG_LOW_DELAY: low-delay decoding, which also
        # stops FFmpeg using frame-threaded decoding (each extra frame thread holds
        # back one frame). It only reaches the decoder if this OpenCV build applies
        # capture options to the codec, which is build dependent and unverified.
        'flags;low_delay',
        # Protocol read/write timeout (microseconds): bounds http(s) connect and
        # reads on FFmpeg 4.x and 5.x.
        'rw_timeout;' + timeout_us,
        # RTSP socket I/O timeout (microseconds) under its FFmpeg 4.x name.
        # FFmpeg 5 renamed it to 'timeout' and ignores this one.
        'stimeout;' + timeout_us,
    ]
    if getAvformatMajorVersion() >= 59:
        # FFmpeg 5+ (avformat 59+) socket I/O timeout (microseconds) for rtsp and
        # http. Left out on FFmpeg 4.x, whose RTSP demuxer reads 'timeout' as a
        # listen timeout and would wait for an incoming connection instead.
        options.append('timeout;' + timeout_us)
    return '|'.join(options)

FFMPEG_CAPTURE_OPTIONS = getFfmpegCaptureOptions()


#**********************
# FastMJPG receiver
#
# Kept identical to FastMJPGReceiver in mate_robotics/Nepi_Connect/tools/
# fastmjpg_viewer.py, which tests it outside NEPI. Change both copies together.

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
