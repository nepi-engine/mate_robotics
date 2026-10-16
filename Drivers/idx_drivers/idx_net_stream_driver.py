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
# stream URL through OpenCV's FFmpeg backend, the only video backend in the NEPI
# container's OpenCV build (GStreamer is not compiled in).
#
# The reader logic is adapted from idx_onvif_generic_driver.py (FFmpeg backend,
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
import threading
import cv2
from urllib.parse import urlsplit

from nepi_sdk import nepi_utils


PKG_NAME = 'IDX_NET_STREAM' # Use in display menus
FILE_TYPE = 'DRIVER'


class NetStreamCamDriver(object):
    SUPPORTED_SCHEMES = ['rtsp', 'rtsps', 'http', 'https']
    DEFAULT_PORTS = dict(rtsp = 554, rtsps = 322, http = 80, https = 443)

    OPEN_TIMEOUT_SEC = 5.0
    READ_TIMEOUT_SEC = 3.0
    MAX_CONSEC_FRAME_FAIL_COUNT = 3 # Grab failures on one open capture before it is re-opened
    REOPEN_DELAY_SEC = 1.0
    FPS_SMOOTHING = 0.1

    def __init__(self, url):
        [supported, msg] = self.is_supported_url(url)
        if supported == False:
            raise ValueError(msg)
        self.url = str(url).strip()
        self.safe_url = self.get_safe_url(self.url)

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
        if hostname is None or hostname == '':
            return False, "No host in stream URL " + safe_url
        return True, "Supported"

    @classmethod
    def probe_stream(cls, url):
        """Open a stream URL, read one frame, and close it again.

        Blocks for at most the open timeout plus the read timeout when the
        installed OpenCV supports its timeout properties; otherwise FFmpeg's own
        socket timeouts and OpenCV's internal FFmpeg interrupt bound the call.

        Args:
            url (str): Stream URL.

        Returns:
            list: [success (bool), msg (str)]. msg is credential-free.
        """
        [supported, msg] = cls.is_supported_url(url)
        if supported == False:
            return False, msg
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


    #**********************
    # Image acquisition

    def imageAcquisitionRunning(self):
        return self.acq_thread is not None and not self.acq_stop_event.is_set()

    def startImageAcquisition(self):
        if self.imageAcquisitionRunning():
            return True, "Already capturing from " + self.safe_url
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
        with self.health_lock:
            fps = self.current_fps
        return True, fps

    def get_health(self):
        """Report stream health for the node's watchdog and logs.

        Returns:
            dict: running (bool) -- acquisition started and not stopped;
                connected (bool) -- a capture is currently open;
                frame_count (int) -- frames received since construction;
                consecutive_failures (int) -- failed opens and reads since the
                    last good frame;
                seconds_since_frame (float or None) -- seconds since the newest
                    frame, or since acquisition started if no frame has arrived
                    since; None when acquisition is not running;
                fps (float) -- smoothed incoming frame rate;
                last_error (str) -- most recent failure message, credential-free.
        """
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
