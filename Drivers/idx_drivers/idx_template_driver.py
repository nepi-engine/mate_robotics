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
# Redistributions in source code must retain this top-level comment bstab.
# Plagiarizing this software to sidestep the license obligations is illegal.
#
# Contact Information:
# ====================
# - mailto:nepi@numurus.com

##############################################################################
# IDX (Imaging / Camera) DRIVER TEMPLATE -- DRIVER (raw hardware I/O)
# ---------------------------------------------------------------------------
# WHY A SEPARATE DRIVER FILE?
#   The serial LSX/PTX/NPX templates do their I/O inline in the node. Cameras
#   are messier -- SDK handles, streaming threads, frame buffers, control
#   tables -- so IDX drivers factor the raw hardware I/O into this standalone
#   class. It imports NO NEPI ROS code (only nepi_utils for a clock) and has a
#   __main__ test harness, so you can exercise the hardware without ROS.
#
# THE INFORMAL DRIVER CONTRACT the node depends on:
#   isConnected()                 -> bool
#   getCameraControls()           -> dict: name -> {'type','value','min','max'}
#                                    ('type' is one of 'int'/'float'/'bool'/'menu';
#                                     a 'menu' control also carries a 'legend' dict
#                                     whose keys are the option labels). The node's
#                                     initSettingsDict() maps these onto the
#                                     nepi_controls CONTROL_TYPES -- so 'min'/'max'
#                                     become 'bounds' and 'legend' becomes 'options'.
#   getCameraControl(name)        -> (success, value)
#   setCameraControl(name, value) -> (success, msg)
#   getFramerate()                -> (success, fps)   NOTE: a pair, not a bare float
#   setFramerate(fps)             -> (success, msg)
#   getCurrentResolution()        -> (width, height)
#   setResolution(resolution_dict)-> (success, msg)   dict is {'width': w, 'height': h}
#   getCurrentFormatAvailableResolutions() -> (success, [ {'width','height'}, ... ])
#   startImageAcquisition()       -> (success, msg)   [opens stream + grab thread]
#   getImage()                    -> (cv2_img, timestamp, success, msg)
#   stopImageAcquisition()        -> (success, msg)
#
# getFramerate is wired STRAIGHT into IDXDeviceIF(getFramerate=...) by the node,
# so keep the (success, fps) shape the shipped drivers use.
#
# COMMON IDIOM (used by the shipped drivers): a background daemon thread calls
# cap.grab() continuously; getImage() does cap.retrieve() on demand. That
# decouples the sensor's native rate from NEPI's pull rate.
##############################################################################

import threading

import cv2  # TODO: replace with your camera SDK if not V4L2/OpenCV

from nepi_sdk import nepi_utils  # get_time() only -- no ROS dependency

MAX_CONSEC_FRAME_FAIL_COUNT = 3


class IdxTemplateDriver:

    def __init__(self, device_path):
        self.device_path = device_path
        self.connected = False
        self.cap = None
        self.img_lock = threading.Lock()
        self.grab_thread = None
        self.grabbing = False
        self.last_frame = None
        self.last_frame_time = None
        self.consec_fail = 0

        # TODO: probe the device and populate its control table. Each entry
        # needs a 'type' the node can map onto a nepi_controls CONTROL_TYPE,
        # plus the current 'value'; numeric controls add 'min'/'max' and menu
        # controls add a 'legend' whose keys are the option labels.
        self.camera_controls = {
            # 'brightness':  {'type': 'int',  'min': 0, 'max': 255, 'value': 128},
            # 'auto_focus':  {'type': 'bool', 'value': True},
            # 'power_line':  {'type': 'menu', 'value': 1,
            #                 'legend': {'Disabled': 0, '50 Hz': 1, '60 Hz': 2}},
        }
        self.width = 1280
        self.height = 720
        self.framerate = 30.0

        # A cheap open/close is a good "is this really our camera?" check.
        self.connected = self._probe()

    # ---- connection ------------------------------------------------------
    def _probe(self):
        try:
            cap = cv2.VideoCapture(self.device_path)
            ok = cap.isOpened()
            cap.release()
            return ok
        except Exception:
            return False

    def isConnected(self):
        return self.connected

    # ---- controls / format ----------------------------------------------
    def getCameraControls(self):
        return self.camera_controls

    def setCameraControl(self, name, value):
        # TODO: push the control to hardware.
        if name in self.camera_controls:
            self.camera_controls[name]['value'] = value
            return True, "Success"
        return False, "Unknown control: " + str(name)

    def getCameraControl(self, name):
        if name in self.camera_controls:
            return True, self.camera_controls[name].get('value')
        return False, None

    def getFramerate(self):
        # A pair, not a bare float -- the node hands this method to
        # IDXDeviceIF(getFramerate=...) unchanged.
        return True, self.framerate

    def setFramerate(self, fps):
        self.framerate = float(fps)
        if self.cap is not None:
            self.cap.set(cv2.CAP_PROP_FPS, self.framerate)
        return True, "Success"

    def getCurrentResolution(self):
        return (self.width, self.height)

    def getCurrentFormatAvailableResolutions(self):
        # TODO: enumerate the resolutions the current pixel format supports.
        # The node turns these into a 'resolution' Selection setting of
        # "<width>:<height>" strings.
        return True, [{'width': self.width, 'height': self.height}]

    def setResolution(self, resolution_dict):
        # The node passes a dict, not two arguments -- match the shipped drivers.
        self.width = int(resolution_dict['width'])
        self.height = int(resolution_dict['height'])
        if self.cap is not None:
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        return True, "Success"

    # ---- streaming -------------------------------------------------------
    def startImageAcquisition(self):
        with self.img_lock:
            if self.grabbing:
                return True, "Already acquiring"
            self.cap = cv2.VideoCapture(self.device_path)
            if not self.cap.isOpened():
                return False, "Failed to open " + str(self.device_path)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            self.cap.set(cv2.CAP_PROP_FPS, self.framerate)
            self.grabbing = True
            self.consec_fail = 0
            self.grab_thread = threading.Thread(target=self._runImgAcqThread, daemon=True)
            self.grab_thread.start()
        return True, "Success"

    def _runImgAcqThread(self):
        # Continuously pull frames off the device so getImage() returns fresh.
        while self.grabbing:
            if self.cap is None:
                break
            ret = self.cap.grab()
            if ret:
                ret, frame = self.cap.retrieve()
                if ret:
                    with self.img_lock:
                        self.last_frame = frame
                        self.last_frame_time = nepi_utils.get_time()
                        self.consec_fail = 0
                    continue
            self.consec_fail += 1
            if self.consec_fail >= MAX_CONSEC_FRAME_FAIL_COUNT:
                # Auto stop+restart on a run of failures (hardware hiccup).
                self.stopImageAcquisition()
                self.startImageAcquisition()
                return

    def getImage(self):
        with self.img_lock:
            if self.last_frame is None:
                return None, None, False, "No frame yet"
            return self.last_frame, self.last_frame_time, True, "Success"

    def stopImageAcquisition(self):
        with self.img_lock:
            self.grabbing = False
            if self.cap is not None:
                self.cap.release()
                self.cap = None
            self.last_frame = None
        return True, "Success"


# ---- standalone hardware test (no ROS) ----------------------------------
if __name__ == '__main__':
    import sys
    path = sys.argv[1] if len(sys.argv) > 1 else '/dev/video0'
    drv = IdxTemplateDriver(path)
    print("connected:", drv.isConnected())
    print(drv.startImageAcquisition())
    for _ in range(30):
        img, ts, ok, msg = drv.getImage()
        if ok:
            print("frame", img.shape, "@", ts)
            break
    drv.stopImageAcquisition()
