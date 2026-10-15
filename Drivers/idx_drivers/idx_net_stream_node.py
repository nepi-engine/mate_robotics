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

# IDX node for one network video stream. Registers the stream as a standard
# NEPI camera through IDXDeviceIF. The camera itself belongs to the remote
# stream source, so there are no camera settings here; max framerate and
# resolution are applied on the NEPI side, by frame skipping and downscaling.
#
# When the stream has delivered no frame for STREAM_LOST_TIMEOUT_SEC the node
# shuts itself down, as the ONVIF node does when it cannot connect, and
# discovery relaunches it once the stream answers a probe again.


import threading

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_drvs
from nepi_sdk import nepi_img

from nepi_api.device_if_idx import IDXDeviceIF
from nepi_api.messages_if import MsgIF


PKG_NAME = 'IDX_NET_STREAM' # Use in display menus
FILE_TYPE = 'NODE'


class NetStreamCamNode:

    # Field of view is left at the IDX defaults: set it on the camera's IDX page
    # to match the remote camera's lens.
    FACTORY_CONTROLS = dict(
    max_framerate = 100, # No NEPI-side limit until the operator sets one; the stream's own rate governs
    frame_id = 'sensor_frame'
    )

    # Longer than the driver needs to notice a dead or stalled stream and try one
    # re-open in-process (read timeout, re-open delay, open timeout)
    STREAM_LOST_TIMEOUT_SEC = 15.0
    STREAM_STALL_WARN_SEC = 2.0
    CHECK_STREAM_INTERVAL_SEC = 1.0
    IMAGE_WAIT_SEC = 0.1 # Longest getColorImg() waits for a new frame
    FRAME_DUE_SLACK_SEC = 0.005 # Arrival jitter tolerated by the max framerate gate

    device_info_dict = dict(device_name = "",
                            path = "",
                            serial_number = "",
                            hw_version = "",
                            sw_version = "")


    # Create threading locks, controls, and status
    img_lock = threading.Lock()
    color_image_acquisition_running = False
    idx_if = None

    max_framerate = 100
    resolution_ratio = 1.0
    next_frame_due = None
    stream_stalled = False

    ################################################
    DEFAULT_NODE_NAME = PKG_NAME.lower() + "_node"
    drv_dict = dict()
    driver = None
    def __init__(self):
        ####  NODE Initialization ####
        nepi_sdk.init_node(name= self.DEFAULT_NODE_NAME)
        self.class_name = type(self).__name__
        self.base_namespace = nepi_sdk.get_base_namespace()
        self.node_name = nepi_sdk.get_node_name()
        self.node_namespace = nepi_sdk.get_node_namespace()

        ##############################
        # Create Msg Class
        self.msg_if = MsgIF()
        self.msg_if.pub_info("Starting Node Initialization Processes")

        ##############################
        # Initialize Class Variables

        # Get required drv driver dict info. The dict is not logged whole: the
        # stream URL in it can carry credentials.
        try:
            self.drv_dict = nepi_sdk.get_param('~drv_dict',dict())
            self.device_name = self.drv_dict['DEVICE_DICT']['device_name']
            self.device_path = self.drv_dict['DEVICE_DICT']['device_path'] # Credential-free URL
            self.stream_url = self.drv_dict['DEVICE_DICT']['stream_url']
            self.driver_path = self.drv_dict['path']
            self.driver_file = self.drv_dict['DRIVER_DICT']['file_name']
            self.driver_module = self.driver_file.split('.')[0]
            self.driver_class_name = self.drv_dict['DRIVER_DICT']['class_name']
        except Exception as e:
            self.msg_if.pub_warn("Failed to load Device Dict " + str(e))#
            nepi_sdk.signal_shutdown(self.node_name + ": Shutting down because no valid Device Dict")
            return

        # import driver class from driver module
        self.msg_if.pub_info("Importing driver class " + self.driver_class_name + " from module " + self.driver_module)
        [success, msg, self.driver_class] = nepi_drvs.importDriverClass(self.driver_file,self.driver_path,self.driver_module,self.driver_class_name)
        if success:
            try:
                self.driver = self.driver_class(self.stream_url)
            except Exception as e:
                # Driver construction raises only credential-free messages
                self.msg_if.pub_warn("Failed to instantiate driver: " + str(e))
        if self.driver is None:
            nepi_sdk.signal_shutdown(self.node_name + ": Shutting down because no driver loaded for stream " + self.device_path)
            return
        self.msg_if.pub_info("Driver ready for stream " + self.device_path)


        # Launch the IDX interface. No getSettingsFunction/setSettingFunction:
        # the remote stream source owns the camera's controls.
        self.msg_if.pub_info("Launching NEPI IDX () interface...")
        self.device_info_dict["device_name"] = self.device_name
        self.device_info_dict["path"] = self.device_path
        self.idx_if = IDXDeviceIF(device_info = self.device_info_dict,
                                    data_source_description = 'camera',
                                    data_ref_description = 'camera_lense',
                                    factoryControls = self.FACTORY_CONTROLS,
                                    get_rtsp_url = self.getRtspUrl,
                                    setResolutionRatio = self.setResolutionRatio,
                                    setMaxFramerate = self.setMaxFramerate,
                                    getFramerate = self.driver.getFramerate,
                                    getColorImage = self.getColorImg,
                                    stopColorImageAcquisition = self.stopColorImg,
                                    data_products = ['color_image'])
        self.msg_if.pub_info(" ... IDX interface running")

        # IDXDeviceIF reloads the saved max framerate and resolution ratio but
        # does not pass them to the callbacks at startup. Pick them up here so a
        # relaunch after a lost stream keeps the operator's settings.
        self.setMaxFramerate(self.idx_if.max_framerate)
        self.setResolutionRatio(self.idx_if.resolution_ratio)

        # Start receiving now, as the V4L2 node does, so the stream watchdog
        # covers a stream that never delivers even before anyone subscribes
        self.img_lock.acquire()
        self.driver.startImageAcquisition()
        self.color_image_acquisition_running = True
        self.img_lock.release()

        nepi_sdk.start_timer_process(self.CHECK_STREAM_INTERVAL_SEC, self.checkStreamCb, oneshot = True)
        nepi_sdk.on_shutdown(self.cleanup_actions)

        ## Initiation Complete
        self.msg_if.pub_info("Initialization Complete")
        # Now start the node
        nepi_sdk.spin()


    #**********************
    # IDX driver functions

    def getRtspUrl(self):
        # Credential-free URL; the IDX status message publishes all three fields
        return [self.device_path, "", ""]

    def setMaxFramerate(self, rate):
        if rate is None:
            return False, 'Got None Max Framerate'
        if rate < 1:
            rate = 1
        if rate > 100:
            rate = 100
        self.max_framerate = rate
        self.next_frame_due = None
        status = True
        err_str = ""
        return status, err_str

    def setResolutionRatio(self, ratio):
        if ratio is None:
            return False, 'Got None Resolution Ratio'
        if ratio < 0.1:
            ratio = 0.1
        if ratio > 1.0:
            ratio = 1.0
        self.resolution_ratio = ratio
        status = True
        err_str = ""
        return status, err_str

    def getColorImg(self):
        # Max framerate as frame skipping: until the next frame is due, return
        # without touching the stream. The grab thread keeps draining it, so the
        # frame taken when one is due is the newest available.
        current_time = nepi_utils.get_time()
        if self.next_frame_due is not None and current_time < self.next_frame_due - self.FRAME_DUE_SLACK_SEC:
            return False, "Waiting for Timer", None, None, None  # Return None data

        encoding = "bgr8"
        self.img_lock.acquire()
        # Always try to start image acquisition -- no big deal if it was already started; driver returns quickly
        ret, msg = self.driver.startImageAcquisition()
        if ret is False:
            self.img_lock.release()
            return ret, msg, None, None, None
        self.color_image_acquisition_running = True
        cv2_img, timestamp, ret, msg = self.driver.getImage(wait_sec = self.IMAGE_WAIT_SEC)
        self.img_lock.release()
        if ret is False:
            return ret, msg, None, None, None

        # Schedule the next frame from this one's arrival time. Due times advance
        # by whole intervals, so a 30 fps stream limited to 20 fps keeps two
        # frames in three instead of dropping to every other frame.
        frame_interval = 1.0 / self.max_framerate
        if self.next_frame_due is None or timestamp - self.next_frame_due > frame_interval:
            self.next_frame_due = timestamp + frame_interval
        else:
            self.next_frame_due += frame_interval

        # Resolution ratio as NEPI-side downscaling
        if self.resolution_ratio < 1.0:
            [cv2_img, img_shape] = nepi_img.adjust_resolution_ratio(cv2_img, self.resolution_ratio)
        return ret, msg, cv2_img, timestamp, encoding

    def stopColorImg(self):
        self.img_lock.acquire()
        ret,msg = self.driver.stopImageAcquisition()
        self.color_image_acquisition_running = False
        self.next_frame_due = None
        self.img_lock.release()
        return ret,msg


    #**********************
    # Stream watchdog

    def checkStreamCb(self, timer):
        health = self.driver.get_health()
        seconds_since_frame = health['seconds_since_frame']
        if seconds_since_frame is not None and seconds_since_frame > self.STREAM_LOST_TIMEOUT_SEC:
            self.msg_if.pub_warn("No frames from stream " + self.device_path + " for " + str(int(seconds_since_frame)) +
                                 " s. Shutting down so discovery can relaunch it when the stream returns")
            nepi_sdk.signal_shutdown(self.node_name + ": Shutting down because stream " + self.device_path + " was lost")
            return
        if seconds_since_frame is not None and seconds_since_frame > self.STREAM_STALL_WARN_SEC:
            if self.stream_stalled == False:
                self.stream_stalled = True
                reason = ""
                if health['last_error'] != "":
                    reason = " (" + health['last_error'] + ")"
                self.msg_if.pub_warn("No frames from stream " + self.device_path + " for over " +
                                     str(int(self.STREAM_STALL_WARN_SEC)) + " s" + reason)
        elif self.stream_stalled == True and health['connected'] == True:
            self.stream_stalled = False
            self.msg_if.pub_info("Stream " + self.device_path + " recovered at " + str(round(health['fps'],1)) + " fps")
        nepi_sdk.start_timer_process(self.CHECK_STREAM_INTERVAL_SEC, self.checkStreamCb, oneshot = True)


    def cleanup_actions(self):
        self.msg_if.pub_info("Shutting down: Executing script cleanup actions")
        if self.driver is not None:
            # Releasing the capture closes the session with the stream source
            # instead of leaving it to time out
            self.driver.stopImageAcquisition()


if __name__ == '__main__':
    node = NetStreamCamNode()
