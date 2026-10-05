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
# IDX (Imaging / Camera) DRIVER TEMPLATE -- NODE
# ---------------------------------------------------------------------------
# Unlike the serial templates, the IDX node does NOT do hardware I/O itself.
# It:
#   1. reads '~drv_dict' (written by the LAUNCH-model discovery node),
#   2. dynamically imports the DRIVER class named in DRIVER_DICT via
#      nepi_drvs.importDriverClass() and constructs it,
#   3. builds its settings controls dict,
#   4. registers an IDXDeviceIF, handing it image-acquisition callbacks,
#   5. spin()s. IDXDeviceIF pulls frames by calling getColorImg() -- there is
#      no publish loop in this file.
#
# getColorImg() returns a 5-TUPLE: (ret, msg, cv2_img, timestamp, encoding).
# Return (False, "...", None, None, None) when there is no new frame.
#
# SETTINGS CONTRACT (the same in every NEPI driver -- see idx_v4l2_node.py,
# which is the reference implementation):
#   initSettingsDict()      build the controls dict once, at startup
#   refreshSettingsDict()   read live values (and bounds/options) back
#   getSettingsFunction()   -> the current controls dict, no arguments
#   setSettingFunction(name, value) -> [success, msg, settings_dict]  (3 items!)
# Only the last two are handed to the device IF. The retired
# capSettings= / factorySettings= / settingUpdateFunction= arguments no longer
# exist on ANY device IF -- passing them raises TypeError at construction.
##############################################################################

import copy
import threading

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_drvs        # importDriverClass
from nepi_sdk import nepi_controls    # controls dict: create/get/set value, bounds, options

from nepi_api.messages_if import MsgIF
from nepi_api.device_if_idx import IDXDeviceIF

PKG_NAME = 'IDX_TEMPLATE'
FILE_TYPE = 'NODE'


class IdxTemplateNode:

    # ---- Runtime-tunable settings. An IDX camera builds these dynamically from
    #      the driver's own control table (see initSettingsDict below) rather
    #      than from a static CAP_SETTINGS table, because which controls a
    #      camera exposes is only known once the device is open. Anything you
    #      want forced away from the camera's own reported value goes here.
    FACTORY_SETTINGS_OVERRIDES = dict()

    # ---- Factory controls IDXDeviceIF expects (FOV + frame id, etc.).
    FACTORY_CONTROLS = dict(width_deg=90, height_deg=60, frame_id='sensor_frame')

    device_info_dict = dict(device_name="", path="", serial_number="",
                            hw_version="", sw_version="")

    max_framerate = 30.0
    cl_img_last_time = None
    idx_if = None

    init_settings_dict = dict()
    settings_dict = dict()

    DEFAULT_NODE_NAME = PKG_NAME.lower() + "_node"

    ##########################################################################
    def __init__(self):
        nepi_sdk.init_node(name=self.DEFAULT_NODE_NAME)
        self.class_name = type(self).__name__
        self.base_namespace = nepi_sdk.get_base_namespace()
        self.node_name = nepi_sdk.get_node_name()
        self.node_namespace = nepi_sdk.get_node_namespace()
        self.msg_if = MsgIF(log_name=self.class_name)
        self.msg_if.pub_info("Starting Node Initialization Processes")

        # ---- Read config + the DRIVER class coordinates from '~drv_dict'.
        try:
            self.drv_dict = nepi_sdk.get_param('~drv_dict', dict())
            self.device_name = self.drv_dict['DEVICE_DICT']['device_name']
            self.device_path = self.drv_dict['DEVICE_DICT']['device_path']
            self.driver_path = self.drv_dict['path']                 # injected by drivers_mgr
            self.driver_file = self.drv_dict['DRIVER_DICT']['file_name']
            self.driver_module = self.driver_file.split('.')[0]
            self.driver_class_name = self.drv_dict['DRIVER_DICT']['class_name']
        except Exception as e:
            self.msg_if.pub_warn("Failed to load Device Dict " + str(e))
            nepi_sdk.signal_shutdown(self.node_name + ": no valid Device Dict")
            return

        # ---- Dynamically import + construct the raw-I/O driver class.
        [success, msg, self.driver_class] = nepi_drvs.importDriverClass(
            self.driver_file, self.driver_path, self.driver_module, self.driver_class_name)
        if not success:
            self.msg_if.pub_warn("Failed to import driver class: " + str(msg))
            nepi_sdk.signal_shutdown(self.node_name + ": driver import failed")
            return
        try:
            self.driver = self.driver_class(self.device_path)   # TODO: match your driver ctor
        except Exception as e:
            self.msg_if.pub_warn("Failed to construct driver: " + str(e))
            nepi_sdk.signal_shutdown(self.node_name + ": driver construct failed")
            return
        if not self.driver.isConnected():
            self.msg_if.pub_warn("Camera not connected at " + str(self.device_path) +
                                 "; shutting down instead of presenting a device that "
                                 "produces no images")
            nepi_sdk.signal_shutdown(self.node_name + ": camera not connected")
            return

        self.img_lock = threading.Lock()

        # ---- Build settings from the driver's control table + fill device_info.
        self.settings_dict = self.initSettingsDict()
        self.settings_dict = self.refreshSettingsDict()
        self.device_info_dict["device_name"] = self.device_name
        self.device_info_dict["path"] = self.device_path
        self.device_info_dict["serial_number"] = ""
        self.device_info_dict["hw_version"] = ""
        self.device_info_dict["sw_version"] = ""

        # ---- Register the camera with NEPI. Pass a getX/stopX pair for every
        #      data product the camera provides; omit the rest. The IF derives
        #      its capability flags from which callbacks are non-None, so a
        #      capability you do not wire up is simply not advertised. This
        #      template provides only a color image.
        self.idx_if = IDXDeviceIF(
            device_info=self.device_info_dict,
            data_source_description='camera',
            data_ref_description='camera_lens',
            getSettingsFunction=self.getSettingsFunction,
            setSettingFunction=self.setSettingFunction,
            factoryControls=self.FACTORY_CONTROLS,
            setMaxFramerate=self.setMaxFramerate,
            getFramerate=self.driver.getFramerate,
            getColorImage=self.getColorImg,
            stopColorImageAcquisition=self.stopColorImg,
            # ---- Optional, all default to None. Wire up only what the camera has:
            # getDepthMap=..., stopDepthMapAcquisition=...,       # depth cameras
            # getPointcloud=..., stopPointcloudAcquisition=...,   # 3D cameras
            # getFOV=..., perspective='pov',                      # field of view
            # get_rtsp_url=...,                                   # IP cameras
            # setResolutionRatio=..., setRangeRatio=...,
            # setContrastRatio=..., setBrightnessRatio=..., setThresholdingRatio=...,
            # setAutoAdjustRatio=..., autoAdjustControls=[...],
            # getNavPoseCb=..., navpose_update_rate=10,           # cameras that report pose
            data_products=['color_image'],
        )

        nepi_sdk.on_shutdown(self.cleanup_actions)
        self.msg_if.pub_info("Initialization complete")
        nepi_sdk.spin()

    #########################################################################
    # SETTINGS  (built from the driver's control table)
    #
    # The settings dict IS a nepi_controls controls dict. Build a plain init
    # dict first -- each entry keyed by setting name, carrying a 'type' from
    # nepi_controls.CONTROL_TYPES, a typed 'default', plus 'bounds' for
    # Int/Float or 'options' for Menu/Selection -- then hand the whole thing to
    # nepi_controls.create_controls_dict().
    #
    # TWO TRAPS when porting an old driver:
    #   * 'Discrete' is NOT a CONTROL_TYPE. create_controls_dict() wraps every
    #     entry in a bare except, so a 'Discrete' setting is silently dropped
    #     and simply never appears in the RUI. A named option list is
    #     'Selection'.
    #   * The retired cap-settings form carried an Int/Float control's min and
    #     max in an 'options' PAIR. Those are 'bounds' now; 'options' on a
    #     numeric control is ignored.
    #########################################################################
    def initSettingsDict(self):
        init_settings_dict = dict()
        for setting_name, info in self.driver.getCameraControls().items():
            setting_dict = dict()

            # TODO: map your driver's own type tokens onto CONTROL_TYPES.
            setting_type = info.get('type', 'int')
            if setting_type == 'int':
                setting_type = 'Int'
            elif setting_type == 'float':
                setting_type = 'Float'
            elif setting_type == 'bool':
                setting_type = 'Toggle'
            elif setting_type == 'menu':
                setting_type = 'Menu'
            setting_dict['type'] = setting_type

            current = info.get('value')
            try:
                if setting_type == 'Int':
                    setting_dict['default'] = int(current)
                    setting_dict['bounds'] = [int(info['min']), int(info['max'])]
                elif setting_type == 'Float':
                    setting_dict['default'] = float(current)
                    setting_dict['bounds'] = [float(info['min']), float(info['max'])]
                elif setting_type == 'Toggle':
                    setting_dict['default'] = (str(current) in ('True', 'true'))
                elif setting_type in ('Menu', 'Selection', 'Selections'):
                    setting_dict['default'] = int(current)
                    setting_dict['options'] = list(info['legend'].keys())
                else:
                    setting_dict['default'] = str(current)
            except Exception as e:
                self.msg_if.pub_warn("Skipping setting " + setting_name + " : " + str(e))
                continue

            if setting_name in self.FACTORY_SETTINGS_OVERRIDES:
                setting_dict['default'] = self.FACTORY_SETTINGS_OVERRIDES[setting_name]
            init_settings_dict[setting_name] = setting_dict

        # TODO: add any setting the camera exposes outside its control table --
        # the shipped drivers add 'resolution' (a Selection of "W:H" strings)
        # and 'framerate' (a Float) here.

        self.init_settings_dict = init_settings_dict
        settings_dict = nepi_controls.create_controls_dict(init_settings_dict)
        self.msg_if.pub_info("Initialized Settings: " +
                             str(nepi_controls.get_values_dict(settings_dict)))
        return settings_dict

    def refreshSettingsDict(self):
        # A camera reports its controls live, so values AND bounds can move.
        settings_dict = copy.deepcopy(self.settings_dict)
        controls = self.driver.getCameraControls()
        for setting_name in settings_dict.keys():
            if setting_name not in controls:
                continue
            info = controls[setting_name]
            try:
                settings_dict = nepi_controls.set_value(settings_dict, setting_name,
                                                        info.get('value'))
                if 'min' in info and 'max' in info:
                    settings_dict = nepi_controls.set_bounds(settings_dict, setting_name,
                                                             [info['min'], info['max']])
            except Exception as e:
                self.msg_if.pub_warn("Failed to refresh setting " + setting_name + " : " + str(e))
        return settings_dict

    def getSettingsFunction(self):
        return self.settings_dict

    def setSettingFunction(self, setting_name, setting_value):
        # SettingsIF replaces its own dict with the THIRD element, so all three
        # must always be returned. It also calls this as a bare statement during
        # SettingsIF.init(), so the return being discarded must be harmless.
        setting_str = setting_name + ":" + str(setting_value)
        if setting_name not in self.settings_dict.keys():
            return False, (self.node_name + " Setting name " + setting_str +
                           " is not supported"), self.settings_dict

        cur_val = nepi_controls.get_value(self.settings_dict, setting_name)
        if str(cur_val) == str(setting_value):
            return True, (self.node_name + " unchanged " + setting_str), self.settings_dict

        # TODO: push the control to hardware. Read and write control values only
        # through nepi_controls -- never by indexing the controls dict directly.
        success, msg = self.driver.setCameraControl(setting_name, setting_value)
        if success:
            msg = (self.node_name + " UPDATED SETTINGS " + setting_str)
            self.settings_dict = self.refreshSettingsDict()
        return success, msg, self.settings_dict

    def setMaxFramerate(self, rate):
        self.max_framerate = max(1.0, min(100.0, float(rate)))
        return True, ""

    #########################################################################
    # IMAGE ACQUISITION CALLBACKS  (the heart of an IDX driver)
    #########################################################################
    def getColorImg(self):
        # Rate-limit to max_framerate so we don't outrun the configured FPS.
        now = nepi_utils.get_time()
        if self.cl_img_last_time is not None:
            if (now - self.cl_img_last_time) < (1.0 / self.max_framerate):
                return False, "Waiting for timer", None, None, None

        with self.img_lock:
            # Always try to start acquisition -- a driver returns quickly when
            # it is already streaming.
            ret, msg = self.driver.startImageAcquisition()
            if ret is False:
                # throttle_s keeps a disconnected camera from filling the log
                # while still surfacing the failure without Debug Mode.
                self.msg_if.pub_warn("Image acquisition failed to start: " + str(msg),
                                     throttle_s=5.0)
                return False, msg, None, None, None
            cv2_img, timestamp, ret, msg = self.driver.getImage()
        if not ret:
            self.msg_if.pub_warn("No color image: " + str(msg), throttle_s=20.0)
            return False, msg, None, None, None
        if timestamp is None:
            timestamp = nepi_utils.get_time()
        self.cl_img_last_time = now
        # 5-tuple: (ret, msg, image, timestamp, encoding)
        return True, "Success", cv2_img, timestamp, "bgr8"

    def stopColorImg(self):
        with self.img_lock:
            ret, msg = self.driver.stopImageAcquisition()
        return ret, msg

    #########################################################################
    def cleanup_actions(self):
        try:
            self.driver.stopImageAcquisition()
        except Exception:
            pass


if __name__ == '__main__':
    node = IdxTemplateNode()
