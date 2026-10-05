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
# PTX (Pan-Tilt) DRIVER TEMPLATE -- NODE
# ---------------------------------------------------------------------------
# Registers a PTXActuatorIF. PTX has the largest callback surface of the
# device types: jog (plain and at a speed ratio), absolute goto, position
# feedback, soft limits, whole-unit and per-axis speed control, homing, and an
# optional navpose (orientation) report. Pass a callback for every capability
# the hardware has and None for the rest -- PTXActuatorIF advertises only what
# you wire up.
#
# HOW THE IF DERIVES ITS CAPABILITY FLAGS (from device_if_ptx.py -- this is
# what makes the pass-a-callback-or-None rule load-bearing):
#   has_absolute_positioning    gotoPositionCb OR gotoPanPositionCb
#   has_seperate_pan_tilt_control  gotoPanPositionCb AND gotoTiltPositionCb
#   has_timed_positioning       movePanCb OR moveTiltCb
#   has_timed_speed_positioning movePanSpeedRatioCb OR moveTiltSpeedRatioCb
#   has_adjustable_speed        getSpeedRatioCb AND setSpeedRatioCb
#   has_seperate_pan_tilt_speed ALL FOUR of get/setPan/TiltSpeedRatioCb
# Wiring three of the four per-axis speed callbacks gets you nothing -- the
# flag stays False and the RUI never shows per-axis speed.
#
# CALLBACK ARITIES (exactly how PTXActuatorIF calls them):
#   stopMovingCb()                          movePanCb(direction, time_s)
#   moveTiltCb(direction, time_s)           movePanSpeedRatioCb(direction, speed_ratio, time_s)
#   moveTiltSpeedRatioCb(direction, speed_ratio, time_s)
#   gotoPositionCb(pan_deg, tilt_deg)       gotoPanPositionCb(pan_deg)
#   gotoTiltPositionCb(tilt_deg)            getPositionCb() -> [pan_deg, tilt_deg]
#   getPositionTimesCb() -> [pan_t, tilt_t] getSoftLimitsCb() -> [min_pan, max_pan, min_tilt, max_tilt]
#   setSoftLimitsCb(min_pan, max_pan, min_tilt, max_tilt)
#   getSpeedMaxCb() -> degrees_per_second    setSpeedMaxCb(degrees_per_second)
#   get/setSpeedRatioCb -- 0.0-1.0           setHomePositionCb(pan_deg, tilt_deg)
#
# SETTINGS CONTRACT (identical in every NEPI driver -- idx_v4l2_node.py is the
# reference implementation):
#   initSettingsDict() / refreshSettingsDict() / getSettingsFunction() /
#   setSettingFunction(name, value) -> [success, msg, settings_dict]
# The retired capSettings= / factorySettings= / settingUpdateFunction=
# arguments no longer exist on ANY device IF -- passing them raises TypeError
# at construction. PTXActuatorIF still ACCEPTS getCapSettingsFunction, but it
# is inert: it is never forwarded to SettingsIF, which has no such parameter.
# Capability data (type, bounds, options) rides in the controls dict itself.
# Do not pass it. A pan-tilt with no settings passes neither settings function.
#
# Angle convention: all angles are DEGREES in the pan-tilt's own frame. The IF
# applies the reverse-axis conversion and the soft-limit clamp BEFORE calling
# these callbacks, so a callback receives a safe value in the device's own
# (non-reversed) frame -- do not re-apply either here.
##############################################################################

import os
import time
import copy
import serial
import serial.tools.list_ports

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_nav          # BLANK_NAVPOSE_DICT
from nepi_sdk import nepi_controls     # controls dict: create/get/set value, bounds, options

from nepi_api.messages_if import MsgIF
from nepi_api.device_if_ptx import PTXActuatorIF

PKG_NAME = 'PTX_TEMPLATE'
FILE_TYPE = 'NODE'


class PtxTemplateNode:

    # ---- Runtime-tunable settings (see lsx_template_node.py for the fully
    #      annotated pattern). 'type' must be a nepi_controls CONTROL_TYPE --
    #      'Discrete' is not one, and a setting declared that way is silently
    #      dropped and never appears in the RUI.
    CAP_SETTINGS = dict(
        status_update_rate_hz={"type": "Int", "name": "status_update_rate_hz", "options": ["1", "20"]},
    )
    FACTORY_SETTINGS = dict(
        status_update_rate_hz={"type": "Int", "name": "status_update_rate_hz", "value": "5"},
    )
    FACTORY_SETTINGS_OVERRIDES = dict()
    settingFunctions = dict(
        status_update_rate_hz={'get': 'getStatusRate', 'set': 'setStatusRate'},
    )

    # ---- FACTORY_CONTROLS: descriptive keys used by the RUI + the three keys
    #      PTXActuatorIF actually merges. The IF copies a key only if it is
    #      already present in its own FACTORY_CONTROLS_DICT, which holds exactly
    #      reverse_pan_enabled / reverse_tilt_enabled / speed_ratio. A key spelt
    #      any other way (reverse_pan_control, say) is silently ignored.
    FACTORY_CONTROLS = dict(
        frame_id='ptx_template_frame',
        pan_joint_name='ptx_template_pan_joint',
        tilt_joint_name='ptx_template_tilt_joint',
        reverse_pan_enabled=False,
        reverse_tilt_enabled=False,
        speed_ratio=0.5,
        status_update_rate_hz=5,
    )

    # ---- factoryLimits: PTXActuatorIF REQUIRES all 8 of these keys.
    LIMITS_DICT = dict(
        max_pan_hardstop_deg=175, min_pan_hardstop_deg=-175,
        max_tilt_hardstop_deg=90, min_tilt_hardstop_deg=-90,
        max_pan_softstop_deg=165, min_pan_softstop_deg=-165,
        max_tilt_softstop_deg=80, min_tilt_softstop_deg=-80,
    )

    device_info_dict = dict(device_name="", path="", serial_number="",
                            hw_version="", sw_version="")

    MAX_POSITION_UPDATE_RATE = 5

    # Bounds on an operator-supplied max speed. Keep the range wide enough to
    # cover the real hardware -- a too-narrow guard silently caps the axis at a
    # fraction of what it can do.
    MIN_SPEED_MAX_DPS = 1
    MAX_SPEED_MAX_DPS = 1000

    serial_port = None
    serial_busy = False
    connected = False
    serial_num = ""
    hw_version = ""
    sw_version = ""

    speed_ratio = 0.5
    pan_speed_ratio = 0.5
    tilt_speed_ratio = 0.5
    speed_max_dps = 30.0
    status_rate_hz = 5
    home_pan_deg = 0.0
    home_tilt_deg = 0.0
    current_position = [0.0, 0.0]       # [pan_deg, tilt_deg]
    position_times = [0.0, 0.0]
    soft_limits = [-165.0, 165.0, -80.0, 80.0]

    init_settings_dict = dict()
    settings_dict = dict()

    ptx_if = None
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

        try:
            self.drv_dict = nepi_sdk.get_param('~drv_dict', dict())
            self.device_name = self.drv_dict['DEVICE_DICT']['device_name']
            self.device_path = self.drv_dict['DEVICE_DICT']['device_path']
            self.port_str = self.drv_dict['DEVICE_DICT']['device_path']
            self.baud_int = int(self.drv_dict['DEVICE_DICT']['baud_str'])
            self.addr_str = self.drv_dict['DEVICE_DICT']['addr_str']
        except Exception as e:
            self.msg_if.pub_warn("Failed to load Device Dict " + str(e))
            nepi_sdk.signal_shutdown(self.node_name + ": no valid Device Dict")
            return

        self.connected = self.connect()
        if not self.connected:
            nepi_sdk.signal_shutdown(self.node_name + ": failed to connect")
            return
        self.msg_if.pub_info("Connected")

        # Seed the node's own soft limits from the factory table before the IF
        # reads them back through getSoftLimitsCb.
        self.soft_limits = [self.LIMITS_DICT['min_pan_softstop_deg'],
                            self.LIMITS_DICT['max_pan_softstop_deg'],
                            self.LIMITS_DICT['min_tilt_softstop_deg'],
                            self.LIMITS_DICT['max_tilt_softstop_deg']]

        self.settings_dict = self.initSettingsDict()
        self.settings_dict = self.refreshSettingsDict()
        self.device_info_dict["device_name"] = self.device_name
        self.device_info_dict["path"] = self.device_path
        self.device_info_dict["serial_number"] = self.serial_num
        self.device_info_dict["hw_version"] = self.hw_version
        self.device_info_dict["sw_version"] = self.sw_version

        # ---- Register the pan-tilt with NEPI.
        self.ptx_if = PTXActuatorIF(
            device_info=self.device_info_dict,
            factoryControls=self.FACTORY_CONTROLS,
            factoryLimits=self.LIMITS_DICT,
            getSettingsFunction=self.getSettingsFunction,
            setSettingFunction=self.setSettingFunction,
            data_source_description='pan_tilt',
            data_ref_description='tilt_axis_center',
            # --- jog / relative motion
            stopMovingCb=self.stopMoving,
            movePanCb=self.movePan,                         # (direction, duration)
            moveTiltCb=self.moveTilt,                       # (direction, duration)
            movePanSpeedRatioCb=self.movePanSpeedRatio,     # (direction, speed_ratio, duration)
            moveTiltSpeedRatioCb=self.moveTiltSpeedRatio,
            # --- soft limits
            getSoftLimitsCb=self.getSoftLimits,
            setSoftLimitsCb=self.setSoftLimits,
            # --- speed control (whole unit, in degrees per second)
            getSpeedMaxCb=self.getSpeedMax,
            setSpeedMaxCb=self.setSpeedMax,
            getSpeedRatioCb=self.getSpeedRatio,
            setSpeedRatioCb=self.setSpeedRatio,
            # --- per-axis speed. All FOUR are needed for the capability flag.
            getPanSpeedRatioCb=self.getPanSpeedRatio,
            setPanSpeedRatioCb=self.setPanSpeedRatio,
            getTiltSpeedRatioCb=self.getTiltSpeedRatio,
            setTiltSpeedRatioCb=self.setTiltSpeedRatio,
            # --- absolute positioning + feedback
            getPositionCb=self.getPosition,                 # -> [pan_deg, tilt_deg]
            getPositionTimesCb=self.getPositionTimes,       # -> [pan_time, tilt_time]
            gotoPositionCb=self.gotoPosition,               # (pan_deg, tilt_deg)
            gotoPanPositionCb=self.gotoPanPosition,
            gotoTiltPositionCb=self.gotoTiltPosition,
            # --- homing
            goHomeCb=self.goHome,
            setHomePositionCb=self.setHomePosition,
            setHomePositionHereCb=self.setHomePositionHere,
            # --- orientation reporting (optional)
            getNavPoseCb=self.getNavPoseDict,
            navpose_update_rate=self.MAX_POSITION_UPDATE_RATE,
            deviceResetCb=self.resetDevice,
            calibrateCenterCB=None,
        )

        # Poll position at MAX_POSITION_UPDATE_RATE via a self-rescheduling timer.
        nepi_sdk.start_timer_process(1.0 / self.MAX_POSITION_UPDATE_RATE,
                                     self.updatePositionHandler, oneshot=True)
        nepi_sdk.on_shutdown(self.cleanup_actions)
        self.msg_if.pub_info("Initialization complete")
        nepi_sdk.spin()

    #########################################################################
    # SETTINGS PLUMBING
    #
    # CAP_SETTINGS' numeric "options" pair becomes 'bounds' in the controls
    # dict: the retired cap-settings form carried an Int/Float control's min
    # and max in an 'options' pair, and 'options' on a numeric control is
    # ignored now.
    #########################################################################
    def initSettingsDict(self):
        init_settings_dict = dict()
        for name, cap_setting in self.CAP_SETTINGS.items():
            setting_type = cap_setting['type']
            setting_dict = {'type': setting_type}
            if 'options' in cap_setting:
                try:
                    if setting_type == 'Int':
                        setting_dict['bounds'] = [int(cap_setting['options'][0]),
                                                  int(cap_setting['options'][1])]
                    elif setting_type == 'Float':
                        setting_dict['bounds'] = [float(cap_setting['options'][0]),
                                                  float(cap_setting['options'][1])]
                    else:
                        setting_dict['options'] = [str(o) for o in cap_setting['options']]
                except Exception as e:
                    self.msg_if.pub_warn("Invalid bounds for setting: " + name + " : " + str(e))

            default = None
            if name in self.FACTORY_SETTINGS:
                default = self.FACTORY_SETTINGS[name].get('value')
            if name in self.FACTORY_SETTINGS_OVERRIDES:
                default = self.FACTORY_SETTINGS_OVERRIDES[name]
            if default is None:
                continue
            try:
                if setting_type == 'Int':
                    default = int(float(default))
                elif setting_type == 'Float':
                    default = float(default)
                elif setting_type == 'Toggle':
                    default = (str(default) in ('True', 'true'))
                else:
                    default = str(default)
            except Exception as e:
                self.msg_if.pub_warn("Invalid factory value for setting: " + name + " : " + str(e))
                continue
            setting_dict['default'] = default
            init_settings_dict[name] = setting_dict

        self.init_settings_dict = init_settings_dict
        settings_dict = nepi_controls.create_controls_dict(init_settings_dict)
        self.msg_if.pub_info("Initialized Settings: " +
                             str(nepi_controls.get_values_dict(settings_dict)))
        return settings_dict

    def refreshSettingsDict(self):
        settings_dict = copy.deepcopy(self.settings_dict)
        for name in settings_dict.keys():
            if name not in self.settingFunctions:
                continue
            try:
                val = getattr(self, self.settingFunctions[name]['get'])()
            except Exception as e:
                self.msg_if.pub_warn("Failed to read setting " + name + " : " + str(e))
                continue
            if val is None:
                continue
            try:
                settings_dict = nepi_controls.set_value(settings_dict, name, val)
            except Exception as e:
                self.msg_if.pub_warn("Failed to apply read setting " + name + " : " + str(e))
        return settings_dict

    def getSettingsFunction(self):
        return self.settings_dict

    def setSettingFunction(self, setting_name, setting_value):
        # SettingsIF replaces its own dict with the THIRD element, so all three
        # must always be returned.
        setting_str = setting_name + ":" + str(setting_value)
        success = False
        if setting_name not in self.settings_dict.keys():
            return False, (self.node_name + " Setting name " + setting_str +
                           " is not supported"), self.settings_dict
        if setting_name not in self.settingFunctions:
            return False, (self.node_name + " Setting name " + setting_str +
                           " has no set function"), self.settings_dict
        try:
            success = (getattr(self, self.settingFunctions[setting_name]['set'])(setting_value) is True)
            msg = (self.node_name + " UPDATED SETTINGS " + setting_str) if success else \
                  (self.node_name + " device rejected setting " + setting_str)
        except Exception as e:
            msg = "Failed to set " + setting_str + " : " + str(e)
            self.msg_if.pub_warn(msg)
        self.settings_dict = self.refreshSettingsDict()
        return success, msg, self.settings_dict

    def getStatusRate(self):
        return self.status_rate_hz

    def setStatusRate(self, val):
        self.status_rate_hz = int(val)
        return True

    #########################################################################
    # PTX CAPABILITY CALLBACKS  (TODO: implement your protocol in each)
    #########################################################################
    def stopMoving(self):
        self.send_msg('#' + self.addr_str + 'STOP')

    def movePan(self, direction, duration):
        # direction: +1 / -1 ; duration: seconds (0 => continuous until stop)
        self.send_msg('#' + self.addr_str + 'PAN' + ('+' if direction >= 0 else '-'))
        if duration > 0:
            nepi_sdk.sleep(duration)
            self.stopMoving()

    def moveTilt(self, direction, duration):
        self.send_msg('#' + self.addr_str + 'TLT' + ('+' if direction >= 0 else '-'))
        if duration > 0:
            nepi_sdk.sleep(duration)
            self.stopMoving()

    def movePanSpeedRatio(self, direction, speed_ratio, duration):
        # Jog at an explicit speed. speed_ratio is 0.0-1.0, NOT degrees per
        # second -- scale it by the axis max before it reaches the hardware.
        speed_dps = float(speed_ratio) * self.speed_max_dps
        self.send_msg('#' + self.addr_str + 'PANS%+d,%.2f' %
                      (1 if direction >= 0 else -1, speed_dps))
        if duration > 0:
            nepi_sdk.sleep(duration)
            self.stopMoving()

    def moveTiltSpeedRatio(self, direction, speed_ratio, duration):
        speed_dps = float(speed_ratio) * self.speed_max_dps
        self.send_msg('#' + self.addr_str + 'TLTS%+d,%.2f' %
                      (1 if direction >= 0 else -1, speed_dps))
        if duration > 0:
            nepi_sdk.sleep(duration)
            self.stopMoving()

    def getSoftLimits(self):
        return self.soft_limits              # [min_pan, max_pan, min_tilt, max_tilt]

    def setSoftLimits(self, min_pan, max_pan, min_tilt, max_tilt):
        self.soft_limits = [min_pan, max_pan, min_tilt, max_tilt]

    def getSpeedMax(self):
        # Degrees per second. PTXActuatorIF calls this during construction, so
        # it must return a real number immediately and must never return None.
        return self.speed_max_dps

    def setSpeedMax(self, speed_max_dps):
        # Report an out-of-range value rather than silently dropping it -- a
        # guard that is too narrow caps the axis at a fraction of the hardware.
        if speed_max_dps < self.MIN_SPEED_MAX_DPS or speed_max_dps > self.MAX_SPEED_MAX_DPS:
            self.msg_if.pub_warn("Max speed " + str(speed_max_dps) + " dps outside " +
                                 str(self.MIN_SPEED_MAX_DPS) + "-" +
                                 str(self.MAX_SPEED_MAX_DPS) + " dps... ignoring")
            return
        self.speed_max_dps = float(speed_max_dps)

    def getSpeedRatio(self):
        return self.speed_ratio

    def setSpeedRatio(self, ratio):
        self.speed_ratio = max(0.0, min(1.0, float(ratio)))
        self.setPanSpeedRatio(self.speed_ratio)
        self.setTiltSpeedRatio(self.speed_ratio)

    def getPanSpeedRatio(self):
        # Never return None: PTXActuatorIF applies math.floor() to this value
        # directly. Fall back to the last commanded ratio when the hardware
        # cannot be read.
        return self.pan_speed_ratio

    def setPanSpeedRatio(self, ratio):
        # A 0.0-1.0 ratio, NOT degrees per second.
        self.pan_speed_ratio = max(0.0, min(1.0, float(ratio)))
        # TODO: push the scaled speed to the pan axis.

    def getTiltSpeedRatio(self):
        return self.tilt_speed_ratio

    def setTiltSpeedRatio(self, ratio):
        self.tilt_speed_ratio = max(0.0, min(1.0, float(ratio)))
        # TODO: push the scaled speed to the tilt axis.

    def getPosition(self):
        return self.current_position         # [pan_deg, tilt_deg]

    def getPositionTimes(self):
        return self.position_times

    def gotoPosition(self, pan_deg, tilt_deg):
        self.send_msg('#' + self.addr_str + 'GOTO,%.2f,%.2f' % (pan_deg, tilt_deg))

    def gotoPanPosition(self, pan_deg):
        self.send_msg('#' + self.addr_str + 'GOTOP,%.2f' % pan_deg)

    def gotoTiltPosition(self, tilt_deg):
        self.send_msg('#' + self.addr_str + 'GOTOT,%.2f' % tilt_deg)

    def goHome(self):
        self.gotoPosition(self.home_pan_deg, self.home_tilt_deg)

    def setHomePosition(self, pan_deg, tilt_deg):
        self.home_pan_deg = pan_deg
        self.home_tilt_deg = tilt_deg

    def setHomePositionHere(self):
        self.home_pan_deg, self.home_tilt_deg = self.getPosition()

    def resetDevice(self):
        self.send_msg('#' + self.addr_str + 'RESET')

    def getNavPoseDict(self):
        # PTX navpose reports ORIENTATION only (degrees, ENU). Fill from feedback.
        #
        # deepcopy is not optional: BLANK_NAVPOSE_DICT is a module-level dict,
        # and mutating it in place corrupts it for every other consumer in the
        # process. The key is 'time_orientation' -- a shipped driver still
        # carries a 'time_oreantation' typo, which writes a key nothing reads.
        navpose = copy.deepcopy(nepi_nav.BLANK_NAVPOSE_DICT)
        navpose['has_orientation'] = True
        navpose['time_orientation'] = nepi_utils.get_time()
        navpose['roll_deg'] = 0.0
        navpose['pitch_deg'] = self.current_position[1]   # tilt
        navpose['yaw_deg'] = self.current_position[0]     # pan
        return navpose

    #########################################################################
    # RAW SERIAL I/O + position poll
    #########################################################################
    def connect(self):
        try:
            self.serial_port = serial.Serial(self.port_str, self.baud_int, timeout=1)
        except Exception as e:
            self.msg_if.pub_warn("Serial open failed: " + str(e))
            return False
        response = self.send_msg('#' + self.addr_str + 'INFO')
        if response is None:
            return False
        # TODO: parse identity fields from your INFO reply.
        return True

    def send_msg(self, ser_msg):
        if self.serial_port is None:
            return None
        while self.serial_busy:
            nepi_sdk.sleep(0.01, 2)
        self.serial_busy = True
        response = None
        try:
            self.serial_port.reset_input_buffer()
            self.serial_port.write(bytearray(ser_msg + '\r', 'utf-8'))
            time.sleep(0.01)
            response = self.serial_port.readline().decode(errors='ignore').strip()
        except Exception as e:
            self.msg_if.pub_warn("Serial comm error: " + str(e))
        finally:
            self.serial_busy = False
        return response

    def updatePositionHandler(self, timer):
        # TODO: query live pan/tilt angles and cache them for getPosition().
        response = self.send_msg('#' + self.addr_str + 'POS?')
        if response is not None:
            try:
                parts = response.replace('#' + self.addr_str, '').split(',')
                self.current_position = [float(parts[-2]), float(parts[-1])]
                t = nepi_utils.get_time()
                self.position_times = [t, t]
            except Exception:
                pass
        # reschedule
        nepi_sdk.start_timer_process(1.0 / self.MAX_POSITION_UPDATE_RATE,
                                     self.updatePositionHandler, oneshot=True)

    def cleanup_actions(self):
        try:
            if self.serial_port is not None:
                self.serial_port.close()
        except Exception:
            pass


if __name__ == '__main__':
    node = PtxTemplateNode()
