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

# SVX (servo) node for one servo channel of a PJRC Teensy 4.1 running the
# teensy_thrusters firmware (protocol 1.1 or later). The Pi-to-Teensy protocol
# is mate_robotics Firmware/teensy_thrusters/PROTOCOL.md; section numbers below
# refer to it.
#
# One SVX device = one servo = one Teensy servo channel, launched by
# svx_teensy_thrusters_discovery.py when the driver's devices option includes
# servos. SVXActuatorIF always lives at <node>/svx, so a node can host only one
# servo; that is why each servo is its own node, as on the Maestro. The board's
# thruster node and its other servo nodes share the one serial port: every
# transaction holds an advisory flock on the port and only accepts the reply
# that names the command it sent (PROTOCOL.md 2.1).
#
# Structure mirrors svx_servo_maestro_node.py: per-servo calibration lives in
# node settings (pulse endpoints and degree range -- they describe the servo,
# not the board), the SVX IF owns the reverse axis and the soft-limit clamp,
# and this node keeps one pure linear degree <-> pulse-width map. The servo's
# calibrated endpoints are also pushed to the firmware as its own range clamp
# (SL), like the Maestro driver pushes its onboard Min/Max.
#
# SPEED: the Teensy has no slew control, so this node slews. A move is sent as
# a series of SS frames, one every SLEW_INTERVAL_SEC, each at most
# speed_ratio x speed_max_dps further along. The servo's first move after the
# node starts (or the board reboots) cannot be slewed, because a limp servo's
# position is unknown: it goes straight to the target at the servo's own speed.
#
# SAFETY:
#   * Servos are not on the thruster watchdog and hold their last pulse
#     (PROTOCOL.md 4.1). The firmware rejects any pulse outside the range this
#     node pushed with SL; nothing is clamped into motion.
#   * A calibration change never moves the servo: the pulse being output is
#     kept, and only the degree it is reported as changes.
#   * On shutdown the servo holds, or goes limp if servo_on_host_loss is limp
#     -- the same choice the firmware applies when every node is gone.
#
# NO FEEDBACK: the servo reports nothing back, so getPosition is the position
# being commanded (mid-slew while slewing), not a measurement.

import copy
import functools
import os
import threading

import serial

try:
    import fcntl  # Linux advisory locking so the board's nodes can share one port
    HAVE_FCNTL = True
except Exception:
    HAVE_FCNTL = False

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_controls

from nepi_api.device_if_svx import SVXActuatorIF
from nepi_api.messages_if import MsgIF

PKG_NAME = 'SVX_TEENSY_THRUSTERS' # Use in display menus
FILE_TYPE = 'NODE'


class SvxTeensyThrustersServoNode:

    # Must match the firmware's ID reply (PROTOCOL.md 6.1 and 11).
    FIRMWARE_NAME = 'teensy_thrusters'
    PROTOCOL_MAJOR = 1
    SERVO_PROTOCOL_MINOR = 1

    # Servo pins in servo order (PROTOCOL.md 4); for the device info only.
    SERVO_PINS = [10, 11, 12, 14]

    # Any rate but 134, which reboots a running Teensy into its bootloader.
    SERIAL_BAUD = 115200
    REPLY_TIMEOUT_SEC = 0.1
    # This many transactions in a row with no reply is treated as a lost link.
    MAX_MISSED_REPLIES = 10
    # SV poll interval: catches a Teensy reboot (scfg 0).
    STATUS_INTERVAL_SEC = 0.5
    # Slew step period.
    SLEW_INTERVAL_SEC = 0.05

    #############################
    # Per-servo calibration (node settings, persisted per device).
    #
    # The defaults are deliberately the narrow 1000-2000 us band, which every
    # hobby servo accepts, so an uncalibrated servo is never driven into its end
    # stops. Widen them to the servo's real range when calibrating; the Axon MAX
    # MK2 and the DS3235 both take 500-2500 us.
    DEFAULT_PULSE_MIN_US = 1000.0
    DEFAULT_PULSE_MAX_US = 2000.0
    DEFAULT_MIN_DEG = -45.0
    DEFAULT_MAX_DEG = 45.0

    # Outer bounds the settings accept: the firmware's servo hard limits
    # (PROTOCOL.md 5.1).
    PULSE_US_MIN = 500.0
    PULSE_US_MAX = 2500.0
    DEG_MIN = -180.0
    DEG_MAX = 180.0
    # Smallest degree span a servo may be calibrated to (the SVX IF enforces a
    # stricter span of its own).
    MIN_DEG_SPAN = 1.0

    DEFAULT_SPEED_RATIO = 0.5
    DEFAULT_SPEED_MAX_DPS = 180.0

    FACTORY_LIMITS_DICT = dict()
    FACTORY_LIMITS_DICT['min_hardstop_deg'] = DEFAULT_MIN_DEG
    FACTORY_LIMITS_DICT['max_hardstop_deg'] = DEFAULT_MAX_DEG
    FACTORY_LIMITS_DICT['min_softstop_deg'] = DEFAULT_MIN_DEG
    FACTORY_LIMITS_DICT['max_softstop_deg'] = DEFAULT_MAX_DEG

    CAP_SETTINGS = {
        'pulse_min_us' : {"type":"Float", "bounds":[PULSE_US_MIN, PULSE_US_MAX]},
        'pulse_max_us' : {"type":"Float", "bounds":[PULSE_US_MIN, PULSE_US_MAX]},
        'min_deg' :      {"type":"Float", "bounds":[DEG_MIN, DEG_MAX]},
        'max_deg' :      {"type":"Float", "bounds":[DEG_MIN, DEG_MAX]}
    }

    settingFunctions = {
        'pulse_min_us' : {'get':'getPulseMinUs', 'set': 'setPulseMinUs'},
        'pulse_max_us' : {'get':'getPulseMaxUs', 'set': 'setPulseMaxUs'},
        'min_deg' :      {'get':'getMinDeg',     'set': 'setMinDeg'},
        'max_deg' :      {'get':'getMaxDeg',     'set': 'setMaxDeg'}
    }

    device_info_dict = dict(device_name = "",
                            path = "",
                            serial_number = "",
                            hw_version = "",
                            sw_version = "")

    serial_num = "Unknown"
    hw_version = "Unknown"
    sw_version = "Unknown"
    svx_if = None

    # Board link
    serial_port = None
    connected = False
    missed_replies = 0
    last_status_time = 0.0

    drv_dict = dict()


    ################################################
    DEFAULT_NODE_NAME = PKG_NAME.lower() + "_servo_node"

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

        # serial_lock: in-process guard around each transaction (paired with the
        # cross-process flock inside transact). motion_lock: the goal/current
        # position state, touched by the SVX IF callbacks and the slew timer.
        self.serial_lock = threading.Lock()
        self.motion_lock = threading.Lock()

        ##############################
        # Get required drv driver dict info
        self.drv_dict = nepi_sdk.get_param('~drv_dict',dict())
        try:
            device_dict = self.drv_dict['DEVICE_DICT']
            self.device_name = device_dict['device_name']
            self.device_path = device_dict['device_path']
            self.port_str = device_dict['device_path']
            self.serial_num = str(device_dict.get('serial_number', 'Unknown'))
            self.servo_index = int(device_dict['servo_index'])
            servo_config = device_dict['servo_config']
            self.servo_rate_hz = int(servo_config['servo_rate_hz'])
            self.hold_on_host_loss = (str(servo_config['servo_on_host_loss']).lower() != 'limp')
        except Exception as e:
            self.msg_if.pub_warn("Failed to load Device Dict " + str(e))
            nepi_sdk.signal_shutdown(self.node_name + ": Shutting down because no valid Device Dict")
            return

        # Per-instance copies, so the settings write to this servo's own limits
        # and never to the class-level dict.
        self.FACTORY_LIMITS_DICT = copy.deepcopy(SvxTeensyThrustersServoNode.FACTORY_LIMITS_DICT)
        self.limits_dict = copy.deepcopy(self.FACTORY_LIMITS_DICT)
        self.pulse_min_us = self.DEFAULT_PULSE_MIN_US
        self.pulse_max_us = self.DEFAULT_PULSE_MAX_US
        self.speed_ratio = self.DEFAULT_SPEED_RATIO
        self.speed_max_dps = self.DEFAULT_SPEED_MAX_DPS
        self.home_pos_deg = 0.0

        # Motion state. out_us is the pulse the Teensy is outputting for this
        # servo (0 = off/limp, position unknown); current_deg is that pulse in
        # degrees; goal_deg is where a slew is heading.
        self.out_us = 0
        self.current_deg = 0.0
        self.goal_deg = 0.0

        ################################################
        self.msg_if.pub_info("Connecting to Teensy " + self.serial_num + " servo " +
                             str(self.servo_index) + " on " + str(self.port_str))
        self.connected = self.connect()
        if self.connected == False:
            self.msg_if.pub_info("Shutting down node, unable to connect to the teensy_thrusters firmware")
            nepi_sdk.signal_shutdown("Unable to connect to servo controller")
            return
        self.msg_if.pub_info("... Connected!")

        # Initialize settings. SettingsIF re-applies persisted values through
        # setSettingFunction, which re-pushes SL with the calibrated range.
        self.settings_dict = self.initSettingsDict()
        self.settings_dict = self.refreshSettingsDict()

        self.device_info_dict["device_name"] = self.device_name
        self.device_info_dict["path"] = self.device_path
        self.device_info_dict["serial_number"] = self.serial_num
        self.device_info_dict["hw_version"] = self.hw_version
        self.device_info_dict["sw_version"] = self.sw_version

        #Factory Control Values
        self.FACTORY_CONTROLS = {
            'reverse_enabled' : False,
            'speed_ratio' : self.DEFAULT_SPEED_RATIO
        }

        self.msg_if.pub_info("Launching NEPI SVX interface...")
        self.svx_if = SVXActuatorIF(device_info = self.device_info_dict,
                                    getSettingsFunction = self.getSettingsFunction,
                                    setSettingFunction = self.setSettingFunction,
                                    factoryControls = self.FACTORY_CONTROLS,
                                    factoryLimits = self.FACTORY_LIMITS_DICT,
                                    stopMovingCb = self.stopMoving,
                                    gotoPositionCb = self.gotoPosition,
                                    getPositionCb = self.getPosition,
                                    setSoftLimitsCb = self.setSoftLimits,
                                    getSoftLimitsCb = self.getSoftLimits,
                                    setSpeedRatioCb = self.setSpeedRatio,
                                    getSpeedRatioCb = self.getSpeedRatio,
                                    setSpeedMaxCb = self.setSpeedMax,
                                    getSpeedMaxCb = self.getSpeedMax,
                                    goHomeCb = self.goHome,
                                    setHomePositionCb = self.setHomePosition,
                                    setHomePositionHereCb = self.setHomePositionHere,
                                    deviceResetCb = self.resetDevice,
                                    msg_if = self.msg_if
                                    )
        self.msg_if.pub_info(" ... SVX interface running")

        # Persisted min_deg/max_deg may have been applied while svx_if was still
        # None, so re-push the calibrated range now that the IF exists (same
        # reason as svx_servo_maestro_node.py).
        self.svx_if.setHardstopLimits(self.limits_dict['min_hardstop_deg'],
                                      self.limits_dict['max_hardstop_deg'])

        nepi_sdk.start_timer_process(self.SLEW_INTERVAL_SEC, self.slewCb)

        # Initialization Complete
        self.msg_if.pub_info("Initialization Complete")
        #Set up node shutdown
        nepi_sdk.on_shutdown(self.cleanup_actions)
        # Spin forever
        nepi_sdk.spin()


    #**********************
    # Device setting functions

    def initSettingsDict(self):
        init_settings_dict = dict()
        for setting_name in self.CAP_SETTINGS.keys():
            setting_dict = copy.deepcopy(self.CAP_SETTINGS[setting_name])
            default = self.readSettingValue(setting_name)
            if default is None:
                continue
            setting_dict['default'] = default
            init_settings_dict[setting_name] = setting_dict
        settings_dict = nepi_controls.create_controls_dict(init_settings_dict)
        self.msg_if.pub_info("Initialized Settings: " + str(nepi_controls.get_values_dict(settings_dict)))
        return settings_dict

    def refreshSettingsDict(self):
        # Per-servo calibration held by this node, so this only reads the
        # current values back. Bounds do not move.
        settings_dict = copy.deepcopy(self.settings_dict)
        for setting_name in settings_dict.keys():
            value = self.readSettingValue(setting_name)
            if value is None:
                continue
            settings_dict = nepi_controls.set_value(settings_dict, setting_name, value)
        return settings_dict

    def readSettingValue(self, setting_name):
        if setting_name not in self.settingFunctions.keys():
            return None
        get_function = getattr(self, self.settingFunctions[setting_name]['get'], None)
        if get_function is None:
            return None
        try:
            return float(get_function())
        except Exception as e:
            self.msg_if.pub_warn("Failed to read setting " + setting_name + " : " + str(e))
            return None

    def getSettingsFunction(self):
        return self.settings_dict

    def setSettingFunction(self, setting_name, setting_value):
        setting_str = setting_name + ":" + str(setting_value)
        success = False
        msg = ""
        if setting_name not in self.settingFunctions.keys():
            msg = (self.node_name + " Setting name " + setting_str + " is not supported")
            return False, msg, self.settings_dict
        set_function = getattr(self, self.settingFunctions[setting_name]['set'], None)
        if set_function is None:
            msg = "Missing set function for setting: " + setting_name
            self.msg_if.pub_warn(msg)
        else:
            # Every setter returns [success, msg].
            [success, msg] = set_function(setting_value)
            if success:
                msg = (self.node_name + " UPDATED SETTINGS " + setting_str)
        self.settings_dict = self.refreshSettingsDict()
        return success, msg, self.settings_dict


    ##############
    ### Settings Functions
    #
    # The pulse endpoints and the degree range are set independently by the UI
    # but only make sense as a pair, so each setter routes through a shared
    # range function that validates the resulting pair as a whole.

    def getPulseMinUs(self):
        return self.pulse_min_us

    def setPulseMinUs(self, val):
        return self.setPulseRange(new_min = val)

    def getPulseMaxUs(self):
        return self.pulse_max_us

    def setPulseMaxUs(self, val):
        return self.setPulseRange(new_max = val)

    def setPulseRange(self, new_min = None, new_max = None):
        try:
            pulse_min = self.pulse_min_us if new_min is None else float(new_min)
            pulse_max = self.pulse_max_us if new_max is None else float(new_max)
        except Exception as e:
            return False, "Pulse width is not a number: " + str(e)
        for val in [pulse_min, pulse_max]:
            if val < self.PULSE_US_MIN or val > self.PULSE_US_MAX:
                return False, ("Pulse width " + str(val) + "us is outside the supported range " +
                               str(self.PULSE_US_MIN) + "-" + str(self.PULSE_US_MAX) + "us")
        if pulse_max <= pulse_min:
            return False, ("Rejecting pulse range " + str(pulse_min) + "-" + str(pulse_max) +
                           "us: max must be greater than min")
        # The firmware gets the same range as its clamp. Its integer limits are
        # rounded inward, so a commanded pulse can never round outside them.
        reply = self.transact('SL,%d,%d,%d' % (self.servo_index, self.firmwareMinUs(pulse_min),
                                               self.firmwareMaxUs(pulse_max)))
        if reply != ['OK', 'SL']:
            return False, "Teensy rejected pulse range " + str(pulse_min) + "-" + str(pulse_max) + "us: " + str(reply)
        with self.motion_lock:
            self.pulse_min_us = pulse_min
            self.pulse_max_us = pulse_max
            self.rebaseOnCurrentPulse()
        return True, ("Pulse range set to " + str(pulse_min) + "-" + str(pulse_max) + "us")

    def getMinDeg(self):
        return self.limits_dict['min_hardstop_deg']

    def setMinDeg(self, val):
        return self.setDegRange(new_min = val)

    def getMaxDeg(self):
        return self.limits_dict['max_hardstop_deg']

    def setMaxDeg(self, val):
        return self.setDegRange(new_max = val)

    def setDegRange(self, new_min = None, new_max = None):
        # The servo's mechanical travel. Open loop, so the operator finds the
        # hardstops and declares them here.
        try:
            min_deg = self.limits_dict['min_hardstop_deg'] if new_min is None else float(new_min)
            max_deg = self.limits_dict['max_hardstop_deg'] if new_max is None else float(new_max)
        except Exception as e:
            return False, "Degree limit is not a number: " + str(e)
        for val in [min_deg, max_deg]:
            if val < self.DEG_MIN or val > self.DEG_MAX:
                return False, ("Degree limit " + str(val) + " is outside the supported range " +
                               str(self.DEG_MIN) + " to " + str(self.DEG_MAX))
        if (max_deg - min_deg) < self.MIN_DEG_SPAN:
            return False, ("Rejecting degree range " + str(min_deg) + " to " + str(max_deg) +
                           ": span must be at least " + str(self.MIN_DEG_SPAN) + " deg")
        # The IF first, so a range it rejects (it enforces its own minimum span)
        # never reaches this node's map. Guarded because SettingsIF can apply a
        # persisted value before the IF exists.
        if self.svx_if is not None:
            if self.svx_if.setHardstopLimits(min_deg, max_deg) == False:
                return False, ("SVX interface rejected degree range " + str(min_deg) + " to " + str(max_deg))
        with self.motion_lock:
            for limits in [self.FACTORY_LIMITS_DICT, self.limits_dict]:
                limits['min_hardstop_deg'] = min_deg
                limits['max_hardstop_deg'] = max_deg
            self.FACTORY_LIMITS_DICT['min_softstop_deg'] = min_deg
            self.FACTORY_LIMITS_DICT['max_softstop_deg'] = max_deg
            self.rebaseOnCurrentPulse()
        return True, ("Degree range set to " + str(min_deg) + " to " + str(max_deg))

    def rebaseOnCurrentPulse(self):
        # SAFETY: a calibration change must not move the servo. Keep the pulse
        # being output and re-express it (and the goal) in the new degrees.
        # Called with motion_lock held.
        if self.out_us != 0:
            self.current_deg = self.us2deg(self.out_us)
        self.goal_deg = self.current_deg


    #######################
    ### SVX IF Callbacks
    #
    # The SVX IF clamps a commanded position to the hardstops and applies the
    # reverse-axis conversion before calling gotoPosition, so this node receives
    # a safe degree value in its own (non-reversed) frame.

    def stopMoving(self):
        # Positional servo: "stop" == hold where it is now, ending any slew.
        with self.motion_lock:
            self.goal_deg = self.current_deg

    def gotoPosition(self, position_deg):
        with self.motion_lock:
            self.goal_deg = float(position_deg)
            jump = (self.out_us == 0)
        if jump:
            # A limp servo's position is unknown, so its first move cannot be
            # slewed: send the target at once (the servo moves at its own speed).
            self.sendPosition(self.goal_deg)

    def getPosition(self):
        return self.current_deg

    def setSoftLimits(self, min_deg, max_deg):
        self.limits_dict['min_softstop_deg'] = min_deg
        self.limits_dict['max_softstop_deg'] = max_deg

    def getSoftLimits(self):
        return self.limits_dict['min_softstop_deg'], self.limits_dict['max_softstop_deg']

    def setSpeedRatio(self, ratio):
        self.speed_ratio = max(0.0, min(1.0, float(ratio)))

    def getSpeedRatio(self):
        return self.speed_ratio

    def setSpeedMax(self, speed_max_dps):
        self.speed_max_dps = max(0.0, float(speed_max_dps))

    def getSpeedMax(self):
        return self.speed_max_dps

    def goHome(self):
        self.gotoPosition(self.home_pos_deg)

    def setHomePosition(self, position_deg):
        self.home_pos_deg = position_deg

    def setHomePositionHere(self):
        self.home_pos_deg = self.current_deg

    def resetDevice(self):
        # Restore this node's factory motion config and send the servo home.
        self.speed_ratio = self.DEFAULT_SPEED_RATIO
        self.speed_max_dps = self.DEFAULT_SPEED_MAX_DPS
        self.limits_dict = copy.deepcopy(self.FACTORY_LIMITS_DICT)
        self.goHome()


    #######################
    ### Unit Conversions (degrees <-> microseconds)
    #
    # Pure linear map; reverse-axis handling lives in the SVX IF, not here.

    def deg2us(self, deg):
        min_deg = self.FACTORY_LIMITS_DICT['min_hardstop_deg']
        max_deg = self.FACTORY_LIMITS_DICT['max_hardstop_deg']
        span_deg = (max_deg - min_deg)
        if span_deg == 0:
            return self.pulse_min_us
        frac = (deg - min_deg) / span_deg
        us = self.pulse_min_us + frac * (self.pulse_max_us - self.pulse_min_us)
        return max(self.pulse_min_us, min(self.pulse_max_us, us))

    def us2deg(self, us):
        min_deg = self.FACTORY_LIMITS_DICT['min_hardstop_deg']
        max_deg = self.FACTORY_LIMITS_DICT['max_hardstop_deg']
        span_us = (self.pulse_max_us - self.pulse_min_us)
        if span_us == 0:
            return min_deg
        frac = (us - self.pulse_min_us) / span_us
        return min_deg + frac * (max_deg - min_deg)

    def firmwareMinUs(self, pulse_min_us):
        return int(-(-pulse_min_us // 1))   # ceil: never below the calibrated min

    def firmwareMaxUs(self, pulse_max_us):
        return int(pulse_max_us // 1)       # floor: never above the calibrated max

    def pulseFor(self, deg):
        # The integer pulse for a degree, inside the firmware's clamp, which is
        # the calibrated range rounded inward.
        us = int(round(self.deg2us(deg)))
        return max(self.firmwareMinUs(self.pulse_min_us), min(self.firmwareMaxUs(self.pulse_max_us), us))


    #######################
    ### Slew and status

    def slewCb(self, timer):
        if self.connected == False or nepi_sdk.is_shutdown():
            return
        now = nepi_utils.get_time()
        step_deg = None
        with self.motion_lock:
            if self.out_us != 0 and self.goal_deg != self.current_deg:
                max_step = self.speed_ratio * self.speed_max_dps * self.SLEW_INTERVAL_SEC
                delta = self.goal_deg - self.current_deg
                if max_step <= 0.0:
                    # Speed 0 holds the servo where it is.
                    self.goal_deg = self.current_deg
                elif abs(delta) <= max_step:
                    step_deg = self.goal_deg
                else:
                    step_deg = self.current_deg + (max_step if delta > 0 else -max_step)
        if step_deg is not None:
            self.sendPosition(step_deg)
        if (now - self.last_status_time) >= self.STATUS_INTERVAL_SEC:
            self.last_status_time = now
            self.pollStatus()

    def sendPosition(self, deg):
        pulse = self.pulseFor(deg)
        reply = self.transact('SS,%d,%d' % (self.servo_index, pulse))
        if reply == ['OK', 'SS']:
            with self.motion_lock:
                self.out_us = pulse
                self.current_deg = deg
            return True
        if reply is not None:
            self.msg_if.pub_warn("Teensy rejected servo " + str(self.servo_index) + " pulse " + str(pulse) +
                                 "us: " + ','.join(reply[1:]), throttle_s = 5.0)
        return False

    def pollStatus(self):
        status = self.parseServoStatus(self.transact('SV'))
        if status is None:
            return
        if status['scfg'] == 0:
            # scfg is 0 only before the first SC since boot, and this node sent
            # one at connect: the Teensy rebooted. Its servos are off and their
            # limits are back at the boot defaults.
            self.msg_if.pub_warn("Teensy " + self.serial_num + " rebooted; reconfiguring servo " +
                                 str(self.servo_index))
            with self.motion_lock:
                was_on = (self.out_us != 0)
                self.out_us = 0
                resume_deg = self.goal_deg
            if self.configureServo() and self.hold_on_host_loss and was_on:
                # hold: put a servo that was being driven back where it was
                # meant to be. A servo never commanded stays off.
                self.gotoPosition(resume_deg)
            return
        pulse = status['pulses_us'][self.servo_index]
        if pulse == 0 and self.out_us != 0:
            # Off without this node asking (another host sent SO or SL): the
            # servo is limp and its position unknown again.
            self.msg_if.pub_warn("Teensy reports servo " + str(self.servo_index) + " off", throttle_s = 10.0)
            with self.motion_lock:
                self.out_us = 0
                self.goal_deg = self.current_deg

    def parseServoStatus(self, fields):
        # $SV,<scfg>,<rate_hz>,<host_loss>,<p0>,<min0>,<max0>,...,<p3>,<min3>,<max3>
        if fields is None or fields[0] != 'SV' or len(fields) < 4 + 3 * (self.servo_index + 1):
            return None
        try:
            servo_count = (len(fields) - 4) // 3
            return dict(scfg = int(fields[1]),
                        rate_hz = int(fields[2]),
                        host_loss = fields[3],
                        pulses_us = [int(fields[4 + 3 * n]) for n in range(servo_count)])
        except Exception:
            return None


    #######################
    ### Connection

    def connect(self):
        try:
            # Not exclusive: the board's other nodes share this port, and each
            # transaction takes the port's flock instead (PROTOCOL.md 2.1).
            self.serial_port = openSharedPort(self.port_str, self.SERIAL_BAUD, self.REPLY_TIMEOUT_SEC)
        except Exception as e:
            self.msg_if.pub_warn("Failed to open serial port at: " + str(self.port_str) + " (" + str(e) + ")")
            self.serial_port = None
            return False

        identity = parseIdentify(self.transact('ID', connecting = True))
        if identity is None or identity['firmware_name'] != self.FIRMWARE_NAME:
            self.msg_if.pub_warn("No teensy_thrusters identify reply on " + str(self.port_str))
            self.closePort()
            return False
        if (identity['protocol_major'] != self.PROTOCOL_MAJOR or
                identity['protocol_minor'] < self.SERVO_PROTOCOL_MINOR):
            self.msg_if.pub_warn("Teensy speaks protocol " + identity['protocol_version'] +
                                 "; servos need " + str(self.PROTOCOL_MAJOR) + "." +
                                 str(self.SERVO_PROTOCOL_MINOR) + " or a later 1.x")
            self.closePort()
            return False
        if self.servo_index >= identity['servo_count']:
            self.msg_if.pub_warn("Teensy has " + str(identity['servo_count']) + " servo channels; no servo " +
                                 str(self.servo_index))
            self.closePort()
            return False

        pin = self.SERVO_PINS[self.servo_index] if self.servo_index < len(self.SERVO_PINS) else '?'
        self.hw_version = "Teensy 4.1 servo " + str(self.servo_index) + " (pin " + str(pin) + ")"
        self.sw_version = ("teensy_thrusters " + identity['firmware_version'] +
                           " (protocol " + identity['protocol_version'] + ")")
        self.msg_if.pub_info("Firmware " + self.sw_version + "; " + self.hw_version + "; " +
                             str(self.servo_rate_hz) + " Hz; on host loss " +
                             ("hold" if self.hold_on_host_loss else "limp"))
        return self.configureServo(connecting = True)

    def configureServo(self, connecting = False):
        # SC then SL (PROTOCOL.md 6.7 and 6.8). The servo stays off until the
        # first goto.
        sc_body = 'SC,%d,%s' % (self.servo_rate_hz, 'HOLD' if self.hold_on_host_loss else 'LIMP')
        reply = self.transact(sc_body, connecting = connecting)
        if reply != ['OK', 'SC']:
            self.msg_if.pub_warn("Teensy rejected servo configuration " + sc_body + ": " + str(reply))
            if connecting:
                self.closePort()
            return False
        sl_body = 'SL,%d,%d,%d' % (self.servo_index, self.firmwareMinUs(self.pulse_min_us),
                                   self.firmwareMaxUs(self.pulse_max_us))
        reply = self.transact(sl_body, connecting = connecting)
        if reply != ['OK', 'SL']:
            self.msg_if.pub_warn("Teensy rejected servo limits " + sl_body + ": " + str(reply))
            if connecting:
                self.closePort()
            return False
        return True

    def transact(self, body, connecting = False):
        # Send one frame and return the reply's fields, or None. Same contract
        # as the thruster node's transact: in-process lock plus the port's
        # flock, input flushed first, and only a reply naming this command is
        # accepted (PROTOCOL.md 2.1).
        reply = None
        link_error = None
        cmd = body[:2]
        with self.serial_lock:
            port = self.serial_port
            if port is None:
                return None
            locked = False
            try:
                if HAVE_FCNTL:
                    fcntl.flock(port.fileno(), fcntl.LOCK_EX)
                    locked = True
                port.reset_input_buffer()
                port.write(encodeFrame(body))
                port.flush()
                deadline = nepi_utils.get_time() + self.REPLY_TIMEOUT_SEC
                while reply is None and nepi_utils.get_time() < deadline:
                    line = port.readline()
                    if line is not None and len(line) > 0:
                        fields = decodeFrame(line)
                        if replyMatches(cmd, fields):
                            reply = fields
            except Exception as e:
                link_error = e
            finally:
                if locked:
                    try:
                        fcntl.flock(port.fileno(), fcntl.LOCK_UN)
                    except Exception:
                        pass

        if link_error is None and reply is not None:
            self.missed_replies = 0
            return reply
        if link_error is None:
            self.missed_replies += 1
            if connecting or self.missed_replies < self.MAX_MISSED_REPLIES:
                return None
            link_error = "no reply to " + str(self.missed_replies) + " frames in a row"
        if connecting:
            self.msg_if.pub_warn("Serial transaction failed: " + str(link_error))
            return None
        self.linkLost(link_error)
        return None

    def linkLost(self, reason):
        # Exit rather than retry in place: a re-plugged or rebooted Teensy can
        # come back on a different /dev path, and discovery relaunches this node
        # on whatever path it finds.
        if self.connected == False:
            return
        self.connected = False
        self.msg_if.pub_warn("Lost serial link to Teensy on " + str(self.port_str) + " (" + str(reason) +
                             "); shutting down so discovery can relaunch it")
        self.closePort()
        nepi_sdk.signal_shutdown("Lost serial link to servo controller")

    def closePort(self):
        with self.serial_lock:
            if self.serial_port is not None:
                try:
                    self.serial_port.close()
                except Exception:
                    pass
            self.serial_port = None


    #######################
    ### Cleanup processes on node shutdown
    def cleanup_actions(self):
        """Apply the servo_on_host_loss choice to this servo and release the port.

        With limp, sends servo-off so the servo stops resisting; with hold, leaves
        the servo at its last pulse, the same choice the firmware makes when every
        node has closed the port.
        """
        self.msg_if.pub_info("Shutting down: Executing script cleanup actions")
        self.connected = False
        if self.serial_port is not None and self.hold_on_host_loss == False:
            try:
                self.transact('SO,%d' % self.servo_index, connecting = True)
            except Exception:
                pass
        self.closePort()


##########################
# Protocol helpers (PROTOCOL.md 3, 2.1 and 6.1). Identical copies of the
# discovery's and the thruster node's -- drivers install flat into one folder
# and the discovery runs inside drivers_mgr, so the files share no module. Keep
# the copies in step.

def frameChecksum(body):
    return functools.reduce(lambda acc, c: acc ^ ord(c), body, 0)


def encodeFrame(body):
    return ('$%s*%02X\n' % (body, frameChecksum(body))).encode('ascii')


def decodeFrame(line):
    # One received line -> list of fields (fields[0] is the command), or None
    # if it is not a well-formed frame with a correct checksum.
    try:
        text = line.decode('ascii').strip()
    except Exception:
        return None
    start = text.rfind('$')
    star = text.rfind('*')
    if start < 0 or star < start or len(text) - star != 3:
        return None
    body = text[start + 1:star]
    try:
        if int(text[star + 1:], 16) != frameChecksum(body):
            return None
    except Exception:
        return None
    fields = body.split(',')
    if len(fields[0]) != 2:
        return None
    return fields


def openSharedPort(path, baud, timeout):
    # Open the board's port while holding its flock (PROTOCOL.md 2.1).
    # pySerial's open() flushes the tty input queue, which every process with
    # the port open shares, so an unlocked open can throw away the reply
    # another node is waiting for. The lock is taken on a short-lived second
    # descriptor because the Serial object's own descriptor does not exist
    # until open() is already running.
    lock_fd = None
    try:
        if HAVE_FCNTL:
            lock_fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
            fcntl.flock(lock_fd, fcntl.LOCK_EX)
        return serial.Serial(path, baud, timeout = timeout, write_timeout = 0.5)
    finally:
        if lock_fd is not None:
            os.close(lock_fd)   # closing the descriptor releases its flock


def replyMatches(cmd, fields):
    # True if fields is the reply to command cmd: $OK,<cmd>, $ER,<code>,<cmd>,
    # or a reply that carries the command's own name (ID, ST, SV).
    if fields is None:
        return False
    if fields[0] == 'OK':
        return len(fields) >= 2 and fields[1] == cmd
    if fields[0] == 'ER':
        return len(fields) >= 3 and fields[2] == cmd
    return fields[0] == cmd


def parseIdentify(fields):
    # $ID,<name>,<protocol>,<firmware>,<thrusters>,<servo_channel>,<serial>[,<servo_count>]
    if fields is None or len(fields) < 7 or fields[0] != 'ID':
        return None
    try:
        version = fields[2].split('.')
        return dict(firmware_name = fields[1],
                    protocol_version = fields[2],
                    protocol_major = int(version[0]),
                    protocol_minor = int(version[1]) if len(version) > 1 else 0,
                    firmware_version = fields[3],
                    thruster_count = int(fields[4]),
                    servo_channel = int(fields[5]),
                    serial_number = fields[6],
                    servo_count = int(fields[7]) if len(fields) > 7 else 0)
    except Exception:
        return None


if __name__ == '__main__':
    node = SvxTeensyThrustersServoNode()
