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
# SVX (Servo) DRIVER TEMPLATE -- NODE
# ---------------------------------------------------------------------------
# Registers an SVXActuatorIF. SVX models ONE SINGLE-AXIS actuator -- one servo,
# one node, one selectable NEPI device. That is the whole shape of the category:
# a pan/tilt turret built from servos is TWO SVX nodes (pan on one channel,
# tilt on another) coordinated above by a PTX driver or an app, not one SVX
# device with two axes. If your hardware has two coupled axes, you want
# ptx_template, not this one.
#
# A servo controller board typically drives several channels over ONE transport
# (one serial port, one bus address). So an SVX driver routinely has SEVERAL
# NODES SHARING ONE DEVICE PATH. Two consequences run through this template:
#   * discovery reconciles a desired set of (path, channel) pairs rather than
#     claiming a path once -- see svx_template_discovery.py;
#   * every transaction on the shared transport is serialized, with an
#     in-process lock AND a cross-process advisory file lock, so two channel
#     nodes never interleave their bytes on the wire.
#
# WHAT BELONGS WHERE. A board-level fact (which port, which protocol, which
# bus address) is a discovery OPTION. A per-actuator fact (travel range, pulse
# endpoints, acceleration) is a NODE SETTING: it describes the servo, not the
# controller, and two servos on one board routinely differ. Settings persist
# per device, so discovery must never overwrite an operator's calibration when
# it relaunches a channel.
#
# HOW THE IF DERIVES ITS CAPABILITY FLAGS -- pass a callback for every
# capability the hardware has and None for the rest; the IF advertises only
# what you wire up, so a None here is a real statement about the hardware.
#
# CALLBACK ARITIES (exactly how SVXActuatorIF calls them):
#   stopMovingCb()                       gotoPositionCb(position_deg)
#   getPositionCb() -> position_deg      setSoftLimitsCb(min_deg, max_deg)
#   getSoftLimitsCb() -> (min_deg, max_deg)
#   get/setSpeedRatioCb -- 0.0-1.0       get/setSpeedMaxCb -- degrees per second
#   get/setSpinDirection                 goHomeCb()
#   setHomePositionCb(position_deg)      setHomePositionHereCb()
#   deviceResetCb()
#
# factoryLimits needs four keys: min/max_hardstop_deg and min/max_softstop_deg.
# Note the IF keeps the softstops MIRRORED to the hardstops -- the hardstops
# are the single clamp range -- so do not design a separate soft band.
#
# The SVX API speaks DEGREES. Converting a degree command into whatever the
# hardware wants (a pulse width, a step count, a raw count) is this driver's
# job. The IF applies the reverse-axis conversion, the soft-limit clamp and the
# ratio math BEFORE calling these callbacks, so a callback receives a safe
# degree value in the device's own (non-reversed) frame. Keep the conversion
# here a single pure map with no reverse logic of its own.
#
# SETTINGS CONTRACT (identical in every NEPI driver -- idx_v4l2_node.py is the
# reference implementation):
#   initSettingsDict() / refreshSettingsDict() / getSettingsFunction() /
#   setSettingFunction(name, value) -> [success, msg, settings_dict]
# The retired capSettings= / factorySettings= / settingUpdateFunction=
# arguments no longer exist on ANY device IF -- passing them raises TypeError
# at construction.
##############################################################################

import copy
import threading

import serial                       # TODO: drop if your transport is not serial

try:
    import fcntl   # Linux advisory locking, so channel nodes can share one port
    HAVE_FCNTL = True
except Exception:
    HAVE_FCNTL = False

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_controls   # controls dict: create/get/set value, bounds, options

from nepi_api.messages_if import MsgIF
from nepi_api.device_if_svx import SVXActuatorIF

PKG_NAME = 'SVX_TEMPLATE'            # TODO: must match params.yaml 'pkg_name'
FILE_TYPE = 'NODE'


class SvxTemplateNode:

    SERIAL_TIMEOUT_SEC = 0.25
    MAX_POSITION_UPDATE_RATE = 5

    #########################################################################
    # PER-ACTUATOR CALIBRATION
    #
    # These describe THE SERVO, not the controller board, so they are node
    # settings (CAP_SETTINGS below) rather than discovery OPTIONS: adjustable
    # live per device in the RUI and persisted per device. The values here are
    # only the bootstrap defaults a servo starts from before anyone calibrates
    # it.
    #########################################################################
    DEFAULT_MIN_DEG = -90.0
    DEFAULT_MAX_DEG = 90.0
    DEFAULT_ACCEL_UNITS = 0

    # Outer bounds the settings will accept.
    DEG_MIN = -180.0
    DEG_MAX = 180.0
    ACCEL_UNITS_MIN = 0
    ACCEL_UNITS_MAX = 255

    # Smallest degree span a servo may be calibrated to. Below this the
    # degree <-> hardware-unit map stops meaning anything and the conversion
    # below would be dividing by ~zero.
    MIN_DEG_SPAN = 1.0

    # factoryLimits: SVXActuatorIF REQUIRES all four of these keys. Deep-copied
    # per instance in __init__ so the settings write to this node's own limits
    # rather than to the class-level dict every instance would share.
    FACTORY_LIMITS_DICT = dict(
        min_hardstop_deg=DEFAULT_MIN_DEG, max_hardstop_deg=DEFAULT_MAX_DEG,
        min_softstop_deg=DEFAULT_MIN_DEG, max_softstop_deg=DEFAULT_MAX_DEG,
    )

    # factoryControls: the IF merges a key only if it already exists in its own
    # FACTORY_CONTROLS_DICT (reverse_enabled / continuous_enabled / speed_ratio
    # / spin_direction). A key spelt any other way is silently ignored.
    FACTORY_CONTROLS = dict(
        reverse_enabled=False,
        speed_ratio=0.5,
    )

    # ---- Runtime-tunable settings (see lsx_template_node.py for the fully
    #      annotated pattern). 'type' must be a nepi_controls CONTROL_TYPE --
    #      'Discrete' is not one, and a setting declared that way is silently
    #      dropped and never appears in the RUI.
    CAP_SETTINGS = dict(
        min_deg={"type": "Float", "name": "min_deg",
                 "options": [str(DEG_MIN), str(DEG_MAX)]},
        max_deg={"type": "Float", "name": "max_deg",
                 "options": [str(DEG_MIN), str(DEG_MAX)]},
        accel_units={"type": "Int", "name": "accel_units",
                     "options": [str(ACCEL_UNITS_MIN), str(ACCEL_UNITS_MAX)]},
    )
    FACTORY_SETTINGS_OVERRIDES = dict()

    settingFunctions = dict(
        min_deg={'get': 'getMinDeg', 'set': 'setMinDeg'},
        max_deg={'get': 'getMaxDeg', 'set': 'setMaxDeg'},
        accel_units={'get': 'getAccelUnits', 'set': 'setAccelUnits'},
    )

    init_settings_dict = dict()
    settings_dict = dict()

    device_info_dict = dict(device_name="", path="", serial_number="",
                            hw_version="", sw_version="")

    serial_num = "Unknown"
    hw_version = "Unknown"
    sw_version = "Unknown"
    svx_if = None

    # Board link
    serial_port = None
    baud_int = 9600
    channel = 0
    connected = False

    # Motion state
    limits_dict = copy.deepcopy(FACTORY_LIMITS_DICT)
    accel_units = DEFAULT_ACCEL_UNITS
    position_deg = 0.0
    goal_deg = 0.0
    home_pos_deg = 0.0
    speed_ratio = 0.5
    speed_max_dps = 20.0

    drv_dict = dict()
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

        # In-process guard around each transaction, paired with the
        # cross-process file lock inside send_cmd().
        self.serial_lock = threading.Lock()

        # ---- Read the config discovery wrote to the param server. The channel
        #      is what makes this node one device among several on one path.
        try:
            self.drv_dict = nepi_sdk.get_param('~drv_dict', dict())
            self.device_name = self.drv_dict['DEVICE_DICT']['device_name']
            self.device_path = self.drv_dict['DEVICE_DICT']['device_path']
            self.port_str = self.drv_dict['DEVICE_DICT']['device_path']
            self.channel = int(self.drv_dict['DEVICE_DICT'].get('channel', 0))
            self.baud_int = int(self.drv_dict['DEVICE_DICT'].get('baud_str', '9600'))
            self.serial_num = str(self.drv_dict['DEVICE_DICT'].get('serial_number', 'Unknown'))
        except Exception as e:
            self.msg_if.pub_warn("Failed to load Device Dict " + str(e))
            nepi_sdk.signal_shutdown(self.node_name + ": no valid Device Dict")
            return

        # Per-instance copy of the calibration limits. The settings write into
        # these, and the class-level dict must not be what they write to.
        self.FACTORY_LIMITS_DICT = copy.deepcopy(SvxTemplateNode.FACTORY_LIMITS_DICT)
        self.limits_dict = copy.deepcopy(self.FACTORY_LIMITS_DICT)

        # ---- Board-level discovery OPTIONS. Anything describing the servo
        #      itself is a node setting instead -- see CAP_SETTINGS above.
        try:
            options = self.drv_dict.get('DISCOVERY_DICT', {}).get('OPTIONS', {})
            # TODO: read your board-level options here (protocol, bus address...).
            _ = options
        except Exception as e:
            self.msg_if.pub_warn("Failed to parse driver OPTIONS, using defaults: " + str(e))

        self.msg_if.pub_info("Connecting on " + str(self.port_str) +
                             " channel " + str(self.channel))
        self.connected = self.connect()
        if not self.connected:
            nepi_sdk.signal_shutdown(self.node_name + ": failed to connect")
            return
        self.msg_if.pub_info("Connected")

        self.settings_dict = self.initSettingsDict()
        self.settings_dict = self.refreshSettingsDict()

        self.device_info_dict["device_name"] = self.device_name
        self.device_info_dict["path"] = self.device_path
        self.device_info_dict["serial_number"] = self.serial_num
        self.device_info_dict["hw_version"] = self.hw_version
        self.device_info_dict["sw_version"] = self.sw_version

        # Push the initial acceleration + speed before the IF comes up.
        self.driverSetAcceleration(self.accel_units)
        self.driverSetSpeedRatio(self.speed_ratio)

        # ---- Register the servo with NEPI.
        self.svx_if = SVXActuatorIF(
            device_info=self.device_info_dict,
            factoryControls=self.FACTORY_CONTROLS,
            factoryLimits=self.FACTORY_LIMITS_DICT,
            getSettingsFunction=self.getSettingsFunction,
            setSettingFunction=self.setSettingFunction,
            data_source_description='servo',
            data_ref_description='servo',
            stopMovingCb=self.stopMoving,
            gotoPositionCb=self.gotoPosition,
            getPositionCb=self.getPosition,
            setSoftLimitsCb=self.setSoftLimits,
            getSoftLimitsCb=self.getSoftLimits,
            setSpeedRatioCb=self.setSpeedRatio,
            getSpeedRatioCb=self.getSpeedRatio,
            setSpeedMaxCb=self.setSpeedMax,
            getSpeedMaxCb=self.getSpeedMax,
            goHomeCb=self.goHome,
            setHomePositionCb=self.setHomePosition,
            setHomePositionHereCb=self.setHomePositionHere,
            deviceResetCb=self.resetDevice,
            # setSpinDirection=..., getSpinDirection=...,  # TODO: continuous-rotation servos
        )
        self.msg_if.pub_info(" ... SVX interface running")

        # A persisted min_deg/max_deg can be applied by SettingsIF while svx_if
        # is still None (mid-construction), so setDegRange() could not push it
        # to the IF and the IF kept the factory range. Re-push the calibrated
        # range now that the IF exists, so its clamp range matches the settings.
        self.svx_if.setHardstopLimits(self.limits_dict['min_hardstop_deg'],
                                      self.limits_dict['max_hardstop_deg'])

        nepi_sdk.start_timer_process(1.0 / self.MAX_POSITION_UPDATE_RATE,
                                     self.updatePositionHandler, oneshot=True)
        nepi_sdk.on_shutdown(self.cleanup_actions)
        self.msg_if.pub_info("Initialization complete")
        nepi_sdk.spin()

    ##########################################################################
    def updatePositionHandler(self, timer):
        # An open-loop servo has no feedback wire, but many controllers report
        # the drive value they are currently transmitting, which is enough for
        # the reported position to track a speed/accel-limited slew.
        stime = nepi_utils.get_time()
        pos_deg = self.driverGetPosition()
        if pos_deg is not None:
            self.position_deg = pos_deg
        gtime = nepi_utils.get_time() - stime
        next_delay = max(0.05, 1.0 / self.MAX_POSITION_UPDATE_RATE - gtime)
        nepi_sdk.start_timer_process(next_delay, self.updatePositionHandler, oneshot=True)

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

            default = self.readSettingValue(name)
            if name in self.FACTORY_SETTINGS_OVERRIDES:
                default = self.FACTORY_SETTINGS_OVERRIDES[name]
            if default is None:
                continue
            setting_dict['default'] = default
            init_settings_dict[name] = setting_dict

        self.init_settings_dict = init_settings_dict
        settings_dict = nepi_controls.create_controls_dict(init_settings_dict)
        self.msg_if.pub_info("Initialized Settings: " +
                             str(nepi_controls.get_values_dict(settings_dict)))
        return settings_dict

    def refreshSettingsDict(self):
        # These are per-servo calibration values this node holds, so this only
        # reads the current values back. Bounds do not move.
        settings_dict = copy.deepcopy(self.settings_dict)
        for name in settings_dict.keys():
            value = self.readSettingValue(name)
            if value is None:
                continue
            settings_dict = nepi_controls.set_value(settings_dict, name, value)
        return settings_dict

    def readSettingValue(self, setting_name):
        # Current value of one setting, typed for its control, or None if it
        # could not be read.
        if setting_name not in self.settingFunctions:
            return None
        setting_type = self.CAP_SETTINGS.get(setting_name, {}).get('type', 'String')
        try:
            val = getattr(self, self.settingFunctions[setting_name]['get'])()
        except Exception as e:
            self.msg_if.pub_warn("Failed to read setting " + setting_name + " : " + str(e))
            return None
        if val is None:
            return None
        try:
            if setting_type == 'Int':
                return int(float(val))
            if setting_type == 'Float':
                return float(val)
            if setting_type == 'Toggle':
                return (val is True)
            return str(val)
        except Exception as e:
            self.msg_if.pub_warn("Failed to convert setting " + setting_name + " : " + str(e))
            return None

    def getSettingsFunction(self):
        return self.settings_dict

    def setSettingFunction(self, setting_name, setting_value):
        # SettingsIF replaces its own dict with the THIRD element, so all three
        # must always be returned. It is also called as a bare statement during
        # SettingsIF.init(), so a discarded return must be harmless.
        setting_str = setting_name + ":" + str(setting_value)
        success = False
        msg = ""
        if setting_name not in self.settings_dict.keys():
            return False, (self.node_name + " Setting name " + setting_str +
                           " is not supported"), self.settings_dict
        if setting_name not in self.settingFunctions:
            return False, ("No set function registered for setting: " +
                           str(setting_name)), self.settings_dict

        set_function = getattr(self, self.settingFunctions[setting_name]['set'], None)
        if set_function is None:
            msg = "Missing set function: " + self.settingFunctions[setting_name]['set']
            self.msg_if.pub_warn(msg)
        else:
            # Every setter here returns [success, msg].
            [success, msg] = set_function(setting_value)
            if success:
                msg = (self.node_name + " UPDATED SETTINGS " + setting_str)

        self.settings_dict = self.refreshSettingsDict()
        return success, msg, self.settings_dict

    ##############
    ### Per-setting getters and setters.
    #
    # min_deg and max_deg are set independently by the UI but only mean
    # anything as a pair, so both setters route through one range function that
    # validates the resulting pair as a whole. Every setter returns
    # [success, msg].

    def getMinDeg(self):
        return self.limits_dict['min_hardstop_deg']

    def setMinDeg(self, val):
        return self.setDegRange(new_min=val)

    def getMaxDeg(self):
        return self.limits_dict['max_hardstop_deg']

    def setMaxDeg(self, val):
        return self.setDegRange(new_max=val)

    def setDegRange(self, new_min=None, new_max=None):
        min_deg = float(self.limits_dict['min_hardstop_deg'] if new_min is None else new_min)
        max_deg = float(self.limits_dict['max_hardstop_deg'] if new_max is None else new_max)
        if (max_deg - min_deg) < self.MIN_DEG_SPAN:
            return False, ("Rejecting degree range " + str(min_deg) + " to " + str(max_deg) +
                           " -- span below " + str(self.MIN_DEG_SPAN) + " deg")
        self.limits_dict['min_hardstop_deg'] = min_deg
        self.limits_dict['max_hardstop_deg'] = max_deg
        # The IF keeps the softstops mirrored to the hardstops -- they are the
        # single clamp range, so push the hardstops and let it mirror them.
        if self.svx_if is not None:
            self.svx_if.setHardstopLimits(min_deg, max_deg)
        return True, "Success"

    def getAccelUnits(self):
        return self.accel_units

    def setAccelUnits(self, val):
        try:
            units = int(float(val))
        except Exception as e:
            return False, "Acceleration not an integer: " + str(e)
        if units < self.ACCEL_UNITS_MIN or units > self.ACCEL_UNITS_MAX:
            return False, ("Acceleration " + str(units) + " outside " +
                           str(self.ACCEL_UNITS_MIN) + "-" + str(self.ACCEL_UNITS_MAX))
        self.accel_units = units
        self.driverSetAcceleration(units)
        return True, "Success"

    #########################################################################
    # SVX CAPABILITY CALLBACKS
    #
    # The IF clamps any commanded position against the limits and applies the
    # reverse-axis conversion before calling these, so a callback receives a
    # safe degree value in the device's own frame.
    #########################################################################
    def stopMoving(self):
        # A positional servo cannot coast: "stop" means hold where it is now.
        # Read the value the controller is currently transmitting and re-command
        # it, which halts any in-progress speed/accel-limited slew.
        pos_deg = self.driverGetPosition()
        if pos_deg is not None:
            self.driverMoveToPosition(pos_deg)

    def gotoPosition(self, position_deg):
        self.goal_deg = position_deg
        self.driverMoveToPosition(position_deg)

    def getPosition(self):
        return self.position_deg

    def setSoftLimits(self, min_deg, max_deg):
        self.limits_dict['min_softstop_deg'] = min_deg
        self.limits_dict['max_softstop_deg'] = max_deg

    def getSoftLimits(self):
        return self.limits_dict['min_softstop_deg'], self.limits_dict['max_softstop_deg']

    def setSpeedRatio(self, ratio):
        self.speed_ratio = max(0.0, min(1.0, float(ratio)))
        self.driverSetSpeedRatio(self.speed_ratio)

    def getSpeedRatio(self):
        return self.speed_ratio

    def setSpeedMax(self, speed_max_dps):
        self.speed_max_dps = float(speed_max_dps)
        # Re-apply so the ratio still means the same fraction of the new max.
        self.driverSetSpeedRatio(self.speed_ratio)

    def getSpeedMax(self):
        return self.speed_max_dps

    def goHome(self):
        self.gotoPosition(self.home_pos_deg)

    def setHomePosition(self, position_deg):
        self.home_pos_deg = position_deg

    def setHomePositionHere(self):
        self.home_pos_deg = self.position_deg

    def resetDevice(self):
        # Restore this node's factory config and re-push it to the hardware.
        self.limits_dict = copy.deepcopy(self.FACTORY_LIMITS_DICT)
        self.speed_ratio = 0.5
        self.accel_units = self.DEFAULT_ACCEL_UNITS
        self.driverSetAcceleration(self.accel_units)
        self.driverSetSpeedRatio(self.speed_ratio)

    #########################################################################
    # UNIT CONVERSION (degrees <-> hardware units)
    #
    # A pure linear map. Reverse-axis handling lives in the SVX IF, not here.
    #########################################################################
    def deg2raw(self, deg):
        # TODO: map a degree value onto your hardware's own command unit
        # (pulse width, step count, raw count...).
        span_deg = (self.limits_dict['max_hardstop_deg'] -
                    self.limits_dict['min_hardstop_deg'])
        if span_deg < self.MIN_DEG_SPAN:
            # setDegRange() enforces MIN_DEG_SPAN, so this is unreachable -- but
            # an unreachable guard is cheaper than a division by ~zero.
            return 0
        frac = (float(deg) - self.limits_dict['min_hardstop_deg']) / span_deg
        return int(round(frac * 1000))

    def raw2deg(self, raw):
        span_deg = (self.limits_dict['max_hardstop_deg'] -
                    self.limits_dict['min_hardstop_deg'])
        return self.limits_dict['min_hardstop_deg'] + (float(raw) / 1000.0) * span_deg

    #########################################################################
    # RAW HARDWARE I/O
    #########################################################################
    def driverMoveToPosition(self, deg):
        # TODO: send the position command for THIS channel.
        self.send_cmd('MOVE %d %d' % (self.channel, self.deg2raw(deg)))

    def driverSetSpeedRatio(self, ratio):
        # TODO: convert the 0.0-1.0 ratio into your hardware's speed unit.
        self.send_cmd('SPEED %d %.3f' % (self.channel, float(ratio) * self.speed_max_dps))

    def driverSetAcceleration(self, units):
        self.send_cmd('ACCEL %d %d' % (self.channel, int(units)))

    def driverGetPosition(self):
        # TODO: read back the value the controller is transmitting. Return None
        # when it cannot be read, so the caller keeps the last good value.
        response = self.send_cmd('POS? %d' % self.channel, read_reply=True)
        if response is None:
            return None
        try:
            return self.raw2deg(float(response))
        except Exception:
            return None

    def connect(self):
        try:
            self.serial_port = serial.Serial(self.port_str, self.baud_int,
                                             timeout=self.SERIAL_TIMEOUT_SEC)
        except Exception as e:
            self.msg_if.pub_warn("Serial open failed on " + str(self.port_str) +
                                 " (" + str(e) + ")")
            return False
        # TODO: a cheap handshake that proves this really is your controller,
        # and not merely that the port opened. Parse the identity out of the
        # reply into serial_num / hw_version / sw_version.
        response = self.send_cmd('INFO?', read_reply=True)
        if response is None:
            self.msg_if.pub_warn("No valid response on " + str(self.port_str))
            try:
                self.serial_port.close()
            except Exception:
                pass
            self.serial_port = None
            return False
        return True

    def send_cmd(self, cmd_str, read_reply=False):
        # Serialized TWICE on purpose: the in-process lock keeps this node's own
        # timer and command callbacks from interleaving, and the advisory file
        # lock on the port keeps SIBLING CHANNEL NODES on the same board from
        # interleaving their bytes. A board with one channel still wants both --
        # it costs nothing and the second channel arrives eventually.
        if self.serial_port is None:
            return None
        response = None
        with self.serial_lock:
            locked = False
            try:
                if HAVE_FCNTL:
                    fcntl.flock(self.serial_port.fileno(), fcntl.LOCK_EX)
                    locked = True
                self.serial_port.reset_input_buffer()
                self.serial_port.write(bytearray(cmd_str + '\r\n', 'utf-8'))
                self.serial_port.flush()
                if read_reply:
                    line = self.serial_port.readline().decode(errors='ignore').strip()
                    response = line if line != "" else None
            except Exception as e:
                self.msg_if.pub_warn("Transaction failed: " + str(e), throttle_s=5.0)
                response = None
            finally:
                if locked:
                    try:
                        fcntl.flock(self.serial_port.fileno(), fcntl.LOCK_UN)
                    except Exception:
                        pass
        return response

    #########################################################################
    def cleanup_actions(self):
        self.msg_if.pub_info("Shutting down: Executing script cleanup actions")
        if self.serial_port is not None:
            try:
                # TODO: leave the channel un-driven so the servo is not held
                # under power after the node exits.
                self.send_cmd('OFF %d' % self.channel)
            except Exception:
                pass
            try:
                self.serial_port.close()
            except Exception:
                pass


if __name__ == '__main__':
    node = SvxTemplateNode()
