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
# LSX (Lighting) DRIVER TEMPLATE -- NODE
# ---------------------------------------------------------------------------
# The node is launched by the discovery script (one node per device). It:
#   1. init_node() and read the '~drv_dict' param the discovery script wrote.
#   2. Connect to the hardware (raw serial here; no separate _driver.py).
#   3. Build the settings controls dict and register an LSXDeviceIF, handing it
#      the callbacks it needs (getStatus, turnOnOff, setIntensityRatio, ...).
#   4. spin(). All ROS topics/services are created by LSXDeviceIF -- this node
#      never touches rospy directly; it only implements device I/O callbacks.
#
# The LSXDeviceIF turns each callback into the standard NEPI LSX ROS API under
# <node_namespace>/lsx/... so every light looks identical to the rest of NEPI.
#
# SETTINGS CONTRACT (identical in every NEPI driver -- idx_v4l2_node.py is the
# reference implementation):
#   initSettingsDict()      build the controls dict once, at startup
#   refreshSettingsDict()   read live values back off the device
#   getSettingsFunction()   -> the current controls dict, no arguments
#   setSettingFunction(name, value) -> [success, msg, settings_dict]  (3 items!)
# Only the last two go to the device IF. The retired capSettings= /
# factorySettings= / settingUpdateFunction= arguments no longer exist on ANY
# device IF -- passing them raises TypeError at construction. A device with no
# settings at all passes NEITHER function; that is the correct state, not an
# omission.
##############################################################################

import os
import copy
import time
import serial                       # TODO: drop if your transport is not serial
import serial.tools.list_ports

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_controls   # controls dict: create/get/set value, bounds, options

from nepi_api.messages_if import MsgIF
from nepi_api.device_if_lsx import LSXDeviceIF

from nepi_interfaces.msg import DeviceLSXStatus

PKG_NAME = 'LSX_TEMPLATE'            # TODO: must match params.yaml 'pkg_name'
FILE_TYPE = 'NODE'

DEFAULT_MIN = 1
DEFAULT_MAX = 100


class LsxTemplateNode:

    # ---- Settings: exposed as NEPI "settings" (tunable at runtime via the UI).
    # These three tables are this node's own source data, NOT an IF argument.
    # initSettingsDict() below turns them into the nepi_controls controls dict
    # the device IF actually consumes.
    #   CAP_SETTINGS  - what CAN be set: type + the value range/option list.
    #   FACTORY_SETTINGS - the defaults each setting starts from.
    #   FACTORY_SETTINGS_OVERRIDES - anything forced away from the factory value.
    # 'type' must be one of nepi_controls.CONTROL_TYPES ("Int", "Float",
    # "Toggle", "Selection", "Menu", "String", ...). 'Discrete' is NOT one of
    # them: create_controls_dict() wraps each entry in a bare except, so a
    # 'Discrete' setting is silently dropped and never appears in the RUI. A
    # named option list is a 'Selection'.
    CAP_SETTINGS = dict(
        min_intensity_percent={"type": "Int", "name": "min_intensity_percent", "options": ["1", "100"]},
        max_intensity_percent={"type": "Int", "name": "max_intensity_percent", "options": ["1", "100"]},
    )
    FACTORY_SETTINGS = dict(
        min_intensity_percent={"type": "Int", "name": "min_intensity_percent", "value": str(DEFAULT_MIN)},
        max_intensity_percent={"type": "Int", "name": "max_intensity_percent", "value": str(DEFAULT_MAX)},
    )
    FACTORY_SETTINGS_OVERRIDES = dict()

    # Maps each setting name to the get/set method on THIS class that talks to hw.
    # (The shipped drivers resolve these via module-level `global` functions +
    #  globals(); using bound methods + getattr, as below, is equivalent and
    #  cleaner for a template. Either is fine -- pick one and be consistent.)
    settingFunctions = dict(
        min_intensity_percent={'get': 'getMinIntensityPercent', 'set': 'setMinIntensityPercent'},
        max_intensity_percent={'get': 'getMaxIntensityPercent', 'set': 'setMaxIntensityPercent'},
    )

    # ---- Factory controls: the initial device state LSXDeviceIF should assume.
    FACTORY_CONTROLS = dict(
        standby_enabled=False,
        on_off_state=False,
        intensity_ratio=0.0,
        strobe_enabled=False,
        blink_interval_sec=2,
        blink_enabled=False,
    )

    # ---- device_info_dict: REQUIRED keys for every *DeviceIF constructor.
    device_info_dict = dict(device_name="", path="", serial_number="",
                            hw_version="", sw_version="")

    # runtime state
    serial_port = None
    serial_busy = False
    connected = False
    on_off_state = False
    standby_state = False
    strobe_state = False
    intensity_ratio = 0.0
    temp_c = 0.0
    min_intensity = DEFAULT_MIN
    max_intensity = DEFAULT_MAX

    self_check_count = 5
    self_check_counter = 0
    lsx_if = None

    init_settings_dict = dict()
    settings_dict = dict()

    DEFAULT_NODE_NAME = PKG_NAME.lower() + "_node"

    ##########################################################################
    def __init__(self):
        ####  NODE Initialization ####
        nepi_sdk.init_node(name=self.DEFAULT_NODE_NAME)
        self.class_name = type(self).__name__
        self.base_namespace = nepi_sdk.get_base_namespace()
        self.node_name = nepi_sdk.get_node_name()
        self.node_namespace = nepi_sdk.get_node_namespace()

        self.msg_if = MsgIF(log_name=self.class_name)
        self.msg_if.pub_info("Starting Node Initialization Processes")

        # ---- Read the config the discovery script wrote to the param server.
        try:
            self.drv_dict = nepi_sdk.get_param('~drv_dict', dict())
            self.device_name = self.drv_dict['DEVICE_DICT']['device_name']
            self.device_path = self.drv_dict['DEVICE_DICT']['device_path']
            self.port_str = self.drv_dict['DEVICE_DICT']['device_path']
            self.baud_str = self.drv_dict['DEVICE_DICT']['baud_str']
            self.baud_int = int(self.baud_str)
            self.addr_str = self.drv_dict['DEVICE_DICT']['addr_str'].zfill(3)
        except Exception as e:
            self.msg_if.pub_warn("Failed to load Device Dict " + str(e))
            nepi_sdk.signal_shutdown(self.node_name + ": no valid Device Dict")
            return

        # ---- Connect to hardware
        self.msg_if.pub_info("Connecting on " + self.port_str + " @ " + self.baud_str)
        self.connected = self.connect()
        if not self.connected:
            nepi_sdk.signal_shutdown(self.node_name + ": failed to connect")
            return
        self.msg_if.pub_info("Connected")

        # ---- Build settings, then fill the required device_info_dict keys
        self.settings_dict = self.initSettingsDict()
        self.settings_dict = self.refreshSettingsDict()
        self.device_info_dict["device_name"] = self.device_name
        self.device_info_dict["path"] = self.device_path
        self.device_info_dict["serial_number"] = self.serial_num
        self.device_info_dict["hw_version"] = self.hw_version
        self.device_info_dict["sw_version"] = self.sw_version

        # ---- Register with NEPI. LSXDeviceIF creates all the ROS plumbing and
        #      calls back into these methods. Pass None for any capability the
        #      device lacks (e.g. no color/kelvin/strobe here).
        self.lsx_if = LSXDeviceIF(
            device_info=self.device_info_dict,
            data_source_description='lighting_device',
            data_ref_description='device',
            getStatusFunction=self.getStatus,
            getSettingsFunction=self.getSettingsFunction,
            setSettingFunction=self.setSettingFunction,
            factoryControls=self.FACTORY_CONTROLS,
            standbyEnableFunction=None,             # TODO: add if device supports standby
            turnOnOffFunction=self.turnOnOff,
            setIntensityRatioFunction=self.setIntensityRatio,
            color_options_list=None,                # TODO: add if RGB-capable
            setColorFunction=None,
            kelvin_limits_list=None,                # TODO: add if tunable-white
            setKelvinFunction=None,
            enableStrobeFunction=None,              # TODO: add if strobe-capable
            blinkOnOffFunction=None,
            reports_temp=True,                      # this device reports temperature
            reports_power=False,
        )

        # ---- Optional health poll: shut down after too many comm failures so
        #      discovery can purge and (optionally) relaunch us.
        nepi_sdk.start_timer_process(0.2, self.check_timer_callback)
        nepi_sdk.on_shutdown(self.cleanup_actions)
        self.msg_if.pub_info("Initialization complete")
        nepi_sdk.spin()

    #########################################################################
    # SETTINGS PLUMBING (identical shape across all NEPI drivers)
    #
    # The settings dict IS a nepi_controls controls dict. Build the plain init
    # dict first, then hand it to nepi_controls.create_controls_dict(). Read and
    # write values through nepi_controls.get_value()/set_value()/set_bounds()/
    # set_options() -- never by indexing the controls dict directly.
    #
    # Note how CAP_SETTINGS' numeric "options" pair becomes 'bounds': the
    # retired cap-settings form carried an Int/Float control's min and max in an
    # 'options' pair, and 'options' on a numeric control is ignored now.
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
            # The default must be TYPED to match 'type' -- a string default on
            # an Int control is dropped by create_controls_dict().
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
        # A serial device holds its own settings, so this only reads current
        # values back. Bounds and options do not move. (A device that reports a
        # live capability set -- a camera -- also re-applies set_bounds()/
        # set_options() here; see idx_template_node.py.)
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
                settings_dict = nepi_controls.set_value(settings_dict, name, int(val))
            except Exception as e:
                self.msg_if.pub_warn("Failed to apply read setting " + name + " : " + str(e))
        return settings_dict

    def getSettingsFunction(self):
        return self.settings_dict

    def setSettingFunction(self, setting_name, setting_value):
        # SettingsIF replaces its own dict with the THIRD element, so all three
        # must always be returned. It is also called as a bare statement during
        # SettingsIF.init(), so a discarded return must be harmless.
        setting_str = setting_name + ":" + str(setting_value)
        success = False
        if setting_name not in self.settings_dict.keys():
            return False, (self.node_name + " Setting name " + setting_str +
                           " is not supported"), self.settings_dict
        if setting_name not in self.settingFunctions:
            return False, (self.node_name + " Setting name " + setting_str +
                           " has no set function"), self.settings_dict

        try:
            set_fn = getattr(self, self.settingFunctions[setting_name]['set'])
            success = (set_fn(setting_value) is True)
            msg = (self.node_name + " UPDATED SETTINGS " + setting_str) if success else \
                  (self.node_name + " device rejected setting " + setting_str)
        except Exception as e:
            msg = "Failed to set " + setting_str + " : " + str(e)
            self.msg_if.pub_warn(msg)

        self.settings_dict = self.refreshSettingsDict()
        return success, msg, self.settings_dict

    # ---- Per-setting hardware get/set. TODO: implement your protocol here.
    def getMinIntensityPercent(self):
        return self.min_intensity

    def setMinIntensityPercent(self, val):
        self.min_intensity = int(val)
        return True

    def getMaxIntensityPercent(self):
        return self.max_intensity

    def setMaxIntensityPercent(self, val):
        self.max_intensity = int(val)
        return True

    #########################################################################
    # LSX CAPABILITY CALLBACKS (what makes this an LSX/lighting device)
    #########################################################################
    def getStatus(self):
        # Refresh cached values from the device, then publish a status msg.
        self.update_status_values()
        status_msg = DeviceLSXStatus()
        status_msg.device_node_name = self.node_name
        status_msg.device_name = self.device_info_dict["device_name"]
        status_msg.device_path = self.device_info_dict["path"]
        status_msg.serial_num = self.device_info_dict["serial_number"]
        status_msg.hw_version = self.device_info_dict["hw_version"]
        status_msg.sw_version = self.device_info_dict["sw_version"]
        status_msg.on_off_state = self.on_off_state
        status_msg.standby_state = self.standby_state
        status_msg.intensity_ratio = self.intensity_ratio
        status_msg.strobe_state = self.strobe_state
        status_msg.blink_state = False
        status_msg.blink_interval = 0
        status_msg.temp_c = int(self.temp_c)
        status_msg.power_w = 0
        return status_msg

    def turnOnOff(self, turn_on):
        # TODO: send the real on/off command to your device.
        cmd = 'ON' if turn_on else 'OFF'
        response = self.send_msg('!' + self.addr_str + ':PWR=' + cmd)
        if response is not None:
            self.on_off_state = bool(turn_on)

    def setIntensityRatio(self, ratio):
        # ratio is 0.0-1.0; map to the device's own units.
        ratio = max(0.0, min(1.0, float(ratio)))
        percent = int(round(ratio * 100))
        response = self.send_msg('!' + self.addr_str + ':LOUT=' + str(percent))
        if response is not None:
            self.intensity_ratio = ratio

    def update_status_values(self):
        # TODO: query the device for live state (power, intensity, temp, ...).
        r = self.send_msg('!' + self.addr_str + ':TEMP?')
        if r is not None:
            try:
                self.temp_c = float(r)
            except Exception:
                pass

    #########################################################################
    # RAW HARDWARE I/O  (serial in-node; use a separate _driver.py for complex
    # transports -- see idx_template for that split.)
    #########################################################################
    def connect(self):
        try:
            self.serial_port = serial.Serial(self.port_str, self.baud_int, timeout=1)
        except Exception as e:
            self.msg_if.pub_warn("Serial open failed: " + str(e))
            return False
        response = self.send_msg('!' + self.addr_str + ':INFO?')
        if response is None or not response.startswith('INFO,'):
            return False
        # TODO: parse your device identity out of the INFO reply.
        parts = response.strip().split(',')
        self.serial_num = parts[1] if len(parts) > 1 else ""
        self.hw_version = parts[2] if len(parts) > 2 else ""
        self.sw_version = parts[3] if len(parts) > 3 else ""
        return True

    def send_msg(self, ser_msg):
        if self.serial_port is None:
            return None
        # crude mutex so the status timer and command callbacks don't interleave
        timeout = 0
        while self.serial_busy and timeout < 100:
            nepi_sdk.sleep(0.01, 2)
            timeout += 1
        self.serial_busy = True
        response = None
        try:
            ser_str = ser_msg + '\r\n'
            self.serial_port.reset_input_buffer()
            self.serial_port.write(bytearray(ser_str, 'utf-8'))
            time.sleep(0.01)
            response = self.serial_port.readline().decode(errors='ignore').strip()
            self.self_check_counter = 0
        except Exception as e:
            self.msg_if.pub_warn("Serial comm error: " + str(e))
            self.self_check_counter += 1
        finally:
            self.serial_busy = False
        return response

    #########################################################################
    def check_timer_callback(self, timer):
        # If comms have failed repeatedly, exit so discovery can recover us.
        if self.self_check_counter >= self.self_check_count:
            self.msg_if.pub_warn("Too many comm failures; shutting down")
            nepi_sdk.signal_shutdown("Too many comm failures")

    def cleanup_actions(self):
        try:
            if self.serial_port is not None:
                self.serial_port.close()
        except Exception:
            pass


if __name__ == '__main__':
    node = LsxTemplateNode()
