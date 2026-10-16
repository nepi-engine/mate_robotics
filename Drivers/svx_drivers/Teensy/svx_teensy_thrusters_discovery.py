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

# Discovery for a PJRC Teensy 4.1 running the teensy_thrusters firmware, which
# generates the pulses for Blue Robotics Basic ESCs driving T200 thrusters and
# for up to four hobby servos. The Pi-to-Teensy protocol is mate_robotics
# Firmware/teensy_thrusters/PROTOCOL.md; section numbers below refer to it.
#
# CALL model, same contract as svx_servo_maestro_discovery.py and the SVX
# template: drivers_mgr instantiates this class once with no arguments and
# calls discoveryFunction() every poll cycle inside its own process, so there
# is no init_node()/spin() here.
#
# PARTS. The devices option selects which parts of each board get nodes, the
# way idx_zed's data_products option selects data products:
#   * thrusters -- ONE node per board (svx_teensy_thrusters_node.py), a
#     MotorsDeviceIF with one motor per thruster, because the thrusters are one
#     vehicle's propulsion and the Teensy keeps them on one watchdog.
#   * servos    -- ONE node PER SERVO (svx_teensy_thrusters_servo_node.py), each
#     an SVXActuatorIF. SVXActuatorIF always lives at <node>/svx, so a node can
#     host only one servo -- the same reason the Maestro runs a node per channel.
# Every node of a board shares its one serial port under a per-transaction
# file lock (PROTOCOL.md 2.1). Each node is a "launch id" (board key + part),
# and each pass reconciles the wanted set against the running set like the
# Maestro discovery: unwanted nodes are killed, missing ones launched, and an
# option change restarts only the part it affects.
#
# WHAT IS DIFFERENT FROM THE MAESTRO:
#   * The USB IDs do not identify the firmware. Every Teensyduino "Serial"
#     sketch enumerates as VID 0x16C0 / PID 0x0483, so a board is opened and
#     asked to identify (PROTOCOL.md 6.1) before any node is launched. The
#     identity is cached per board while any of its nodes runs, so discovery
#     never opens a port its nodes are using.
#   * Boards are keyed by their USB serial number, not their /dev/ttyACM index,
#     so a board that re-enumerates on a different ttyACM is the same device
#     with the same node names and saved config -- never a second set of nodes.
#   * Every OPTION is parsed and validated here and handed to the nodes in
#     DEVICE_DICT. An invalid option launches nothing new for its part, is
#     logged, and leaves that part's running nodes alone: a typo in the RUI
#     must not stop the thrusters.
#
# SAFETY: discovery only ever sends the identify command. It never sends a
# thrust (SA) or servo (SS) frame. Opening and closing a port that no node
# holds toggles DTR, which the firmware answers by holding every thruster at
# stop (PROTOCOL.md 9).

import copy
import functools
import math
import time

import serial
from serial.tools import list_ports

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_drvs
from nepi_sdk import nepi_system

PKG_NAME = 'SVX_TEENSY_THRUSTERS'
FILE_TYPE = 'DISCOVERY'


class SvxTeensyThrustersDiscovery:

    # Teensyduino USB type "Serial" IDs (Docs/tbd/pjrc_teensy_4_1.md). The
    # bootloader (PID 0x0478) is deliberately not matched: a board in its
    # bootloader runs no firmware and outputs no pulses.
    TEENSY_VENDOR_ID = 0x16C0
    TEENSY_SERIAL_PRODUCT_ID = 0x0483

    # Must match the firmware's ID reply (PROTOCOL.md 6.1 and 11).
    FIRMWARE_NAME = 'teensy_thrusters'
    PROTOCOL_MAJOR = 1
    # Servo commands arrived in 1.1 (PROTOCOL.md 11).
    SERVO_PROTOCOL_MINOR = 1

    # Any rate but 134, which reboots a running Teensy into its bootloader.
    PROBE_BAUD = 115200
    PROBE_REPLY_TIMEOUT_SEC = 0.5
    # A port that gave no usable reply is re-probed no sooner than this.
    PROBE_RETRY_SEC = 10
    # A board that answered as other firmware, or another protocol major, is
    # re-probed this rarely: it is someone else's Teensy until reflashed.
    FOREIGN_RECHECK_SEC = 60
    # A node that exited on its own (lost serial link, failed handshake) is
    # relaunched no sooner than this, so a node that keeps failing cannot spin.
    RELAUNCH_DELAY_SEC = 10

    # devices option -> parts (PROTOCOL.md has no notion of parts; this is the
    # driver's). Kept as an explicit table so an unknown value is rejected.
    DEVICES_OPTIONS = {
        'thrusters': ['thrusters'],
        'servos': ['servos'],
        'thrusters,servos': ['thrusters', 'servos'],
    }
    THRUSTERS = 'thrusters'
    SERVOS = 'servos'

    # esc_model -> fixed pulse parameters (Docs/Motors/bluerobotics_basic_esc.md:
    # the documented 1100/1500/1900 us, deadband 1500 +/- 25 us). Used here only
    # to validate; the node resolves esc_model with an identical table. Keep the
    # two in step.
    ESC_MODELS = {
        'BlueRobotics Basic ESC (BESC30-R3)': dict(pulse_min_us = 1100, pulse_stop_us = 1500,
                                                   pulse_max_us = 1900, deadband_us = 25),
    }
    CUSTOM_ESC_MODEL = 'Custom'

    # Limits enforced by the firmware's CF command (PROTOCOL.md 5), checked
    # here first so a bad option is reported by name instead of as a
    # rejected frame.
    PULSE_RATES_HZ = [50, 100, 200, 400]
    PULSE_HARD_MIN_US = 800
    PULSE_HARD_MAX_US = 2200
    PERIOD_GUARD_US = 100
    DEADBAND_MAX_US = 200
    WATCHDOG_MS_RANGE = (50, 1000)
    ARMING_HOLD_S_RANGE = (2, 15)
    COMMAND_TIMEOUT_S_RANGE = (0.1, 10.0)

    # Servo limits (PROTOCOL.md 5.1). SERVO_COUNT_RANGE's upper end is the
    # firmware's servo count; the identify reply is checked against it too.
    SERVO_RATES_HZ = [50, 100, 200, 333]
    SERVO_COUNT_RANGE = (1, 4)
    SERVO_HOST_LOSS_OPTIONS = ['hold', 'limp']

    node_launch_name = "teensy_thrusters"

    # launch id -> {node_name, sub_process, path, board_key, part, config}
    active_devices_dict = dict()
    active_paths_list = []
    dont_retry_list = []
    # board key -> identity from its last successful identify reply. Kept while
    # any node of the board runs, so a part can be (re)launched without opening
    # a port the board's other nodes are using.
    identity_dict = dict()
    # board key -> earliest time it may be probed again
    probe_after_dict = dict()
    # launch id -> time its node was found to have exited on its own
    exit_times_dict = dict()
    # key -> last message logged, for messages logged only when they change
    last_log_dict = dict()

    retry = True


    ################################################
    def __init__(self):
        self.logger = nepi_sdk.logger()
        time.sleep(1)
        self.logger.log_info("Starting Initialization")
        self.logger.log_info("Initialization Complete")


    ##########  DRV Standard Discovery Function
    def discoveryFunction(self, available_paths_list, active_paths_list, base_namespace, drv_dict, retry_enabled = True):
        self.drv_dict = drv_dict
        self.available_paths_list = available_paths_list
        self.active_paths_list = active_paths_list
        self.base_namespace = base_namespace

        ##################################
        # Resolve and validate every option. Each part's config is a dict when
        # valid, None when its part is not selected, and the string 'invalid'
        # (already logged) when an option it needs is bad.
        options = dict()
        try:
            options = self.drv_dict.get('DISCOVERY_DICT', {}).get('OPTIONS', {})
        except Exception as e:
            self.logger.log_warn("Failed to read options " + str(e))
        parts = self.resolveParts(options)

        # Retry behavior
        self.retry = retry_enabled
        if self.retry == True:
            self.dont_retry_list = []

        ### Purge nodes whose port has disappeared from /dev
        for launch_id in list(self.active_devices_dict.keys()):
            if self.active_devices_dict[launch_id]['path'] not in self.available_paths_list:
                self.killDevice(launch_id)

        ### Reap nodes that exited on their own. A node exits when it loses its
        ### serial link; without this its stale entry would block the relaunch.
        for launch_id in list(self.active_devices_dict.keys()):
            sub_process = self.active_devices_dict[launch_id]['sub_process']
            if sub_process is not None and sub_process.poll() is not None:
                self.logger.log_warn("Node " + self.active_devices_dict[launch_id]['node_name'] +
                                     " exited; relaunching in " + str(self.RELAUNCH_DELAY_SEC) +
                                     "s if its board is still present")
                self.exit_times_dict[launch_id] = time.time()
                self.killDevice(launch_id)

        ### Enumerate Teensy serial ports
        [boards, enumerate_ok] = self.findBoards()
        if enumerate_ok == False:
            # Enumeration failed, which is not the same as finding no boards.
            # Reconciling against an empty list would tear every node down.
            return self.active_paths_list
        if parts is None:
            # The devices option itself is invalid: change nothing.
            return self.active_paths_list
        nothing_wanted = (parts[self.THRUSTERS] is None and parts[self.SERVOS] is None)

        ### Forget identities of boards that are gone or have no running node;
        ### such a board is re-probed before anything is launched for it.
        present_keys = [b['key'] for b in boards]
        for key in list(self.identity_dict.keys()):
            if key not in present_keys or not self.boardHasNodes(key):
                del self.identity_dict[key]

        ### Build the wanted set of launch ids, probing any board that needs it
        now = time.time()
        desired_dict = dict()
        keep_ids = []   # running nodes of a part whose options are invalid
        for board in boards:
            key = board['key']
            if key in self.dont_retry_list:
                continue
            identity = self.identity_dict.get(key)
            if identity is None:
                if nothing_wanted or self.boardHasNodes(key):
                    continue
                if board['path'] in self.active_paths_list:
                    # Claimed by another driver's node; never open a port in use.
                    continue
                if now < self.probe_after_dict.get(key, 0):
                    continue
                identity = self.probeBoard(board)
                if identity is None:
                    continue
                self.identity_dict[key] = identity

            thruster_cfg = parts[self.THRUSTERS]
            if thruster_cfg == 'invalid':
                keep_ids.append(self.launchId(key, self.THRUSTERS))
            elif thruster_cfg is not None and self.checkThrustersFit(thruster_cfg, identity, board):
                launch_id = self.launchId(key, self.THRUSTERS)
                desired_dict[launch_id] = dict(board = board, part = self.THRUSTERS, servo_index = None,
                                               config = thruster_cfg)

            servo_cfg = parts[self.SERVOS]
            if servo_cfg == 'invalid':
                keep_ids.extend(self.launchIdsOf(key, self.SERVOS))
            elif servo_cfg is not None and self.checkServosFit(servo_cfg, identity, board):
                node_cfg = dict(servo_rate_hz = servo_cfg['servo_rate_hz'],
                                servo_on_host_loss = servo_cfg['servo_on_host_loss'])
                for n in range(servo_cfg['servo_count']):
                    launch_id = self.launchId(key, self.SERVOS, n)
                    desired_dict[launch_id] = dict(board = board, part = self.SERVOS, servo_index = n,
                                                   config = node_cfg)

        ### Kill running nodes that are no longer wanted (part deselected, servo
        ### count lowered, board gone) or whose options changed. A node whose
        ### part has invalid options keeps running on its last valid options.
        for launch_id in list(self.active_devices_dict.keys()):
            if launch_id in keep_ids:
                continue
            if launch_id not in desired_dict:
                self.killDevice(launch_id)
            elif self.active_devices_dict[launch_id]['config'] != desired_dict[launch_id]['config']:
                self.logger.log_warn("Driver options changed; restarting node " +
                                     self.active_devices_dict[launch_id]['node_name'])
                self.killDevice(launch_id)

        ### Launch every wanted node that is not running
        for launch_id in sorted(desired_dict.keys()):
            if launch_id in self.active_devices_dict:
                continue
            if launch_id in self.exit_times_dict:
                if time.time() - self.exit_times_dict[launch_id] < self.RELAUNCH_DELAY_SEC:
                    continue
                del self.exit_times_dict[launch_id]
            entry = desired_dict[launch_id]
            board = entry['board']
            self.launchDeviceNode(launch_id, board, self.identity_dict[board['key']],
                                  entry['part'], entry['servo_index'], entry['config'])

        ### Claim each board's path once any of its nodes is live
        for entry in self.active_devices_dict.values():
            if entry['path'] not in self.active_paths_list:
                self.active_paths_list.append(entry['path'])
        return self.active_paths_list
    ################################################


    ##########  Device specific calls

    def logOnChange(self, key, msg, warn = True):
        # Discovery runs every few seconds, so a standing condition is logged once
        # when it appears or changes rather than on every pass. msg None clears
        # the condition, so it is logged again if it comes back.
        if msg is None:
            self.last_log_dict.pop(key, None)
            return
        if self.last_log_dict.get(key) == msg:
            return
        self.last_log_dict[key] = msg
        if warn == True:
            self.logger.log_warn(msg)
        else:
            self.logger.log_info(msg)


    def launchId(self, board_key, part, servo_index = None):
        if part == self.SERVOS:
            return board_key + "|servo" + str(servo_index)
        return board_key + "|" + part


    def launchIdsOf(self, board_key, part):
        return [lid for lid, e in self.active_devices_dict.items()
                if e['board_key'] == board_key and e['part'] == part]


    def boardHasNodes(self, board_key):
        return any(e['board_key'] == board_key for e in self.active_devices_dict.values())


    def resolveParts(self, options):
        # {'thrusters': cfg, 'servos': cfg}, each a dict, None (not selected) or
        # 'invalid'; or None if the devices option itself is invalid.
        devices = self.optionValue(options, 'devices', 'thrusters,servos').replace(' ', '')
        if devices not in self.DEVICES_OPTIONS:
            self.logOnChange('devices', "Invalid devices option '" + devices + "'; must be one of " +
                             str(list(self.DEVICES_OPTIONS.keys())) + ". Nothing launched or stopped")
            return None
        self.logOnChange('devices', "Driver devices: " + devices, warn = False)
        selected = self.DEVICES_OPTIONS[devices]
        parts = {self.THRUSTERS: None, self.SERVOS: None}
        if self.THRUSTERS in selected:
            parts[self.THRUSTERS] = self.resolveThrusterConfig(options)
        else:
            self.logOnChange('config', None, warn = False)
        if self.SERVOS in selected:
            parts[self.SERVOS] = self.resolveServoConfig(options)
        else:
            self.logOnChange('servo_config', None, warn = False)
        return parts


    def resolveThrusterConfig(self, options):
        # Turn the thruster OPTIONS into the validated configuration the
        # thruster node uses, or 'invalid'. drivers_mgr stores every value as a
        # string and writes operator edits without range checks, so every value
        # is parsed and range-checked here.
        try:
            esc_model = self.optionValue(options, 'esc_model', 'BlueRobotics Basic ESC (BESC30-R3)')
            if esc_model in self.ESC_MODELS:
                pulses = copy.deepcopy(self.ESC_MODELS[esc_model])
            elif esc_model == self.CUSTOM_ESC_MODEL:
                pulses = dict(
                    pulse_min_us = self.intOption(options, 'custom_pulse_min_us', 1100,
                                                  self.PULSE_HARD_MIN_US, self.PULSE_HARD_MAX_US),
                    pulse_stop_us = self.intOption(options, 'custom_pulse_stop_us', 1500,
                                                   self.PULSE_HARD_MIN_US, self.PULSE_HARD_MAX_US),
                    pulse_max_us = self.intOption(options, 'custom_pulse_max_us', 1900,
                                                  self.PULSE_HARD_MIN_US, self.PULSE_HARD_MAX_US),
                    deadband_us = self.intOption(options, 'custom_deadband_us', 25,
                                                 0, self.DEADBAND_MAX_US))
            else:
                raise ValueError("esc_model '" + esc_model + "' is not a known ESC")

            pulse_rate_hz = self.intOption(options, 'pulse_rate_hz', 200, 50, 400)
            if pulse_rate_hz not in self.PULSE_RATES_HZ:
                raise ValueError("pulse_rate_hz must be one of " + str(self.PULSE_RATES_HZ))

            pmin = pulses['pulse_min_us']
            pstop = pulses['pulse_stop_us']
            pmax = pulses['pulse_max_us']
            dband = pulses['deadband_us']
            if not (pmin < pstop - dband and pstop + dband < pmax):
                raise ValueError("pulse widths must satisfy min < stop - deadband and stop + deadband < max (got " +
                                 str(pmin) + "/" + str(pstop) + "/" + str(pmax) + " us, deadband " + str(dband) + " us)")
            if pmax + self.PERIOD_GUARD_US > 1000000 // pulse_rate_hz:
                raise ValueError("pulse max " + str(pmax) + " us does not fit a " + str(pulse_rate_hz) + " Hz frame")

            thruster_channels = self.parseIndexList(self.optionValue(options, 'thruster_channels', '0,1,2,3,4,5'),
                                                    'thruster_channels')
            if len(thruster_channels) == 0:
                raise ValueError("thruster_channels lists no channels")
            reversed_thrusters = self.parseIndexList(self.optionValue(options, 'reversed_thrusters', ''),
                                                     'reversed_thrusters')
            for index in reversed_thrusters:
                if index >= len(thruster_channels):
                    raise ValueError("reversed_thrusters entry " + str(index) + " is not a motor index (0-" +
                                     str(len(thruster_channels) - 1) + ")")

            # The parsed option values, not the effective pulses: the node
            # resolves esc_model into pulse widths itself (its ESC_MODELS table
            # mirrors the one above). custom_* are passed only for Custom.
            config = dict(
                esc_model = esc_model,
                thruster_model = self.optionValue(options, 'thruster_model', 'BlueRobotics T200 (R2)'),
                thruster_channels = thruster_channels,
                reversed_thrusters = reversed_thrusters,
                max_thrust_ratio = self.floatOption(options, 'max_thrust_ratio', 1.0, 0.0, 1.0),
                pulse_rate_hz = pulse_rate_hz,
                watchdog_ms = self.intOption(options, 'watchdog_ms', 200, *self.WATCHDOG_MS_RANGE),
                arming_hold_s = self.intOption(options, 'arming_hold_s', 7, *self.ARMING_HOLD_S_RANGE),
                command_timeout_s = self.floatOption(options, 'command_timeout_s', 1.0, *self.COMMAND_TIMEOUT_S_RANGE),
            )
            if esc_model == self.CUSTOM_ESC_MODEL:
                config['custom_pulse_min_us'] = pmin
                config['custom_pulse_stop_us'] = pstop
                config['custom_pulse_max_us'] = pmax
                config['custom_deadband_us'] = dband
        except Exception as e:
            self.logOnChange('config', "Invalid thruster options, no thruster node launched: " + str(e))
            return 'invalid'
        self.logOnChange('config', "Thruster options: " + str(config), warn = False)
        return config


    def resolveServoConfig(self, options):
        # The servo OPTIONS, validated, or 'invalid'. Per-servo calibration is
        # not here: it is each servo node's own settings.
        try:
            servo_count = self.intOption(options, 'servo_count', 1, *self.SERVO_COUNT_RANGE)
            servo_rate_hz = self.intOption(options, 'servo_rate_hz', 50, 50, 333)
            if servo_rate_hz not in self.SERVO_RATES_HZ:
                raise ValueError("servo_rate_hz must be one of " + str(self.SERVO_RATES_HZ))
            host_loss = self.optionValue(options, 'servo_on_host_loss', 'hold').lower()
            if host_loss not in self.SERVO_HOST_LOSS_OPTIONS:
                raise ValueError("servo_on_host_loss must be one of " + str(self.SERVO_HOST_LOSS_OPTIONS))
            config = dict(servo_count = servo_count,
                          servo_rate_hz = servo_rate_hz,
                          servo_on_host_loss = host_loss)
        except Exception as e:
            self.logOnChange('servo_config', "Invalid servo options, no servo node launched: " + str(e))
            return 'invalid'
        self.logOnChange('servo_config', "Servo options: " + str(config), warn = False)
        return config


    def optionValue(self, options, name, default):
        value = options.get(name, {}).get('value', default)
        if value is None:
            value = default
        return str(value).strip()


    def intOption(self, options, name, default, min_val, max_val):
        text = self.optionValue(options, name, str(default))
        try:
            fval = float(text)
        except Exception:
            raise ValueError(name + " '" + text + "' is not a number")
        if math.isnan(fval) or fval != int(fval):
            raise ValueError(name + " '" + text + "' is not a whole number")
        val = int(fval)
        if val < min_val or val > max_val:
            raise ValueError(name + " " + str(val) + " is outside " + str(min_val) + "-" + str(max_val))
        return val


    def floatOption(self, options, name, default, min_val, max_val):
        text = self.optionValue(options, name, str(default))
        try:
            val = float(text)
        except Exception:
            raise ValueError(name + " '" + text + "' is not a number")
        if math.isnan(val) or val < min_val or val > max_val:
            raise ValueError(name + " " + text + " is outside " + str(min_val) + "-" + str(max_val))
        return val


    def parseIndexList(self, text, name):
        # "0,1,2" -> [0, 1, 2]; "" -> []. Order is kept. Unlike the Maestro's
        # channel list there is no fallback: thruster order IS the motor
        # mapping, so a bad list is rejected rather than guessed.
        values = []
        for tok in str(text).replace(';', ',').split(','):
            tok = tok.strip()
            if tok == '':
                continue
            try:
                value = int(tok)
            except Exception:
                raise ValueError(name + " entry '" + tok + "' is not a whole number")
            if value < 0:
                raise ValueError(name + " entry " + str(value) + " is negative")
            if value in values:
                raise ValueError(name + " lists " + str(value) + " twice")
            values.append(value)
        return values


    def checkThrustersFit(self, config, identity, board):
        count = identity['thruster_count']
        bad = [c for c in config['thruster_channels'] if c >= count]
        if len(bad) > 0:
            self.logOnChange('channels:' + board['key'], "thruster_channels " + str(bad) +
                             " out of range for the Teensy on " + board['path'] +
                             ", which has thruster channels 0-" + str(count - 1) + "; no thruster node launched")
            return False
        self.logOnChange('channels:' + board['key'], None, warn = False)
        return True


    def checkServosFit(self, config, identity, board):
        board_str = "Teensy " + str(board['serial_number']) + " on " + board['path']
        if identity['protocol_minor'] < self.SERVO_PROTOCOL_MINOR or identity['servo_count'] == 0:
            self.logOnChange('servos:' + board['key'], board_str + " runs protocol " + identity['protocol_version'] +
                             ", which has no servo commands; reflash it with firmware 1.1 or later. No servo nodes launched")
            return False
        if config['servo_count'] > identity['servo_count']:
            self.logOnChange('servos:' + board['key'], "servo_count " + str(config['servo_count']) + " is more than the " +
                             str(identity['servo_count']) + " servo channels of the " + board_str +
                             "; no servo nodes launched")
            return False
        self.logOnChange('servos:' + board['key'], None, warn = False)
        return True


    def findBoards(self):
        # Every port with the Teensyduino "Serial" IDs. Returns [boards, ok];
        # ok is False only when enumeration itself failed.
        boards = []
        try:
            for p in list_ports.comports():
                if getattr(p, 'vid', None) != self.TEENSY_VENDOR_ID:
                    continue
                if getattr(p, 'pid', None) != self.TEENSY_SERIAL_PRODUCT_ID:
                    continue
                serial_number = getattr(p, 'serial_number', None)
                if serial_number is None or str(serial_number).strip() == '':
                    # Every Teensy reports one; fall back to the port so a
                    # board without one is still usable, just not stable.
                    key = p.device
                    serial_number = 'Unknown'
                else:
                    serial_number = str(serial_number).strip()
                    key = serial_number
                boards.append({'key': key, 'path': p.device, 'serial_number': serial_number})
        except Exception as e:
            self.logger.log_warn("Failed to enumerate serial ports: " + str(e))
            return [], False
        return boards, True


    def probeBoard(self, board):
        # Send the identify command and parse the reply (PROTOCOL.md 6.1).
        # Called only for a board with no running node. Returns the identity
        # dict for a compatible teensy_thrusters board, otherwise None (and
        # schedules the next probe).
        key = board['key']
        path_str = board['path']
        reply = None
        port = None
        try:
            # exclusive holds the port's flock for the whole probe, so the probe
            # can never interleave with a node's transaction (PROTOCOL.md 2.1).
            port = serial.Serial(path_str, self.PROBE_BAUD, timeout = 0.1,
                                 write_timeout = 0.5, exclusive = True)
            port.reset_input_buffer()
            port.write(encodeFrame('ID'))
            port.flush()
            deadline = time.time() + self.PROBE_REPLY_TIMEOUT_SEC
            while time.time() < deadline:
                line = port.readline()
                if line is None or len(line) == 0:
                    continue
                fields = decodeFrame(line)
                if fields is not None and fields[0] == 'ID':
                    reply = fields
                    break
        except Exception as e:
            self.logOnChange('probe:' + key, "Could not probe Teensy " + str(board['serial_number']) +
                             " on " + path_str + ": " + str(e))
        finally:
            if port is not None:
                try:
                    port.close()
                except Exception:
                    pass

        board_str = "Teensy " + str(board['serial_number']) + " on " + path_str
        if reply is None:
            self.logOnChange('probe:' + key, board_str + " gave no identify reply; not teensy_thrusters " +
                             "firmware, or still booting. Retrying in " + str(self.PROBE_RETRY_SEC) + "s")
            self.probe_after_dict[key] = time.time() + self.PROBE_RETRY_SEC
            return None

        identity = parseIdentify(reply)
        if identity is None or identity['firmware_name'] != self.FIRMWARE_NAME:
            self.logOnChange('probe:' + key, board_str + " runs other firmware (" + str(reply[1:2]) +
                             "); ignoring it, rechecking every " + str(self.FOREIGN_RECHECK_SEC) + "s")
            self.probe_after_dict[key] = time.time() + self.FOREIGN_RECHECK_SEC
            return None
        if identity['protocol_major'] != self.PROTOCOL_MAJOR:
            self.logOnChange('probe:' + key, board_str + " speaks protocol " + identity['protocol_version'] +
                             "; this driver speaks major version " + str(self.PROTOCOL_MAJOR) +
                             ". Reflash the firmware or update the driver. Rechecking every " +
                             str(self.FOREIGN_RECHECK_SEC) + "s")
            self.probe_after_dict[key] = time.time() + self.FOREIGN_RECHECK_SEC
            return None
        if board['serial_number'] not in ['Unknown', identity['serial_number']]:
            # Informational only: both come from the same fuse, so a mismatch
            # means a firmware or core change worth knowing about.
            self.logOnChange('serial:' + key, board_str + " reports serial " + identity['serial_number'] +
                             " in its identify reply")
        self.logOnChange('probe:' + key, board_str + " identified: firmware " + identity['firmware_version'] +
                         ", protocol " + identity['protocol_version'] + ", " +
                         str(identity['thruster_count']) + " thruster channels, " +
                         str(identity['servo_count']) + " servo channels", warn = False)
        return identity


    def boardIdString(self, board):
        # The serial number is stable across reboots and re-enumeration; the
        # port basename is not. Node names must be plain identifiers.
        if board['serial_number'] == 'Unknown':
            board_id = board['path'].split('/')[-1]
        else:
            board_id = board['serial_number']
        return ''.join(c if (c.isalnum() or c == '_') else '_' for c in board_id)


    def launchDeviceNode(self, launch_id, board, identity, part, servo_index, config):
        key = board['key']
        path_str = board['path']
        node_file = self.drv_dict['NODE_DICT']['file_name']
        device_name = self.node_launch_name + "_" + self.boardIdString(board)
        if part == self.SERVOS:
            # The servo node is the thruster node's sibling file. Derived from
            # NODE_DICT so a setup_new_driver.sh rename keeps the pair together.
            node_file = node_file.replace('_node.py', '_servo_node.py')
            device_name = device_name + "_servo" + str(servo_index)
        node_name = nepi_system.get_device_alias(device_name)
        self.logger.log_info("Launching node: " + node_name + " on " + path_str)

        # Deep-copied so a launch never mutates the drv_dict drivers_mgr owns.
        node_drv_dict = copy.deepcopy(self.drv_dict)
        device_dict = {
            'device_name': device_name,
            'device_path': path_str,
            'serial_number': board['serial_number'],
            'firmware_version': identity['firmware_version'],
            'protocol_version': identity['protocol_version'],
            'thruster_count': identity['thruster_count'],
            'servo_count': identity['servo_count'],
        }
        if part == self.SERVOS:
            # Only board-level servo options. The servo's own calibration is the
            # node's settings, persisted per device, and discovery must not
            # overwrite an operator's calibration on every relaunch.
            device_dict['servo_index'] = servo_index
            device_dict['servo_config'] = copy.deepcopy(config)
        else:
            device_dict['thruster_config'] = copy.deepcopy(config)
        node_drv_dict['DEVICE_DICT'] = device_dict
        dict_param_name = nepi_sdk.create_namespace(self.base_namespace, node_name + "/drv_dict")
        nepi_sdk.set_param(dict_param_name, node_drv_dict)

        [success, msg, sub_process] = nepi_drvs.launchDriverNode(node_file, node_name, device_path = path_str)
        if success:
            self.active_devices_dict[launch_id] = {
                'node_name': node_name,
                'sub_process': sub_process,
                'path': path_str,
                'board_key': key,
                'part': part,
                'config': copy.deepcopy(config)
            }
            if path_str not in self.active_paths_list:
                self.active_paths_list.append(path_str)
            self.logger.log_info("Launched node: " + node_name)
        else:
            self.logger.log_warn("Failed to launch node: " + node_name + " with msg: " + str(msg))
            self.exit_times_dict[launch_id] = time.time()
            if self.retry == False:
                self.dont_retry_list.append(key)
        return success


    def killDevice(self, launch_id):
        if launch_id not in self.active_devices_dict:
            return
        entry = self.active_devices_dict[launch_id]
        self.logger.log_info("Killing node: " + entry['node_name'])
        nepi_drvs.killDriverNode(entry['node_name'], entry['sub_process'])
        del self.active_devices_dict[launch_id]
        # Release the board's path only once none of its nodes still uses it.
        still_active = any(e['path'] == entry['path'] for e in self.active_devices_dict.values())
        if not still_active and entry['path'] in self.active_paths_list:
            self.active_paths_list.remove(entry['path'])


    def killAllDevices(self, active_paths_list):
        for launch_id in list(self.active_devices_dict.keys()):
            entry = self.active_devices_dict[launch_id]
            if self.retry == False and entry['board_key'] not in self.dont_retry_list:
                self.dont_retry_list.append(entry['board_key'])
            nepi_drvs.killDriverNode(entry['node_name'], entry['sub_process'])
            if entry['path'] in active_paths_list:
                active_paths_list.remove(entry['path'])
            if entry['path'] in self.active_paths_list:
                self.active_paths_list.remove(entry['path'])
        self.active_devices_dict = dict()
        self.identity_dict = dict()
        self.exit_times_dict = dict()
        self.probe_after_dict = dict()
        nepi_sdk.sleep(1)
        return active_paths_list


##########################
# Protocol helpers (PROTOCOL.md 3 and 6.1). The two nodes carry identical
# copies of encodeFrame/decodeFrame -- drivers install flat into one folder and
# the discovery runs inside drivers_mgr, so the files share no module. Keep the
# copies in step.

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


def parseIdentify(fields):
    # $ID,<name>,<protocol>,<firmware>,<thrusters>,<servo_channel>,<serial>[,<servo_count>].
    # <servo_count> arrived in 1.1; a 1.0 reply without it means no servos.
    # Fields beyond these are ignored: a later minor version may append some.
    if len(fields) < 7 or fields[0] != 'ID':
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
    SvxTeensyThrustersDiscovery()
