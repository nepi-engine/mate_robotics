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
# generates the servo pulses for Blue Robotics Basic ESCs driving T200
# thrusters. The Pi-to-Teensy protocol is mate_robotics
# Firmware/teensy_thrusters/PROTOCOL.md; section numbers below refer to it.
#
# CALL model, same contract as svx_servo_maestro_discovery.py and the SVX
# template: drivers_mgr instantiates this class once with no arguments and
# calls discoveryFunction() every poll cycle inside its own process, so there
# is no init_node()/spin() here.
#
# WHAT IS DIFFERENT FROM THE MAESTRO:
#   * One node per BOARD, not per channel. The node registers a MotorsDeviceIF
#     with one motor per thruster, because the thrusters are one vehicle's
#     propulsion and the Teensy keeps them on one watchdog.
#   * The USB IDs do not identify the firmware. Every Teensyduino "Serial"
#     sketch enumerates as VID 0x16C0 / PID 0x0483, so each candidate port is
#     opened and asked to identify (PROTOCOL.md 6.1). Only a board that answers
#     as teensy_thrusters with the protocol major this driver speaks gets a node.
#   * Boards are keyed by their USB serial number, not their /dev/ttyACM index,
#     so a board that re-enumerates on a different ttyACM after a USB reset is
#     the same device with the same node name and saved config -- never a
#     second node.
#   * Every OPTION is parsed and validated here, once, and handed to the node
#     in DEVICE_DICT['thruster_config']. An invalid option launches nothing
#     and is logged; an option change restarts the node so the new
#     configuration is sent to the Teensy and re-armed.
#
# SAFETY: discovery only ever sends the identify command. It never sends a
# thrust (SA) frame. Opening and closing a port toggles DTR, which the
# firmware answers by holding every thruster at stop (PROTOCOL.md 9).

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

    node_launch_name = "teensy_thrusters"

    # board key (USB serial number) -> {node_name, sub_process, path, config}
    active_devices_dict = dict()
    active_paths_list = []
    dont_retry_list = []
    # board key -> earliest time it may be probed again
    probe_after_dict = dict()
    # board key -> time its node was found to have exited on its own
    exit_times_dict = dict()
    # key -> last message logged, for messages logged only when they change
    last_log_dict = dict()

    retry = True


    ################################################
    def __init__(self):
        self.log_name = PKG_NAME.lower() + "_discovery"
        self.logger = nepi_sdk.logger(log_name = self.log_name)
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
        # Resolve and validate every option. None means invalid, already logged.
        options = dict()
        try:
            options = self.drv_dict.get('DISCOVERY_DICT', {}).get('OPTIONS', {})
        except Exception as e:
            self.logger.log_warn(self.log_name + ": Failed to read options " + str(e))
        config = self.resolveConfig(options)

        # Retry behavior
        self.retry = retry_enabled
        if self.retry == True:
            self.dont_retry_list = []

        ### Purge nodes whose port has disappeared from /dev
        for key in list(self.active_devices_dict.keys()):
            if self.active_devices_dict[key]['path'] not in self.available_paths_list:
                self.killDevice(key)

        ### Reap nodes that exited on their own. The node exits when it loses its
        ### serial link; without this its stale entry would block the relaunch.
        for key in list(self.active_devices_dict.keys()):
            sub_process = self.active_devices_dict[key]['sub_process']
            if sub_process is not None and sub_process.poll() is not None:
                self.logger.log_warn("Node " + self.active_devices_dict[key]['node_name'] +
                                     " exited; relaunching in " + str(self.RELAUNCH_DELAY_SEC) +
                                     "s if its board is still present")
                self.exit_times_dict[key] = time.time()
                self.killDevice(key)

        ### An option change restarts the node, so the new configuration reaches
        ### the Teensy (CF) and the ESCs are re-armed. An INVALID option change
        ### leaves running nodes alone: a typo in the RUI must not stop the
        ### thrusters, and the running node keeps its last valid configuration.
        if config is not None:
            for key in list(self.active_devices_dict.keys()):
                if self.active_devices_dict[key]['config'] != config:
                    self.logger.log_warn("Driver options changed; restarting node " +
                                         self.active_devices_dict[key]['node_name'])
                    self.killDevice(key)

        ### Enumerate Teensy serial ports
        [boards, enumerate_ok] = self.findBoards()
        if enumerate_ok == False:
            # Enumeration failed, which is not the same as finding no boards.
            return self.active_paths_list

        if config is None:
            return self.active_paths_list

        ### Probe and launch every board that is not running yet
        now = time.time()
        for board in boards:
            key = board['key']
            path_str = board['path']
            if key in self.active_devices_dict:
                continue
            if key in self.dont_retry_list:
                continue
            if path_str in self.active_paths_list:
                # Claimed by another driver's node; never open a port in use.
                continue
            if now < self.probe_after_dict.get(key, 0):
                continue
            if key in self.exit_times_dict:
                if now - self.exit_times_dict[key] < self.RELAUNCH_DELAY_SEC:
                    continue
                del self.exit_times_dict[key]

            identity = self.probeBoard(board)
            if identity is None:
                continue
            if self.checkChannelsFit(config, identity, board) == False:
                self.probe_after_dict[key] = time.time() + self.PROBE_RETRY_SEC
                continue
            self.launchDeviceNode(board, identity, config)

        return self.active_paths_list
    ################################################


    ##########  Device specific calls

    def logOnChange(self, key, msg, warn = True):
        # Discovery runs every few seconds, so a standing condition is logged once
        # when it appears or changes rather than on every pass.
        if self.last_log_dict.get(key) == msg:
            return
        self.last_log_dict[key] = msg
        if warn == True:
            self.logger.log_warn(msg)
        else:
            self.logger.log_info(msg)


    def resolveConfig(self, options):
        # Turn the OPTIONS into the validated configuration the node uses, or
        # None when any option is invalid. drivers_mgr stores every value as a
        # string and writes operator edits without range checks, so every
        # value is parsed and range-checked here.
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
            self.logOnChange('config', "Invalid driver options, no thruster node launched: " + str(e))
            return None
        self.logOnChange('config', "Driver options: " + str(config), warn = False)
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


    def checkChannelsFit(self, config, identity, board):
        count = identity['thruster_count']
        bad = [c for c in config['thruster_channels'] if c >= count]
        if len(bad) > 0:
            self.logOnChange('channels:' + board['key'], "thruster_channels " + str(bad) +
                             " out of range for the Teensy on " + board['path'] +
                             ", which has thruster channels 0-" + str(count - 1) + "; no node launched")
            return False
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
        # Returns the identity dict for a compatible teensy_thrusters board,
        # otherwise None (and schedules the next probe).
        key = board['key']
        path_str = board['path']
        reply = None
        port = None
        try:
            # exclusive: never share a port with a process that holds it.
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
                         str(identity['thruster_count']) + " thruster channels", warn = False)
        return identity


    def boardIdString(self, board):
        # The serial number is stable across reboots and re-enumeration; the
        # port basename is not. Node names must be plain identifiers.
        if board['serial_number'] == 'Unknown':
            board_id = board['path'].split('/')[-1]
        else:
            board_id = board['serial_number']
        return ''.join(c if (c.isalnum() or c == '_') else '_' for c in board_id)


    def launchDeviceNode(self, board, identity, config):
        key = board['key']
        path_str = board['path']
        file_name = self.drv_dict['NODE_DICT']['file_name']
        device_name = self.node_launch_name + "_" + self.boardIdString(board)
        node_name = nepi_system.get_device_alias(device_name)
        self.logger.log_info("Launching node: " + node_name + " on " + path_str)

        # Deep-copied so a launch never mutates the drv_dict drivers_mgr owns.
        # The node reads its configuration from DEVICE_DICT['thruster_config'],
        # which is the validated form of the OPTIONS above.
        node_drv_dict = copy.deepcopy(self.drv_dict)
        node_drv_dict['DEVICE_DICT'] = {
            'device_name': device_name,
            'device_path': path_str,
            'serial_number': board['serial_number'],
            'firmware_version': identity['firmware_version'],
            'protocol_version': identity['protocol_version'],
            'thruster_count': identity['thruster_count'],
            'servo_channel': identity['servo_channel'],
            'thruster_config': copy.deepcopy(config)
        }
        dict_param_name = nepi_sdk.create_namespace(self.base_namespace, node_name + "/drv_dict")
        nepi_sdk.set_param(dict_param_name, node_drv_dict)

        [success, msg, sub_process] = nepi_drvs.launchDriverNode(file_name, node_name, device_path = path_str)
        if success:
            self.active_devices_dict[key] = {
                'node_name': node_name,
                'sub_process': sub_process,
                'path': path_str,
                'config': copy.deepcopy(config)
            }
            if path_str not in self.active_paths_list:
                self.active_paths_list.append(path_str)
            self.logger.log_info("Launched node: " + node_name)
        else:
            self.logger.log_warn("Failed to launch node: " + node_name + " with msg: " + str(msg))
            self.probe_after_dict[key] = time.time() + self.RELAUNCH_DELAY_SEC
            if self.retry == False:
                self.dont_retry_list.append(key)
        return success


    def killDevice(self, key):
        if key not in self.active_devices_dict:
            return
        entry = self.active_devices_dict[key]
        self.logger.log_info("Killing node: " + entry['node_name'])
        nepi_drvs.killDriverNode(entry['node_name'], entry['sub_process'])
        del self.active_devices_dict[key]
        if entry['path'] in self.active_paths_list:
            self.active_paths_list.remove(entry['path'])


    def killAllDevices(self, active_paths_list):
        for key in list(self.active_devices_dict.keys()):
            entry = self.active_devices_dict[key]
            if self.retry == False:
                self.dont_retry_list.append(key)
            nepi_drvs.killDriverNode(entry['node_name'], entry['sub_process'])
            if entry['path'] in active_paths_list:
                active_paths_list.remove(entry['path'])
            if entry['path'] in self.active_paths_list:
                self.active_paths_list.remove(entry['path'])
        self.active_devices_dict = dict()
        self.exit_times_dict = dict()
        self.probe_after_dict = dict()
        nepi_sdk.sleep(1)
        return active_paths_list


##########################
# Protocol helpers (PROTOCOL.md 3 and 6.1). The node carries an identical copy
# of encodeFrame/decodeFrame -- drivers install flat into one folder and the
# discovery runs inside drivers_mgr, so the two files share no module. Keep the
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
    # $ID,<name>,<protocol>,<firmware>,<thrusters>,<servo_channel>,<serial>.
    # Fields beyond these are ignored: a later minor version may append some.
    if len(fields) < 7 or fields[0] != 'ID':
        return None
    try:
        protocol_major = int(fields[2].split('.')[0])
        return dict(firmware_name = fields[1],
                    protocol_version = fields[2],
                    protocol_major = protocol_major,
                    firmware_version = fields[3],
                    thruster_count = int(fields[4]),
                    servo_channel = int(fields[5]),
                    serial_number = fields[6])
    except Exception:
        return None


if __name__ == '__main__':
    SvxTeensyThrustersDiscovery()
