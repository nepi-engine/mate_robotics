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

# Thruster node for a PJRC Teensy 4.1 running the teensy_thrusters firmware.
# The Teensy generates the servo pulses for Blue Robotics Basic ESCs, each
# driving one T200 thruster. The Pi-to-Teensy protocol is mate_robotics
# Firmware/teensy_thrusters/PROTOCOL.md; section numbers below refer to it.
#
# Structure mirrors svx_servo_maestro_node.py (init, drv_dict read, serial lock,
# device_info_dict, exit-on-link-loss), but this node registers a
# MotorsDeviceIF -- one motor per thruster -- not an SVXActuatorIF. It lives in
# svx_drivers only because deploy_nepi_drivers.sh installs category folders.
# MotorsDeviceIF takes no settings functions, so every configuration choice is
# a driver OPTION, resolved by discovery into DEVICE_DICT['thruster_config'].
#
# SHARED PORT: the board's servos are separate nodes
# (svx_teensy_thrusters_servo_node.py, one per servo) on the same serial port.
# Every transaction holds an advisory flock on the port and only accepts the
# reply that names the command it sent (PROTOCOL.md 2.1). This node exiting
# does not drop DTR while a servo node still holds the port, so the Teensy
# watchdog -- not DTR -- is what stops the thrusters in that case.
#
# COMMAND SEMANTICS (from connect_device_if_motor.py, the consumer side):
#   set_speed(ratio)      ratio 0.0 (off) to 1.0 (max). A non-zero ratio runs
#                         the thruster in its set direction; 0.0 stops it. The
#                         in-repo consumers (custom_robot's RBX, wpilib_if)
#                         drive with set_speed alone, so it must move.
#   set_direction(+1/-1)  sets the direction; takes effect at once if running.
#                         +1 is the ESC's forward pulse range (above stop).
#   go_direction()        runs "in its currently set direction at its set
#                         speed until a stop_motor command is issued".
#   stop_motor()          stops; the set speed and direction are kept, so a
#                         later go_direction resumes at them.
# motor_name is "motor_0".."motor_N" (N+1 thrusters, in thruster_channels
# order) or "all".
#
# SAFETY LAYERS, outermost first:
#   1. Teensy watchdog (firmware): no SA for watchdog_ms -> all stop. The only
#      stop if this node or the Pi dies (PROTOCOL.md 8).
#   2. Keepalive (here): SA with the full command set every watchdog_ms / 4,
#      whether or not anything changed.
#   3. Per-thruster command timeout (here): a running thruster that has had no
#      MotorCommand for command_timeout_s is stopped. MotorsDeviceIF has no
#      keepalive of its own, so this is what stops a thruster whose
#      controller went quiet while this node stayed up.
#   4. Arm hold (firmware AR): at startup and after a Teensy reboot, all stop
#      for arming_hold_s while the ESCs initialize.
#
# NO FEEDBACK: the ESCs and T200s report nothing, so motor_max_speed,
# motor_speed and motor_position are always 0.0, and motor_speed_ratio is the
# ratio being commanded, not a measurement.

import copy
import functools
import math
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

from nepi_api.device_if_motor import MotorsDeviceIF
from nepi_api.messages_if import MsgIF

PKG_NAME = 'SVX_TEENSY_THRUSTERS' # Use in display menus
FILE_TYPE = 'NODE'


class SvxTeensyThrustersNode:

    # Must match the firmware's ID reply (PROTOCOL.md 6.1 and 11).
    FIRMWARE_NAME = 'teensy_thrusters'
    PROTOCOL_MAJOR = 1

    # esc_model -> pulse parameters (Docs/Motors/bluerobotics_basic_esc.md: the
    # documented 1100/1500/1900 us, not the internal 1112/1512/1912, and a
    # 1500 +/- 25 us deadband). Mirrors the table discovery validates with.
    ESC_MODELS = {
        'BlueRobotics Basic ESC (BESC30-R3)': dict(pulse_min_us = 1100, pulse_stop_us = 1500,
                                                   pulse_max_us = 1900, deadband_us = 25),
    }
    CUSTOM_ESC_MODEL = 'Custom'

    # Any rate but 134, which reboots a running Teensy into its bootloader.
    SERIAL_BAUD = 115200
    REPLY_TIMEOUT_SEC = 0.1
    # Keepalive period = watchdog_ms / 4, held to this range (PROTOCOL.md 6.2).
    KEEPALIVE_MIN_SEC = 0.02
    KEEPALIVE_MAX_SEC = 0.1
    # ST poll interval: the rate MotorsDeviceIF publishes MotorsStatus at.
    STATUS_INTERVAL_SEC = 0.5
    # This many transactions in a row with no reply is treated as a lost link
    # (a hung firmware keeps USB up but stops answering).
    MAX_MISSED_REPLIES = 10

    device_info_dict = dict(device_name = "",
                            path = "",
                            serial_number = "",
                            hw_version = "",
                            sw_version = "")

    serial_num = "Unknown"
    hw_version = "Unknown"
    sw_version = "Unknown"
    motors_if = None

    # Board link
    serial_port = None
    connected = False
    missed_replies = 0
    last_status_time = 0.0
    last_rx_errors = None

    # Teensy state from the last ST reply (PROTOCOL.md 6.6)
    teensy_state = 'IDLE'

    drv_dict = dict()


    ################################################
    DEFAULT_NODE_NAME = PKG_NAME.lower() + "_node"

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

        # serial_lock serializes transactions on the port (keepalive timer,
        # startup, shutdown). state_lock guards the per-thruster command state,
        # which the MotorCommand callbacks, the status publisher and the
        # keepalive timer all touch from different threads.
        self.serial_lock = threading.Lock()
        self.state_lock = threading.Lock()

        ##############################
        # Get required drv driver dict info
        self.drv_dict = nepi_sdk.get_param('~drv_dict',dict())
        try:
            device_dict = self.drv_dict['DEVICE_DICT']
            self.device_name = device_dict['device_name']
            self.device_path = device_dict['device_path']
            self.port_str = device_dict['device_path']
            self.serial_num = str(device_dict.get('serial_number', 'Unknown'))
            self.thruster_count = int(device_dict['thruster_count'])
            self.config = copy.deepcopy(device_dict['thruster_config'])
        except Exception as e:
            self.msg_if.pub_warn("Failed to load Device Dict " + str(e))
            nepi_sdk.signal_shutdown(self.node_name + ": Shutting down because no valid Device Dict")
            return

        ##############################
        # Resolve the configuration discovery validated
        try:
            self.resolveConfig()
        except Exception as e:
            self.msg_if.pub_warn("Invalid thruster configuration, shutting down: " + str(e))
            nepi_sdk.signal_shutdown(self.node_name + ": invalid thruster configuration")
            return

        # Per-thruster command state. motor_i drives Teensy channel
        # thruster_channels[i].
        self.motor_names = ['motor_' + str(i) for i in range(len(self.thruster_channels))]
        self.motors = []
        for i in range(len(self.thruster_channels)):
            self.motors.append(dict(speed_ratio = 0.0,   # last set_speed, 0.0-1.0
                                    direction = 1,       # last set_direction, +1/-1
                                    running = False,     # set_speed > 0 or go_direction, until stop
                                    last_cmd_time = 0.0, # last MotorCommand for this thruster
                                    pulse_us = self.stop_us))
        self.clamp_warned = set()

        ################################################
        self.msg_if.pub_info("Connecting to Teensy " + self.serial_num + " on " + str(self.port_str))
        self.connected = self.connect()
        if self.connected == False:
            self.msg_if.pub_info("Shutting down node, unable to connect to the teensy_thrusters firmware")
            nepi_sdk.signal_shutdown("Unable to connect to thruster controller")
            return
        self.msg_if.pub_info("... Connected!")

        self.device_info_dict["device_name"] = self.device_name
        self.device_info_dict["path"] = self.device_path
        self.device_info_dict["serial_number"] = self.serial_num
        self.device_info_dict["hw_version"] = self.hw_version
        self.device_info_dict["sw_version"] = self.sw_version

        # Thrusters cannot steer, so both rotation callbacks are None and every
        # motor reports has_rotation False. MotorsDeviceIF still registers the
        # rotation topics and ignores them.
        self.msg_if.pub_info("Launching NEPI Motors interface...")
        self.motors_if = MotorsDeviceIF(device_info = self.device_info_dict,
                                        data_source_description = 'thrusters',
                                        data_ref_description = 'vehicle',
                                        getMotorsStatusFunction = self.getMotorsStatus,
                                        setSpeedFunction = self.setSpeed,
                                        setDirectionFunction = self.setDirection,
                                        goDirectionFunction = self.goDirection,
                                        stopMotorFunction = self.stopMotor,
                                        gotoRotationFunction = None,
                                        setRotationSpeedFunction = None,
                                        msg_if = self.msg_if
                                        )
        self.msg_if.pub_info(" ... Motors interface running")

        # The keepalive runs for the life of the node: it is what keeps the
        # Teensy watchdog fed while the thrusters are meant to run.
        nepi_sdk.start_timer_process(self.keepalive_sec, self.keepaliveCb)

        # Initialization Complete
        self.msg_if.pub_info("Initialization Complete")
        #Set up node shutdown
        nepi_sdk.on_shutdown(self.cleanup_actions)
        # Spin forever
        nepi_sdk.spin()


    #######################
    ### Configuration

    def resolveConfig(self):
        # Effective pulse parameters from esc_model, or from the custom_*
        # options when esc_model is Custom. Discovery has already range-checked
        # every value; the relationships are re-checked here because the
        # pulse math below divides the span either side of the deadband.
        cfg = self.config
        esc_model = str(cfg['esc_model'])
        if esc_model in self.ESC_MODELS:
            pulses = self.ESC_MODELS[esc_model]
            self.min_us = int(pulses['pulse_min_us'])
            self.stop_us = int(pulses['pulse_stop_us'])
            self.max_us = int(pulses['pulse_max_us'])
            self.deadband_us = int(pulses['deadband_us'])
        elif esc_model == self.CUSTOM_ESC_MODEL:
            self.min_us = int(cfg['custom_pulse_min_us'])
            self.stop_us = int(cfg['custom_pulse_stop_us'])
            self.max_us = int(cfg['custom_pulse_max_us'])
            self.deadband_us = int(cfg['custom_deadband_us'])
        else:
            raise ValueError("unknown esc_model '" + esc_model + "'")
        if not (self.min_us < self.stop_us - self.deadband_us and
                self.stop_us + self.deadband_us < self.max_us):
            raise ValueError("pulse widths " + str([self.min_us, self.stop_us, self.max_us]) +
                             " with deadband " + str(self.deadband_us) + " leave no drive range")

        self.esc_model = esc_model
        self.thruster_model = str(cfg['thruster_model'])
        self.thruster_channels = [int(c) for c in cfg['thruster_channels']]
        self.reversed_thrusters = [int(i) for i in cfg['reversed_thrusters']]
        self.max_thrust_ratio = float(cfg['max_thrust_ratio'])
        self.pulse_rate_hz = int(cfg['pulse_rate_hz'])
        self.watchdog_ms = int(cfg['watchdog_ms'])
        self.arming_hold_s = int(cfg['arming_hold_s'])
        self.command_timeout_s = float(cfg['command_timeout_s'])

        if len(self.thruster_channels) == 0:
            raise ValueError("no thruster channels")
        for channel in self.thruster_channels:
            if channel < 0 or channel >= self.thruster_count:
                raise ValueError("thruster channel " + str(channel) + " is not 0-" + str(self.thruster_count - 1))

        self.channel_mask = 0
        for channel in self.thruster_channels:
            self.channel_mask |= (1 << channel)
        self.keepalive_sec = min(self.KEEPALIVE_MAX_SEC,
                                 max(self.KEEPALIVE_MIN_SEC, self.watchdog_ms / 4000.0))


    #######################
    ### Pulse mapping

    def ratioToPulse(self, index, ratio, direction):
        # Stop is stop_us. A non-zero command is stop_us +/- (deadband_us + ratio
        # x the span from the deadband edge to max_us or min_us), so the
        # smallest non-zero command already clears the ESC deadband.
        # max_thrust_ratio caps the ratio BEFORE the map; the reversed_thrusters
        # mirror about stop_us is applied AFTER it.
        ratio = min(float(ratio), self.max_thrust_ratio)
        if ratio <= 0.0:
            return self.stop_us
        if direction >= 0:
            pulse = self.stop_us + self.deadband_us + ratio * (self.max_us - self.stop_us - self.deadband_us)
        else:
            pulse = self.stop_us - self.deadband_us - ratio * (self.stop_us - self.min_us - self.deadband_us)
        if index in self.reversed_thrusters:
            pulse = 2 * self.stop_us - pulse
        pulse = int(round(pulse))
        # Only a Custom ESC whose stop is off-center can mirror past an end.
        # Limit the magnitude to the end it passed: the direction stays as
        # commanded, and the Teensy would otherwise reject the whole frame.
        if pulse < self.min_us or pulse > self.max_us:
            if index not in self.clamp_warned:
                self.clamp_warned.add(index)
                self.msg_if.pub_warn("Reversed " + self.motor_names[index] + " mirrors outside " +
                                     str(self.min_us) + "-" + str(self.max_us) + " us with this Custom ESC; " +
                                     "limiting it to the range end")
            pulse = max(self.min_us, min(self.max_us, pulse))
        return pulse


    #######################
    ### MotorsDeviceIF callbacks

    def targetIndexes(self, motor_name):
        name = str(motor_name)
        if name == 'all':
            return list(range(len(self.motor_names)))
        if name in self.motor_names:
            return [self.motor_names.index(name)]
        self.msg_if.pub_warn("Ignoring command for unknown motor '" + name + "' (have " +
                             str(self.motor_names) + " and 'all')", throttle_s = 5.0)
        return []

    def setSpeed(self, motor_name, speed_ratio):
        try:
            ratio = float(speed_ratio)
        except Exception:
            ratio = float('nan')
        if math.isnan(ratio) or ratio < 0.0 or ratio > 1.0:
            # Rejected, not clamped: a bad ratio is a caller bug, and clamping
            # it would turn that bug into thrust.
            self.msg_if.pub_warn("Ignoring set_speed " + str(speed_ratio) + " for " + str(motor_name) +
                                 ": speed_ratio must be 0.0-1.0", throttle_s = 5.0)
            return
        now = nepi_utils.get_time()
        with self.state_lock:
            for i in self.targetIndexes(motor_name):
                self.motors[i]['speed_ratio'] = ratio
                self.motors[i]['running'] = (ratio > 0.0)
                self.motors[i]['last_cmd_time'] = now

    def setDirection(self, motor_name, direction):
        # Same reduction as ConnectMotorsDeviceIF.set_direction: >= 0 is +1.
        new_dir = 1 if direction >= 0 else -1
        now = nepi_utils.get_time()
        with self.state_lock:
            for i in self.targetIndexes(motor_name):
                self.motors[i]['direction'] = new_dir
                self.motors[i]['last_cmd_time'] = now

    def goDirection(self, motor_name):
        now = nepi_utils.get_time()
        with self.state_lock:
            for i in self.targetIndexes(motor_name):
                self.motors[i]['running'] = True
                self.motors[i]['last_cmd_time'] = now

    def stopMotor(self, motor_name):
        now = nepi_utils.get_time()
        with self.state_lock:
            for i in self.targetIndexes(motor_name):
                self.motors[i]['running'] = False
                self.motors[i]['last_cmd_time'] = now

    def getMotorsStatus(self):
        # motor_enable is False while the Teensy holds every thruster at stop
        # for a reason of its own: arming, or a tripped watchdog.
        # No feedback exists (no ESC or T200 data link), so motor_max_speed,
        # motor_speed and motor_position are always 0.0, and motor_speed_ratio
        # is the ratio being commanded right now (0.0 when stopped), not a
        # measurement.
        enabled = (self.connected == True and self.teensy_state not in ['ARMING', 'TRIPPED'])
        motors_list = []
        with self.state_lock:
            for i in range(len(self.motors)):
                motor = self.motors[i]
                ratio = 0.0
                if enabled and motor['running']:
                    ratio = min(motor['speed_ratio'], self.max_thrust_ratio)
                motors_list.append(dict(motor_name = self.motor_names[i],
                                        motor_enable = enabled,
                                        motor_dir = motor['direction'],
                                        motor_max_speed = 0.0,
                                        motor_speed_ratio = ratio,
                                        motor_speed = 0.0,
                                        motor_position = 0.0))
        return motors_list


    #######################
    ### Keepalive and status

    def keepaliveCb(self, timer):
        if self.connected == False or nepi_sdk.is_shutdown():
            return
        now = nepi_utils.get_time()

        # Per-thruster command timeout, then the full command set.
        values = [self.stop_us] * self.thruster_count
        with self.state_lock:
            for i in range(len(self.motors)):
                motor = self.motors[i]
                if motor['running'] and (now - motor['last_cmd_time']) > self.command_timeout_s:
                    motor['running'] = False
                    self.msg_if.pub_warn("No command for " + self.motor_names[i] + " in " +
                                         str(self.command_timeout_s) + "s; stopping it")
                if motor['running']:
                    motor['pulse_us'] = self.ratioToPulse(i, motor['speed_ratio'], motor['direction'])
                else:
                    motor['pulse_us'] = self.stop_us
                values[self.thruster_channels[i]] = motor['pulse_us']

        # During an arm hold the Teensy refuses SA (ER,ARMING) and counts each
        # refusal in rx_errors, so skip SA until the ST poll shows the hold has
        # ended. The watchdog runs only in RUN, so this cannot trip it.
        reply = None
        if self.teensy_state != 'ARMING':
            reply = self.transact('SA,' + ','.join(str(v) for v in values))
        if reply is not None:
            if reply[:2] == ['ER', 'ARMING']:
                self.teensy_state = 'ARMING'
            elif reply[0] == 'ER':
                self.msg_if.pub_warn("Teensy rejected thrust frame: " + ','.join(reply[1:]), throttle_s = 5.0)
            elif reply[:2] == ['OK', 'SA']:
                self.teensy_state = 'RUN'

        if (now - self.last_status_time) >= self.STATUS_INTERVAL_SEC:
            self.last_status_time = now
            self.pollStatus()

    def pollStatus(self):
        reply = self.transact('ST')
        status = self.parseStatus(reply)
        if status is None:
            return
        self.teensy_state = status['state']
        if self.last_rx_errors is not None and status['rx_errors'] > self.last_rx_errors:
            self.msg_if.pub_warn("Teensy has rejected " + str(status['rx_errors'] - self.last_rx_errors) +
                                 " more frames (total " + str(status['rx_errors']) + ")", throttle_s = 10.0)
        self.last_rx_errors = status['rx_errors']
        if status['cfg'] == 0:
            # cfg is 0 only before the first CF since boot, and this node sent
            # one at connect: the Teensy rebooted. Its outputs dropped long
            # enough for the ESCs' ~0.3 s signal-loss stop, so configure and
            # re-arm it.
            self.msg_if.pub_warn("Teensy " + self.serial_num + " rebooted; reconfiguring and re-arming")
            self.configureAndArm()

    def parseStatus(self, fields):
        # $ST,<state>,<cfg>,<dtr>,<arm_ms_left>,<rate_hz>,<watchdog_ms>,<mask>,
        #     <min_us>,<stop_us>,<max_us>,<rx_errors>,<uptime_ms>,<p0>..<p7>
        if fields is None or fields[0] != 'ST' or len(fields) < 13 + self.thruster_count:
            return None
        try:
            return dict(state = fields[1],
                        cfg = int(fields[2]),
                        dtr = int(fields[3]),
                        arm_ms_left = int(fields[4]),
                        rate_hz = int(fields[5]),
                        watchdog_ms = int(fields[6]),
                        mask = int(fields[7], 16),
                        min_us = int(fields[8]),
                        stop_us = int(fields[9]),
                        max_us = int(fields[10]),
                        rx_errors = int(fields[11]),
                        uptime_ms = int(fields[12]),
                        pulses_us = [int(p) for p in fields[13:13 + self.thruster_count]])
        except Exception:
            return None


    #######################
    ### Connection

    def connect(self):
        try:
            # Not exclusive: pySerial's exclusive holds the port's flock for the
            # whole open, which would lock out the board's servo nodes. Each
            # transaction takes the flock instead (PROTOCOL.md 2.1), and
            # discovery never probes a board while any of its nodes runs.
            self.serial_port = openSharedPort(self.port_str, self.SERIAL_BAUD, self.REPLY_TIMEOUT_SEC)
        except Exception as e:
            self.msg_if.pub_warn("Failed to open serial port at: " + str(self.port_str) + " (" + str(e) + ")")
            self.serial_port = None
            return False

        identity = self.parseIdentify(self.transact('ID', connecting = True))
        if identity is None or identity['firmware_name'] != self.FIRMWARE_NAME:
            self.msg_if.pub_warn("No teensy_thrusters identify reply on " + str(self.port_str))
            self.closePort()
            return False
        if identity['protocol_major'] != self.PROTOCOL_MAJOR:
            self.msg_if.pub_warn("Teensy speaks protocol " + identity['protocol_version'] +
                                 "; this driver speaks major version " + str(self.PROTOCOL_MAJOR))
            self.closePort()
            return False
        if identity['thruster_count'] != self.thruster_count:
            self.msg_if.pub_warn("Teensy reports " + str(identity['thruster_count']) +
                                 " thruster channels; discovery saw " + str(self.thruster_count))
            self.closePort()
            return False

        self.hw_version = ("Teensy 4.1; ESC: " + self.esc_model + "; thruster: " + self.thruster_model)
        self.sw_version = ("teensy_thrusters " + identity['firmware_version'] +
                           " (protocol " + identity['protocol_version'] + ")")
        self.msg_if.pub_info("Firmware " + self.sw_version + "; " + self.hw_version + "; " +
                             str(len(self.thruster_channels)) + " thrusters on channels " +
                             str(self.thruster_channels) + "; pulses " +
                             str([self.min_us, self.stop_us, self.max_us]) + " us, deadband " +
                             str(self.deadband_us) + " us; reversed " + str(self.reversed_thrusters) +
                             "; max_thrust_ratio " + str(self.max_thrust_ratio) + "; " +
                             str(self.pulse_rate_hz) + " Hz; watchdog " + str(self.watchdog_ms) +
                             " ms; keepalive " + str(round(self.keepalive_sec * 1000)) + " ms")

        return self.configureAndArm(connecting = True)

    def configureAndArm(self, connecting = False):
        # CF then AR (PROTOCOL.md 6.5 and 6.4). Every thruster's run state is
        # cleared first, so nothing resumes on its own when the hold ends: a
        # thruster moves again only on a fresh command.
        with self.state_lock:
            for motor in self.motors:
                motor['running'] = False
        cf_body = 'CF,%d,%d,%02X,%d,%d,%d' % (self.pulse_rate_hz, self.watchdog_ms, self.channel_mask,
                                              self.min_us, self.stop_us, self.max_us)
        reply = self.transact(cf_body, connecting = connecting)
        if reply != ['OK', 'CF']:
            self.msg_if.pub_warn("Teensy rejected configuration " + cf_body + ": " + str(reply))
            if connecting:
                self.closePort()
            return False
        reply = self.transact('AR,%d' % (self.arming_hold_s * 1000), connecting = connecting)
        if reply != ['OK', 'AR']:
            self.msg_if.pub_warn("Teensy rejected arm hold: " + str(reply))
            if connecting:
                self.closePort()
            return False
        self.teensy_state = 'ARMING'
        self.msg_if.pub_info("Arming: holding every thruster at stop for " + str(self.arming_hold_s) + "s")
        return True

    def parseIdentify(self, fields):
        # $ID,<name>,<protocol>,<firmware>,<thrusters>,<servo_channel>,<serial>
        if fields is None or len(fields) < 7 or fields[0] != 'ID':
            return None
        try:
            return dict(firmware_name = fields[1],
                        protocol_version = fields[2],
                        protocol_major = int(fields[2].split('.')[0]),
                        firmware_version = fields[3],
                        thruster_count = int(fields[4]),
                        servo_channel = int(fields[5]),
                        serial_number = fields[6])
        except Exception:
            return None

    def transact(self, body, connecting = False):
        # Send one frame and return the reply's fields, or None. The protocol is
        # strictly request/reply. The in-process lock and the cross-process
        # flock together make this the only transaction on the port; the input
        # buffer is flushed first, and a reply that does not name this command
        # (a late reply to another node's timed-out request) is skipped.
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
        # on whatever path it finds. Closing the port drops DTR, which the
        # firmware answers with all-stop (PROTOCOL.md 9).
        if self.connected == False:
            return
        self.connected = False
        self.msg_if.pub_warn("Lost serial link to Teensy on " + str(self.port_str) + " (" + str(reason) +
                             "); shutting down so discovery can relaunch it")
        self.closePort()
        nepi_sdk.signal_shutdown("Lost serial link to thruster controller")

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
        """Stop every thruster and release the serial port on node shutdown.

        Sends the stop-all command, then closes the port. Closing drops DTR,
        which the firmware also answers with all-stop, so the thrusters stop
        even if the stop frame is lost.
        """
        self.msg_if.pub_info("Shutting down: Executing script cleanup actions")
        self.connected = False
        if self.serial_port is not None:
            try:
                self.transact('SP', connecting = True)
            except Exception:
                pass
        self.closePort()


##########################
# Protocol helpers (PROTOCOL.md 3 and 2.1). encodeFrame/decodeFrame are an
# identical copy of the discovery's, and replyMatches of the servo node's --
# drivers install flat into one folder and the discovery runs inside
# drivers_mgr, so the files share no module. Keep the copies in step.

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


if __name__ == '__main__':
    node = SvxTeensyThrustersNode()
