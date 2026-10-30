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

# NPX (navpose) node for a Blue Robotics Bar02 or Bar30 depth sensor wired to
# the I2C port of a PJRC Teensy 4.1 running the teensy_thrusters firmware
# (protocol 1.2 or later). The Pi-to-Teensy protocol is mate_robotics
# Firmware/teensy_thrusters/PROTOCOL.md; section numbers below refer to it.
# Sensor facts are from Docs/npx/bluerobotics_bar_depth_sensor.md ("the Bar
# doc").
#
# Launched by svx_teensy_thrusters_discovery.py when the driver's devices
# option includes depth_sensor: an NPX node from an SVX driver, because the
# sensor can only be reached through the Teensy's port. The board's thruster
# and servo nodes share that port: every transaction holds an advisory flock
# on the port and only accepts the reply that names the command it sent
# (PROTOCOL.md 2.1).
#
# SPLIT OF WORK: the Teensy samples the sensor in the background and hands back
# raw 24-bit readings (DV) and the factory calibration words (DP); this node
# does the conversion. The math differs per model, the model cannot be read
# from the chip, and here it is unit-tested against the datasheet examples
# (Drivers/tests/test_svx_teensy_thrusters_depth.py).
#
# WHAT IT PUBLISHES: depth through the NPXDeviceIF (has_depth only), and the
# compensated pressure and temperature as sensor_msgs/FluidPressure (pascals)
# and sensor_msgs/Temperature (deg C) on <node>/pressure and
# <node>/temperature, because the NavPose dict has no field for either.
#
# STALE DATA: until the first good reading the NavPose callback returns None,
# so NPXDeviceIF waits to create its NavPose publishers. After that a lost
# sensor keeps its last reading and timestamp, with has_depth False. Returning
# None there would make NPXDeviceIF publish a blank NavPose, which reads as
# depth 0 -- the surface -- the worst wrong value a depth reading can take.

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

from sensor_msgs.msg import FluidPressure, Temperature

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_nav

from nepi_api.device_if_npx import NPXDeviceIF
from nepi_api.messages_if import MsgIF

PKG_NAME = 'SVX_TEENSY_THRUSTERS' # Use in display menus
FILE_TYPE = 'NODE'

# Standard gravity, as Blue Robotics' libraries use (Bar doc, Depth equation)
GRAVITY_M_S2 = 9.80665

# Bar model -> its MS5837 variant's conversion constants (Bar doc, First-order
# calculation). The shifts are the powers of two in the datasheet formulas:
#   OFF  = C2 * 2^off_c2  + (C4 * dT) / 2^off_dt
#   SENS = C1 * 2^sens_c1 + (C3 * dT) / 2^sens_dt
#   P    = (D1 * SENS / 2^21 - OFF) / 2^p_shift, in 1/p_per_mbar mbar
# max_mbar is the top of the model's range (Bar doc, At a Glance). The keys
# are the depth_model option values; the discovery holds the same list.
BAR_MODELS = {
    'BlueRobotics Bar02 (MS5837-02BA)': dict(chip = '02BA', off_c2 = 17, off_dt = 6, sens_c1 = 16, sens_dt = 7,
                                             p_shift = 15, p_per_mbar = 100, max_mbar = 2000.0),
    'BlueRobotics Bar30 (MS5837-30BA)': dict(chip = '30BA', off_c2 = 16, off_dt = 7, sens_c1 = 15, sens_dt = 8,
                                             p_shift = 13, p_per_mbar = 10, max_mbar = 30000.0),
}
# The OSR values the firmware's DC command takes (PROTOCOL.md 6.12)
DEPTH_OSR_VALUES = [256, 512, 1024, 2048, 4096, 8192]
FLUID_DENSITY_KG_M3_RANGE = (950.0, 1100.0)
SURFACE_PRESSURE_MBAR_RANGE = (700.0, 1100.0)
# The startup model check accepts a first reading this close to
# surface_pressure_mbar, about 3 m of water. The wrong model reads about 20
# times too high or too low, far outside it.
MODEL_CHECK_WINDOW_MBAR = 300.0


def promCrc4(words):
    # The datasheet's CRC-4 over PROM words 0-6, line for line (Bar doc, CRC-4
    # check). Compare the result with the top four bits of word 0. Bits that
    # n_rem grows past 16 never shift back down, so Python's unbounded ints
    # give the C result.
    n_prom = [int(w) & 0xFFFF for w in words[:7]] + [0]
    n_prom[0] = n_prom[0] & 0x0FFF
    n_rem = 0
    for cnt in range(16):
        if cnt % 2 == 1:
            n_rem ^= (n_prom[cnt >> 1] & 0x00FF)
        else:
            n_rem ^= (n_prom[cnt >> 1] >> 8)
        for n_bit in range(8):
            if n_rem & 0x8000:
                n_rem = (n_rem << 1) ^ 0x3000
            else:
                n_rem = (n_rem << 1)
    return (n_rem >> 12) & 0x000F


def promValid(words):
    # True for seven calibration words that pass the CRC and are not blank: an
    # all-zero PROM passes the CRC, and 0 or 0xFFFF coefficients mean the bus
    # read nothing real. The firmware checks the same before it reports OK.
    if words is None or len(words) != 7:
        return False
    for w in words[1:]:
        if w <= 0 or w >= 0xFFFF:
            return False
    return promCrc4(words) == (words[0] >> 12)


def barFirstOrder(model, prom, d1, d2):
    # The datasheet's first-order values in its integer units. // floors, as
    # the datasheet's worked examples do; C's truncating / misses them by one.
    c = BAR_MODELS[model]
    dT = d2 - prom[5] * 2**8
    TEMP = 2000 + (dT * prom[6]) // 2**23
    OFF = prom[2] * 2**c['off_c2'] + (prom[4] * dT) // 2**c['off_dt']
    SENS = prom[1] * 2**c['sens_c1'] + (prom[3] * dT) // 2**c['sens_dt']
    P = (d1 * SENS // 2**21 - OFF) // 2**c['p_shift']
    return dict(dT = dT, TEMP = TEMP, OFF = OFF, SENS = SENS, P = P)


def barCompensate(model, prom, d1, d2):
    # [pressure_mbar, temperature_c] after the second-order compensation
    # (Bar doc, Second-order compensation).
    c = BAR_MODELS[model]
    first = barFirstOrder(model, prom, d1, d2)
    dT = first['dT']
    TEMP = first['TEMP']
    if c['chip'] == '30BA':
        if TEMP < 2000:
            Ti = (3 * dT * dT) // 2**33
            OFFi = (3 * (TEMP - 2000)**2) // 2**1
            SENSi = (5 * (TEMP - 2000)**2) // 2**3
            if TEMP < -1500:
                OFFi = OFFi + 7 * (TEMP + 1500)**2
                SENSi = SENSi + 4 * (TEMP + 1500)**2
        else:
            Ti = (2 * dT * dT) // 2**37
            OFFi = ((TEMP - 2000)**2) // 2**4
            SENSi = 0
    else:
        if TEMP < 2000:
            Ti = (11 * dT * dT) // 2**35
            OFFi = (31 * (TEMP - 2000)**2) // 2**3
            SENSi = (63 * (TEMP - 2000)**2) // 2**5
        else:
            Ti = 0
            OFFi = 0
            SENSi = 0
    OFF2 = first['OFF'] - OFFi
    SENS2 = first['SENS'] - SENSi
    pressure_mbar = ((d1 * SENS2 // 2**21 - OFF2) // 2**c['p_shift']) / float(c['p_per_mbar'])
    temperature_c = (TEMP - Ti) / 100.0
    return [pressure_mbar, temperature_c]


def depthFromPressure(pressure_mbar, surface_pressure_mbar, fluid_density_kg_m3):
    # Blue Robotics' depth equation, with the configured surface pressure in
    # place of the libraries' fixed 1013 mbar (Bar doc, Depth equation).
    return (pressure_mbar - surface_pressure_mbar) * 100.0 / (fluid_density_kg_m3 * GRAVITY_M_S2)


def modelCheckText(model, prom, d1, d2, surface_pressure_mbar, fluid_density_kg_m3):
    # A warning when a first reading suggests depth_model is wrong, else None.
    # In air or at the surface only the right model's math reads near
    # surface_pressure_mbar (Bar doc, Plausibility check). A sensor that
    # starts underwater gets a warning too, so the text says what it saw and
    # nothing is refused.
    pressure = barCompensate(model, prom, d1, d2)[0]
    if abs(pressure - surface_pressure_mbar) <= MODEL_CHECK_WINDOW_MBAR:
        return None
    for other in sorted(BAR_MODELS.keys()):
        if other == model:
            continue
        other_pressure = barCompensate(other, prom, d1, d2)[0]
        if abs(other_pressure - surface_pressure_mbar) <= MODEL_CHECK_WINDOW_MBAR:
            return ("First depth sensor reading is %.1f mbar as a %s, but %.1f mbar as a %s, near "
                    "surface_pressure_mbar %.1f. If the sensor is in air or at the surface, set depth_model "
                    "to %s" % (pressure, model, other_pressure, other, surface_pressure_mbar, other))
    depth = depthFromPressure(pressure, surface_pressure_mbar, fluid_density_kg_m3)
    return ("First depth sensor reading is %.1f mbar, %.2f m deep. If the sensor is in air or at the "
            "surface, check depth_model and surface_pressure_mbar" % (pressure, depth))


class SvxTeensyThrustersDepthNode:

    # Must match the firmware's ID reply (PROTOCOL.md 6.1 and 11).
    FIRMWARE_NAME = 'teensy_thrusters'
    PROTOCOL_MAJOR = 1
    DEPTH_PROTOCOL_MINOR = 2

    # Any rate but 134, which reboots a running Teensy into its bootloader.
    SERIAL_BAUD = 115200
    REPLY_TIMEOUT_SEC = 0.1
    # This many transactions in a row with no reply is treated as a lost link.
    MAX_MISSED_REPLIES = 10
    # DV poll period. The Teensy samples at most 50 times a second, so 20 Hz
    # gets a fresh sample on nearly every poll.
    POLL_INTERVAL_SEC = 0.05
    # A sample older than this is stale: has_depth goes False.
    STALE_SEC = 1.0
    # depth_m_per_sec is the depth change over this much of the latest
    # samples; two neighboring samples differ mostly by sensor noise.
    RATE_WINDOW_SEC = 0.5
    # Standing problems (no sensor, bad calibration) are repeated this rarely.
    WARN_THROTTLE_SEC = 10.0
    # The firmware reports SEARCHING from DC until its first probe finishes,
    # and re-probes once a second, so "no sensor" is not reported for this
    # long after sampling is (re)started.
    STARTUP_GRACE_SEC = 2.5

    device_info_dict = dict(device_name = "",
                            path = "",
                            serial_number = "",
                            hw_version = "",
                            sw_version = "")

    serial_num = "Unknown"
    hw_version = "Unknown"
    sw_version = "Unknown"
    npx_if = None

    # Board link
    serial_port = None
    connected = False
    missed_replies = 0

    drv_dict = dict()


    ################################################
    DEFAULT_NODE_NAME = PKG_NAME.lower() + "_depth_node"

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
        # cross-process flock inside transact). navpose_lock: the reading,
        # written by the poll timer and read by NPXDeviceIF's timer.
        self.serial_lock = threading.Lock()
        self.navpose_lock = threading.Lock()

        ##############################
        # Get required drv driver dict info
        self.drv_dict = nepi_sdk.get_param('~drv_dict',dict())
        try:
            device_dict = self.drv_dict['DEVICE_DICT']
            self.device_name = device_dict['device_name']
            self.device_path = device_dict['device_path']
            self.port_str = device_dict['device_path']
            self.serial_num = str(device_dict.get('serial_number', 'Unknown'))
            self.config = device_dict['depth_config']
            self.resolveConfig()
        except Exception as e:
            self.msg_if.pub_warn("Failed to load Device Dict " + str(e))
            nepi_sdk.signal_shutdown(self.node_name + ": Shutting down because no valid Device Dict")
            return

        self.initReadingState()

        ################################################
        self.msg_if.pub_info("Connecting to Teensy " + self.serial_num + " depth sensor on " + str(self.port_str))
        self.connected = self.connect()
        if self.connected == False:
            self.msg_if.pub_info("Shutting down node, unable to connect to the teensy_thrusters firmware")
            nepi_sdk.signal_shutdown("Unable to connect to depth sensor controller")
            return
        self.msg_if.pub_info("... Connected!")

        self.device_info_dict["device_name"] = self.device_name
        self.device_info_dict["path"] = self.device_path
        self.device_info_dict["serial_number"] = self.serial_num
        self.device_info_dict["hw_version"] = self.hw_version
        self.device_info_dict["sw_version"] = self.sw_version

        self.pressure_pub = nepi_sdk.create_publisher(nepi_sdk.create_namespace(self.node_namespace, 'pressure'),
                                                      FluidPressure, queue_size = 1)
        self.temperature_pub = nepi_sdk.create_publisher(nepi_sdk.create_namespace(self.node_namespace, 'temperature'),
                                                         Temperature, queue_size = 1)

        self.msg_if.pub_info("Launching NEPI NPX interface...")
        self.npx_if = NPXDeviceIF(device_info = self.device_info_dict,
                                  data_source_description = 'depth_sensor',
                                  data_ref_description = 'sensor_center',
                                  frame_depth = 'DEPTH',
                                  getNavPoseCb = self.getNavPoseCb,
                                  max_navpose_update_rate = int(round(1.0 / self.POLL_INTERVAL_SEC)),
                                  msg_if = self.msg_if
                                  )
        self.msg_if.pub_info(" ... NPX interface running")

        nepi_sdk.start_timer_process(self.POLL_INTERVAL_SEC, self.pollCb)

        # Initialization Complete
        self.msg_if.pub_info("Initialization Complete")
        #Set up node shutdown
        nepi_sdk.on_shutdown(self.cleanup_actions)
        # Spin forever
        nepi_sdk.spin()


    def resolveConfig(self):
        # The depth_config the discovery built, checked again: the discovery
        # and this node install as separate files. Raises ValueError.
        config = self.config
        self.depth_model = str(config['depth_model'])
        if self.depth_model not in BAR_MODELS:
            raise ValueError("depth_model '" + self.depth_model + "' must be one of " + str(sorted(BAR_MODELS.keys())))
        self.model = BAR_MODELS[self.depth_model]
        self.depth_osr = int(config['depth_osr'])
        if self.depth_osr not in DEPTH_OSR_VALUES:
            raise ValueError("depth_osr " + str(self.depth_osr) + " must be one of " + str(DEPTH_OSR_VALUES))
        self.fluid_density_kg_m3 = float(config['fluid_density_kg_m3'])
        if not (FLUID_DENSITY_KG_M3_RANGE[0] <= self.fluid_density_kg_m3 <= FLUID_DENSITY_KG_M3_RANGE[1]):
            raise ValueError("fluid_density_kg_m3 " + str(self.fluid_density_kg_m3) + " is outside " +
                             str(FLUID_DENSITY_KG_M3_RANGE))
        self.surface_pressure_mbar = float(config['surface_pressure_mbar'])
        if not (SURFACE_PRESSURE_MBAR_RANGE[0] <= self.surface_pressure_mbar <= SURFACE_PRESSURE_MBAR_RANGE[1]):
            raise ValueError("surface_pressure_mbar " + str(self.surface_pressure_mbar) + " is outside " +
                             str(SURFACE_PRESSURE_MBAR_RANGE))


    def initReadingState(self):
        # navpose_dict stays None until the first good reading (see STALE DATA
        # at the top of this file).
        self.navpose_dict = None
        self.prom = None
        self.prom_detects = None
        self.last_seq = None
        self.history = []
        self.model_checked = False
        self.configured_time = 0.0


    #######################
    ### NPX IF Callback

    def getNavPoseCb(self):
        with self.navpose_lock:
            if self.navpose_dict is None:
                return None
            return copy.deepcopy(self.navpose_dict)


    #######################
    ### Reading

    def pollCb(self, timer):
        if self.connected == False or nepi_sdk.is_shutdown():
            return
        status = parseDepthValue(self.transact('DV'))
        if status is None:
            return
        if status['dcfg'] == 0:
            # dcfg is 0 only before the first DC since boot, and this node sent
            # one at connect: the Teensy rebooted.
            self.msg_if.pub_warn("Teensy " + self.serial_num + " rebooted; restarting its depth sensor")
            self.markStale()
            self.configureSensor()
            return
        if status['state'] == 'OFF':
            # Another host turned sampling off -- a depth node that was shutting
            # down while this one started. Turn it back on.
            self.msg_if.pub_warn("Teensy depth sensor sampling was off; restarting it", throttle_s = self.WARN_THROTTLE_SEC)
            self.markStale()
            self.configureSensor()
            return
        if status['state'] != 'OK':
            self.markStale()
            self.warnState(status['state'])
            return
        if status['seq'] == self.last_seq:
            # No new sample since the last poll. The Teensy keeps the last one
            # while a few reads fail in a row, so its age is what says stale.
            if status['age_ms'] > self.STALE_SEC * 1000.0:
                self.markStale()
            return
        if status['detects'] != self.prom_detects:
            # A new detection, possibly a different sensor: its own calibration.
            if self.readProm(status['detects']) == False:
                self.markStale()
                return
        self.last_seq = status['seq']
        sample_time = nepi_utils.get_time() - status['age_ms'] / 1000.0
        self.updateReading(status['d1'], status['d2'], sample_time)


    def warnState(self, state):
        if state == 'SEARCHING':
            if nepi_utils.get_time() - self.configured_time < self.STARTUP_GRACE_SEC:
                return
            self.msg_if.pub_warn("No depth sensor answers at I2C address 0x76 on Teensy " + self.serial_num +
                                 " (SDA pin 18, SCL pin 19); check its wiring, power and pull-ups",
                                 throttle_s = self.WARN_THROTTLE_SEC)
        elif state == 'BAD_PROM':
            self.msg_if.pub_warn("A device answers at I2C address 0x76 on Teensy " + self.serial_num +
                                 " but its calibration words fail the CRC check: not a Bar sensor (an MS8607 " +
                                 "also uses 0x76), or a bad connection", throttle_s = self.WARN_THROTTLE_SEC)
        else:
            self.msg_if.pub_warn("Teensy reports depth sensor state " + str(state), throttle_s = self.WARN_THROTTLE_SEC)


    def readProm(self, detects):
        prom = parseDepthProm(self.transact('DP'))
        if prom is None:
            return False
        [prom_detects, words] = prom
        if prom_detects != detects:
            # Re-detected between DV and DP; the next poll pairs them again.
            return False
        if promValid(words) == False:
            # The firmware checked these before reporting OK, so this means
            # the words were damaged on the way.
            self.msg_if.pub_warn("Depth sensor calibration words fail the CRC check: " + str(words),
                                 throttle_s = self.WARN_THROTTLE_SEC)
            return False
        self.prom = words
        self.prom_detects = prom_detects
        self.msg_if.pub_info("Depth sensor found on Teensy " + self.serial_num + "; calibration words " + str(words))
        return True


    def updateReading(self, d1, d2, sample_time):
        [pressure_mbar, temperature_c] = barCompensate(self.depth_model, self.prom, d1, d2)
        depth_m = depthFromPressure(pressure_mbar, self.surface_pressure_mbar, self.fluid_density_kg_m3)
        if self.model_checked == False:
            # Once per node start, not per detection: a sensor re-detected
            # after a glitch underwater would warn for no reason.
            self.model_checked = True
            text = modelCheckText(self.depth_model, self.prom, d1, d2,
                                  self.surface_pressure_mbar, self.fluid_density_kg_m3)
            if text is not None:
                self.msg_if.pub_warn(text)
            else:
                self.msg_if.pub_info("First depth sensor reading: %.1f mbar, %.2f C, depth %.3f m" %
                                     (pressure_mbar, temperature_c, depth_m))
        if pressure_mbar > self.model['max_mbar']:
            self.msg_if.pub_warn("Depth sensor reads %.0f mbar, beyond the %.0f mbar top of a %s's range" %
                                 (pressure_mbar, self.model['max_mbar'], self.depth_model),
                                 throttle_s = self.WARN_THROTTLE_SEC)
        rate = self.depthRate(sample_time, depth_m)
        with self.navpose_lock:
            if self.navpose_dict is None:
                self.navpose_dict = copy.deepcopy(nepi_nav.BLANK_NAVPOSE_DICT)
                self.navpose_dict['frame_depth'] = 'DEPTH'
            self.navpose_dict['has_depth'] = True
            self.navpose_dict['time_depth'] = sample_time
            self.navpose_dict['depth_m'] = depth_m
            self.navpose_dict['depth_m_per_sec'] = rate
        self.publishPressureTemperature(sample_time, pressure_mbar, temperature_c)


    def depthRate(self, sample_time, depth_m):
        # Positive while getting deeper.
        self.history.append((sample_time, depth_m))
        while len(self.history) > 1 and (sample_time - self.history[0][0]) > self.RATE_WINDOW_SEC:
            self.history.pop(0)
        (first_time, first_depth) = self.history[0]
        if sample_time - first_time <= 0.0:
            return 0.0
        return (depth_m - first_depth) / (sample_time - first_time)


    def markStale(self):
        with self.navpose_lock:
            if self.navpose_dict is not None:
                self.navpose_dict['has_depth'] = False
                self.navpose_dict['depth_m_per_sec'] = 0.0
        self.history = []


    def publishPressureTemperature(self, sample_time, pressure_mbar, temperature_c):
        stamp = nepi_sdk.msg_stamp_from_sec(sample_time)
        pressure_msg = FluidPressure()
        pressure_msg.header.stamp = stamp
        pressure_msg.header.frame_id = self.node_name
        pressure_msg.fluid_pressure = pressure_mbar * 100.0
        pressure_msg.variance = 0.0   # unknown
        nepi_sdk.publish_pub(self.pressure_pub, pressure_msg)
        temperature_msg = Temperature()
        temperature_msg.header.stamp = stamp
        temperature_msg.header.frame_id = self.node_name
        temperature_msg.temperature = temperature_c
        temperature_msg.variance = 0.0   # unknown
        nepi_sdk.publish_pub(self.temperature_pub, temperature_msg)


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
                identity['protocol_minor'] < self.DEPTH_PROTOCOL_MINOR):
            self.msg_if.pub_warn("Teensy speaks protocol " + identity['protocol_version'] +
                                 "; the depth sensor needs " + str(self.PROTOCOL_MAJOR) + "." +
                                 str(self.DEPTH_PROTOCOL_MINOR) + " or a later 1.x")
            self.closePort()
            return False

        self.hw_version = self.depth_model + " on Teensy 4.1 I2C (SDA 18, SCL 19)"
        self.sw_version = ("teensy_thrusters " + identity['firmware_version'] +
                           " (protocol " + identity['protocol_version'] + ")")
        self.msg_if.pub_info("Firmware " + self.sw_version + "; " + self.hw_version + "; OSR " +
                             str(self.depth_osr) + "; fluid density " + str(self.fluid_density_kg_m3) +
                             " kg/m3; surface pressure " + str(self.surface_pressure_mbar) + " mbar")
        return self.configureSensor(connecting = True)

    def configureSensor(self, connecting = False):
        # DC starts the Teensy sampling (PROTOCOL.md 6.12); the first reading
        # follows within about a second, once the sensor has been probed.
        body = 'DC,%d' % self.depth_osr
        reply = self.transact(body, connecting = connecting)
        if reply != ['OK', 'DC']:
            self.msg_if.pub_warn("Teensy rejected depth sensor configuration " + body + ": " + str(reply))
            if connecting:
                self.closePort()
            return False
        self.configured_time = nepi_utils.get_time()
        return True

    def transact(self, body, connecting = False):
        # Send one frame and return the reply's fields, or None. Same contract
        # as the thruster and servo nodes' transact: in-process lock plus the
        # port's flock, input flushed first, and only a reply naming this
        # command is accepted (PROTOCOL.md 2.1).
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
        self.markStale()
        self.msg_if.pub_warn("Lost serial link to Teensy on " + str(self.port_str) + " (" + str(reason) +
                             "); shutting down so discovery can relaunch it")
        self.closePort()
        nepi_sdk.signal_shutdown("Lost serial link to depth sensor controller")

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
        """Stop the Teensy sampling the depth sensor and release the port.

        Sends DC,0 so a board whose depth sensor part was deselected leaves its
        I2C bus alone again, as before the part was selected.
        """
        self.msg_if.pub_info("Shutting down: Executing script cleanup actions")
        self.connected = False
        if self.serial_port is not None:
            try:
                self.transact('DC,0', connecting = True)
            except Exception:
                pass
        self.closePort()


##########################
# Protocol helpers (PROTOCOL.md 3, 2.1 and 6.1). Identical copies of the
# discovery's and the thruster and servo nodes' -- drivers install flat into one
# folder and the discovery runs inside drivers_mgr, so the files share no
# module. Keep the copies in step.

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
    # or a reply that carries the command's own name (ID, ST, SV, DV, DP).
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


def parseDepthValue(fields):
    # $DV,<dcfg>,<state>,<osr>,<detects>,<seq>,<age_ms>,<d1>,<d2>,<errors>
    # (PROTOCOL.md 6.13). Fields beyond these are ignored: a later minor
    # version may append some.
    if fields is None or len(fields) < 10 or fields[0] != 'DV':
        return None
    try:
        return dict(dcfg = int(fields[1]),
                    state = fields[2],
                    osr = int(fields[3]),
                    detects = int(fields[4]),
                    seq = int(fields[5]),
                    age_ms = int(fields[6]),
                    d1 = int(fields[7]),
                    d2 = int(fields[8]),
                    errors = int(fields[9]))
    except Exception:
        return None


def parseDepthProm(fields):
    # $DP,<detects>,<w0>,...,<w6> (PROTOCOL.md 6.14) -> [detects, [w0..w6]]
    if fields is None or len(fields) < 9 or fields[0] != 'DP':
        return None
    try:
        return [int(fields[1]), [int(w) for w in fields[2:9]]]
    except Exception:
        return None


if __name__ == '__main__':
    node = SvxTeensyThrustersDepthNode()
