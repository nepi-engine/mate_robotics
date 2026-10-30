#!/usr/bin/env python3
##
## Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
##
## This file is part of nepi-engine
## (see https://github.com/nepi-engine).
##
## License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
##

# Tests for the depth sensor part of the svx_teensy_thrusters driver:
# Drivers/svx_drivers/Teensy/svx_teensy_thrusters_depth_node.py, and the
# devices and depth options in svx_teensy_thrusters_discovery.py. Standard
# library only. Run from this folder:
#   python3 -B -m unittest -v test_svx_teensy_thrusters_depth
# or with the thrust map tests:
#   python3 -B -m unittest test_svx_teensy_thrusters test_svx_teensy_thrusters_depth
#
# There is no ROS and no serial port here. Stand-in modules go into
# sys.modules only so the driver files import; they are added with setdefault,
# so either test file can be imported first. The node class is never built
# (its __init__ starts ROS): instances come from Class.__new__ with only what
# the method under test reads, and FakeTeensy answers DC, DV and DP the way
# PROTOCOL.md 6.12-6.14 says the firmware does.
#
# Reference values are the MS5837 datasheet examples quoted in
# Docs/npx/bluerobotics_bar_depth_sensor.md. The CRC reference vectors were
# produced by compiling that document's C crc4() verbatim.

import copy
import os
import sys
import threading
import types
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..', 'svx_drivers', 'Teensy'))


class FakeClock:
    def __init__(self):
        self.now = 1000.0

    def time(self):
        return self.now


class FakeMsg:
    # sensor_msgs FluidPressure / Temperature: a header plus plain fields
    def __init__(self):
        self.header = types.SimpleNamespace(stamp = None, frame_id = '')


CLOCK = FakeClock()
PUBLISHED = []


def stubModules():
    names = ['nepi_sdk', 'nepi_sdk.nepi_sdk', 'nepi_sdk.nepi_utils', 'nepi_sdk.nepi_drvs', 'nepi_sdk.nepi_system',
             'nepi_sdk.nepi_nav', 'nepi_api', 'nepi_api.device_if_npx', 'nepi_api.messages_if',
             'sensor_msgs', 'sensor_msgs.msg', 'serial', 'serial.tools', 'serial.tools.list_ports']
    for name in names:
        sys.modules.setdefault(name, types.ModuleType(name))
    for name in names:
        if '.' in name:
            [parent, child] = name.rsplit('.', 1)
            setattr(sys.modules[parent], child, sys.modules[name])
    # The names the driver files use from them
    sys.modules['nepi_api.device_if_npx'].NPXDeviceIF = None
    sys.modules['nepi_api.messages_if'].MsgIF = None
    sys.modules['sensor_msgs.msg'].FluidPressure = FakeMsg
    sys.modules['sensor_msgs.msg'].Temperature = FakeMsg
    sys.modules['nepi_sdk.nepi_utils'].get_time = CLOCK.time
    sdk = sys.modules['nepi_sdk.nepi_sdk']
    sdk.is_shutdown = lambda: False
    sdk.msg_stamp_from_sec = lambda t: t
    sdk.publish_pub = lambda pub, msg: PUBLISHED.append((pub, msg))
    # The keys of nepi_nav.BLANK_NAVPOSE_DICT the depth node touches, at
    # their real defaults
    sys.modules['nepi_sdk.nepi_nav'].BLANK_NAVPOSE_DICT = {
        'frame_depth': 'MSL', 'has_location': False, 'has_depth': False,
        'time_depth': 0.0, 'depth_m': 0.0, 'depth_m_per_sec': 0.0}

stubModules()

import svx_teensy_thrusters_depth_node as depth_node
import svx_teensy_thrusters_discovery

Node = depth_node.SvxTeensyThrustersDepthNode
Discovery = svx_teensy_thrusters_discovery.SvxTeensyThrustersDiscovery

BAR02 = 'BlueRobotics Bar02 (MS5837-02BA)'
BAR30 = 'BlueRobotics Bar30 (MS5837-30BA)'

# Datasheet examples (Bar doc, Unit-test vectors): C1-C6, D1, D2, expected
BAR30_C = [34982, 36352, 20328, 22354, 26646, 26146]
BAR30_D1 = 4958179
BAR30_D2 = 6815414
BAR30_FIRST = dict(dT = -5962, TEMP = 1981, OFF = 2381323464, SENS = 1145816755, P = 39998)
BAR02_C = [46372, 43981, 29059, 27842, 31553, 28165]
BAR02_D1 = 6465444
BAR02_D2 = 8077636
BAR02_FIRST = dict(dT = 68, TEMP = 2000, OFF = 5764707214, SENS = 3039050829, P = 110002)

# PROM words 0-6 and the CRC the Bar doc's C crc4() returns for them
CRC_VECTORS = [([43079, 41365, 7149, 53051, 33377, 25040, 449], 7),
               ([59598, 40344, 61432, 27, 33601, 47129, 35093], 9),
               ([38286, 37589, 20946, 41920, 9865, 31416, 49625], 10)]

SURFACE_MBAR = 1013.25
FRESH = 997.0


def validProm(coefficients):
    # PROM words 0-6 for C1-C6, with the CRC in word 0's top four bits
    words = [0] + list(coefficients)
    words[0] = depth_node.promCrc4(words) << 12
    return words


BAR30_PROM = validProm(BAR30_C)
BAR02_PROM = validProm(BAR02_C)


def d1ForPressure(model, prom, d2, target_mbar):
    # The raw D1 that reads target_mbar, by bisection: pressure rises with D1
    lo = 0
    hi = 2**24 - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if depth_node.barCompensate(model, prom, mid, d2)[0] < target_mbar:
            lo = mid
        else:
            hi = mid
    return hi


def referenceCompensate(chip, C, D1, D2):
    # The Bar doc's formulas in floating point, as Blue Robotics' Python
    # library computes them: an independent check on the integer version.
    # C is [C1..C6].
    (C1, C2, C3, C4, C5, C6) = C
    dT = D2 - C5 * 256.0
    TEMP = 2000.0 + dT * C6 / 2**23
    if chip == '30BA':
        OFF = C2 * 2**16 + (C4 * dT) / 2**7
        SENS = C1 * 2**15 + (C3 * dT) / 2**8
        if TEMP < 2000:
            Ti = 3 * dT**2 / 2**33
            OFFi = 3 * (TEMP - 2000)**2 / 2**1
            SENSi = 5 * (TEMP - 2000)**2 / 2**3
            if TEMP < -1500:
                OFFi = OFFi + 7 * (TEMP + 1500)**2
                SENSi = SENSi + 4 * (TEMP + 1500)**2
        else:
            Ti = 2 * dT**2 / 2**37
            OFFi = (TEMP - 2000)**2 / 2**4
            SENSi = 0
        p_mbar = (((D1 * (SENS - SENSi)) / 2**21 - (OFF - OFFi)) / 2**13) / 10.0
    else:
        OFF = C2 * 2**17 + (C4 * dT) / 2**6
        SENS = C1 * 2**16 + (C3 * dT) / 2**7
        if TEMP < 2000:
            Ti = 11 * dT**2 / 2**35
            OFFi = 31 * (TEMP - 2000)**2 / 2**3
            SENSi = 63 * (TEMP - 2000)**2 / 2**5
        else:
            Ti = OFFi = SENSi = 0
        p_mbar = (((D1 * (SENS - SENSi)) / 2**21 - (OFF - OFFi)) / 2**15) / 100.0
    return [p_mbar, (TEMP - Ti) / 100.0]


class FakeLogger:
    def __init__(self):
        self.warnings = []

    def log_warn(self, msg):
        self.warnings.append(msg)

    def log_info(self, msg):
        pass


class FakeMsgIF:
    def __init__(self):
        self.warnings = []
        self.infos = []

    def pub_warn(self, msg, throttle_s = None):
        self.warnings.append(msg)

    def pub_info(self, msg, throttle_s = None):
        self.infos.append(msg)


class FakeTeensy:
    # Answers DC, DV and DP as PROTOCOL.md 6.12-6.14 describes, through a real
    # frame encode and decode.
    def __init__(self, prom, d1, d2):
        self.prom = list(prom)
        self.d1 = d1
        self.d2 = d2
        self.dcfg = 0
        self.state = 'OFF'
        self.osr = 0
        self.detects = 0
        self.seq = 0
        self.age_ms = 0
        self.sent = []

    def transact(self, body, connecting = False):
        self.sent.append(body)
        fields = body.split(',')
        if fields[0] == 'DC':
            osr = int(fields[1])
            self.dcfg = 1
            if osr == 0:
                self.state = 'OFF'
            else:
                self.osr = osr
                if self.state == 'OFF':
                    self.state = 'SEARCHING'
            reply = 'OK,DC'
        elif fields[0] == 'DV':
            osr = 0 if self.state == 'OFF' else self.osr
            reply = 'DV,%d,%s,%d,%d,%d,%d,%d,%d,0' % (self.dcfg, self.state, osr, self.detects, self.seq,
                                                     self.age_ms, self.d1, self.d2)
        elif fields[0] == 'DP':
            reply = 'DP,%d,' % self.detects + ','.join(str(w) for w in self.prom)
        else:
            return None
        return depth_node.decodeFrame(depth_node.encodeFrame(reply))

    def detect(self):
        self.state = 'OK'
        self.detects += 1

    def sample(self, d1 = None, age_ms = 5):
        if d1 is not None:
            self.d1 = d1
        self.seq += 1
        self.age_ms = age_ms


def makeNode(teensy = None, **config):
    node = Node.__new__(Node)
    node.config = dict(depth_model = BAR02, depth_osr = 1024, fluid_density_kg_m3 = FRESH,
                       surface_pressure_mbar = SURFACE_MBAR)
    node.config.update(config)
    node.resolveConfig()
    node.msg_if = FakeMsgIF()
    node.serial_num = '21616540'
    node.node_name = 'teensy_thrusters_21616540_depth'
    node.navpose_lock = threading.Lock()
    node.pressure_pub = 'pressure'
    node.temperature_pub = 'temperature'
    node.connected = True
    node.initReadingState()
    if teensy is not None:
        node.transact = teensy.transact
        node.configureSensor()   # as connect() does
    return node


def makeDiscovery():
    disc = Discovery.__new__(Discovery)
    disc.logger = FakeLogger()
    disc.last_log_dict = dict()   # class attributes; give each instance its own
    disc.active_devices_dict = dict()
    disc.identity_dict = dict()
    disc.probe_after_dict = dict()
    disc.exit_times_dict = dict()
    disc.active_paths_list = []
    disc.dont_retry_list = []
    return disc


def optionsDict(values):
    # The shape drivers_mgr hands the discovery. A multi-select value is a
    # list; every other value a string.
    return {name: {'value': value if isinstance(value, list) else str(value)} for (name, value) in values.items()}


class TestConversion(unittest.TestCase):

    def test_first_order_matches_datasheet(self):
        self.assertEqual(depth_node.barFirstOrder(BAR30, BAR30_PROM, BAR30_D1, BAR30_D2), BAR30_FIRST)
        self.assertEqual(depth_node.barFirstOrder(BAR02, BAR02_PROM, BAR02_D1, BAR02_D2), BAR02_FIRST)

    def test_compensated_examples(self):
        # Bar02 at exactly 20 C gets no second-order correction
        self.assertEqual(depth_node.barCompensate(BAR02, BAR02_PROM, BAR02_D1, BAR02_D2), [1100.02, 20.0])
        # Bar30 at 19.81 C: the low-temperature correction is below one count
        self.assertEqual(depth_node.barCompensate(BAR30, BAR30_PROM, BAR30_D1, BAR30_D2), [3999.8, 19.81])

    def test_second_order_against_float_reference(self):
        # Cold, warm and (Bar30) below -15 C, across the D2 range: the integer
        # math stays within a count or two of the float formulas
        cases = [(BAR30, '30BA', BAR30_PROM, BAR30_D1, 0.25), (BAR02, '02BA', BAR02_PROM, BAR02_D1, 0.03)]
        for (model, chip, prom, d1, tolerance_mbar) in cases:
            for d2 in range(prom[5] * 256 - 2000000, prom[5] * 256 + 2000001, 100000):
                got = depth_node.barCompensate(model, prom, d1, d2)
                expected = referenceCompensate(chip, prom[1:], d1, d2)
                what = model + " D2 " + str(d2) + ": " + str((got, expected))
                self.assertLessEqual(abs(got[0] - expected[0]), tolerance_mbar, what)
                self.assertLessEqual(abs(got[1] - expected[1]), 0.02, what)
        # The sweep reaches the very-cold Bar30 branch
        self.assertLess(depth_node.barFirstOrder(BAR30, BAR30_PROM, BAR30_D1, BAR30_PROM[5] * 256 - 2000000)['TEMP'],
                        -1500)

    def test_wrong_model_is_about_20x(self):
        # Bar doc, Plausibility check: about 22000 and about 200 mbar
        self.assertAlmostEqual(depth_node.barCompensate(BAR30, BAR02_PROM, BAR02_D1, BAR02_D2)[0], 22000, delta = 100)
        self.assertAlmostEqual(depth_node.barCompensate(BAR02, BAR30_PROM, BAR30_D1, BAR30_D2)[0], 200, delta = 5)

    def test_depth_equation(self):
        one_meter_mbar = FRESH * depth_node.GRAVITY_M_S2 / 100.0
        self.assertEqual(depth_node.depthFromPressure(SURFACE_MBAR, SURFACE_MBAR, FRESH), 0.0)
        self.assertAlmostEqual(depth_node.depthFromPressure(SURFACE_MBAR + one_meter_mbar, SURFACE_MBAR, FRESH), 1.0)
        self.assertAlmostEqual(depth_node.depthFromPressure(SURFACE_MBAR + one_meter_mbar, SURFACE_MBAR, 1029.0),
                               FRESH / 1029.0)
        self.assertLess(depth_node.depthFromPressure(SURFACE_MBAR - 5.0, SURFACE_MBAR, FRESH), 0.0)


class TestProm(unittest.TestCase):

    def test_crc_matches_datasheet_code(self):
        for (words, crc) in CRC_VECTORS:
            self.assertEqual(depth_node.promCrc4(words), crc, str(words))

    def test_valid_prom_and_every_single_bit_error(self):
        for prom in (BAR30_PROM, BAR02_PROM):
            self.assertTrue(depth_node.promValid(prom))
            for word in range(7):
                for bit in range(16):
                    damaged = list(prom)
                    damaged[word] ^= (1 << bit)
                    self.assertFalse(depth_node.promValid(damaged), str((word, bit)))

    def test_blank_prom_rejected(self):
        # An all-zero PROM passes the CRC, so the CRC alone is not enough
        self.assertEqual(depth_node.promCrc4([0] * 7), 0)
        self.assertFalse(depth_node.promValid([0] * 7))
        self.assertFalse(depth_node.promValid(validProm([0xFFFF] + BAR02_C[1:])))
        self.assertFalse(depth_node.promValid(BAR02_PROM[:6]))
        self.assertFalse(depth_node.promValid(None))


class TestFrames(unittest.TestCase):

    def roundTrip(self, body):
        return depth_node.decodeFrame(depth_node.encodeFrame(body))

    def test_depth_value(self):
        fields = self.roundTrip('DV,1,OK,1024,3,1532,12,6465444,8077636,0')
        self.assertTrue(depth_node.replyMatches('DV', fields))
        self.assertEqual(depth_node.parseDepthValue(fields),
                         dict(dcfg = 1, state = 'OK', osr = 1024, detects = 3, seq = 1532, age_ms = 12,
                              d1 = 6465444, d2 = 8077636, errors = 0))
        # A later minor version may append fields
        self.assertEqual(depth_node.parseDepthValue(self.roundTrip('DV,1,OK,1024,3,1532,12,1,2,0,99'))['d2'], 2)
        self.assertIsNone(depth_node.parseDepthValue(self.roundTrip('DV,1,OK,1024')))
        self.assertIsNone(depth_node.parseDepthValue(self.roundTrip('DV,1,OK,x,3,1532,12,1,2,0')))
        self.assertIsNone(depth_node.parseDepthValue(None))

    def test_depth_prom(self):
        fields = self.roundTrip('DP,3,' + ','.join(str(w) for w in BAR02_PROM))
        self.assertTrue(depth_node.replyMatches('DP', fields))
        self.assertEqual(depth_node.parseDepthProm(fields), [3, BAR02_PROM])
        self.assertIsNone(depth_node.parseDepthProm(self.roundTrip('DP,3,1,2')))
        # An error reply names the command it answers, and is not a DP reply
        error = self.roundTrip('ER,FIELDS,DP')
        self.assertTrue(depth_node.replyMatches('DP', error))
        self.assertIsNone(depth_node.parseDepthProm(error))


class TestModelCheck(unittest.TestCase):

    def check(self, model, prom, d1, d2):
        return depth_node.modelCheckText(model, prom, d1, d2, SURFACE_MBAR, FRESH)

    def test_right_model_in_air(self):
        # The Bar02 example reads 1100 mbar, within the window of the surface
        self.assertIsNone(self.check(BAR02, BAR02_PROM, BAR02_D1, BAR02_D2))
        d1 = d1ForPressure(BAR30, BAR30_PROM, BAR30_D2, SURFACE_MBAR)
        self.assertIsNone(self.check(BAR30, BAR30_PROM, d1, BAR30_D2))

    def test_wrong_model_named(self):
        text = self.check(BAR30, BAR02_PROM, BAR02_D1, BAR02_D2)
        self.assertIn("set depth_model to " + BAR02, text)
        d1 = d1ForPressure(BAR30, BAR30_PROM, BAR30_D2, SURFACE_MBAR)
        text = self.check(BAR02, BAR30_PROM, d1, BAR30_D2)
        self.assertIn("set depth_model to " + BAR30, text)

    def test_underwater_start(self):
        d1 = d1ForPressure(BAR02, BAR02_PROM, BAR02_D2, 1500.0)
        text = self.check(BAR02, BAR02_PROM, d1, BAR02_D2)
        self.assertIn("check depth_model and surface_pressure_mbar", text)
        self.assertNotIn("set depth_model", text)


class TestNodeReading(unittest.TestCase):

    def setUp(self):
        CLOCK.now = 1000.0
        del PUBLISHED[:]
        self.teensy = FakeTeensy(BAR02_PROM, BAR02_D1, BAR02_D2)
        self.node = makeNode(self.teensy)

    def poll(self, seconds = 0.05):
        CLOCK.now += seconds
        self.node.pollCb(None)

    def test_connect_starts_sampling(self):
        self.assertEqual(self.teensy.sent, ['DC,1024'])
        node = makeNode(FakeTeensy(BAR02_PROM, BAR02_D1, BAR02_D2), depth_osr = 8192)
        self.assertEqual(node.transact.__self__.sent, ['DC,8192'])

    def test_no_navpose_and_no_warning_before_first_reading(self):
        for k in range(10):
            self.poll()
        self.assertIsNone(self.node.getNavPoseCb())
        self.assertEqual(self.node.msg_if.warnings, [])
        # Past the grace period, a sensor that never answers is reported
        self.poll(self.node.STARTUP_GRACE_SEC)
        self.assertIsNone(self.node.getNavPoseCb())
        self.assertIn("No depth sensor answers at I2C address 0x76", self.node.msg_if.warnings[-1])

    def test_first_reading(self):
        self.teensy.detect()
        self.teensy.sample(age_ms = 20)
        self.poll()
        self.assertEqual(self.teensy.sent[-2:], ['DV', 'DP'])
        navpose = self.node.getNavPoseCb()
        self.assertTrue(navpose['has_depth'])
        self.assertEqual(navpose['frame_depth'], 'DEPTH')
        self.assertAlmostEqual(navpose['time_depth'], CLOCK.now - 0.020)
        self.assertAlmostEqual(navpose['depth_m'], (1100.02 - SURFACE_MBAR) * 100.0 / (FRESH * 9.80665))
        self.assertEqual(navpose['depth_m_per_sec'], 0.0)
        # Pressure in pascals and temperature in deg C, stamped with the sample time
        [(pressure_pub, pressure), (temperature_pub, temperature)] = PUBLISHED
        self.assertEqual((pressure_pub, temperature_pub), ('pressure', 'temperature'))
        self.assertAlmostEqual(pressure.fluid_pressure, 110002.0)
        self.assertEqual(temperature.temperature, 20.0)
        self.assertAlmostEqual(pressure.header.stamp, CLOCK.now - 0.020)
        # In range for a Bar02 in air, so the model check stays quiet
        self.assertEqual(self.node.msg_if.warnings, [])
        # The PROM is read once per detection, not per sample
        self.teensy.sample()
        self.poll()
        self.assertEqual(self.teensy.sent[-1], 'DV')

    def test_stale_keeps_last_reading(self):
        self.teensy.detect()
        self.teensy.sample()
        self.poll()
        depth = self.node.getNavPoseCb()['depth_m']
        # No new sample, but still young: unchanged
        self.teensy.age_ms = 500
        self.poll()
        self.assertTrue(self.node.getNavPoseCb()['has_depth'])
        # Too old: stale, never zeroed
        self.teensy.age_ms = 1500
        self.poll()
        navpose = self.node.getNavPoseCb()
        self.assertFalse(navpose['has_depth'])
        self.assertEqual(navpose['depth_m'], depth)
        # The sensor stops answering: stale again, still not zeroed
        self.teensy.sample()
        self.poll()
        self.assertTrue(self.node.getNavPoseCb()['has_depth'])
        self.teensy.state = 'SEARCHING'
        self.poll()
        navpose = self.node.getNavPoseCb()
        self.assertFalse(navpose['has_depth'])
        self.assertEqual(navpose['depth_m'], depth)

    def test_reboot_and_off_restart_sampling(self):
        self.teensy.detect()
        self.teensy.sample()
        self.poll()
        # The Teensy rebooted: dcfg 0 and nothing sampled
        self.teensy.dcfg = 0
        self.teensy.state = 'OFF'
        self.poll()
        self.assertEqual(self.teensy.sent[-1], 'DC,1024')
        self.assertFalse(self.node.getNavPoseCb()['has_depth'])
        self.assertIn("rebooted", self.node.msg_if.warnings[-1])
        # Another host turned sampling off
        self.teensy.state = 'OFF'
        self.poll()
        self.assertEqual(self.teensy.sent[-1], 'DC,1024')

    def test_new_detection_rereads_prom(self):
        self.teensy.detect()
        self.teensy.sample()
        self.poll()
        # A different sensor is plugged in: its own calibration, its own reading
        self.teensy.prom = BAR30_PROM
        self.teensy.d1 = d1ForPressure(BAR02, BAR30_PROM, BAR30_D2, 1200.0)
        self.teensy.d2 = BAR30_D2
        self.teensy.detect()
        self.teensy.sample()
        self.poll()
        self.assertEqual(self.teensy.sent[-1], 'DP')
        self.assertEqual(self.node.prom, BAR30_PROM)
        self.assertAlmostEqual(self.node.getNavPoseCb()['depth_m'],
                               depth_node.depthFromPressure(1200.0, SURFACE_MBAR, FRESH), delta = 0.001)

    def test_prom_from_another_detection_is_not_used(self):
        self.teensy.detect()
        self.teensy.sample()
        real_transact = self.teensy.transact

        def raceTransact(body, connecting = False):
            # Re-detected between DV and DP
            if body == 'DP':
                self.teensy.detects += 1
            return real_transact(body, connecting)
        self.node.transact = raceTransact
        self.poll()
        self.assertIsNone(self.node.getNavPoseCb())
        self.assertIsNone(self.node.prom)

    def test_damaged_prom_not_used(self):
        damaged = list(BAR02_PROM)
        damaged[3] ^= 0x0100
        self.teensy.prom = damaged
        self.teensy.detect()
        self.teensy.sample()
        self.poll()
        self.assertIsNone(self.node.getNavPoseCb())
        self.assertIn("fail the CRC check", self.node.msg_if.warnings[-1])

    def test_depth_rate(self):
        self.teensy.detect()
        one_meter_mbar = FRESH * depth_node.GRAVITY_M_S2 / 100.0
        # Descending at 0.2 m/s, sampled every 0.05 s
        for k in range(20):
            pressure = SURFACE_MBAR + one_meter_mbar * (1.0 + 0.2 * 0.05 * k)
            self.teensy.sample(d1 = d1ForPressure(BAR02, BAR02_PROM, BAR02_D2, pressure), age_ms = 0)
            self.poll()
        self.assertAlmostEqual(self.node.getNavPoseCb()['depth_m_per_sec'], 0.2, delta = 0.002)

    def test_model_warning_once_and_over_range(self):
        node = makeNode(self.teensy, depth_model = BAR30)
        self.teensy.detect()
        self.teensy.sample()
        node.pollCb(None)
        self.assertEqual(len(node.msg_if.warnings), 1)
        self.assertIn("set depth_model to " + BAR02, node.msg_if.warnings[0])
        self.teensy.sample()
        node.pollCb(None)
        self.assertEqual(len(node.msg_if.warnings), 1)
        # A Bar02 past 2000 mbar is beyond its range
        self.teensy.detect()
        self.teensy.sample(d1 = d1ForPressure(BAR02, BAR02_PROM, BAR02_D2, 2100.0))
        self.poll()
        self.assertIn("beyond the 2000 mbar top", self.node.msg_if.warnings[-1])

    def test_node_rechecks_config(self):
        for change in (dict(depth_model = 'BlueRobotics Bar100'), dict(depth_osr = 1000),
                       dict(fluid_density_kg_m3 = 1.0), dict(surface_pressure_mbar = 500.0)):
            with self.assertRaises(ValueError, msg = str(change)):
                makeNode(**change)


class TestDevicesOption(unittest.TestCase):

    def test_option_list_forms(self):
        disc = makeDiscovery()
        default = ['thrusters', 'servos']
        cases = [(['thrusters', 'depth_sensor'], ['thrusters', 'depth_sensor']),
                 # What devices held when it was a single Selection
                 ('thrusters,servos', ['thrusters', 'servos']),
                 ('servos', ['servos']),
                 # What an engine without multi-select driver options stores
                 ("['thrusters', 'servos']", ['thrusters', 'servos']),
                 (' thrusters , depth_sensor ', ['thrusters', 'depth_sensor']),
                 (['servos', 'servos', 'thrusters'], ['servos', 'thrusters']),
                 ([], []),
                 ('', [])]
        for (value, expected) in cases:
            self.assertEqual(disc.optionList({'devices': {'value': value}}, 'devices', default), expected, repr(value))
        self.assertEqual(disc.optionList({}, 'devices', default), default)
        self.assertEqual(disc.optionList({'devices': {'value': None}}, 'devices', default), default)

    def test_parts(self):
        disc = makeDiscovery()
        parts = disc.resolveParts(optionsDict(dict(devices = ['thrusters', 'depth_sensor'])))
        self.assertEqual(sorted(parts.keys()), sorted(Discovery.PARTS))
        self.assertIsInstance(parts['thrusters'], dict)
        self.assertIsNone(parts['servos'])
        self.assertEqual(parts['depth_sensor'], dict(depth_model = BAR02, depth_osr = 1024,
                                                     fluid_density_kg_m3 = 997.0, surface_pressure_mbar = 1013.25))
        # The default is thrusters and servos, as before depth existed
        parts = disc.resolveParts({})
        self.assertIsInstance(parts['thrusters'], dict)
        self.assertIsInstance(parts['servos'], dict)
        self.assertIsNone(parts['depth_sensor'])
    def test_unknown_part_changes_nothing(self):
        disc = makeDiscovery()
        self.assertIsNone(disc.resolveParts(optionsDict(dict(devices = ['thrusters', 'lights']))))
        self.assertIn("'lights'", disc.logger.warnings[-1])

    def test_empty_selection_changes_nothing(self):
        # The RUI cannot clear a multi-select; an old engine empties a saved value
        for value in ([], ''):
            disc = makeDiscovery()
            self.assertIsNone(disc.resolveParts(optionsDict(dict(devices = value))), repr(value))
            self.assertIn("update nepi_engine", disc.logger.warnings[-1])

    def test_depth_options(self):
        disc = makeDiscovery()
        config = disc.resolveDepthConfig(optionsDict(dict(depth_model = BAR30, depth_osr = '8192',
                                                          fluid_density_kg_m3 = '1029',
                                                          surface_pressure_mbar = '985.5')))
        self.assertEqual(config, dict(depth_model = BAR30, depth_osr = 8192, fluid_density_kg_m3 = 1029.0,
                                      surface_pressure_mbar = 985.5))
        for (name, value) in (('depth_model', 'Bar100'), ('depth_osr', '1000'), ('depth_osr', '16384'),
                              ('fluid_density_kg_m3', '1.0'), ('fluid_density_kg_m3', 'abc'),
                              ('surface_pressure_mbar', '1013.25 mbar'), ('surface_pressure_mbar', '500')):
            disc = makeDiscovery()
            self.assertEqual(disc.resolveDepthConfig(optionsDict({name: value})), 'invalid', name + " " + value)
            self.assertIn(name, disc.logger.warnings[-1])

    def test_node_accepts_discovery_config(self):
        config = makeDiscovery().resolveDepthConfig(optionsDict(dict(depth_osr = '256')))
        node = Node.__new__(Node)
        node.config = config
        node.resolveConfig()
        self.assertEqual(node.depth_osr, 256)
        self.assertEqual(node.depth_model, BAR02)
        self.assertEqual(sorted(depth_node.BAR_MODELS.keys()), sorted(Discovery.DEPTH_MODELS))
        self.assertEqual(depth_node.DEPTH_OSR_VALUES, Discovery.DEPTH_OSR_VALUES)


class FakeProcess:
    def poll(self):
        return None


IDENTITY_1_2 = dict(firmware_name = 'teensy_thrusters', protocol_version = '1.2', protocol_major = 1,
                    protocol_minor = 2, firmware_version = '1.2.0', thruster_count = 8, servo_channel = 8,
                    serial_number = '21616540', servo_count = 4)
BOARD = {'key': '21616540', 'path': '/dev/ttyACM0', 'serial_number': '21616540'}


class TestReconcile(unittest.TestCase):

    def makeDisc(self, identity = IDENTITY_1_2):
        disc = makeDiscovery()
        disc.launched = []
        disc.killed = []
        disc.findBoards = lambda: ([dict(BOARD)], True)
        disc.probeBoard = lambda board: dict(identity)

        def launch(launch_id, board, identity, part, servo_index, config):
            disc.launched.append(launch_id)
            disc.active_devices_dict[launch_id] = dict(node_name = launch_id, sub_process = FakeProcess(),
                                                       path = board['path'], board_key = board['key'],
                                                       part = part, config = copy.deepcopy(config))
            return True

        def kill(launch_id):
            disc.killed.append(launch_id)
            del disc.active_devices_dict[launch_id]
        disc.launchDeviceNode = launch
        disc.killDevice = kill
        return disc

    def runPass(self, disc, devices):
        drv_dict = {'DISCOVERY_DICT': {'OPTIONS': optionsDict(dict(devices = devices))}}
        disc.discoveryFunction([BOARD['path']], disc.active_paths_list, '/nepi/device1', drv_dict)

    def test_selecting_and_deselecting_parts(self):
        disc = self.makeDisc()
        self.runPass(disc, ['thrusters', 'servos'])
        self.assertEqual(disc.launched, ['21616540|servo0', '21616540|thrusters'])
        # Adding the depth sensor launches only its node
        self.runPass(disc, ['thrusters', 'servos', 'depth_sensor'])
        self.assertEqual(disc.launched[2:], ['21616540|depth_sensor'])
        self.assertEqual(disc.killed, [])
        # Dropping servos stops only the servo node
        self.runPass(disc, ['thrusters', 'depth_sensor'])
        self.assertEqual(disc.killed, ['21616540|servo0'])
        self.assertEqual(len(disc.launched), 3)
        # A legacy saved string means the same parts as the list
        self.runPass(disc, 'thrusters,servos')
        self.assertEqual(disc.killed, ['21616540|servo0', '21616540|depth_sensor'])
        self.assertEqual(disc.launched[3:], ['21616540|servo0'])
        # An unknown entry, or a value emptied by an old engine, changes nothing
        for devices in (['thrusters', 'lights'], []):
            self.runPass(disc, devices)
            self.assertEqual(len(disc.killed), 2)
            self.assertEqual(len(disc.launched), 4)

    def test_depth_needs_protocol_1_2(self):
        identity = dict(IDENTITY_1_2, protocol_version = '1.1', protocol_minor = 1, firmware_version = '1.1.0')
        disc = self.makeDisc(identity)
        self.runPass(disc, ['thrusters', 'depth_sensor'])
        self.assertEqual(disc.launched, ['21616540|thrusters'])
        self.assertIn("reflash it with firmware 1.2", disc.logger.warnings[-1])


if __name__ == '__main__':
    unittest.main()
