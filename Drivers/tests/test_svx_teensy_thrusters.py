#!/usr/bin/env python3
##
## Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
##
## This file is part of nepi-engine
## (see https://github.com/nepi-engine).
##
## License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
##

# Tests for the thrust map of Drivers/svx_drivers/Teensy/svx_teensy_thrusters_node.py
# and its options in svx_teensy_thrusters_discovery.py. Standard library only.
# Run from this folder:
#   python3 -B -m unittest -v test_svx_teensy_thrusters
#
# There is no ROS and no serial port here. Empty stand-ins for nepi_sdk,
# nepi_api and pyserial go into sys.modules only so the driver files import.
# The node class is never built (its __init__ starts ROS): instances come from
# Class.__new__ with only what the method under test reads. Node configs come
# from the discovery's resolveThrusterConfig, so the tests also run the
# options-to-node contract.
#
# The reference pulses in the thrust map tests come from an independent
# prototype of the map on the same 4-decimal table, so they are checked to
# +/- 1 us.

import os
import sys
import types
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..', 'svx_drivers', 'Teensy'))


def stubModules():
    names = ['nepi_sdk', 'nepi_sdk.nepi_sdk', 'nepi_sdk.nepi_utils', 'nepi_sdk.nepi_drvs', 'nepi_sdk.nepi_system',
             'nepi_api', 'nepi_api.device_if_motor', 'nepi_api.messages_if',
             'serial', 'serial.tools', 'serial.tools.list_ports']
    for name in names:
        sys.modules[name] = types.ModuleType(name)
    for name in names:
        if '.' in name:
            [parent, child] = name.rsplit('.', 1)
            setattr(sys.modules[parent], child, sys.modules[name])
    # The names the driver files import from them
    sys.modules['nepi_api.device_if_motor'].MotorsDeviceIF = None
    sys.modules['nepi_api.messages_if'].MsgIF = None

stubModules()

import svx_teensy_thrusters_node
import svx_teensy_thrusters_discovery

Node = svx_teensy_thrusters_node.SvxTeensyThrustersNode
Discovery = svx_teensy_thrusters_discovery.SvxTeensyThrustersDiscovery
TABLE = svx_teensy_thrusters_node.T200_THRUST_KGF
VOLTS = svx_teensy_thrusters_node.T200_THRUST_VOLTS
interpolateThrustRows = svx_teensy_thrusters_node.interpolateThrustRows
thrustToPulse = svx_teensy_thrusters_node.thrustToPulse

BASIC_ESC = 'BlueRobotics Basic ESC (BESC30-R3)'
T200 = 'BlueRobotics T200 (R2)'
THRUST_MAPS = ['thrust', 'thrust_symmetric']
# F5(a): exact-zero force runs around 1500 us, inclusive
ZERO_RUNS = {10.0: (1460, 1540), 12.0: (1464, 1536), 14.0: (1468, 1532),
             16.0: (1472, 1528), 18.0: (1472, 1528), 20.0: (1476, 1528)}


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

    def pub_warn(self, msg, throttle_s = None):
        self.warnings.append(msg)

    def pub_info(self, msg, throttle_s = None):
        pass


def makeDiscovery():
    disc = Discovery.__new__(Discovery)
    disc.logger = FakeLogger()
    disc.last_log_dict = dict()   # a class attribute; give each instance its own
    return disc


def optionsDict(values):
    # The shape drivers_mgr hands the discovery: every value a string
    return {name: {'value': str(value)} for (name, value) in values.items()}


def resolveOptions(**values):
    return makeDiscovery().resolveThrusterConfig(optionsDict(values))


def makeNode(config, thruster_count = 8):
    node = Node.__new__(Node)
    node.config = config
    node.thruster_count = thruster_count
    node.msg_if = FakeMsgIF()
    node.resolveConfig()
    node.motor_names = ['motor_' + str(i) for i in range(len(node.thruster_channels))]
    node.clamp_warned = set()
    return node


def nodeFromOptions(**values):
    config = resolveOptions(**values)
    if not isinstance(config, dict):
        raise AssertionError("options " + str(values) + " resolved to " + str(config))
    return makeNode(config)


def oldRatioToPulse(index, ratio, direction, min_us, stop_us, max_us, deadband_us,
                    max_thrust_ratio, reversed_thrusters):
    # ratioToPulse as it was before the thrust map: the reference for 'pulse'
    ratio = min(float(ratio), max_thrust_ratio)
    if ratio <= 0.0:
        return stop_us
    if direction >= 0:
        pulse = stop_us + deadband_us + ratio * (max_us - stop_us - deadband_us)
    else:
        pulse = stop_us - deadband_us - ratio * (stop_us - min_us - deadband_us)
    if index in reversed_thrusters:
        pulse = 2 * stop_us - pulse
    pulse = int(round(pulse))
    if pulse < min_us or pulse > max_us:
        pulse = max(min_us, min(max_us, pulse))
    return pulse


def envelopeAt(envelope, pulse):
    # The envelope's thrust at a pulse between its rows, linearly
    for k in range(1, len(envelope)):
        (p0, t0) = envelope[k - 1]
        (p1, t1) = envelope[k]
        if min(p0, p1) <= pulse <= max(p0, p1):
            return t0 + (pulse - p0) / float(p1 - p0) * (t1 - t0)
    raise ValueError("pulse " + str(pulse) + " is outside the envelope")


class TestThrustTable(unittest.TestCase):

    # T1
    def test_table_rows_and_values(self):
        self.assertEqual(len(TABLE), 201)
        self.assertEqual([row[0] for row in TABLE], list(range(1100, 1901, 4)))
        self.assertEqual(VOLTS, [10.0, 12.0, 14.0, 16.0, 18.0, 20.0])
        for row in TABLE:
            self.assertEqual(len(row), 7)
        at12 = dict((row[0], row[1 + VOLTS.index(12.0)]) for row in TABLE)
        self.assertEqual(at12[1100], -2.9037)
        self.assertEqual(at12[1104], -2.9166)
        self.assertEqual(at12[1500], 0.0)

    # T1
    def test_zero_runs(self):
        pulses = [row[0] for row in TABLE]
        for v in VOLTS:
            force = [row[1 + VOLTS.index(v)] for row in TABLE]
            lo = hi = pulses.index(1500)
            while lo > 0 and force[lo - 1] == 0.0:
                lo -= 1
            while hi < len(force) - 1 and force[hi + 1] == 0.0:
                hi += 1
            self.assertEqual((pulses[lo], pulses[hi]), ZERO_RUNS[v], str(v) + " V")

    # B5: envelopes are monotone, end at the full-scale thrust, and step over the dips
    def test_envelopes(self):
        node = nodeFromOptions(thrust_map = 'thrust', supply_voltage_v = 12.0)
        curve = node.thrust_curve
        for envelope in (curve['fwd'], curve['rev']):
            self.assertEqual(envelope[0][0], 1500)
            for k in range(1, len(envelope)):
                self.assertGreaterEqual(envelope[k][1], envelope[k - 1][1])
        self.assertEqual(curve['fwd'][-1][0], 1900)
        self.assertEqual(curve['rev'][-1][0], 1100)
        self.assertEqual(curve['fwd_max'], 3.7142)
        # 12 V reverse peaks at 1104 (2.9166) and dips at 1100 (2.9037)
        self.assertEqual(curve['rev_max'], 2.9166)
        self.assertEqual(dict(curve['rev'])[1100], 2.9166)
        # 12 V forward dips 1892 -> 1896
        self.assertEqual(dict(curve['fwd'])[1896], 3.7058)

    # T7, and B2's exact sheets and range
    def test_voltage_interpolation(self):
        rows = interpolateThrustRows(VOLTS, TABLE, 13.0)
        i12 = 1 + VOLTS.index(12.0)
        i14 = 1 + VOLTS.index(14.0)
        self.assertEqual(len(rows), len(TABLE))
        for (row, (pulse, kgf)) in zip(TABLE, rows):
            self.assertEqual(pulse, row[0])
            self.assertAlmostEqual(kgf, (row[i12] + row[i14]) / 2.0, delta = 1e-9)
        for v in VOLTS:
            column = [(row[0], row[1 + VOLTS.index(v)]) for row in TABLE]
            self.assertEqual(interpolateThrustRows(VOLTS, TABLE, v), column)
        for v in (9.9, 20.1, float('nan')):
            with self.assertRaises(ValueError):
                interpolateThrustRows(VOLTS, TABLE, v)


class TestPulseMap(unittest.TestCase):

    # T2: 'pulse' is the old formula exactly, Basic ESC and an off-center Custom ESC
    def test_pulse_map_matches_old_formula(self):
        escs = [dict(esc_model = BASIC_ESC, thruster_model = T200),
                dict(esc_model = 'Custom', thruster_model = 'Custom', custom_pulse_min_us = 1100,
                     custom_pulse_stop_us = 1600, custom_pulse_max_us = 1900, custom_deadband_us = 25)]
        clamped = 0
        for esc in escs:
            for cap in (1.0, 0.6):
                node = nodeFromOptions(thrust_map = 'pulse', max_thrust_ratio = cap, reversed_thrusters = '1,3', **esc)
                self.assertEqual(node.thrust_curve, None)
                for index in range(len(node.motor_names)):
                    for ratio in (0, 0.001, 0.3, 0.999, 1.0, 1.5):
                        for direction in (1, -1):
                            expected = oldRatioToPulse(index, ratio, direction, node.min_us, node.stop_us,
                                                       node.max_us, node.deadband_us, cap, [1, 3])
                            self.assertEqual(node.ratioToPulse(index, ratio, direction), expected,
                                             str((esc['esc_model'], cap, index, ratio, direction)))
                clamped += len(node.clamp_warned)
        # The off-center Custom ESC mirrors past 1900 us, so the clamp ran
        self.assertGreater(clamped, 0)


class TestThrustMap(unittest.TestCase):

    def assertPulse(self, got, expected, what):
        self.assertLessEqual(abs(got - expected), 1, what + ": got " + str(got) + ", expected " + str(expected))

    def checkPulses(self, thrust_map, volts, cases):
        node = nodeFromOptions(thrust_map = thrust_map, supply_voltage_v = volts)
        for (ratio, fwd, rev) in cases:
            what = thrust_map + " " + str(volts) + " V ratio " + str(ratio)
            self.assertPulse(node.ratioToPulse(0, ratio, 1), fwd, what + " forward")
            self.assertPulse(node.ratioToPulse(0, ratio, -1), rev, what + " reverse")
        return node

    # T3
    def test_thrust_12v(self):
        self.checkPulses('thrust', 12.0, [(1e-6, 1536, 1464), (0.1, 1597, 1404), (0.5, 1753, 1249),
                                          (1.0, 1900, 1104)])

    # T4
    def test_thrust_symmetric_12v(self):
        self.checkPulses('thrust_symmetric', 12.0, [(0.1, 1586, 1404), (0.5, 1718, 1249), (1.0, 1835, 1104)])

    # T5: at 20 V the data's reverse zero band starts inside the deadband
    def test_deadband_guard(self):
        node = self.checkPulses('thrust', 20.0, [(1e-6, 1528, 1474)])
        self.assertEqual(node.ratioToPulse(0, 1e-6, -1), 1474)
        unguarded = int(round(thrustToPulse(node.thrust_curve['rev'], 1e-6 * node.full_rev_kgf)))
        self.assertGreater(unguarded, 1474)
        self.checkPulses('thrust', 16.0, [(1e-6, 1528, 1472)])

    # T6
    def test_monotonic_and_outside_deadband(self):
        for volts in (10.0, 12.0, 13.3, 16.0, 20.0):
            for thrust_map in THRUST_MAPS:
                node = nodeFromOptions(thrust_map = thrust_map, supply_voltage_v = volts)
                last = None
                for k in range(1, 1001):
                    ratio = k / 1000.0
                    pulses = (node.ratioToPulse(0, ratio, 1), node.ratioToPulse(0, ratio, -1))
                    what = thrust_map + " " + str(volts) + " V ratio " + str(ratio) + ": " + str(pulses)
                    for pulse in pulses:
                        self.assertTrue(1100 <= pulse <= 1900, what)
                        self.assertFalse(1475 <= pulse <= 1525, what)
                    if last is not None:
                        self.assertGreaterEqual(pulses[0], last[0], what)
                        self.assertLessEqual(pulses[1], last[1], what)
                    last = pulses

    # T8
    def test_symmetric_balance(self):
        for volts in (12.0, 16.0):
            node = nodeFromOptions(thrust_map = 'thrust_symmetric', supply_voltage_v = volts)
            for k in range(1, 11):
                ratio = k / 10.0
                fwd = envelopeAt(node.thrust_curve['fwd'], node.ratioToPulse(0, ratio, 1))
                rev = envelopeAt(node.thrust_curve['rev'], node.ratioToPulse(0, ratio, -1))
                self.assertLessEqual(abs(fwd - rev), 0.06,
                                     str(volts) + " V ratio " + str(ratio) + ": " + str((fwd, rev)))

    # T9: reversal flips the direction into the map, not the pulse out of it
    def test_reversed_thruster(self):
        for thrust_map in THRUST_MAPS:
            node = nodeFromOptions(thrust_map = thrust_map, reversed_thrusters = '1')
            for ratio in (1e-6, 0.001, 0.1, 0.37, 0.5, 0.9, 1.0):
                self.assertEqual(node.ratioToPulse(1, ratio, 1), node.ratioToPulse(0, ratio, -1))
                self.assertEqual(node.ratioToPulse(1, ratio, -1), node.ratioToPulse(0, ratio, 1))
        # A mirror of full forward (1900) would be 1100; full reverse peaks at 1104
        node = nodeFromOptions(thrust_map = 'thrust', supply_voltage_v = 12.0, reversed_thrusters = '1')
        self.assertNotEqual(node.ratioToPulse(1, 1.0, 1), 2 * node.stop_us - node.ratioToPulse(0, 1.0, 1))

    # T10
    def test_max_thrust_ratio_caps_before_map(self):
        node = nodeFromOptions(thrust_map = 'thrust', max_thrust_ratio = 0.5)
        for direction in (1, -1):
            self.assertEqual(node.ratioToPulse(0, 0.8, direction), node.ratioToPulse(0, 0.5, direction))
        self.assertEqual(node.ratioToPulse(0, 0.0, 1), node.stop_us)

    # B8
    def test_log_text(self):
        node = nodeFromOptions(thrust_map = 'thrust', supply_voltage_v = 12.0)
        self.assertEqual(node.thrustMapText(),
                         "thrust_map thrust at 12.0 V, full scale 3.71 kgf forward / 2.92 kgf reverse")
        node = nodeFromOptions(thrust_map = 'thrust_symmetric', supply_voltage_v = 12.0)
        self.assertEqual(node.thrustMapText(),
                         "thrust_map thrust_symmetric at 12.0 V, full scale 2.92 kgf forward / 2.92 kgf reverse")
        self.assertEqual(nodeFromOptions(thrust_map = 'pulse').thrustMapText(), "thrust_map pulse")

    # B3: the node re-checks what discovery checks
    def test_node_rechecks_config(self):
        base = resolveOptions(thrust_map = 'pulse', esc_model = 'Custom', thruster_model = 'Custom')
        for change in (dict(thrust_map = 'thrust', supply_voltage_v = 12.0),
                       dict(thrust_map = 'thrust', supply_voltage_v = 12.0, esc_model = BASIC_ESC),
                       dict(thrust_map = 'thrust', supply_voltage_v = 12.0, thruster_model = T200),
                       dict(thrust_map = 'linear'),
                       dict(thruster_model = 'T100')):
            config = dict(base)
            config.update(change)
            with self.assertRaises(ValueError, msg = str(change)):
                makeNode(config)
        config = resolveOptions(thrust_map = 'thrust')
        config['supply_voltage_v'] = 25.0
        with self.assertRaises(ValueError):
            makeNode(config)


class TestThrustOptions(unittest.TestCase):

    def assertInvalid(self, names, **values):
        disc = makeDiscovery()
        self.assertEqual(disc.resolveThrusterConfig(optionsDict(values)), 'invalid', str(values))
        for name in names:
            self.assertIn(name, disc.logger.warnings[-1])

    # T11
    def test_invalid(self):
        custom_esc = dict(esc_model = 'Custom', custom_pulse_min_us = 1100, custom_pulse_stop_us = 1500,
                          custom_pulse_max_us = 1900, custom_deadband_us = 25)
        for thrust_map in THRUST_MAPS:
            self.assertInvalid(['thruster_model', 'Custom'], thrust_map = thrust_map, thruster_model = 'Custom')
            self.assertInvalid(['esc_model', 'Custom', BASIC_ESC], thrust_map = thrust_map, **custom_esc)
            for volts in ('9.9', '20.1', 'abc'):
                self.assertInvalid(['supply_voltage_v'], thrust_map = thrust_map, supply_voltage_v = volts)
        # A thrust map is the default, so Custom without thrust_map is invalid too
        self.assertInvalid(['esc_model'], **custom_esc)
        for thrust_map in ('pulse', 'thrust'):
            self.assertInvalid(['thruster_model', 'T100'], thrust_map = thrust_map, thruster_model = 'T100')
        self.assertInvalid(['thrust_map', 'linear'], thrust_map = 'linear')

    # T11
    def test_valid(self):
        config = resolveOptions(thrust_map = 'pulse', supply_voltage_v = 'abc')
        self.assertIsInstance(config, dict)
        self.assertEqual(config['thrust_map'], 'pulse')
        self.assertNotIn('supply_voltage_v', config)
        config = resolveOptions()
        self.assertIsInstance(config, dict)
        self.assertEqual(config['thrust_map'], 'thrust')
        self.assertEqual(config['thruster_model'], T200)
        self.assertEqual(config['supply_voltage_v'], 12.0)
        # pulse works with any thruster_model and esc_model
        config = resolveOptions(thrust_map = 'pulse', thruster_model = 'Custom', esc_model = 'Custom')
        self.assertIsInstance(config, dict)
        self.assertNotIn('supply_voltage_v', config)
        for volts in ('10', '13.3', '20.0'):
            self.assertEqual(resolveOptions(supply_voltage_v = volts)['supply_voltage_v'], float(volts))

    # B2: supply_voltage_v restarts the node only under a thrust map
    def test_supply_voltage_restart_scope(self):
        self.assertEqual(resolveOptions(thrust_map = 'pulse', supply_voltage_v = '12.0'),
                         resolveOptions(thrust_map = 'pulse', supply_voltage_v = '16.0'))
        self.assertNotEqual(resolveOptions(thrust_map = 'thrust', supply_voltage_v = '12.0'),
                            resolveOptions(thrust_map = 'thrust', supply_voltage_v = '16.0'))


if __name__ == '__main__':
    unittest.main()
