#!/usr/bin/env python3
##
## Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
##
## This file is part of nepi-engine
## (see https://github.com/nepi-engine).
##
## License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
##

# Tests for the MateRovLink class in Drivers/rbx_drivers/rbx_mate_rov_link_node.py
# and the probe in rbx_mate_rov_link_discovery.py, against the real Pi side,
# Nepi_Connect/pi/nepi_link.py. Standard library only. Run from this folder:
#   python3 -B -m unittest -v test_rbx_mate_rov_link
#
# There is no ROS here. Empty stand-ins for nepi_sdk, nepi_api and the two ROS
# message packages go into sys.modules only so the driver files import; the
# node class is never built, and the discovery class only as far as its probe.
#
# Bench tests join one MateRovLink and one NepiLink over real UDP on
# 127.0.0.1, with free ports and NEPI_LAN_IP 127.0.0.1, under one injected
# clock; the NepiLink's threads are not started, so each step runs in a fixed
# order. TestLoopback starts both sides' threads in real time.

import json
import math
import os
import random
import socket
import sys
import threading
import time
import types
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..', 'pi'))
sys.path.insert(0, os.path.join(HERE, '..', '..', 'Drivers', 'rbx_drivers'))


def stubModules():
    names = ['nepi_sdk', 'nepi_sdk.nepi_sdk', 'nepi_sdk.nepi_nav', 'nepi_sdk.nepi_utils', 'nepi_sdk.nepi_drvs',
             'nepi_sdk.nepi_system', 'nepi_sdk.nepi_controls', 'nepi_api', 'nepi_api.device_if_rbx',
             'nepi_api.device_if_svx', 'nepi_api.messages_if',
             'std_msgs', 'std_msgs.msg', 'nepi_interfaces', 'nepi_interfaces.msg']
    for name in names:
        sys.modules[name] = types.ModuleType(name)
    for name in names:
        if '.' in name:
            [parent, child] = name.rsplit('.', 1)
            setattr(sys.modules[parent], child, sys.modules[name])
    # The names the driver files import from them
    sys.modules['nepi_api.device_if_rbx'].RBXRobotIF = None
    sys.modules['nepi_api.device_if_svx'].SVXActuatorIF = None
    sys.modules['nepi_api.messages_if'].MsgIF = None
    sys.modules['std_msgs.msg'].String = None
    sys.modules['std_msgs.msg'].Empty = None
    sys.modules['nepi_interfaces.msg'].AxisControls = None

stubModules()

import nepi_link
import rbx_mate_rov_link_node
import rbx_mate_rov_link_discovery
import rbx_mate_rov_link_servo_node

MateRovLink = rbx_mate_rov_link_node.MateRovLink
MateRovLinkDiscovery = rbx_mate_rov_link_discovery.MateRovLinkDiscovery
ImuMount = rbx_mate_rov_link_node.ImuMount

ROV = '127.0.0.1' # Both the ROV Pi's address and NEPI_LAN_IP here
OTHER = '127.0.0.2' # A sender that is not the ROV
WALL_EPOCH = 1792200000.0
SURGE_0_1 = 5706 # stick_value(0.1)
BENCH_LINE = '$TNCTL,0,0,0,0,100,100,100,100,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,*FF'
# Two claw servos on the Pi: D-pad up/down and the A button
CLAW_SERVOS = dict(NEPI_SERVO_1_NAME = 'claw_pitch', NEPI_SERVO_1_INPUT = 'ABS_HAT0Y',
                   NEPI_SERVO_2_NAME = 'claw_grip', NEPI_SERVO_2_INPUT = 'BTN_SOUTH')
HAT0Y_SLOT = nepi_link.GAMEPAD_SLOTS['ABS_HAT0Y']
SOUTH_SLOT = nepi_link.GAMEPAD_SLOTS['BTN_SOUTH']


def teensyAngles(mount, roll_deg, pitch_deg, yaw_deg):
    # What the team's sketch prints for a ROV at this orientation with its
    # board mounted this way: the board's rotation as a quaternion, through
    # new_ctl.ino:285-287 verbatim, printed to 2 decimals
    rov = mount.rotationFromEuler(roll_deg, pitch_deg, yaw_deg)
    m = [[sum(rov[i][k] * mount.matrix[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
    trace = m[0][0] + m[1][1] + m[2][2]
    if trace > 0:
        s = math.sqrt(trace + 1.0) * 2
        [qr, qi, qj, qk] = [0.25 * s, (m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s, (m[1][0] - m[0][1]) / s]
    elif m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2
        [qr, qi, qj, qk] = [(m[2][1] - m[1][2]) / s, 0.25 * s, (m[0][1] + m[1][0]) / s, (m[0][2] + m[2][0]) / s]
    elif m[1][1] > m[2][2]:
        s = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2
        [qr, qi, qj, qk] = [(m[0][2] - m[2][0]) / s, (m[0][1] + m[1][0]) / s, 0.25 * s, (m[1][2] + m[2][1]) / s]
    else:
        s = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2
        [qr, qi, qj, qk] = [(m[1][0] - m[0][1]) / s, (m[0][2] + m[2][0]) / s, (m[1][2] + m[2][1]) / s, 0.25 * s]
    [sqr, sqi, sqj, sqk] = [qr * qr, qi * qi, qj * qj, qk * qk]
    yaw = math.atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr))
    pitch = math.asin(max(-1.0, min(1.0, -2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr))))
    roll = math.atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr))
    return [round(math.degrees(angle), 2) for angle in (roll, pitch, yaw)]


def angleError(a_deg, b_deg):
    return abs((a_deg - b_deg + 180.0) % 360.0 - 180.0)


class FakeClock(object):

    def __init__(self, now = 1000.0):
        self.now = now

    def __call__(self):
        return self.now

    def advance(self, seconds):
        self.now += seconds


def freeUdpPort():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('127.0.0.1', 0))
    port = sock.getsockname()[1]
    sock.close()
    return port

def freeUdpPorts():
    status_port = freeUdpPort()
    demand_port = freeUdpPort()
    while demand_port == status_port:
        demand_port = freeUdpPort()
    return status_port, demand_port

def waitFor(predicate, timeout_sec = 1.0):
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.002)
    return predicate()

def pilot(rb = 0, ly = 0, slots = None):
    inputs = [0] * 18
    inputs[nepi_link.RB_SLOT] = rb
    inputs[nepi_link.LY_SLOT] = ly
    if slots is not None:
        for [slot, value] in slots.items():
            inputs[slot] = value
    return inputs

def statusBytes(**changes):
    status = dict(v = 1, type = 'status', seq = 0, t = WALL_EPOCH, assist = False, assist_reason = 'start',
                  pilot_surge = False, pilot_sway = False, pilot_yaw = False, pilot_depth = False, slow = False,
                  out_lx = 0, out_ly = 0, out_lt = 0, out_rt = 0, demand_seq = -1, demand_age_ms = -1,
                  demand_bad = 0, tn = None, tn_age_ms = -1, tn_count = 0, tn_bad = 0)
    status.update(changes)
    return json.dumps(status).encode('utf-8')

def openSocket(bind_ip):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((bind_ip, 0))
    return sock


class Bench(object):
    """One MateRovLink and one NepiLink joined over UDP on 127.0.0.1, one clock.

    step() is one 100 ms cycle in a fixed order: the link's send tick, the
    demand across UDP into the Pi's handle_demand(), the pilot's inputs
    through the Pi's merge, then one Pi status across UDP into the link's
    receive thread, and the clock advances 0.1 s.
    """

    def __init__(self, **overrides):
        self.clock = FakeClock()
        self.wall_clock = lambda: WALL_EPOCH + self.clock.now
        [self.status_port, self.demand_port] = freeUdpPorts()
        self.config = dict(NEPI_LAN_IP = ROV, NEPI_STATUS_UDP_PORT = self.status_port,
                           NEPI_DEMAND_UDP_PORT = self.demand_port)
        self.config.update(overrides)
        self.pi_logs = []
        self.link_logs = []
        self.pi = None
        self.restartPi()
        # Stands in for the Pi's demand port and status socket
        self.demand_rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.demand_rx.bind((ROV, self.demand_port))
        self.demand_rx.settimeout(1.0)
        self.status_tx = openSocket(ROV)
        self.link = None
        self.restartLink()
        self.merged = None

    def restartPi(self):
        self.pi = nepi_link.NepiLink(clock = self.clock, wall_clock = self.wall_clock, log = self.pi_logs.append)
        [on, msg] = self.pi.configure(self.config)
        if not on:
            raise AssertionError('Pi did not configure: ' + msg)

    def restartLink(self):
        if self.link is not None:
            self.link.stop()
        self.link = MateRovLink(ROV, self.status_port, self.demand_port, bind_ip = ROV, clock = self.clock,
                                wall_clock = self.wall_clock, log = self.link_logs.append)
        [success, msg] = self.link.start()
        if not success:
            raise AssertionError('link did not start: ' + msg)

    def close(self):
        self.link.stop()
        self.demand_rx.close()
        self.status_tx.close()

    def sendStatus(self, datagram, sock = None):
        if sock is None:
            sock = self.status_tx
        seq = json.loads(datagram.decode('utf-8'))['seq']
        bad = self.link.status_bad
        sock.sendto(datagram, (ROV, self.status_port))
        if not waitFor(lambda: self.link.status_seq == seq or self.link.status_bad > bad):
            raise AssertionError('status ' + str(seq) + ' never reached the link')

    def step(self, command = None, rb = 0, status = True, ly = 0, slots = None):
        if command is not None:
            self.link.set_teleop_velocity(*command)
        demand = None
        [sent, msg] = self.link.send_demand()
        if sent:
            [datagram, sender] = self.demand_rx.recvfrom(65536)
            demand = json.loads(datagram.decode('utf-8'))
            self.pi.handle_demand(datagram, sender[0])
        self.merged = self.pi.merge_inputs(pilot(rb = rb, ly = ly, slots = slots))
        if status:
            self.sendStatus(self.pi.nextStatusDatagram())
        self.clock.advance(0.1)
        return demand

    def steps(self, count, command = None, status = True):
        return [self.step(command, status = status) for i in range(count)]

    def piState(self):
        status = self.pi.build_status()
        return status['assist'], status['assist_reason']

    def nothingSent(self):
        self.demand_rx.settimeout(0.05)
        try:
            self.demand_rx.recvfrom(65536)
            return False
        except socket.timeout:
            return True
        finally:
            self.demand_rx.settimeout(1.0)

    def handOver(self):
        # The pilot presses and releases RB; NEPI must already be ready
        self.step(rb = 1)
        if self.piState() != (True, 'pilot_button'):
            raise AssertionError('RB did not hand over: ' + str(self.piState()))

    def engage(self, command):
        # Link up and NEPI ready, RB, then the command's first demand
        self.steps(3)
        self.handOver()
        return self.step(command)


class BenchCase(unittest.TestCase):

    def setUp(self):
        self.bench = Bench(NEPI_SURGE_SIGN = 1, NEPI_SWAY_SIGN = 1, NEPI_YAW_SIGN = 1)

    def tearDown(self):
        self.bench.close()


class TestIdle(BenchCase):

    def test_idle_sends_active_true_every_tick_and_the_pilot_keeps_the_rov(self):
        self.assertEqual(MateRovLink.DEMAND_PERIOD_SEC, 0.1)
        b = self.bench
        self.assertIsNone(b.step()) # No status yet: the link is down
        self.assertEqual(b.pi.topside_tag(), ',NEPI:NOTREADY')
        demands = b.steps(10)
        for demand in demands:
            self.assertIsNotNone(demand)
            self.assertTrue(demand['active'])
            self.assertEqual([demand[name] for name in ['surge', 'sway', 'heave', 'yaw', 'pitch', 'roll']], [0.0] * 6)
        # One demand per 0.1 s tick, each accepted in order
        self.assertEqual([d['seq'] for d in demands], list(range(demands[0]['seq'], demands[0]['seq'] + 10)))
        status = b.pi.build_status()
        self.assertEqual((status['demand_seq'], status['demand_bad']), (demands[-1]['seq'], 0))
        # Ready, but only RB hands the ROV over
        self.assertEqual(b.piState(), (False, 'start'))
        self.assertEqual(b.pi.topside_tag(), ',NEPI:OFF')
        health = b.link.get_health()
        self.assertEqual((health['link_up'], health['nepi_control'], health['assist']), (True, False, False))
        self.assertTrue(b.link.is_link_up())
        self.assertFalse(b.link.is_nepi_control())
        # Logged once, not per message
        self.assertEqual(len(b.link_logs), 1, b.link_logs)


class TestHandover(BenchCase):

    def test_rb_hands_over_and_the_command_merges(self):
        b = self.bench
        demand = b.engage((0.1, 0.0, 0.0, 0.0))
        self.assertEqual((demand['active'], demand['surge'], demand['sway'], demand['yaw']), (True, 0.1, 0.0, 0.0))
        self.assertEqual(b.pi.settings['NEPI_SURGE_SIGN'], 1)
        self.assertEqual(b.merged[nepi_link.LY_SLOT], SURGE_0_1)
        self.assertEqual(b.pi.topside_tag(), ',NEPI:ON')
        health = b.link.get_health()
        self.assertEqual((health['nepi_control'], health['assist'], health['assist_reason']), (True, True, 'pilot_button'))
        self.assertTrue(b.link.is_nepi_control())
        self.assertGreaterEqual(health['demand_age_ms'], 0)
        self.assertTrue(any(line.startswith('NEPI control on') for line in b.link_logs), b.link_logs)

    def test_commands_before_the_handover_are_dropped(self):
        b = self.bench
        command = (0.1, 0.0, 0.0, 0.0)
        demands = b.steps(4, command)
        self.assertEqual([d['surge'] for d in demands[1:]], [0.0] * 3)
        self.assertIsNone(b.link.cmd)
        self.assertEqual(b.merged[nepi_link.LY_SLOT], 0)
        # The handover starts neutral; only a command after it moves the ROV
        b.handOver()
        self.assertEqual(b.merged[nepi_link.LY_SLOT], 0)
        self.assertEqual(b.step()['surge'], 0.0)
        self.assertEqual(b.step(command)['surge'], 0.1)
        self.assertEqual(b.merged[nepi_link.LY_SLOT], SURGE_0_1)

    def test_ros_axes_map_to_protocol_axes(self):
        b = self.bench
        demand = b.engage((0.0, 0.1, 0.0, 0.05))
        self.assertEqual((demand['surge'], demand['sway'], demand['yaw']), (0.0, -0.1, -0.05))
        # Negative yaw goes on RT, not LT; negative sway is left stick left
        self.assertEqual(b.merged[nepi_link.LT_SLOT], 0)
        self.assertEqual(b.merged[nepi_link.RT_SLOT], nepi_link.trigger_value(0.05, 10, 255))
        self.assertGreater(b.merged[nepi_link.RT_SLOT], 0)
        self.assertEqual(b.merged[nepi_link.LX_SLOT], nepi_link.stick_value(-0.1))
        # Up (ROS linear.z positive) is negative heave, which drives RY
        demand = b.step((0.0, 0.0, 0.1, 0.0))
        self.assertEqual((demand['active'], demand['surge'], demand['sway'], demand['heave'], demand['yaw']),
                         (True, 0.0, 0.0, -0.1, 0.0))
        self.assertEqual(b.merged[nepi_link.RY_SLOT], nepi_link.stick_value(-0.1))
        # Heave back to 0 leaves RY at 0, so the Teensy holds the new depth
        b.step((0.1, 0.0, 0.0, 0.0))
        self.assertEqual((b.merged[nepi_link.LY_SLOT], b.merged[nepi_link.RY_SLOT]), (SURGE_0_1, 0))
        # Each axis is clamped to -1 to 1; the Pi applies its own caps
        demand = b.step((2.0, -3.0, 4.0, 5.0))
        self.assertEqual((demand['surge'], demand['sway'], demand['heave'], demand['yaw']), (1.0, 1.0, -1.0, -1.0))
        self.assertEqual(b.pi.build_status()['demand_bad'], 0)

    def test_bad_command_values_are_not_stored(self):
        b = self.bench
        b.engage((0.1, 0.0, 0.0, 0.0))
        for bad in [float('nan'), float('inf'), True, '0.1', None]:
            [success, msg] = b.link.set_teleop_velocity(bad, 0.0, 0.0, 0.0)
            self.assertFalse(success)
            [success, msg] = b.link.set_teleop_velocity(0.0, 0.0, bad, 0.0)
            self.assertFalse(success)
        self.assertEqual(b.link.cmd, [0.1, 0.0, 0.0, 0.0])


class TestNepiControl(BenchCase):

    def test_rb_again_hands_the_rov_back(self):
        b = self.bench
        command = (0.1, 0.0, 0.0, 0.0)
        b.engage(command)
        b.step(command, rb = 1)
        self.assertEqual(b.piState(), (False, 'pilot_button'))
        self.assertFalse(b.link.is_nepi_control())
        # Commands still arriving are dropped, and NEPI stays ready
        demands = b.steps(3, command)
        self.assertEqual([(d['active'], d['surge']) for d in demands], [(True, 0.0)] * 3)
        self.assertEqual(b.merged[nepi_link.LY_SLOT], 0)
        self.assertEqual(b.pi.topside_tag(), ',NEPI:OFF')
        self.assertTrue(any(line.startswith('NEPI control off (pilot_button)') for line in b.link_logs), b.link_logs)
        # RB once more hands it over again
        b.handOver()
        self.assertEqual(b.step(command)['surge'], 0.1)
        self.assertEqual(b.merged[nepi_link.LY_SLOT], SURGE_0_1)

    def test_a_stick_past_the_takeover_point_hands_the_rov_back(self):
        b = self.bench
        command = (0.1, 0.0, 0.0, 0.0)
        b.engage(command)
        b.step(command, ly = 20000)
        self.assertEqual(b.piState(), (False, 'pilot_stick'))
        self.assertEqual(b.merged[nepi_link.LY_SLOT], 20000)
        self.assertEqual(b.link.get_health()['assist_reason'], 'pilot_stick')
        self.assertFalse(b.link.is_nepi_control())
        self.assertEqual(b.step(command)['surge'], 0.0)

    def test_stale_command_goes_neutral_and_nepi_keeps_the_rov(self):
        self.assertEqual(MateRovLink.TELEOP_CMD_TIMEOUT_SEC, 0.5)
        b = self.bench
        b.engage((0.1, 0.0, 0.0, 0.0))
        # Fresh for 0.4 s after the command, neutral by 0.7 s
        self.assertEqual([d['surge'] for d in b.steps(4)], [0.1] * 4)
        b.steps(2)
        demand = b.step()
        self.assertEqual((demand['active'], demand['surge']), (True, 0.0))
        self.assertEqual(b.merged[nepi_link.LY_SLOT], 0)
        self.assertEqual(b.piState(), (True, 'pilot_button'))
        self.assertEqual(b.pi.topside_tag(), ',NEPI:ON')

    def test_stop_goes_neutral_and_nepi_keeps_the_rov(self):
        b = self.bench
        b.engage((0.1, 0.0, 0.0, 0.0))
        [success, msg] = b.link.go_stop()
        self.assertTrue(success, msg)
        self.assertIsNone(b.link.cmd)
        demand = b.step()
        self.assertEqual((demand['active'], demand['surge']), (True, 0.0))
        self.assertEqual(b.merged[nepi_link.LY_SLOT], 0)
        self.assertEqual(b.piState(), (True, 'pilot_button'))

    def test_release_to_pilot_sends_one_active_false(self):
        b = self.bench
        command = (0.1, 0.0, 0.0, 0.0)
        b.engage(command)
        [success, msg] = b.link.release_to_pilot()
        self.assertTrue(success, msg)
        demands = b.steps(3, command)
        self.assertEqual([d['active'] for d in demands], [False, True, True])
        self.assertEqual([d['surge'] for d in demands], [0.0] * 3)
        self.assertEqual(b.piState(), (False, 'nepi_off'))
        self.assertFalse(b.link.is_nepi_control())
        self.assertEqual(b.pi.topside_tag(), ',NEPI:OFF')
        # The pilot can hand it over again
        b.handOver()
        self.assertTrue(b.link.is_nepi_control())

    def test_release_while_the_link_is_down_fails(self):
        b = self.bench
        [success, msg] = b.link.release_to_pilot()
        self.assertFalse(success)
        self.assertIn('Link down', msg)

    def test_restarted_pi_script_starts_with_the_pilot(self):
        b = self.bench
        command = (0.1, 0.0, 0.0, 0.0)
        b.engage(command)
        # The Pi script restarts: over 1 s of silence while commands keep coming
        b.steps(12, command, status = False)
        self.assertFalse(b.link.is_link_up())
        self.assertFalse(b.link.is_nepi_control())
        b.restartPi()
        self.assertIsNone(b.step(command)) # Its first status brings the link up
        demands = b.steps(3, command)
        self.assertEqual([(d['active'], d['surge']) for d in demands], [(True, 0.0)] * 3)
        self.assertEqual(b.piState(), (False, 'start'))
        self.assertFalse(b.link.is_nepi_control())
        b.handOver()
        self.assertEqual(b.step(command)['surge'], 0.1)


class TestLinkDown(BenchCase):

    def test_status_silence_takes_the_link_down_and_stops_demands(self):
        b = self.bench
        command = (0.1, 0.0, 0.0, 0.0)
        b.pi.on_teensy_line(BENCH_LINE)
        b.engage(command)
        self.assertEqual(MateRovLink.LINK_TIMEOUT_SEC, 0.5)
        # 0.1 s to 0.4 s after the last status: still up, still sending
        for i in range(4):
            self.assertIsNotNone(b.step(command, status = False))
        b.steps(2, command, status = False)
        # Over 0.5 s: down, not ready, and nothing sent
        self.assertFalse(b.link.is_link_up())
        self.assertFalse(b.link.is_nepi_control())
        self.assertFalse(b.link.get_health()['link_up'])
        [sent, msg] = b.link.send_demand()
        self.assertFalse(sent, msg)
        self.assertTrue(b.nothingSent())
        self.assertTrue(any(line.startswith('Link down') for line in b.link_logs), b.link_logs)
        # The Teensy telemetry ages with the status, so the navpose skips it
        [tn, tn_age_ms] = b.link.get_tn()
        self.assertIsNotNone(tn)
        self.assertGreater(tn_age_ms, rbx_mate_rov_link_node.MateRovLinkNode.TN_MAX_AGE_MS)
        self.assertGreater(b.link.get_health()['demand_age_ms'], 500)

    def test_status_from_the_wrong_address_is_ignored(self):
        b = self.bench
        other = openSocket(OTHER)
        try:
            b.sendStatus(b.pi.nextStatusDatagram(), other)
        finally:
            other.close()
        self.assertEqual(b.link.get_health()['status_bad'], 1)
        self.assertFalse(b.link.is_link_up())
        self.assertFalse(b.link.send_demand()[0])
        # The same status from the ROV address is accepted
        b.sendStatus(b.pi.nextStatusDatagram())
        self.assertTrue(b.link.is_link_up())


class TestSeq(BenchCase):

    def test_restarted_link_is_ahead_of_its_previous_run(self):
        b = self.bench
        demands = b.steps(6)
        last_seq = demands[-1]['seq']
        b.restartLink()
        self.assertIsNone(b.step()) # Down until its first status
        demand = b.step()
        # Within SEQ_RESTART_SEC of the old run's last demand, and still accepted
        self.assertGreater(demand['seq'], last_seq)
        status = b.pi.build_status()
        self.assertEqual((status['demand_seq'], status['demand_bad']), (demand['seq'], 0))
        # A link that restarted at seq 0 would have been dropped
        stale = json.dumps(dict(demand, seq = 0)).encode('utf-8')
        self.assertFalse(b.pi.handle_demand(stale, ROV)[0])

    def test_first_seq_comes_from_the_wall_clock_and_fits(self):
        link = MateRovLink(ROV, wall_clock = lambda: WALL_EPOCH)
        self.assertEqual(link.seq, int(WALL_EPOCH * MateRovLink.SEQ_PER_SEC) & MateRovLink.SEQ_MAX)
        link = MateRovLink(ROV, wall_clock = lambda: 1.0e12)
        self.assertTrue(0 <= link.seq <= MateRovLink.SEQ_MAX)


class TestNavPose(unittest.TestCase):
    # getNavPoseCb on a stand-in self, since the node class is never built

    def setUp(self):
        rbx_mate_rov_link_node.nepi_utils.get_time = lambda: 1000.0
        self.tn = None
        self.tn_age_ms = -1
        link = types.SimpleNamespace(get_tn = lambda: [self.tn, self.tn_age_ms])
        self.node = types.SimpleNamespace(link = link, TN_MAX_AGE_MS = rbx_mate_rov_link_node.MateRovLinkNode.TN_MAX_AGE_MS,
                                          navpose_dict = dict(has_depth = False, has_orientation = False),
                                          imu_mount = ImuMount())

    def navpose(self):
        return rbx_mate_rov_link_node.MateRovLinkNode.getNavPoseCb(self.node)

    def test_none_until_the_first_fresh_telemetry(self):
        # None makes NPXDeviceIF wait instead of fixing has_depth False for good
        self.assertIsNone(self.navpose())
        self.tn = dict(depth_m = '0.03', depth_m_s = '0.00', yaw_deg = '0.00', pitch_deg = '0.00', roll_deg = '0.00',
                       yaw_deg_s = '0.00', pitch_deg_s = '0.00', roll_deg_s = '0.00')
        self.tn_age_ms = self.node.TN_MAX_AGE_MS + 1
        self.assertIsNone(self.navpose())
        self.tn_age_ms = 100
        navpose = self.navpose()
        self.assertTrue(navpose['has_depth'])
        self.assertEqual(navpose['depth_m'], 0.03)
        self.assertAlmostEqual(navpose['time_depth'], 999.9)
        self.assertTrue(navpose['has_orientation'])
        # Stale after that: the last navpose, not None
        self.tn_age_ms = self.node.TN_MAX_AGE_MS + 1
        self.assertIs(self.navpose(), navpose)

    def test_depth_rate_frame_and_the_flat_mount(self):
        # The flat mount passes the angles through and reads the rate columns
        # as gyro X, Y, Z, so the sketch's yaw_deg_s lands on roll
        self.tn = dict(depth_m = 1.25, depth_m_s = -0.12, yaw_deg = 40.0, pitch_deg = -3.0, roll_deg = 2.0,
                       yaw_deg_s = 10.0, pitch_deg_s = 2.0, roll_deg_s = 3.0)
        self.tn_age_ms = 100
        navpose = self.navpose()
        self.assertEqual(navpose['frame_depth'], 'DEPTH')
        self.assertEqual(navpose['depth_m_per_sec'], -0.12)
        self.assertEqual((navpose['roll_deg'], navpose['pitch_deg'], navpose['yaw_deg']), (2.0, -3.0, 40.0))
        self.assertEqual((navpose['roll_deg_per_sec'], navpose['pitch_deg_per_sec'], navpose['yaw_deg_per_sec']),
                         (10.0, 2.0, 3.0))

    def test_side_mount_turns_orientation_and_rates_into_the_rov_frame(self):
        # The team's mount (2026-10-26): X vertical. Y left is assumed here.
        self.node.imu_mount = ImuMount('X_UP_Y_LEFT')
        [roll, pitch, yaw] = teensyAngles(self.node.imu_mount, 2.0, -3.0, 40.0)
        self.tn = dict(depth_m = 1.25, depth_m_s = 0.0, yaw_deg = yaw, pitch_deg = pitch, roll_deg = roll,
                       yaw_deg_s = 10.0, pitch_deg_s = 2.0, roll_deg_s = 3.0)
        self.tn_age_ms = 100
        navpose = self.navpose()
        self.assertLess(angleError(navpose['roll_deg'], 2.0), 0.02)
        self.assertLess(angleError(navpose['pitch_deg'], -3.0), 0.02)
        self.assertLess(angleError(navpose['yaw_deg'], 40.0), 0.02)
        # Gyro X is up (yaw), Y is left (pitch), and Z = X x Y points back, so roll is -Z
        self.assertEqual((navpose['roll_deg_per_sec'], navpose['pitch_deg_per_sec'], navpose['yaw_deg_per_sec']),
                         (-3.0, 2.0, 10.0))


class TestImuMount(unittest.TestCase):

    def test_24_right_handed_mounts_flat_first(self):
        options = rbx_mate_rov_link_node.IMU_MOUNT_OPTIONS
        self.assertEqual(len(options), 24)
        self.assertEqual(len(set(options)), 24)
        self.assertEqual(options[0], rbx_mate_rov_link_node.IMU_MOUNT_FLAT)
        for name in options:
            m = ImuMount(name).matrix
            det = (m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
                   + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]))
            self.assertEqual(det, 1, name)

    def test_unknown_mount_is_refused(self):
        for name in ['', 'X_UP_Y_UP', 'X_UP_Y_DOWN', 'x_forward_y_left', None]:
            with self.assertRaises(ValueError):
                ImuMount(name)

    def test_flat_passes_through_unchanged(self):
        mount = ImuMount()
        self.assertEqual(mount.orientation_to_rov(1.5, -2.25, 170.0), [1.5, -2.25, 170.0])
        self.assertEqual(mount.rates_to_rov(1.0, 2.0, 3.0), [1.0, 2.0, 3.0])

    def test_every_mount_recovers_the_rov_from_the_printed_angles(self):
        # Any heading, tilts within 5 deg, through the sketch's own conversion
        # and its 2-decimal print
        rng = random.Random(1)
        for name in rbx_mate_rov_link_node.IMU_MOUNT_OPTIONS:
            mount = ImuMount(name)
            for _ in range(200):
                pose = [rng.uniform(-5, 5), rng.uniform(-5, 5), rng.uniform(-180, 180)]
                result = mount.orientation_to_rov(*teensyAngles(mount, *pose))
                for (got, want) in zip(result, pose):
                    self.assertLess(angleError(got, want), 0.02, (name, pose, result))

    def test_side_mount_yaw_is_unusable_unconverted(self):
        # Why the mount exists: on its side, the sketch's yaw follows a half
        # degree of tilt, not the heading
        mount = ImuMount('X_UP_Y_LEFT')
        [roll, pitch, yaw] = teensyAngles(mount, 0.5, 0.0, 30.0)
        self.assertAlmostEqual(abs(pitch), 89.5, places = 1)
        self.assertGreater(angleError(yaw, 30.0), 45.0)
        self.assertLess(angleError(mount.orientation_to_rov(roll, pitch, yaw)[2], 30.0), 0.02)


class TestStatusValidation(unittest.TestCase):

    def setUp(self):
        self.clock = FakeClock()
        self.link = MateRovLink(ROV, clock = self.clock, wall_clock = lambda: WALL_EPOCH, log = lambda msg: None)

    def assertDropped(self, datagram, source = ROV):
        bad = self.link.status_bad
        [accepted, msg] = self.link.handle_status(datagram, source)
        self.assertFalse(accepted, msg)
        self.assertEqual(self.link.status_bad, bad + 1)
        self.assertIsNone(self.link.status)

    def test_nepi_link_status_is_accepted(self):
        pi = nepi_link.NepiLink(clock = self.clock, wall_clock = lambda: WALL_EPOCH, log = lambda msg: None)
        pi.configure(dict(NEPI_LAN_IP = ROV))
        pi.on_teensy_line('$TNCTL,0,0,0,0,100,100,100,100,1.25,0.00,-12.50,3.00,-4.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,*FF')
        [accepted, msg] = self.link.handle_status(pi.nextStatusDatagram(), ROV)
        self.assertTrue(accepted, msg)
        [tn, tn_age_ms] = self.link.get_tn()
        self.assertEqual((tn['depth_m'], tn['yaw_deg'], tn['pitch_deg'], tn['roll_deg']), (1.25, -12.5, 3.0, -4.0))
        self.assertEqual(tn_age_ms, 0)
        self.clock.advance(0.3)
        self.assertEqual(self.link.get_tn()[1], 300)

    def test_malformed_statuses_are_dropped(self):
        self.assertDropped(b'not json')
        self.assertDropped(b'\xff\xfe')
        self.assertDropped(b'[1, 2]')
        self.assertDropped(statusBytes(v = 2))
        self.assertDropped(statusBytes(v = True))
        self.assertDropped(statusBytes(type = 'demand'))
        self.assertDropped(statusBytes(seq = -1))
        self.assertDropped(statusBytes(seq = 1.5))
        self.assertDropped(statusBytes(assist = 1))
        self.assertDropped(statusBytes(slow = None))
        self.assertDropped(statusBytes(assist_reason = 3))
        self.assertDropped(statusBytes(tn_age_ms = -2))
        self.assertDropped(statusBytes(tn = [1]))
        self.assertDropped(statusBytes(tn = dict(depth_m = '1', yaw_deg = 0, pitch_deg = 0, roll_deg = 0)))
        self.assertDropped(statusBytes(tn = dict(depth_m = 0, yaw_deg = 0, pitch_deg = 0)))
        self.assertDropped(statusBytes(tn = dict(depth_m = 0, depth_m_s = 0, yaw_deg = 0, pitch_deg = 0, roll_deg = 0,
                                                 yaw_deg_s = 0, pitch_deg_s = 0)))
        message = json.loads(statusBytes().decode('utf-8'))
        del message['tn']
        self.assertDropped(json.dumps(message).encode('utf-8'))
        self.assertDropped(statusBytes(pad = 'x' * 1200))
        self.assertDropped(statusBytes(), OTHER)
        self.assertDropped(statusBytes(servos = [1]))
        self.assertDropped(statusBytes(servos = dict(claw = 1)))
        self.assertDropped(statusBytes(servos = dict(Claw = False)))
        self.assertDropped(statusBytes(servos = dict(a = False, b = False, c = False, d = False, e = False)))

    def test_a_status_without_servos_lists_none(self):
        [accepted, msg] = self.link.handle_status(statusBytes(), ROV)
        self.assertTrue(accepted, msg)
        self.assertEqual(self.link.get_health()['servos'], dict())
        self.clock.advance(0.1)
        self.assertTrue(self.link.handle_status(statusBytes(seq = 1, servos = dict(claw = True)), ROV)[0])
        self.assertEqual(self.link.get_health()['servos'], dict(claw = dict(value = 0.0, pilot = True)))

    def test_status_seq_order_and_restart(self):
        self.assertTrue(self.link.handle_status(statusBytes(seq = 5), ROV)[0])
        self.clock.advance(0.1)
        self.assertFalse(self.link.handle_status(statusBytes(seq = 5), ROV)[0])
        self.assertFalse(self.link.handle_status(statusBytes(seq = 0), ROV)[0])
        self.assertTrue(self.link.handle_status(statusBytes(seq = 6), ROV)[0])
        # More than 1 s after the last accepted one, a restarted Pi's seq 0 is accepted
        self.clock.advance(1.01)
        self.assertTrue(self.link.handle_status(statusBytes(seq = 0), ROV)[0])


class TestServos(unittest.TestCase):

    def setUp(self):
        self.bench = Bench(NEPI_SURGE_SIGN = 1, NEPI_SWAY_SIGN = 1, NEPI_YAW_SIGN = 1, **CLAW_SERVOS)

    def tearDown(self):
        self.bench.close()

    def servoInputs(self):
        return self.bench.merged[HAT0Y_SLOT], self.bench.merged[SOUTH_SLOT]

    def test_the_pi_lists_its_servos(self):
        b = self.bench
        self.assertEqual(b.link.get_health()['servos'], dict())
        b.steps(3)
        self.assertEqual(b.link.get_health()['servos'], dict(claw_pitch = dict(value = 0.0, pilot = False),
                                                             claw_grip = dict(value = 0.0, pilot = False)))

    def test_pilot_mode_refuses_servo_values(self):
        b = self.bench
        b.steps(3)
        [success, msg] = b.link.set_servo('claw_pitch', 1.0)
        self.assertFalse(success)
        self.assertIn('NEPI does not have the ROV', msg)
        demand = b.step(slots = {HAT0Y_SLOT: -1})
        self.assertEqual(demand['servos'], dict())
        # The pilot's own D-pad passes straight through
        self.assertEqual(self.servoInputs(), (-1, 0))

    def test_nepi_control_moves_and_holds_the_servos(self):
        b = self.bench
        b.steps(3)
        b.handOver()
        self.assertTrue(b.link.set_servo('claw_pitch', -1.0)[0])
        self.assertTrue(b.link.set_servo('claw_grip', 1.0)[0])
        demand = b.step()
        self.assertEqual(demand['servos'], dict(claw_pitch = -1.0, claw_grip = 1.0))
        self.assertEqual(self.servoInputs(), (-1, 1))
        # Unlike a teleop command, a servo value never goes stale
        demands = b.steps(10)
        self.assertEqual(demands[-1]['servos'], dict(claw_pitch = -1.0, claw_grip = 1.0))
        self.assertEqual(self.servoInputs(), (-1, 1))
        self.assertEqual(b.link.get_health()['servos']['claw_pitch'], dict(value = -1.0, pilot = False))
        self.assertEqual(b.pi.build_status()['demand_bad'], 0)

    def test_values_are_clamped_and_rounded(self):
        b = self.bench
        b.steps(3)
        b.handOver()
        self.assertTrue(b.link.set_servo('claw_pitch', 3.0)[0])
        self.assertTrue(b.link.set_servo('claw_grip', 0.123456789)[0])
        self.assertEqual(b.step()['servos'], dict(claw_pitch = 1.0, claw_grip = 0.1235))
        for bad in [float('nan'), float('inf'), True, '0.5', None]:
            self.assertFalse(b.link.set_servo('claw_pitch', bad)[0])
        [success, msg] = b.link.set_servo('wrist', 0.5)
        self.assertFalse(success)
        self.assertIn('no servo by that name', msg)
        self.assertEqual(b.link.servo_cmd, dict(claw_pitch = 1.0, claw_grip = 0.123456789))

    def test_pilot_wins_that_input_only(self):
        b = self.bench
        b.steps(3)
        b.handOver()
        b.link.set_servo('claw_pitch', 1.0)
        b.link.set_servo('claw_grip', 1.0)
        b.step((0.1, 0.0, 0.0, 0.0), slots = {HAT0Y_SLOT: -1})
        self.assertEqual(self.servoInputs(), (-1, 1))
        self.assertEqual(b.merged[nepi_link.LY_SLOT], SURGE_0_1)
        self.assertTrue(b.link.is_nepi_control())
        self.assertTrue(b.link.get_health()['servos']['claw_pitch']['pilot'])
        # Released, NEPI's value is back
        b.step()
        self.assertEqual(self.servoInputs(), (1, 1))
        self.assertFalse(b.link.get_health()['servos']['claw_pitch']['pilot'])

    def test_handing_back_drops_the_values(self):
        b = self.bench
        b.steps(3)
        b.handOver()
        b.link.set_servo('claw_pitch', 1.0)
        b.step()
        self.assertEqual(self.servoInputs(), (1, 0))
        b.step(rb = 1)
        self.assertFalse(b.link.is_nepi_control())
        self.assertEqual(b.step()['servos'], dict())
        self.assertEqual(self.servoInputs(), (0, 0))
        self.assertEqual(b.link.get_health()['servos']['claw_pitch']['value'], 0.0)
        # A new handover starts neutral
        b.handOver()
        self.assertEqual(b.step()['servos'], dict())
        self.assertEqual(self.servoInputs(), (0, 0))

    def test_release_to_pilot_drops_the_values_and_stop_keeps_them(self):
        b = self.bench
        b.steps(3)
        b.handOver()
        b.link.set_servo('claw_pitch', 1.0)
        b.link.go_stop()
        self.assertEqual(b.step()['servos'], dict(claw_pitch = 1.0))
        b.link.release_to_pilot()
        demand = b.step()
        self.assertEqual((demand['active'], demand['servos']), (False, dict()))
        self.assertEqual(b.link.servo_cmd, dict())


class TestServoDegrees(unittest.TestCase):

    def test_each_half_scales_on_its_own(self):
        to_value = rbx_mate_rov_link_servo_node.deg_to_value
        to_deg = rbx_mate_rov_link_servo_node.value_to_deg
        self.assertEqual([to_value(d, -90.0, 90.0) for d in [-90.0, -45.0, 0.0, 45.0, 90.0, 120.0]],
                         [-1.0, -0.5, 0.0, 0.5, 1.0, 1.0])
        # An Axon MAX MK2 in servo mode can reach about 175 deg each way
        self.assertEqual([to_value(d, -175.0, 50.0) for d in [-175.0, -87.5, 0.0, 25.0, 50.0]],
                         [-1.0, -0.5, 0.0, 0.5, 1.0])
        for value in [-1.0, -0.3, 0.0, 0.7, 1.0]:
            self.assertAlmostEqual(to_value(to_deg(value, -175.0, 50.0), -175.0, 50.0), value)
        self.assertEqual(to_deg(-2.0, -175.0, 50.0), -175.0)


class FakeMessage(object):

    def __init__(self, data = None):
        self.data = data


class TestServoNodeGate(unittest.TestCase):
    """The servo node's own NEPI control check, on a node built without ROS."""

    def setUp(self):
        self.now = 1000.0
        self.published = []
        self.warnings = []
        node_module = rbx_mate_rov_link_servo_node
        self.saved = (node_module.String, getattr(node_module.nepi_utils, 'get_time', None))
        node_module.String = FakeMessage
        node_module.nepi_utils.get_time = lambda: self.now
        node = node_module.MateRovLinkServoNode.__new__(node_module.MateRovLinkServoNode)
        node.servo_name = 'claw_pitch'
        node.link_lock = threading.Lock()
        node.link_status_time = None
        node.nepi_control = False
        node.servo_value = 0.0
        node.pilot_holds = False
        node.last_refused_warn = None
        node.limits_dict = dict(min_hardstop_deg = -175.0, max_hardstop_deg = 50.0)
        node.set_servo_pub = types.SimpleNamespace(publish = lambda msg: self.published.append(json.loads(msg.data)))
        node.msg_if = types.SimpleNamespace(pub_warn = lambda msg, **kwargs: self.warnings.append(msg))
        self.node = node

    def tearDown(self):
        rbx_mate_rov_link_servo_node.String = self.saved[0]
        rbx_mate_rov_link_servo_node.nepi_utils.get_time = self.saved[1]

    def linkStatus(self, nepi_control, servos = None):
        health = dict(link_up = True, nepi_control = nepi_control, servos = dict())
        if servos is not None:
            health['servos'] = servos
        self.node.linkStatusCb(FakeMessage(json.dumps(health)))

    def test_refused_until_nepi_has_the_rov(self):
        node = self.node
        node.gotoPosition(25.0) # No link_status yet
        self.linkStatus(False)
        node.gotoPosition(25.0)
        self.assertEqual(self.published, [])
        self.assertEqual(len(self.warnings), 1) # At most once per REFUSED_WARN_SEC
        self.assertIn('NEPI does not have the ROV', self.warnings[0])
        self.linkStatus(True)
        node.gotoPosition(25.0)
        node.gotoPosition(-87.5)
        self.assertEqual(self.published, [dict(name = 'claw_pitch', value = 0.5), dict(name = 'claw_pitch', value = -0.5)])

    def test_stale_link_status_counts_as_pilot_mode(self):
        self.linkStatus(True)
        self.now += self.node.LINK_STATUS_TIMEOUT_SEC + 0.1
        self.node.gotoPosition(25.0)
        self.assertEqual(self.published, [])

    def test_position_is_the_value_nepi_sends(self):
        node = self.node
        self.assertEqual(node.getPosition(), 0.0)
        self.linkStatus(True, dict(claw_pitch = dict(value = -0.5, pilot = True)))
        self.assertEqual((node.getPosition(), node.pilot_holds), (-87.5, True))
        # Dropped when NEPI control ends
        self.linkStatus(False, dict(claw_pitch = dict(value = 0.0, pilot = False)))
        self.assertEqual(node.getPosition(), 0.0)


class TestLoopback(unittest.TestCase):

    def setUp(self):
        [self.status_port, self.demand_port] = freeUdpPorts()
        self.logs = []
        self.pi = nepi_link.NepiLink(log = self.logs.append)
        self.link = MateRovLink(ROV, self.status_port, self.demand_port, bind_ip = ROV, log = self.logs.append)
        self.merged = None

    def tearDown(self):
        self.link.stop()
        self.pi.stop()

    def tick(self, seconds, command = None, until = None):
        # The node's 10 Hz timer and the team's main loop, in real time
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if command is not None:
                self.link.set_teleop_velocity(*command)
            self.link.send_demand()
            time.sleep(0.02)
            self.merged = self.pi.merge_inputs([0] * 18)
            if until is not None and until():
                return True
            time.sleep(0.08)
        return until is None or until()

    def test_threads_hand_over_release_and_link_down(self):
        [success, msg] = self.link.start()
        self.assertTrue(success, msg)
        config = dict(NEPI_LAN_IP = ROV, NEPI_STATUS_UDP_PORT = self.status_port,
                      NEPI_DEMAND_UDP_PORT = self.demand_port)
        [on, msg] = self.pi.start(config, 'test')
        self.assertTrue(on, msg)
        self.assertTrue(waitFor(self.link.is_link_up, 2.0))
        # Idle: NEPI is ready, and the pilot keeps the ROV
        self.assertTrue(self.tick(2.0, until = lambda: self.pi.build_status()['nepi_ready']))
        self.tick(0.3)
        self.assertEqual(self.pi.build_status()['assist'], False)
        # The pilot presses RB; the link sees NEPI control, and the command merges
        self.pi.merge_inputs(pilot(rb = 1))
        self.assertTrue(self.tick(1.0, until = self.link.is_nepi_control))
        self.assertTrue(self.tick(2.0, (0.1, 0.0, 0.0, 0.0), until = lambda: self.merged[nepi_link.LY_SLOT] == SURGE_0_1))
        self.assertEqual(self.link.get_health()['assist_reason'], 'pilot_button')
        # NEPI hands it back: one active false, and the Pi reports nepi_off
        self.assertTrue(self.link.release_to_pilot()[0])
        self.assertTrue(self.tick(1.0, until = lambda: self.link.get_health()['assist_reason'] == 'nepi_off'))
        self.assertFalse(self.link.is_nepi_control())
        self.assertEqual(self.pi.build_status()['demand_bad'], 0)
        # The Pi stops: the link goes down within 0.5 s and sends nothing
        self.pi.stop()
        self.assertTrue(waitFor(lambda: not self.link.is_link_up(), 1.5))
        self.assertFalse(self.link.send_demand()[0])
        # stop() closes the status port
        self.link.stop()
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.bind((ROV, self.status_port))
        finally:
            sock.close()


class FakeStatusSender(object):
    """Send a datagram every 100 ms from bind_ip to the ROV address and port.

    The default datagram is a real nepi_link status.
    """

    def __init__(self, bind_ip, port, datagram = None):
        self.pi = nepi_link.NepiLink(log = lambda msg: None)
        self.pi.configure(dict(NEPI_LAN_IP = ROV))
        self.datagram = datagram
        self.port = port
        self.sock = openSocket(bind_ip)
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target = self.run)
        self.thread.daemon = True
        self.thread.start()

    def run(self):
        while not self.stop_event.is_set():
            datagram = self.datagram
            if datagram is None:
                datagram = self.pi.nextStatusDatagram()
            self.sock.sendto(datagram, (ROV, self.port))
            self.stop_event.wait(0.1)

    def stop(self):
        self.stop_event.set()
        self.thread.join(1.0)
        self.sock.close()


class TestDiscoveryProbe(unittest.TestCase):

    def setUp(self):
        self.port = freeUdpPort()
        self.sender = None
        # Only as far as the probe needs: no logger, no drv_dict
        self.discovery = MateRovLinkDiscovery.__new__(MateRovLinkDiscovery)

    def tearDown(self):
        if self.sender is not None:
            self.sender.stop()

    def probe(self):
        start = time.monotonic()
        found = self.discovery.checkForMateRovLinkDevice(ROV, str(self.port))
        return found, time.monotonic() - start

    def assertPortFree(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.bind(('', self.port))
        finally:
            sock.close()

    def test_status_from_the_rov_is_found(self):
        self.sender = FakeStatusSender(ROV, self.port)
        [found, elapsed] = self.probe()
        self.assertTrue(found)
        self.assertLess(elapsed, 0.5)
        self.assertPortFree()

    def test_status_from_the_wrong_address_is_not_found(self):
        self.sender = FakeStatusSender(OTHER, self.port)
        [found, elapsed] = self.probe()
        self.assertFalse(found)
        self.assertGreater(elapsed, 1.4)
        self.assertLess(elapsed, 2.0)
        self.assertPortFree()

    def test_a_demand_is_not_a_status(self):
        demand = json.dumps(dict(v = 1, type = 'demand', seq = 0, t = WALL_EPOCH, active = False, surge = 0.0,
                                 sway = 0.0, yaw = 0.0, pitch = 0.0, roll = 0.0)).encode('utf-8')
        self.sender = FakeStatusSender(ROV, self.port, demand)
        self.assertFalse(self.probe()[0])

    def test_silence_is_not_found(self):
        self.assertFalse(self.probe()[0])
        self.assertPortFree()

    def test_the_probe_reads_the_servo_names(self):
        servos = dict(claw_pitch = False, claw_grip = True, Bad = False)
        self.sender = FakeStatusSender(ROV, self.port, statusBytes(servos = servos))
        self.assertEqual(self.discovery.probeStatus(ROV, str(self.port)), (True, ['claw_grip', 'claw_pitch']))
        self.sender.stop()
        self.sender = FakeStatusSender(ROV, self.port, statusBytes())
        self.assertEqual(self.discovery.probeStatus(ROV, str(self.port)), (True, []))

    def test_a_port_held_by_a_running_link_is_never_probed(self):
        link = MateRovLink(ROV, self.port, freeUdpPort(), log = lambda msg: None)
        [success, msg] = link.start()
        self.assertTrue(success, msg)
        try:
            self.sender = FakeStatusSender(ROV, self.port)
            [found, elapsed] = self.probe()
            self.assertFalse(found)
            self.assertLess(elapsed, 0.5)
            self.assertTrue(waitFor(link.is_link_up, 1.0)) # The running link still has the port
        finally:
            link.stop()


if __name__ == '__main__':
    unittest.main()
