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
import os
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
             'nepi_sdk.nepi_system', 'nepi_api', 'nepi_api.device_if_rbx', 'nepi_api.messages_if',
             'std_msgs', 'std_msgs.msg', 'nepi_interfaces', 'nepi_interfaces.msg']
    for name in names:
        sys.modules[name] = types.ModuleType(name)
    for name in names:
        if '.' in name:
            [parent, child] = name.rsplit('.', 1)
            setattr(sys.modules[parent], child, sys.modules[name])
    # The names the driver files import from them
    sys.modules['nepi_api.device_if_rbx'].RBXRobotIF = None
    sys.modules['nepi_api.messages_if'].MsgIF = None
    sys.modules['std_msgs.msg'].String = None
    sys.modules['nepi_interfaces.msg'].AxisControls = None

stubModules()

import nepi_link
import rbx_mate_rov_link_node
import rbx_mate_rov_link_discovery

MateRovLink = rbx_mate_rov_link_node.MateRovLink
MateRovLinkDiscovery = rbx_mate_rov_link_discovery.MateRovLinkDiscovery

ROV = '127.0.0.1' # Both the ROV Pi's address and NEPI_LAN_IP here
OTHER = '127.0.0.2' # A sender that is not the ROV
WALL_EPOCH = 1792200000.0
SURGE_0_1 = 5706 # stick_value(0.1)
BENCH_LINE = '$TNCTL,0,0,0,0,100,100,100,100,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,*FF'


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

def pilot(rb = 0):
    inputs = [0] * 18
    inputs[nepi_link.RB_SLOT] = rb
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

    def step(self, command = None, rb = 0, status = True):
        if command is not None:
            self.link.set_teleop_velocity(*command)
        demand = None
        [sent, msg] = self.link.send_demand()
        if sent:
            [datagram, sender] = self.demand_rx.recvfrom(65536)
            demand = json.loads(datagram.decode('utf-8'))
            self.pi.handle_demand(datagram, sender[0])
        self.merged = self.pi.merge_inputs(pilot(rb = rb))
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

    def engage(self, command):
        # Link up, idle active false, then the command's first active true
        self.steps(3)
        demand = self.step(command)
        if not demand['active'] or self.piState() != (True, 'nepi_on'):
            raise AssertionError('did not engage: ' + str(self.piState()))
        return demand


class BenchCase(unittest.TestCase):

    def setUp(self):
        self.bench = Bench(NEPI_SURGE_SIGN = 1, NEPI_SWAY_SIGN = 1, NEPI_YAW_SIGN = 1)

    def tearDown(self):
        self.bench.close()


class TestIdle(BenchCase):

    def test_idle_sends_active_false_every_tick_and_assist_stays_off(self):
        self.assertEqual(MateRovLink.DEMAND_PERIOD_SEC, 0.1)
        b = self.bench
        self.assertIsNone(b.step()) # No status yet: the link is down
        demands = b.steps(10)
        for demand in demands:
            self.assertIsNotNone(demand)
            self.assertFalse(demand['active'])
            self.assertEqual([demand[name] for name in ['surge', 'sway', 'yaw', 'pitch', 'roll']], [0.0] * 5)
        # One demand per 0.1 s tick, each accepted in order
        self.assertEqual([d['seq'] for d in demands], list(range(demands[0]['seq'], demands[0]['seq'] + 10)))
        status = b.pi.build_status()
        self.assertEqual((status['demand_seq'], status['demand_bad']), (demands[-1]['seq'], 0))
        self.assertEqual(b.piState(), (False, 'start'))
        health = b.link.get_health()
        self.assertEqual((health['link_up'], health['latched'], health['assist']), (True, False, False))
        self.assertTrue(b.link.is_link_up())
        # Logged once, not per message
        self.assertEqual(len(b.link_logs), 1, b.link_logs)


class TestEngage(BenchCase):

    def test_fresh_command_turns_assist_on_and_merges_surge(self):
        b = self.bench
        demand = b.engage((0.1, 0.0, 0.0, 0.0))
        self.assertEqual((demand['surge'], demand['sway'], demand['yaw']), (0.1, 0.0, 0.0))
        self.assertEqual(b.pi.settings['NEPI_SURGE_SIGN'], 1)
        self.assertEqual(b.merged[nepi_link.LY_SLOT], SURGE_0_1)
        b.step((0.1, 0.0, 0.0, 0.0))
        health = b.link.get_health()
        self.assertEqual((health['assist'], health['assist_reason'], health['latched']), (True, 'nepi_on', False))
        self.assertGreaterEqual(health['demand_age_ms'], 0)

    def test_ros_axes_map_to_protocol_axes(self):
        b = self.bench
        demand = b.engage((0.0, 0.1, 0.0, 0.05))
        self.assertEqual((demand['surge'], demand['sway'], demand['yaw']), (0.0, -0.1, -0.05))
        # Negative yaw goes on RT, not LT; negative sway is left stick left
        self.assertEqual(b.merged[nepi_link.LT_SLOT], 0)
        self.assertEqual(b.merged[nepi_link.RT_SLOT], nepi_link.trigger_value(0.05, 10, 255))
        self.assertGreater(b.merged[nepi_link.RT_SLOT], 0)
        self.assertEqual(b.merged[nepi_link.LX_SLOT], nepi_link.stick_value(-0.1))
        # linear.z is ignored: the Teensy holds depth
        demand = b.step((0.0, 0.0, 0.7, 0.0))
        self.assertEqual((demand['active'], demand['surge'], demand['sway'], demand['yaw']), (True, 0.0, 0.0, 0.0))
        # Each axis is clamped to -1 to 1; the Pi applies its own caps
        demand = b.step((2.0, -3.0, 0.0, 5.0))
        self.assertEqual((demand['surge'], demand['sway'], demand['yaw']), (1.0, 1.0, -1.0))
        self.assertEqual(b.pi.build_status()['demand_bad'], 0)

    def test_bad_command_values_are_not_stored(self):
        b = self.bench
        b.engage((0.1, 0.0, 0.0, 0.0))
        for bad in [float('nan'), float('inf'), True, '0.1', None]:
            [success, msg] = b.link.set_teleop_velocity(bad, 0.0, 0.0, 0.0)
            self.assertFalse(success)
        self.assertEqual(b.link.cmd, [0.1, 0.0, 0.0])


class TestLatch(BenchCase):

    def test_pilot_button_latches_until_commands_pause_for_one_second(self):
        b = self.bench
        command = (0.1, 0.0, 0.0, 0.0)
        b.engage(command)
        b.step(command)
        b.step(command, rb = 1)
        self.assertEqual(b.piState(), (False, 'pilot_button'))
        # Commands keep coming, but the link holds active false
        demands = b.steps(5, command)
        self.assertEqual([d['active'] for d in demands], [False] * 5)
        self.assertTrue(b.link.get_health()['latched'])
        self.assertEqual(b.piState(), (False, 'pilot_button'))
        self.assertEqual(b.merged[nepi_link.LY_SLOT], 0)
        # A 0.5 s pause is not enough
        b.steps(5)
        self.assertFalse(b.step(command)['active'])
        self.assertTrue(b.link.get_health()['latched'])
        # A 1 s pause clears the latch, and the next command engages again
        b.steps(9)
        self.assertTrue(b.link.get_health()['latched'])
        b.steps(2)
        self.assertFalse(b.link.get_health()['latched'])
        self.assertTrue(b.step(command)['active'])
        self.assertEqual(b.piState(), (True, 'nepi_on'))
        self.assertEqual(b.merged[nepi_link.LY_SLOT], SURGE_0_1)
        latch_logs = [line for line in b.link_logs if line.startswith('Latch')]
        self.assertEqual(len(latch_logs), 2, b.link_logs)
        self.assertIn('pilot_button', latch_logs[0])

    def test_stale_command_sends_active_false_and_does_not_latch(self):
        b = self.bench
        command = (0.1, 0.0, 0.0, 0.0)
        b.engage(command)
        # Still fresh at 0.1 s and 0.2 s, stale after 0.25 s
        self.assertEqual([d['active'] for d in b.steps(3)], [True, True, False])
        self.assertEqual(b.piState(), (False, 'nepi_off'))
        self.assertFalse(b.link.get_health()['latched'])
        # Within 1 s, a fresh command engages again at once
        self.assertTrue(b.step(command)['active'])
        self.assertEqual(b.piState(), (True, 'nepi_on'))

    def test_restarted_pi_script_latches_after_half_a_second(self):
        b = self.bench
        command = (0.1, 0.0, 0.0, 0.0)
        b.engage(command)
        # The Pi script restarts: over 1 s of silence while commands keep coming
        b.steps(12, command, status = False)
        self.assertFalse(b.link.is_link_up())
        b.restartPi()
        self.assertIsNone(b.step(command)) # Its first status brings the link up
        # Its first-ever demand is active true, which it ignores
        actives = [d['active'] for d in b.steps(8, command)]
        self.assertEqual(actives[:5], [True] * 5)
        self.assertEqual(actives[6:], [False] * 2)
        self.assertTrue(b.link.get_health()['latched'])
        self.assertEqual(b.piState(), (False, 'start'))
        # Pause 1 s, then the Pi sees a false-to-true edge
        b.steps(11)
        self.assertTrue(b.step(command)['active'])
        self.assertEqual(b.piState(), (True, 'nepi_on'))

    def test_stop_clears_the_command_and_latches(self):
        b = self.bench
        command = (0.1, 0.0, 0.0, 0.0)
        b.engage(command)
        [success, msg] = b.link.go_stop()
        self.assertTrue(success, msg)
        self.assertIsNone(b.link.cmd)
        self.assertTrue(b.link.get_health()['latched'])
        demands = [b.step()] + b.steps(5, command)
        self.assertEqual([d['active'] for d in demands], [False] * 6)
        self.assertEqual(b.piState(), (False, 'nepi_off'))
        self.assertTrue(b.link.get_health()['latched'])


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
        message = json.loads(statusBytes().decode('utf-8'))
        del message['tn']
        self.assertDropped(json.dumps(message).encode('utf-8'))
        self.assertDropped(statusBytes(pad = 'x' * 1200))
        self.assertDropped(statusBytes(), OTHER)

    def test_status_seq_order_and_restart(self):
        self.assertTrue(self.link.handle_status(statusBytes(seq = 5), ROV)[0])
        self.clock.advance(0.1)
        self.assertFalse(self.link.handle_status(statusBytes(seq = 5), ROV)[0])
        self.assertFalse(self.link.handle_status(statusBytes(seq = 0), ROV)[0])
        self.assertTrue(self.link.handle_status(statusBytes(seq = 6), ROV)[0])
        # More than 1 s after the last accepted one, a restarted Pi's seq 0 is accepted
        self.clock.advance(1.01)
        self.assertTrue(self.link.handle_status(statusBytes(seq = 0), ROV)[0])


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

    def test_threads_engage_release_and_link_down(self):
        [success, msg] = self.link.start()
        self.assertTrue(success, msg)
        config = dict(NEPI_LAN_IP = ROV, NEPI_STATUS_UDP_PORT = self.status_port,
                      NEPI_DEMAND_UDP_PORT = self.demand_port)
        [on, msg] = self.pi.start(config, 'test')
        self.assertTrue(on, msg)
        self.assertTrue(waitFor(self.link.is_link_up, 2.0))
        # Idle: active false reaches the Pi, assist stays off
        self.assertTrue(self.tick(2.0, until = lambda: self.pi.build_status()['demand_seq'] >= 0))
        self.tick(0.3)
        self.assertEqual(self.pi.build_status()['assist'], False)
        # A command engages, and the link sees assist come on
        self.assertTrue(self.tick(2.0, (0.1, 0.0, 0.0, 0.0), until = lambda: self.merged[nepi_link.LY_SLOT] == SURGE_0_1))
        self.assertTrue(self.tick(1.0, (0.1, 0.0, 0.0, 0.0), until = lambda: self.link.get_health()['assist']))
        self.assertEqual(self.link.get_health()['assist_reason'], 'nepi_on')
        # Commands stop: active false, and the Pi reports nepi_off
        self.assertTrue(self.tick(1.0, until = lambda: self.link.get_health()['assist_reason'] == 'nepi_off'))
        self.assertFalse(self.link.get_health()['latched'])
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
