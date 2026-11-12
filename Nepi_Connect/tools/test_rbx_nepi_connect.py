#!/usr/bin/env python3
##
## Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
##
## This file is part of nepi-engine
## (see https://github.com/nepi-engine).
##
## License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
##

# Tests for the NepiConnectLink class in Drivers/rbx_drivers/rbx_nepi_connect_node.py
# and the probe and announcement parser in rbx_nepi_connect_discovery.py,
# against the real robot side: the nepi_connect package in Nepi_Connect/pi,
# driven through nepi_link.py's four hooks and the hooks adapter, with the
# real vehicles/mate_coral_crusaders.yaml. Standard library plus PyYAML. Run
# from this folder:
#   python3 -B -m unittest -v test_rbx_nepi_connect
#
# There is no ROS here. Empty stand-ins for nepi_sdk, nepi_api and the two ROS
# message packages go into sys.modules only so the driver files import; the
# node class is never built, and the discovery class only as far as its probe.
#
# Bench tests join one NepiConnectLink and one robot over real UDP on
# 127.0.0.1, with free ports and network.nepi_ip 127.0.0.1, under one injected
# clock; the robot's threads are not started, so each step runs in a fixed
# order. The link is built, as on NEPI, from the robot's own vehicle message
# through discovery's parse_vehicle_message. TestLoopback starts both sides'
# threads in real time, with discovery's probe in between.
#
# The tests carried over from version 1 were written against version 1's
# defaults (every sign +1, triggers 10 to 255), so the robot gets those two
# values on top of the team file, as in Nepi_Connect/pi/test_nepi_link.py.

import copy
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

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..', 'pi'))
sys.path.insert(0, os.path.join(HERE, '..', '..', 'Drivers', 'rbx_drivers'))


def stubModules():
    names = ['nepi_sdk', 'nepi_sdk.nepi_sdk', 'nepi_sdk.nepi_nav', 'nepi_sdk.nepi_utils', 'nepi_sdk.nepi_drvs',
             'nepi_sdk.nepi_system', 'nepi_sdk.nepi_controls', 'nepi_api', 'nepi_api.device_if_rbx',
             'nepi_api.device_if_npx', 'nepi_api.device_if_svx', 'nepi_api.messages_if',
             'std_msgs', 'std_msgs.msg', 'nepi_interfaces', 'nepi_interfaces.msg']
    for name in names:
        sys.modules[name] = types.ModuleType(name)
    for name in names:
        if '.' in name:
            [parent, child] = name.rsplit('.', 1)
            setattr(sys.modules[parent], child, sys.modules[name])
    # The names the driver files import from them
    sys.modules['nepi_api.device_if_rbx'].RBXRobotIF = None
    sys.modules['nepi_api.device_if_npx'].NPXDeviceIF = None
    sys.modules['nepi_api.device_if_svx'].DeviceSVXIF = None
    sys.modules['nepi_api.messages_if'].MsgIF = None
    sys.modules['std_msgs.msg'].String = None
    sys.modules['std_msgs.msg'].Empty = None

stubModules()

import nepi_link
from nepi_connect import control
from nepi_connect import link as nc_link
from nepi_connect import mapping
from nepi_connect import vehicle as vehicle_mod
from nepi_connect.adapters import hooks
import rbx_nepi_connect_node
import rbx_nepi_connect_discovery
import rbx_nepi_connect_servo_node

NepiConnectLink = rbx_nepi_connect_node.NepiConnectLink
NepiConnectNode = rbx_nepi_connect_node.NepiConnectNode
NepiConnectDiscovery = rbx_nepi_connect_discovery.NepiConnectDiscovery
parse_vehicle_message = rbx_nepi_connect_discovery.parse_vehicle_message
ImuMount = rbx_nepi_connect_node.ImuMount

ROV = '127.0.0.1' # Both the robot's address and its network.nepi_ip here
OTHER = '127.0.0.2' # A sender that is not the robot
WALL_EPOCH = 1792200000.0
SURGE_0_1 = 5706 # stick(0.1)
BENCH_LINE = '$TNCTL,0,0,0,0,100,100,100,100,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,*FF'
TEAM_FILE = os.path.join(HERE, '..', 'vehicles', 'mate_coral_crusaders.yaml')
with open(TEAM_FILE) as team_file:
    TEAM_RAW = yaml.safe_load(team_file)
TEAM = vehicle_mod.validate_vehicle(TEAM_RAW)[0]
SLOTS = dict((name, entry['slot']) for (name, entry) in TEAM['control']['inputs'].items())
LX_SLOT = SLOTS['ABS_X']
LY_SLOT = SLOTS['ABS_Y']
RY_SLOT = SLOTS['ABS_RY']
RB_SLOT = SLOTS['BTN_TR']
LT_SLOT = SLOTS['ABS_Z']
RT_SLOT = SLOTS['ABS_RZ']
HAT0Y_SLOT = SLOTS['ABS_HAT0Y']
SOUTH_SLOT = SLOTS['BTN_SOUTH']
# Two claw servos on the robot: D-pad up/down and the A button
CLAW_SERVOS = dict(claw_pitch = dict(input = 'ABS_HAT0Y'), claw_grip = dict(input = 'BTN_SOUTH'))
ALL_AXES_ZERO = dict(surge = 0.0, sway = 0.0, heave = 0.0, yaw = 0.0)

# A made-up rover: surge and yaw on RC sticks around 1500, depth only (a
# pressure sensor, no IMU), one servo and two cameras
ROVER_YAML = '''
vehicle: {name: test_rover, kind: rover}
network: {nepi_ip: 127.0.0.1, status_port: 6700, demand_port: 6701}
control:
  toggle_input: SWITCH
  inputs:
    THROTTLE: {slot: 0, min: 1000, neutral: 1500, max: 2000, deadzone: 20}
    STEER:    {slot: 1, min: 1000, neutral: 1500, max: 2000, deadzone: 20}
    SWITCH:   {slot: 2, min: 1000, neutral: 1000, max: 2000, deadzone: 500}
    GRIP:     {slot: 3, min: 1000, neutral: 1000, max: 2000, deadzone: 500}
  axes:
    surge: {kind: stick, input: THROTTLE, sign: 1, cap: 0.4}
    yaw:   {kind: stick, input: STEER, sign: 1, cap: 0.4}
  servos:
    grip: {input: GRIP}
telemetry:
  format: csv_line
  prefix: 'TLM'
  fields:
    - {name: depth, type: float}
    - {name: depth_rate, type: float}
    - {name: battery_mv, type: int}
  nav:
    depth: {depth_m: depth, depth_m_s: depth_rate}
  raw: [battery_mv]
cameras:
  - {name: front, transport: http, port: 8080}
  - {name: rear, transport: fastmjpg, port: 5610}
'''


def robotAngles(mount, roll_deg, pitch_deg, yaw_deg):
    # What the MATE team's sketch prints for a robot at this orientation with
    # its board mounted this way: the board's rotation as a quaternion, through
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

def stick(demand):
    entry = TEAM['control']['inputs']['ABS_Y']
    return mapping.stick_value(demand, entry['min'], entry['neutral'], entry['max'], entry['deadzone'])

def robotRaw(status_port = 5700, demand_port = 5701, servos = None, team = False):
    # The real team file, sending to this computer, with version 1's defaults unless team
    raw = copy.deepcopy(TEAM_RAW)
    raw['network'].update(nepi_ip = ROV, status_port = status_port, demand_port = demand_port)
    if not team:
        raw['control']['axes']['surge']['sign'] = 1
        for name in ['ABS_Z', 'ABS_RZ']:
            raw['control']['inputs'][name].update(deadzone = 10, max = 255)
    if servos is not None:
        raw['control']['servos'] = copy.deepcopy(servos)
    return raw

def makeRobot(raw, clock = None, wall_clock = None, logs = None):
    kwargs = dict(name = 'nepi_link')
    if clock is not None:
        kwargs.update(clock = clock, wall_clock = wall_clock)
    kwargs['log'] = logs.append if logs is not None else (lambda msg: None)
    robot = control.NepiControl(**kwargs)
    [on, msg] = robot.configure(raw)
    if not on:
        raise AssertionError('robot did not configure: ' + msg)
    return robot

def announcement(robot, source = ROV):
    # The robot's vehicle message, as discovery parses it for the node
    datagram = nc_link.encode_message(robot.build_vehicle_message())
    [vehicle, msg] = parse_vehicle_message(datagram, source, ROV)
    if vehicle is None:
        raise AssertionError('vehicle message rejected: ' + msg)
    return vehicle

TEAM_ANNOUNCEMENT = announcement(makeRobot(robotRaw(team = True)))

def pilot(rb = 0, ly = 0, slots = None):
    inputs = [0] * 18
    inputs[RB_SLOT] = rb
    inputs[LY_SLOT] = ly
    if slots is not None:
        for [slot, value] in slots.items():
            inputs[slot] = value
    return inputs

def statusBytes(**changes):
    status = dict(v = 2, type = 'status', seq = 0, t = WALL_EPOCH, control = False, control_reason = 'start',
                  nepi_ready = False, pilot = dict(surge = False, sway = False, heave = False, yaw = False),
                  servos = dict(), out = dict(ABS_Y = 0, ABS_X = 0, ABS_RY = 0, ABS_Z = 0, ABS_RZ = 0),
                  demand_seq = -1, demand_age_ms = -1, demand_bad = 0, nav = None, nav_age_ms = -1, nav_count = 0,
                  nav_bad = 0, raw = None)
    status.update(changes)
    return json.dumps(status).encode('utf-8')

def vehicleBytes(**changes):
    message = makeRobot(robotRaw(team = True)).build_vehicle_message()
    message.update(changes)
    return json.dumps(message).encode('utf-8')

def openSocket(bind_ip):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((bind_ip, 0))
    return sock


class HooksInstalled(object):
    # Points nepi_link.py's hooks at one robot, and back on exit

    def __init__(self, robot):
        self.robot = robot
        self.saved = hooks._adapter

    def install(self, robot = None):
        if robot is not None:
            self.robot = robot
        hooks._adapter = hooks.HooksAdapter(self.robot)

    def restore(self):
        hooks._adapter = self.saved


class Bench(object):
    """One NepiConnectLink and one robot joined over UDP on 127.0.0.1, one clock.

    The robot's code calls nepi_link.py's hooks, which reach the robot's
    nepi_connect through the hooks adapter. step() is one 100 ms cycle in a
    fixed order: the link's send tick, the demand across UDP into the robot's
    handle_demand(), the pilot's inputs through nepi_link.merge_inputs(), then
    one robot status across UDP into the link's receive thread, and the clock
    advances 0.1 s.
    """

    def __init__(self, servos = None):
        self.clock = FakeClock()
        self.wall_clock = lambda: WALL_EPOCH + self.clock.now
        [self.status_port, self.demand_port] = freeUdpPorts()
        self.raw = robotRaw(self.status_port, self.demand_port, servos)
        self.pi_logs = []
        self.link_logs = []
        self.pi = None
        self.hooks = HooksInstalled(None)
        self.restartPi()
        # Stands in for the robot's demand port and status socket
        self.demand_rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.demand_rx.bind((ROV, self.demand_port))
        self.demand_rx.settimeout(1.0)
        self.status_tx = openSocket(ROV)
        self.link = None
        self.restartLink()
        self.merged = None

    def restartPi(self):
        self.pi = makeRobot(self.raw, self.clock, self.wall_clock, self.pi_logs)
        self.hooks.install(self.pi)

    def restartLink(self):
        if self.link is not None:
            self.link.stop()
        self.link = NepiConnectLink(ROV, announcement(self.pi), self.status_port, bind_ip = ROV, clock = self.clock,
                                    wall_clock = self.wall_clock, log = self.link_logs.append)
        [success, msg] = self.link.start()
        if not success:
            raise AssertionError('link did not start: ' + msg)

    def close(self):
        self.link.stop()
        self.demand_rx.close()
        self.status_tx.close()
        self.hooks.restore()

    def sendStatus(self, datagram, sock = None):
        if sock is None:
            sock = self.status_tx
        message = json.loads(datagram.decode('utf-8'))
        seq = message['seq']
        bad = self.link.status_bad
        sock.sendto(datagram, (ROV, self.status_port))
        if message['type'] == 'vehicle':
            reached = lambda: self.link.vehicle_seq == seq or self.link.status_bad > bad
        else:
            reached = lambda: self.link.status_seq == seq or self.link.status_bad > bad
        if not waitFor(reached):
            raise AssertionError(message['type'] + ' ' + str(seq) + ' never reached the link')

    def step(self, command = None, rb = 0, status = True, ly = 0, slots = None):
        if command is not None:
            self.link.set_teleop_velocity(*command)
        demand = None
        [sent, msg] = self.link.send_demand()
        if sent:
            [datagram, sender] = self.demand_rx.recvfrom(65536)
            demand = json.loads(datagram.decode('utf-8'))
            self.pi.handle_demand(datagram, sender[0])
        self.merged = nepi_link.merge_inputs(pilot(rb = rb, ly = ly, slots = slots))
        if status:
            self.sendStatus(self.pi.nextStatusDatagram())
        self.clock.advance(0.1)
        return demand

    def steps(self, count, command = None, status = True):
        return [self.step(command, status = status) for i in range(count)]

    def piState(self):
        status = self.pi.build_status()
        return status['control'], status['control_reason']

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
        self.bench = Bench()

    def tearDown(self):
        self.bench.close()


class TestIdle(BenchCase):

    def test_idle_sends_active_true_every_tick_and_the_pilot_keeps_the_rov(self):
        self.assertEqual(NepiConnectLink.DEMAND_PERIOD_SEC, 0.1)
        b = self.bench
        self.assertIsNone(b.step()) # No status yet: the link is down
        self.assertEqual(nepi_link.topside_tag(), ',NEPI:NOTREADY')
        demands = b.steps(10)
        for demand in demands:
            self.assertIsNotNone(demand)
            self.assertTrue(demand['active'])
            self.assertEqual(demand['axes'], ALL_AXES_ZERO)
            self.assertEqual(demand['v'], 2)
        # One demand per 0.1 s tick, each accepted in order
        self.assertEqual([d['seq'] for d in demands], list(range(demands[0]['seq'], demands[0]['seq'] + 10)))
        status = b.pi.build_status()
        self.assertEqual((status['demand_seq'], status['demand_bad']), (demands[-1]['seq'], 0))
        # Ready, but only RB hands the ROV over
        self.assertEqual(b.piState(), (False, 'start'))
        self.assertEqual(nepi_link.topside_tag(), ',NEPI:OFF')
        health = b.link.get_health()
        self.assertEqual((health['link_up'], health['nepi_control'], health['control']), (True, False, False))
        self.assertTrue(b.link.is_link_up())
        self.assertFalse(b.link.is_nepi_control())
        # Logged once, not per message
        self.assertEqual(len(b.link_logs), 1, b.link_logs)


class TestHandover(BenchCase):

    def test_rb_hands_over_and_the_command_merges(self):
        b = self.bench
        demand = b.engage((0.1, 0.0, 0.0, 0.0))
        self.assertEqual((demand['active'], demand['axes']['surge'], demand['axes']['sway'], demand['axes']['yaw']),
                         (True, 0.1, 0.0, 0.0))
        self.assertEqual(b.pi.vehicle['control']['axes'][0]['sign'], 1)
        self.assertEqual(b.merged[LY_SLOT], SURGE_0_1)
        self.assertEqual(nepi_link.topside_tag(), ',NEPI:ON')
        health = b.link.get_health()
        self.assertEqual((health['nepi_control'], health['control'], health['control_reason']),
                         (True, True, 'pilot_button'))
        self.assertTrue(b.link.is_nepi_control())
        self.assertGreaterEqual(health['demand_age_ms'], 0)
        self.assertTrue(any(line.startswith('NEPI control on') for line in b.link_logs), b.link_logs)

    def test_commands_before_the_handover_are_dropped(self):
        b = self.bench
        command = (0.1, 0.0, 0.0, 0.0)
        demands = b.steps(4, command)
        self.assertEqual([d['axes']['surge'] for d in demands[1:]], [0.0] * 3)
        self.assertIsNone(b.link.cmd)
        self.assertEqual(b.merged[LY_SLOT], 0)
        # The handover starts neutral; only a command after it moves the ROV
        b.handOver()
        self.assertEqual(b.merged[LY_SLOT], 0)
        self.assertEqual(b.step()['axes']['surge'], 0.0)
        self.assertEqual(b.step(command)['axes']['surge'], 0.1)
        self.assertEqual(b.merged[LY_SLOT], SURGE_0_1)

    def test_ros_axes_map_to_protocol_axes(self):
        b = self.bench
        demand = b.engage((0.0, 0.1, 0.0, 0.05))
        self.assertEqual((demand['axes']['surge'], demand['axes']['sway'], demand['axes']['yaw']), (0.0, -0.1, -0.05))
        # Negative yaw goes on RT, not LT; negative sway is left stick left
        self.assertEqual(b.merged[LT_SLOT], 0)
        self.assertEqual(b.merged[RT_SLOT], mapping.trigger_value(0.05, 0, 255, 10))
        self.assertGreater(b.merged[RT_SLOT], 0)
        self.assertEqual(b.merged[LX_SLOT], stick(-0.1))
        # Up (ROS linear.z positive) is negative heave, which drives RY
        demand = b.step((0.0, 0.0, 0.1, 0.0))
        self.assertEqual((demand['active'], demand['axes']), (True, dict(surge = 0.0, sway = 0.0, heave = -0.1,
                                                                         yaw = 0.0)))
        self.assertEqual(b.merged[RY_SLOT], stick(-0.1))
        # Heave back to 0 leaves RY at 0, so the Teensy holds the new depth
        b.step((0.1, 0.0, 0.0, 0.0))
        self.assertEqual((b.merged[LY_SLOT], b.merged[RY_SLOT]), (SURGE_0_1, 0))
        # Each axis is clamped to -1 to 1; the robot applies its own caps
        demand = b.step((2.0, -3.0, 4.0, 5.0))
        self.assertEqual(demand['axes'], dict(surge = 1.0, sway = 1.0, heave = -1.0, yaw = -1.0))
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
        self.assertEqual([(d['active'], d['axes']['surge']) for d in demands], [(True, 0.0)] * 3)
        self.assertEqual(b.merged[LY_SLOT], 0)
        self.assertEqual(nepi_link.topside_tag(), ',NEPI:OFF')
        self.assertTrue(any(line.startswith('NEPI control off (pilot_button)') for line in b.link_logs), b.link_logs)
        # RB once more hands it over again
        b.handOver()
        self.assertEqual(b.step(command)['axes']['surge'], 0.1)
        self.assertEqual(b.merged[LY_SLOT], SURGE_0_1)

    def test_a_stick_past_the_takeover_point_hands_the_rov_back(self):
        b = self.bench
        command = (0.1, 0.0, 0.0, 0.0)
        b.engage(command)
        b.step(command, ly = 20000)
        self.assertEqual(b.piState(), (False, 'pilot_stick'))
        self.assertEqual(b.merged[LY_SLOT], 20000)
        self.assertEqual(b.link.get_health()['control_reason'], 'pilot_stick')
        self.assertFalse(b.link.is_nepi_control())
        self.assertEqual(b.step(command)['axes']['surge'], 0.0)

    def test_stale_command_goes_neutral_and_nepi_keeps_the_rov(self):
        self.assertEqual(NepiConnectLink.TELEOP_CMD_TIMEOUT_SEC, 0.5)
        b = self.bench
        b.engage((0.1, 0.0, 0.0, 0.0))
        # Fresh for 0.4 s after the command, neutral by 0.7 s
        self.assertEqual([d['axes']['surge'] for d in b.steps(4)], [0.1] * 4)
        b.steps(2)
        demand = b.step()
        self.assertEqual((demand['active'], demand['axes']['surge']), (True, 0.0))
        self.assertEqual(b.merged[LY_SLOT], 0)
        self.assertEqual(b.piState(), (True, 'pilot_button'))
        self.assertEqual(nepi_link.topside_tag(), ',NEPI:ON')

    def test_stop_goes_neutral_and_nepi_keeps_the_rov(self):
        b = self.bench
        b.engage((0.1, 0.0, 0.0, 0.0))
        [success, msg] = b.link.go_stop()
        self.assertTrue(success, msg)
        self.assertIsNone(b.link.cmd)
        demand = b.step()
        self.assertEqual((demand['active'], demand['axes']['surge']), (True, 0.0))
        self.assertEqual(b.merged[LY_SLOT], 0)
        self.assertEqual(b.piState(), (True, 'pilot_button'))

    def test_release_to_pilot_sends_one_active_false(self):
        b = self.bench
        command = (0.1, 0.0, 0.0, 0.0)
        b.engage(command)
        [success, msg] = b.link.release_to_pilot()
        self.assertTrue(success, msg)
        demands = b.steps(3, command)
        self.assertEqual([d['active'] for d in demands], [False, True, True])
        self.assertEqual([d['axes']['surge'] for d in demands], [0.0] * 3)
        self.assertEqual(b.piState(), (False, 'nepi_off'))
        self.assertFalse(b.link.is_nepi_control())
        self.assertEqual(nepi_link.topside_tag(), ',NEPI:OFF')
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
        # The robot's script restarts: over 1 s of silence while commands keep coming
        b.steps(12, command, status = False)
        self.assertFalse(b.link.is_link_up())
        self.assertFalse(b.link.is_nepi_control())
        b.restartPi()
        self.assertIsNone(b.step(command)) # Its first status brings the link up
        demands = b.steps(3, command)
        self.assertEqual([(d['active'], d['axes']['surge']) for d in demands], [(True, 0.0)] * 3)
        self.assertEqual(b.piState(), (False, 'start'))
        self.assertFalse(b.link.is_nepi_control())
        b.handOver()
        self.assertEqual(b.step(command)['axes']['surge'], 0.1)


class TestLinkDown(BenchCase):

    def test_status_silence_takes_the_link_down_and_stops_demands(self):
        b = self.bench
        command = (0.1, 0.0, 0.0, 0.0)
        nepi_link.on_teensy_line(BENCH_LINE)
        b.engage(command)
        self.assertEqual(NepiConnectLink.LINK_TIMEOUT_SEC, 0.5)
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
        # The telemetry ages with the status, so the navpose skips it
        [nav, nav_age_ms] = b.link.get_nav()
        self.assertIsNotNone(nav)
        self.assertGreater(nav_age_ms, NepiConnectNode.NAV_MAX_AGE_MS)
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
        # The same status from the robot address is accepted
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
        link = NepiConnectLink(ROV, TEAM_ANNOUNCEMENT, wall_clock = lambda: WALL_EPOCH)
        self.assertEqual(link.seq, int(WALL_EPOCH * NepiConnectLink.SEQ_PER_SEC) & NepiConnectLink.SEQ_MAX)
        link = NepiConnectLink(ROV, TEAM_ANNOUNCEMENT, wall_clock = lambda: 1.0e12)
        self.assertTrue(0 <= link.seq <= NepiConnectLink.SEQ_MAX)

    def test_vehicle_messages_keep_their_own_seq(self):
        b = self.bench
        b.steps(2)
        first = b.pi.nextVehicleDatagram()
        b.sendStatus(first)
        self.assertEqual((b.link.vehicle_seq, b.link.status_bad), (0, 0))
        # A status seq lower than the vehicle seq is fine, and vice versa
        b.sendStatus(b.pi.nextStatusDatagram())
        b.status_tx.sendto(first, (ROV, b.status_port))
        self.assertTrue(waitFor(lambda: b.link.status_bad == 1)) # The repeated vehicle message
        self.assertTrue(b.link.is_link_up())


class TestNavPose(unittest.TestCase):
    # getNavPoseCb on a stand-in self, since the node class is never built

    def setUp(self):
        rbx_nepi_connect_node.nepi_utils.get_time = lambda: 1000.0
        self.nav = None
        self.nav_age_ms = -1
        link = types.SimpleNamespace(get_nav = lambda: [self.nav, self.nav_age_ms])
        self.node = types.SimpleNamespace(link = link, NAV_MAX_AGE_MS = NepiConnectNode.NAV_MAX_AGE_MS,
                                          navpose_dict = dict(has_depth = False, has_orientation = False),
                                          imu_mount = ImuMount(), nav_seen = False, has_depth_nav = True,
                                          has_orientation_nav = True)

    def navpose(self):
        return NepiConnectNode.getNavPoseCb(self.node)

    def boardNav(self, depth_m, depth_m_s, yaw_deg, pitch_deg, roll_deg, yaw_deg_s, pitch_deg_s, roll_deg_s):
        # The MATE vehicle file's mapping: the sketch's rate columns are gyro X, Y and Z
        return dict(depth_m = depth_m, depth_m_s = depth_m_s, roll_deg = roll_deg, pitch_deg = pitch_deg,
                    yaw_deg = yaw_deg, gyro_x_deg_s = yaw_deg_s, gyro_y_deg_s = pitch_deg_s, gyro_z_deg_s = roll_deg_s)

    def test_none_until_the_first_fresh_telemetry(self):
        # None makes NPXDeviceIF wait instead of fixing has_depth False for good
        self.assertIsNone(self.navpose())
        self.nav = self.boardNav('0.03', '0.00', '0.00', '0.00', '0.00', '0.00', '0.00', '0.00')
        self.nav_age_ms = self.node.NAV_MAX_AGE_MS + 1
        self.assertIsNone(self.navpose())
        self.nav_age_ms = 100
        navpose = self.navpose()
        self.assertTrue(navpose['has_depth'])
        self.assertEqual(navpose['depth_m'], 0.03)
        self.assertAlmostEqual(navpose['time_depth'], 999.9)
        self.assertTrue(navpose['has_orientation'])
        # Stale after that: the last navpose, not None
        self.nav_age_ms = self.node.NAV_MAX_AGE_MS + 1
        self.assertIs(self.navpose(), navpose)

    def test_depth_rate_frame_and_the_flat_mount(self):
        # The flat mount passes the angles through and reads the rate columns
        # as gyro X, Y, Z, so the sketch's yaw_deg_s lands on roll
        self.nav = self.boardNav(1.25, -0.12, 40.0, -3.0, 2.0, 10.0, 2.0, 3.0)
        self.nav_age_ms = 100
        navpose = self.navpose()
        self.assertEqual(navpose['frame_depth'], 'DEPTH')
        self.assertEqual(navpose['depth_m_per_sec'], -0.12)
        self.assertEqual((navpose['roll_deg'], navpose['pitch_deg'], navpose['yaw_deg']), (2.0, -3.0, 40.0))
        self.assertEqual((navpose['roll_deg_per_sec'], navpose['pitch_deg_per_sec'], navpose['yaw_deg_per_sec']),
                         (10.0, 2.0, 3.0))

    def test_side_mount_turns_orientation_and_rates_into_the_rov_frame(self):
        # The team's mount (2026-10-26): X vertical. Y left is assumed here.
        self.node.imu_mount = ImuMount('X_UP_Y_LEFT')
        [roll, pitch, yaw] = robotAngles(self.node.imu_mount, 2.0, -3.0, 40.0)
        self.nav = self.boardNav(1.25, 0.0, yaw, pitch, roll, 10.0, 2.0, 3.0)
        self.nav_age_ms = 100
        navpose = self.navpose()
        self.assertLess(angleError(navpose['roll_deg'], 2.0), 0.02)
        self.assertLess(angleError(navpose['pitch_deg'], -3.0), 0.02)
        self.assertLess(angleError(navpose['yaw_deg'], 40.0), 0.02)
        # Gyro X is up (yaw), Y is left (pitch), and Z = X x Y points back, so roll is -Z
        self.assertEqual((navpose['roll_deg_per_sec'], navpose['pitch_deg_per_sec'], navpose['yaw_deg_per_sec']),
                         (-3.0, 2.0, 10.0))

    def test_depth_only_vehicle_leaves_orientation_unset(self):
        self.node.has_orientation_nav = False
        self.nav = dict(depth_m = 2.5, depth_m_s = 0.1)
        self.nav_age_ms = 50
        navpose = self.navpose()
        self.assertTrue(navpose['has_depth'])
        self.assertEqual(navpose['depth_m'], 2.5)
        self.assertFalse(navpose['has_orientation'])
        self.assertNotIn('roll_deg', navpose)

    def test_vehicle_with_no_navigation_never_sends_a_navpose(self):
        self.node.has_depth_nav = False
        self.node.has_orientation_nav = False
        self.nav = dict()
        self.nav_age_ms = 50
        self.assertIsNone(self.navpose())
        self.assertIsNone(self.navpose())


class TestImuMount(unittest.TestCase):

    def test_24_right_handed_mounts_flat_first(self):
        options = rbx_nepi_connect_node.IMU_MOUNT_OPTIONS
        self.assertEqual(len(options), 24)
        self.assertEqual(len(set(options)), 24)
        self.assertEqual(options[0], rbx_nepi_connect_node.IMU_MOUNT_FLAT)
        for name in options:
            m = ImuMount(name).matrix
            det = (m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
                   + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]))
            self.assertEqual(det, 1, name)

    def test_robot_side_offers_the_same_mounts(self):
        self.assertEqual(vehicle_mod.IMU_MOUNT_OPTIONS, rbx_nepi_connect_node.IMU_MOUNT_OPTIONS)
        self.assertEqual(vehicle_mod.IMU_MOUNT_FLAT, rbx_nepi_connect_node.IMU_MOUNT_FLAT)

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
        for name in rbx_nepi_connect_node.IMU_MOUNT_OPTIONS:
            mount = ImuMount(name)
            for _ in range(200):
                pose = [rng.uniform(-5, 5), rng.uniform(-5, 5), rng.uniform(-180, 180)]
                result = mount.orientation_to_rov(*robotAngles(mount, *pose))
                for (got, want) in zip(result, pose):
                    self.assertLess(angleError(got, want), 0.02, (name, pose, result))

    def test_side_mount_yaw_is_unusable_unconverted(self):
        # Why the mount exists: on its side, the sketch's yaw follows a half
        # degree of tilt, not the heading
        mount = ImuMount('X_UP_Y_LEFT')
        [roll, pitch, yaw] = robotAngles(mount, 0.5, 0.0, 30.0)
        self.assertAlmostEqual(abs(pitch), 89.5, places = 1)
        self.assertGreater(angleError(yaw, 30.0), 45.0)
        self.assertLess(angleError(mount.orientation_to_rov(roll, pitch, yaw)[2], 30.0), 0.02)


class TestStatusValidation(unittest.TestCase):

    def setUp(self):
        self.clock = FakeClock()
        self.link = NepiConnectLink(ROV, TEAM_ANNOUNCEMENT, clock = self.clock, wall_clock = lambda: WALL_EPOCH,
                                    log = lambda msg: None)

    def assertDropped(self, datagram, source = ROV):
        bad = self.link.status_bad
        [accepted, msg] = self.link.handle_status(datagram, source)
        self.assertFalse(accepted, msg)
        self.assertEqual(self.link.status_bad, bad + 1)
        self.assertIsNone(self.link.status)

    def test_robot_status_is_accepted(self):
        robot = makeRobot(robotRaw(team = True), self.clock, lambda: WALL_EPOCH)
        robot.on_telemetry_line('$TNCTL,0,0,0,0,100,100,100,100,1.25,0.00,-12.50,3.00,-4.00,0.00,0.00,0.00,0.00,0.00,'
                                '0.00,0.00,*FF')
        [accepted, msg] = self.link.handle_status(robot.nextStatusDatagram(), ROV)
        self.assertTrue(accepted, msg)
        [nav, nav_age_ms] = self.link.get_nav()
        self.assertEqual((nav['depth_m'], nav['yaw_deg'], nav['pitch_deg'], nav['roll_deg']), (1.25, -12.5, 3.0, -4.0))
        self.assertEqual(nav_age_ms, 0)
        self.clock.advance(0.3)
        self.assertEqual(self.link.get_nav()[1], 300)
        self.assertEqual(self.link.get_health()['raw']['hfl_pwr'], 100)

    def test_malformed_statuses_are_dropped(self):
        full_nav = dict((name, 0) for name in TEAM_ANNOUNCEMENT['nav'])
        self.assertDropped(b'not json')
        self.assertDropped(b'\xff\xfe')
        self.assertDropped(b'[1, 2]')
        self.assertDropped(statusBytes(v = 1))
        self.assertDropped(statusBytes(v = True))
        self.assertDropped(statusBytes(type = 'demand'))
        self.assertDropped(statusBytes(seq = -1))
        self.assertDropped(statusBytes(seq = 1.5))
        self.assertDropped(statusBytes(control = 1))
        self.assertDropped(statusBytes(pilot = dict(surge = False, sway = False, heave = False)))
        self.assertDropped(statusBytes(pilot = dict(surge = None, sway = False, heave = False, yaw = False)))
        self.assertDropped(statusBytes(control_reason = 3))
        self.assertDropped(statusBytes(control_reason = 'because'))
        self.assertDropped(statusBytes(nav_age_ms = -2))
        self.assertDropped(statusBytes(nav = [1]))
        self.assertDropped(statusBytes(nav = dict(full_nav, depth_m = '1')))
        nav = dict(full_nav)
        del nav['gyro_z_deg_s']
        self.assertDropped(statusBytes(nav = nav))
        self.assertDropped(statusBytes(nav = dict(depth_m = 0, depth_m_s = 0)))
        self.assertDropped(statusBytes(raw = dict(a = 'x')))
        self.assertDropped(statusBytes(pad = 'x' * 1200))
        self.assertDropped(statusBytes(), OTHER)
        self.assertDropped(statusBytes(servos = [1]))
        self.assertDropped(statusBytes(servos = dict(claw = 1)))
        self.assertDropped(statusBytes(servos = dict(Claw = False)))
        self.assertDropped(statusBytes(servos = dict(a = False, b = False, c = False, d = False, e = False)))
        # A full nav is accepted
        self.assertTrue(self.link.handle_status(statusBytes(nav = full_nav), ROV)[0])

    def test_missing_optional_parts_mean_none(self):
        # A robot need not have every part: a status with no pilot, nav,
        # nav_age_ms, raw or servos field (or a null one) is accepted as having none
        for missing in [None, 'omit']:
            message = json.loads(statusBytes(seq = 0 if missing is None else 1).decode('utf-8'))
            for field in ['pilot', 'nav', 'nav_age_ms', 'raw', 'servos']:
                if missing is None:
                    message[field] = None
                else:
                    message.pop(field, None)
            [accepted, msg] = self.link.handle_status(json.dumps(message).encode('utf-8'), ROV)
            self.assertTrue(accepted, msg)
            status = self.link.status
            self.assertEqual(status['pilot'], dict(surge = False, sway = False, heave = False, yaw = False))
            self.assertEqual((status['nav'], status['nav_age_ms'], status['raw'], status['servos']),
                             (None, -1, None, dict()))
            self.clock.advance(0.1)
        # The rest of the status is still required
        message = json.loads(statusBytes(seq = 2).decode('utf-8'))
        del message['demand_age_ms']
        bad = self.link.status_bad
        [accepted, msg] = self.link.handle_status(json.dumps(message).encode('utf-8'), ROV)
        self.assertFalse(accepted, msg)
        self.assertIn('demand_age_ms', msg)
        self.assertEqual(self.link.status_bad, bad + 1)

    def test_a_status_with_no_servos_lists_none(self):
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
        # More than 1 s after the last accepted one, a restarted robot's seq 0 is accepted
        self.clock.advance(1.01)
        self.assertTrue(self.link.handle_status(statusBytes(seq = 0), ROV)[0])


class TestLinkStatus(unittest.TestCase):
    # What the rbx node publishes on link_status, from get_health()

    def setUp(self):
        self.clock = FakeClock()

    def test_health_fields_and_their_v1_names_are_gone(self):
        link = NepiConnectLink(ROV, TEAM_ANNOUNCEMENT, clock = self.clock, log = lambda msg: None)
        health = link.get_health()
        for name in ['running', 'link_up', 'nepi_control', 'control', 'control_reason', 'pilot', 'demand_age_ms',
                     'nav_age_ms', 'status_bad', 'servos', 'raw', 'vehicle', 'kind', 'cameras', 'vehicle_changed']:
            self.assertIn(name, health)
        for name in ['assist', 'assist_reason', 'pilot_surge', 'pilot_depth', 'slow', 'tn_age_ms']:
            self.assertNotIn(name, health)
        self.assertEqual((health['vehicle'], health['kind'], health['cameras']), ('mate_coral_crusaders', 'ROV', []))
        self.assertEqual(health['pilot'], dict(surge = False, sway = False, heave = False, yaw = False))
        json.dumps(health)

    def test_camera_urls(self):
        rover = makeRobot(yaml.safe_load(ROVER_YAML))
        link = NepiConnectLink('192.168.68.22', announcement(rover), log = lambda msg: None)
        self.assertEqual(link.get_health()['cameras'],
                         [dict(name = 'front', transport = 'http', url = 'http://192.168.68.22:8080/stream'),
                          dict(name = 'rear', transport = 'fastmjpg', url = 'fastmjpg://0.0.0.0:5610')])

    def test_changed_announcement_is_reported(self):
        link = NepiConnectLink(ROV, TEAM_ANNOUNCEMENT, clock = self.clock, log = lambda msg: None)
        # The same announcement again: nothing changed, and nothing counts as bad
        self.assertEqual(link.handle_status(vehicleBytes(seq = 0), ROV), (True, 'Vehicle unchanged'))
        self.assertFalse(link.get_health()['vehicle_changed'])
        self.assertIsNone(link.get_vehicle_change())
        # A new cap
        self.clock.advance(0.5)
        axes = dict(TEAM_ANNOUNCEMENT['axes'], yaw = 0.0)
        self.assertEqual(link.handle_status(vehicleBytes(seq = 1, axes = axes), ROV), (True, 'Vehicle changed'))
        self.assertTrue(link.get_health()['vehicle_changed'])
        self.assertEqual(link.get_vehicle_change()['axes']['yaw'], 0.0)
        # The node keeps the launch announcement
        self.assertEqual(link.axis_names, ['surge', 'sway', 'heave', 'yaw'])
        # Back to the launch one
        self.clock.advance(0.5)
        link.handle_status(vehicleBytes(seq = 2), ROV)
        self.assertFalse(link.get_health()['vehicle_changed'])
        self.assertEqual(link.status_bad, 0)

    def test_node_warns_once_per_change(self):
        warnings = []
        published = []
        link = NepiConnectLink(ROV, TEAM_ANNOUNCEMENT, clock = self.clock, log = lambda msg: None)
        node = types.SimpleNamespace(link = link, msg_if = types.SimpleNamespace(pub_warn = warnings.append),
                                     link_status_pub = types.SimpleNamespace(publish = published.append),
                                     getStateInd = lambda: 0, state_ind_last = 0,
                                     rbx_if = types.SimpleNamespace(publish_status = lambda: None),
                                     vehicle_change_seen = None)
        saved = rbx_nepi_connect_node.String
        rbx_nepi_connect_node.String = lambda data = None: types.SimpleNamespace(data = data)
        try:
            tick = lambda: NepiConnectNode.sendDemandCb(node, None)
            tick()
            self.assertEqual(warnings, [])
            link.handle_status(vehicleBytes(seq = 0, kind = 'AUV'), ROV)
            for i in range(3):
                tick()
            self.assertEqual(len(warnings), 1)
            self.assertIn('Disable and enable the driver', warnings[0])
            self.clock.advance(0.5)
            link.handle_status(vehicleBytes(seq = 1, kind = 'USV'), ROV)
            tick()
            tick()
            self.assertEqual(len(warnings), 2)
            self.assertIn('"USV"', warnings[1])
            self.assertEqual(json.loads(published[-1].data)['vehicle_changed'], True)
        finally:
            rbx_nepi_connect_node.String = saved


class TestAnnouncement(unittest.TestCase):

    def test_team_announcement(self):
        self.assertEqual(TEAM_ANNOUNCEMENT['name'], 'mate_coral_crusaders')
        self.assertEqual(TEAM_ANNOUNCEMENT['demand_port'], 5701)
        self.assertEqual(TEAM_ANNOUNCEMENT['axes'], dict(surge = 0.2, sway = 0.2, heave = 0.2, yaw = 0.1))
        self.assertEqual(rbx_nepi_connect_node.axis_capabilities(TEAM_ANNOUNCEMENT),
                         dict(has_x_control = True, has_y_control = True, has_z_control = True,
                              has_roll_control = False, has_pitch_control = False, has_yaw_control = True))
        self.assertEqual(rbx_nepi_connect_node.navpose_groups(TEAM_ANNOUNCEMENT), [True, True])

    def test_second_vehicle_capabilities_navpose_and_demands(self):
        rover = makeRobot(yaml.safe_load(ROVER_YAML))
        vehicle = announcement(rover)
        self.assertEqual(rbx_nepi_connect_node.axis_capabilities(vehicle),
                         dict(has_x_control = True, has_y_control = False, has_z_control = False,
                              has_roll_control = False, has_pitch_control = False, has_yaw_control = True))
        self.assertEqual(rbx_nepi_connect_node.navpose_groups(vehicle), [True, False])
        self.assertEqual((vehicle['servos'], vehicle['raw'], vehicle['demand_port']), (['grip'], ['battery_mv'], 6701))
        # Its link sends only surge and yaw, which its robot merges around 1500
        clock = FakeClock()
        link = NepiConnectLink(ROV, vehicle, clock = clock, wall_clock = lambda: WALL_EPOCH, log = lambda msg: None)
        rover.on_telemetry_line('TLM,1.50,0.02,11800')
        link.status_time = None
        sent = []
        link.demand_sock = types.SimpleNamespace(sendto = lambda datagram, address: sent.append((datagram, address)))
        self.assertTrue(link.handle_status(rover.nextStatusDatagram(), ROV)[0])
        self.assertEqual(link.get_nav()[0], dict(depth_m = 1.5, depth_m_s = 0.02))
        self.assertEqual(link.get_health()['raw'], dict(battery_mv = 11800))
        link.send_demand()
        demand = json.loads(sent[-1][0].decode('utf-8'))
        self.assertEqual((sent[-1][1], demand['axes']), ((ROV, 6701), dict(surge = 0.0, yaw = 0.0)))
        self.assertTrue(rover.handle_demand(sent[-1][0], ROV)[0])

    def test_vehicle_parts_are_optional(self):
        # A robot need not have every part. A vehicle message with no axes,
        # servos, nav, raw, cameras or imu_mount field announces none of them.
        message = json.loads(vehicleBytes().decode('utf-8'))
        for field in ['axes', 'servos', 'nav', 'raw', 'cameras', 'imu_mount']:
            del message[field]
        [vehicle, msg] = parse_vehicle_message(json.dumps(message).encode('utf-8'), ROV, ROV)
        self.assertIsNotNone(vehicle, msg)
        self.assertEqual((vehicle['axes'], vehicle['servos'], vehicle['nav'], vehicle['raw'], vehicle['cameras'],
                          vehicle['imu_mount']), (dict(), [], [], [], [], 'X_FORWARD_Y_LEFT'))
        self.assertEqual(rbx_nepi_connect_node.axis_capabilities(vehicle),
                         dict(has_x_control = False, has_y_control = False, has_z_control = False,
                              has_roll_control = False, has_pitch_control = False, has_yaw_control = False))
        self.assertEqual(rbx_nepi_connect_node.navpose_groups(vehicle), [False, False])
        # Null is the same as missing
        null_message = dict(message, axes = None, servos = None)
        self.assertEqual(parse_vehicle_message(json.dumps(null_message).encode('utf-8'), ROV, ROV)[0], vehicle)
        # The node built from it sees a later message that spells the parts out
        # empty, or leaves them out, as the same robot
        clock = FakeClock()
        link = NepiConnectLink(ROV, message, clock = clock, log = lambda msg: None)
        self.assertEqual(link.axis_names, [])
        empty = dict(message, seq = 1, axes = dict(), servos = [], nav = [], raw = [], cameras = [],
                     imu_mount = 'X_FORWARD_Y_LEFT')
        self.assertEqual(link.handle_status(json.dumps(empty).encode('utf-8'), ROV), (True, 'Vehicle unchanged'))
        # With no axes there is nothing to teleop, even while NEPI has the robot
        node = types.SimpleNamespace(getSettingSelection = lambda name: 'TRUE',
                                     link = types.SimpleNamespace(axis_names = [], is_nepi_control = lambda: True))
        self.assertFalse(NepiConnectNode.teleopControlsReady(node))
        node.link.axis_names = ['surge']
        self.assertTrue(NepiConnectNode.teleopControlsReady(node))

    def test_parse_vehicle_message_rejects(self):
        good = vehicleBytes()
        self.assertIsNotNone(parse_vehicle_message(good, ROV, ROV)[0])
        cases = [
            [vehicleBytes(), OTHER, 'expecting the robot'],
            [statusBytes(), ROV, 'not a vehicle message'],
            [b'not json', ROV, 'not UTF-8 JSON'],
            [vehicleBytes(v = 1), ROV, 'version 1'],
            [vehicleBytes(v = 3), ROV, 'v must be 2'],
            [vehicleBytes(seq = -1), ROV, 'seq'],
            [vehicleBytes(t = 'now'), ROV, 't must be'],
            [vehicleBytes(name = 'Mate ROV'), ROV, 'name'],
            [vehicleBytes(kind = 'k' * 33), ROV, 'kind'],
            [vehicleBytes(demand_port = 0), ROV, 'demand_port'],
            [vehicleBytes(axes = dict(pitch = 0.1)), ROV, 'axes'],
            [vehicleBytes(axes = dict(surge = 1.5)), ROV, 'axes'],
            [vehicleBytes(axes = dict(surge = True)), ROV, 'axes'],
            [vehicleBytes(servos = ['a', 'b', 'c', 'd', 'e']), ROV, 'servos'],
            [vehicleBytes(servos = ['Claw']), ROV, 'servos'],
            [vehicleBytes(servos = ['claw', 'claw']), ROV, 'servos'],
            [vehicleBytes(nav = ['depth_m']), ROV, 'nav'],
            [vehicleBytes(nav = ['altitude_m']), ROV, 'nav'],
            [vehicleBytes(raw = ['Bad Name']), ROV, 'raw'],
            [vehicleBytes(cameras = [dict(name = 'a', transport = 'rtsp', port = 1)]), ROV, 'camera'],
            [vehicleBytes(cameras = [dict(name = 'a', transport = 'http', port = 1),
                                     dict(name = 'a', transport = 'http', port = 2)]), ROV, 'camera'],
            [vehicleBytes(imu_mount = 'flat'), ROV, 'imu_mount'],
            [vehicleBytes(pad = 'x' * 1200), ROV, 'over 1200']]
        for [datagram, source, reason] in cases:
            [vehicle, msg] = parse_vehicle_message(datagram, source, ROV)
            self.assertIsNone(vehicle, reason)
            self.assertIn(reason, msg)
        # Unknown extra fields are ignored
        self.assertIsNotNone(parse_vehicle_message(vehicleBytes(battery = 'full'), ROV, ROV)[0])


class TestServos(unittest.TestCase):

    def setUp(self):
        self.bench = Bench(servos = CLAW_SERVOS)

    def tearDown(self):
        self.bench.close()

    def servoInputs(self):
        return self.bench.merged[HAT0Y_SLOT], self.bench.merged[SOUTH_SLOT]

    def test_the_pi_lists_its_servos(self):
        b = self.bench
        self.assertEqual(b.link.get_health()['servos'], dict())
        self.assertEqual(b.link.servo_names, ['claw_pitch', 'claw_grip'])
        b.steps(3)
        self.assertEqual(b.link.get_health()['servos'], dict(claw_pitch = dict(value = 0.0, pilot = False),
                                                             claw_grip = dict(value = 0.0, pilot = False)))

    def test_pilot_mode_refuses_servo_values(self):
        b = self.bench
        b.steps(3)
        [success, msg] = b.link.set_servo('claw_pitch', 1.0)
        self.assertFalse(success)
        self.assertIn('NEPI does not have the robot', msg)
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
        self.assertEqual(b.merged[LY_SLOT], SURGE_0_1)
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
        to_value = rbx_nepi_connect_servo_node.deg_to_value
        to_deg = rbx_nepi_connect_servo_node.value_to_deg
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
        node_module = rbx_nepi_connect_servo_node
        self.saved = (node_module.String, getattr(node_module.nepi_utils, 'get_time', None))
        node_module.String = FakeMessage
        node_module.nepi_utils.get_time = lambda: self.now
        node = node_module.NepiConnectServoNode.__new__(node_module.NepiConnectServoNode)
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
        rbx_nepi_connect_servo_node.String = self.saved[0]
        rbx_nepi_connect_servo_node.nepi_utils.get_time = self.saved[1]

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
        self.assertIn('NEPI does not have the robot', self.warnings[0])
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

    def test_reads_the_link_status_the_rbx_node_publishes(self):
        # The servo node reads only nepi_control and servos, which kept their shape
        link = NepiConnectLink(ROV, dict(TEAM_ANNOUNCEMENT, servos = ['claw_pitch']), log = lambda msg: None)
        self.node.linkStatusCb(FakeMessage(json.dumps(link.get_health())))
        self.assertEqual((self.node.nepi_control, self.node.servo_value), (False, 0.0))


class TestLoopback(unittest.TestCase):

    def setUp(self):
        [self.status_port, self.demand_port] = freeUdpPorts()
        self.logs = []
        self.pi = makeRobot(robotRaw(self.status_port, self.demand_port), logs = self.logs)
        self.hooks = HooksInstalled(self.pi)
        self.hooks.install()
        self.link = None
        self.merged = None

    def tearDown(self):
        if self.link is not None:
            self.link.stop()
        self.pi.stop()
        self.hooks.restore()

    def tick(self, seconds, command = None, until = None):
        # The node's 10 Hz timer and the team's main loop, in real time
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if command is not None:
                self.link.set_teleop_velocity(*command)
            self.link.send_demand()
            time.sleep(0.02)
            self.merged = nepi_link.merge_inputs([0] * 18)
            if until is not None and until():
                return True
            time.sleep(0.08)
        return until is None or until()

    def test_discovery_then_threads_hand_over_release_and_link_down(self):
        [on, msg] = self.pi.start(robotRaw(self.status_port, self.demand_port), 'test')
        self.assertTrue(on, msg)
        # Discovery hears the vehicle message and launches the node from it
        discovery = NepiConnectDiscovery.__new__(NepiConnectDiscovery)
        [vehicle, note] = discovery.probeVehicle(ROV, str(self.status_port))
        self.assertIsNotNone(vehicle, note)
        self.assertEqual((vehicle['name'], vehicle['demand_port']), ('mate_coral_crusaders', self.demand_port))
        self.link = NepiConnectLink(ROV, vehicle, self.status_port, bind_ip = ROV, log = self.logs.append)
        [success, msg] = self.link.start()
        self.assertTrue(success, msg)
        self.assertTrue(waitFor(self.link.is_link_up, 2.0))
        # Idle: NEPI is ready, and the pilot keeps the ROV
        self.assertTrue(self.tick(2.0, until = lambda: self.pi.build_status()['nepi_ready']))
        self.tick(0.3)
        self.assertEqual(self.pi.build_status()['control'], False)
        # The robot's vehicle messages keep arriving and match the launch one
        self.assertTrue(waitFor(lambda: self.link.vehicle_seq >= 1, 1.5))
        self.assertFalse(self.link.get_health()['vehicle_changed'])
        # The pilot presses RB; the link sees NEPI control, and the command merges
        nepi_link.merge_inputs(pilot(rb = 1))
        self.assertTrue(self.tick(1.0, until = self.link.is_nepi_control))
        self.assertTrue(self.tick(2.0, (0.1, 0.0, 0.0, 0.0), until = lambda: self.merged[LY_SLOT] == SURGE_0_1))
        self.assertEqual(self.link.get_health()['control_reason'], 'pilot_button')
        # NEPI hands it back: one active false, and the robot reports nepi_off
        self.assertTrue(self.link.release_to_pilot()[0])
        self.assertTrue(self.tick(1.0, until = lambda: self.link.get_health()['control_reason'] == 'nepi_off'))
        self.assertFalse(self.link.is_nepi_control())
        self.assertEqual(self.pi.build_status()['demand_bad'], 0)
        self.assertEqual(self.link.status_bad, 0)
        # The robot stops: the link goes down within 0.5 s and sends nothing
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


class FakeRobotSender(object):
    """Send from bind_ip to the robot address and port every 100 ms.

    By default a real robot: a status every 100 ms and a vehicle message
    every 0.5 s, the first one at once. With datagram, that datagram every
    100 ms instead.
    """

    def __init__(self, bind_ip, port, datagram = None):
        self.pi = makeRobot(robotRaw(team = True))
        self.datagram = datagram
        self.port = port
        self.sock = openSocket(bind_ip)
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target = self.run)
        self.thread.daemon = True
        self.thread.start()

    def run(self):
        count = 0
        while not self.stop_event.is_set():
            datagrams = [self.datagram]
            if self.datagram is None:
                datagrams = [self.pi.nextStatusDatagram()]
                if count % 5 == 0:
                    datagrams.insert(0, self.pi.nextVehicleDatagram())
            for datagram in datagrams:
                self.sock.sendto(datagram, (ROV, self.port))
            count += 1
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
        self.discovery = NepiConnectDiscovery.__new__(NepiConnectDiscovery)

    def tearDown(self):
        if self.sender is not None:
            self.sender.stop()

    def probe(self):
        start = time.monotonic()
        found = self.discovery.checkForNepiConnectDevice(ROV, str(self.port))
        return found, time.monotonic() - start

    def assertPortFree(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.bind(('', self.port))
        finally:
            sock.close()

    def test_vehicle_message_from_the_robot_is_found(self):
        self.sender = FakeRobotSender(ROV, self.port)
        [found, elapsed] = self.probe()
        self.assertTrue(found)
        # Vehicle messages come every 0.5 s, so the probe waits up to that
        self.assertLess(elapsed, 0.8)
        self.assertPortFree()

    def test_vehicle_message_from_the_wrong_address_is_not_found(self):
        self.sender = FakeRobotSender(OTHER, self.port)
        [found, elapsed] = self.probe()
        self.assertFalse(found)
        self.assertGreater(elapsed, 1.4)
        self.assertLess(elapsed, 2.0)
        self.assertPortFree()

    def test_a_demand_is_not_a_vehicle_message(self):
        demand = json.dumps(dict(v = 2, type = 'demand', seq = 0, t = WALL_EPOCH, active = False,
                                 axes = ALL_AXES_ZERO, servos = dict())).encode('utf-8')
        self.sender = FakeRobotSender(ROV, self.port, demand)
        self.assertFalse(self.probe()[0])

    def test_statuses_alone_are_not_found(self):
        self.sender = FakeRobotSender(ROV, self.port, statusBytes())
        self.assertEqual(self.discovery.probeVehicle(ROV, str(self.port)), (None, ''))

    def test_a_version_1_robot_is_named(self):
        old = json.dumps(dict(v = 1, type = 'status', seq = 0, t = WALL_EPOCH, assist = False)).encode('utf-8')
        self.sender = FakeRobotSender(ROV, self.port, old)
        [vehicle, note] = self.discovery.probeVehicle(ROV, str(self.port))
        self.assertIsNone(vehicle)
        self.assertIn('version 1', note)

    def test_silence_is_not_found(self):
        self.assertFalse(self.probe()[0])
        self.assertPortFree()

    def test_the_probe_reads_the_announcement(self):
        servos = ['claw_pitch', 'claw_grip']
        self.sender = FakeRobotSender(ROV, self.port, vehicleBytes(servos = servos))
        [vehicle, note] = self.discovery.probeVehicle(ROV, str(self.port))
        self.assertEqual(vehicle['servos'], servos)
        self.assertEqual(vehicle['axes'], TEAM_ANNOUNCEMENT['axes'])
        self.sender.stop()
        self.sender = FakeRobotSender(ROV, self.port, vehicleBytes())
        self.assertEqual(self.discovery.probeVehicle(ROV, str(self.port))[0], TEAM_ANNOUNCEMENT)

    def test_a_port_held_by_a_running_link_is_never_probed(self):
        link = NepiConnectLink(ROV, TEAM_ANNOUNCEMENT, self.port, log = lambda msg: None)
        [success, msg] = link.start()
        self.assertTrue(success, msg)
        try:
            self.sender = FakeRobotSender(ROV, self.port)
            [found, elapsed] = self.probe()
            self.assertFalse(found)
            self.assertLess(elapsed, 0.5)
            self.assertTrue(waitFor(link.is_link_up, 1.0)) # The running link still has the port
        finally:
            link.stop()


if __name__ == '__main__':
    unittest.main()
