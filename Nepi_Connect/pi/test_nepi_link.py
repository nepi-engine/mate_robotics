#!/usr/bin/env python3
##
## Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
##
## This file is part of nepi-engine
## (see https://github.com/nepi-engine).
##
## License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
##

# Tests for the nepi_connect package and the nepi_link.py hook file. Standard
# library plus PyYAML. Run from this folder:
#   python3 -B -m unittest -v test_nepi_link
#
# Every test loads the real vehicles/mate_coral_crusaders.yaml. The tests
# carried over from version 1 were written against version 1's defaults
# (every sign +1, triggers 10 to 255), so makeLink() applies V1_DEFAULTS on
# top of the team file and their numbers stand; makeLink(team = True), and
# TestTeamFile, use the team file exactly as it is. TestSecondVehicle drives
# a made-up boat, to show nothing MATE-specific lives in the core.
#
# Every timing test uses an injected clock. Only TestLoopback opens sockets,
# and it sends to 127.0.0.1 alone, with free ports and real time; it drives
# the link with tools/fake_nepi.py's DemandSender and StatusReceiver.

import copy
import json
import math
import os
import socket
import sys
import tempfile
import time
import unittest

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..', 'tools'))

import nepi_link
import fake_nepi
from nepi_connect import control
from nepi_connect import link as nc_link
from nepi_connect import mapping
from nepi_connect import telemetry
from nepi_connect import vehicle
from nepi_connect.adapters import Adapter
from nepi_connect.adapters import hooks


VEHICLES = os.path.join(HERE, '..', 'vehicles')
TEAM_FILE = os.path.join(VEHICLES, 'mate_coral_crusaders.yaml')
BENCH_FILE = os.path.join(VEHICLES, 'mate_bench.yaml')
TEMPLATE_FILE = os.path.join(VEHICLES, 'template.yaml')
NEPI_CONNECT_DIR = os.path.join(HERE, 'nepi_connect')

with open(TEAM_FILE) as team_file:
    TEAM_RAW = yaml.safe_load(team_file)
TEAM = vehicle.validate_vehicle(TEAM_RAW)[0]
NEPI = TEAM['network']['nepi_ip']
SLOTS = dict((name, entry['slot']) for (name, entry) in TEAM['control']['inputs'].items())
LX_SLOT = SLOTS['ABS_X']
LY_SLOT = SLOTS['ABS_Y']
RX_SLOT = SLOTS['ABS_RX']
RY_SLOT = SLOTS['ABS_RY']
LB_SLOT = SLOTS['BTN_TL']
RB_SLOT = SLOTS['BTN_TR']
LT_SLOT = SLOTS['ABS_Z']
RT_SLOT = SLOTS['ABS_RZ']
HAT0Y_SLOT = SLOTS['ABS_HAT0Y']
SOUTH_SLOT = SLOTS['BTN_SOUTH']
TN_NAMES = [field['name'] for field in TEAM['telemetry']['fields']]
AXES = list(nc_link.AXIS_NAMES)

BENCH_LINE = '$TNCTL,0,0,0,0,100,100,100,100,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,*FF'
STATUS_FIELDS = ['v', 'type', 'seq', 't', 'control', 'control_reason', 'nepi_ready', 'pilot', 'servos', 'out',
                 'demand_seq', 'demand_age_ms', 'demand_bad', 'nav', 'nav_age_ms', 'nav_count', 'nav_bad', 'raw']
VEHICLE_FIELDS = ['v', 'type', 'seq', 't', 'name', 'kind', 'demand_port', 'axes', 'servos', 'nav', 'raw', 'cameras',
                  'imu_mount']
# Version 1's defaults where the team file differs: surge sign +1, triggers 10 to 255
V1_DEFAULTS = dict(signs = dict(surge = 1), trigger = (10, 255))
# Two claw servos, the way the team's sketch might drive them: D-pad up/down
# and the A button
CLAW_SERVOS = dict(claw_pitch = dict(input = 'ABS_HAT0Y'), claw_grip = dict(input = 'BTN_SOUTH'))
TAKEOVER_STICK = 8192 # takeover_fraction 0.25 of 32767, rounded up
TAKEOVER_TRIGGER = 64 # 0.25 of version 1's default trigger max 255, rounded up
OMIT = object()


class FakeClock(object):

    def __init__(self, now = 1000.0):
        self.now = now

    def __call__(self):
        return self.now

    def advance(self, seconds):
        self.now += seconds


def makeRaw(team = False, caps = None, signs = None, trigger = None, servos = None, takeover = None, nepi_ip = None):
    # The real team file, with the version 1 defaults unless team, plus the changes given
    raw = copy.deepcopy(TEAM_RAW)
    changes = dict()
    if not team:
        changes.update(signs = dict(V1_DEFAULTS['signs']), trigger = V1_DEFAULTS['trigger'])
    if signs is not None:
        changes.setdefault('signs', dict()).update(signs)
    if trigger is not None:
        changes['trigger'] = trigger
    for [name, value] in changes.get('signs', dict()).items():
        raw['control']['axes'][name]['sign'] = value
    if 'trigger' in changes:
        for name in ['ABS_Z', 'ABS_RZ']:
            [raw['control']['inputs'][name]['deadzone'], raw['control']['inputs'][name]['max']] = changes['trigger']
    for [name, value] in (caps or dict()).items():
        raw['control']['axes'][name]['cap'] = value
    if servos is not None:
        raw['control']['servos'] = copy.deepcopy(servos)
    if takeover is not None:
        raw['control']['takeover_fraction'] = takeover
    if nepi_ip is not None:
        raw['network']['nepi_ip'] = nepi_ip
    return raw

def makeLink(**changes):
    clock = FakeClock()
    logs = []
    link = control.NepiControl(clock = clock, wall_clock = lambda: 1792200000.25, log = logs.append, name = 'nepi_link')
    [on, msg] = link.configure(makeRaw(**changes))
    if not on:
        raise AssertionError('link did not configure: ' + msg)
    return link, clock, logs

def validate(raw):
    return vehicle.validate_vehicle(raw)

def demandBytes(seq, active, surge = 0.0, sway = 0.0, yaw = 0.0, heave = OMIT, **changes):
    axes = dict(surge = surge, sway = sway, yaw = yaw)
    if heave is not OMIT:
        axes['heave'] = heave
    message = dict(v = 2, type = 'demand', seq = seq, t = 1792200000.25, active = active, axes = axes)
    message.update(changes)
    return json.dumps(message).encode('utf-8')

def pilot(lx = 0, ly = 0, rx = 0, ry = 0, lt = 0, rt = 0, lb = 0, rb = 0):
    inputs = [0] * 18
    inputs[LX_SLOT] = lx
    inputs[LY_SLOT] = ly
    inputs[RX_SLOT] = rx
    inputs[RY_SLOT] = ry
    inputs[LT_SLOT] = lt
    inputs[RT_SLOT] = rt
    inputs[LB_SLOT] = lb
    inputs[RB_SLOT] = rb
    return inputs

def stick(demand):
    entry = TEAM['control']['inputs']['ABS_Y']
    return mapping.stick_value(demand, entry['min'], entry['neutral'], entry['max'], entry['deadzone'])

def topsideTag(link):
    # nepi_link.topside_tag() for this link, through the real hooks adapter
    saved = hooks._adapter
    hooks._adapter = hooks.HooksAdapter(link)
    try:
        return nepi_link.topside_tag()
    finally:
        hooks._adapter = saved

def engage(link, clock, surge = 0.0, sway = 0.0, yaw = 0.0, seq = 0, heave = 0.0):
    # NEPI is ready (an active true demand), then the pilot presses and releases RB
    [accepted, msg] = link.handle_demand(demandBytes(seq, True, surge, sway, yaw, heave = heave), NEPI)
    if not accepted:
        raise AssertionError('engage demand dropped: ' + msg)
    link.merge_inputs(pilot(rb = 1))
    link.merge_inputs(pilot())
    if not link.build_status()['control']:
        raise AssertionError('RB did not turn NEPI control on')
    return seq + 1

def freeUdpPort():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('127.0.0.1', 0))
    port = sock.getsockname()[1]
    sock.close()
    return port


class TestMapping(unittest.TestCase):

    def test_stick_table(self):
        self.assertEqual(stick(0.01), 3000)
        self.assertEqual(stick(0.2), 8713)
        self.assertEqual(stick(1.0), 32767)
        self.assertEqual(stick(-0.2), -8713)
        self.assertEqual(stick(0.0099), 0)
        self.assertEqual(stick(0.0), 0)

    def test_trigger_table_10_255(self):
        self.assertEqual(mapping.trigger_value(0.01, 0, 255, 10), 10)
        self.assertEqual(mapping.trigger_value(0.1, 0, 255, 10), 32)
        self.assertEqual(mapping.trigger_value(1.0, 0, 255, 10), 255)
        self.assertEqual(mapping.trigger_value(0.0099, 0, 255, 10), 0)

    def test_trigger_table_40_1023(self):
        self.assertEqual(mapping.trigger_value(0.01, 0, 1023, 40), 40)
        self.assertEqual(mapping.trigger_value(1.0, 0, 1023, 40), 1023)

    def test_merge_uses_table_with_caps_at_one(self):
        link, clock, logs = makeLink(caps = dict(surge = 1.0, sway = 1.0, yaw = 1.0))
        engage(link, clock, surge = 1.0, sway = 0.2, yaw = 0.1)
        out = link.merge_inputs(pilot())
        self.assertEqual(out[LY_SLOT], 32767)
        self.assertEqual(out[LX_SLOT], 8713)
        self.assertEqual(out[LT_SLOT], 32)
        self.assertEqual(out[RT_SLOT], 0)

    def test_negative_yaw_uses_right_trigger(self):
        link, clock, logs = makeLink(caps = dict(yaw = 1.0))
        engage(link, clock, yaw = -1.0)
        out = link.merge_inputs(pilot())
        self.assertEqual(out[LT_SLOT], 0)
        self.assertEqual(out[RT_SLOT], 255)

    def test_caps_clamp_before_mapping(self):
        link, clock, logs = makeLink(trigger = (40, 1023))
        engage(link, clock, surge = 1.0, sway = -1.0, yaw = 1.0)
        out = link.merge_inputs(pilot())
        self.assertEqual(out[LY_SLOT], 8713) # surge cap 0.2
        self.assertEqual(out[LX_SLOT], -8713) # sway cap 0.2
        self.assertEqual(out[LT_SLOT], 129) # yaw cap 0.1: 40 + 0.09 / 0.99 * 983 = 129.4

    def test_signs(self):
        link, clock, logs = makeLink(signs = dict(surge = -1, sway = -1, yaw = -1))
        engage(link, clock, surge = 0.2, sway = 0.2, yaw = 0.1)
        out = link.merge_inputs(pilot())
        self.assertEqual(out[LY_SLOT], -8713)
        self.assertEqual(out[LX_SLOT], -8713)
        self.assertEqual(out[LT_SLOT], 0)
        self.assertEqual(out[RT_SLOT], 32)

    def test_tiny_and_zero_demands_send_zero(self):
        link, clock, logs = makeLink()
        engage(link, clock, surge = 0.005, sway = 0.0, yaw = 0.0, heave = -0.005)
        out = link.merge_inputs(pilot(ly = 1500, lx = -2000, ry = 2500, lt = 5, rt = 3))
        self.assertEqual(out[LY_SLOT], 0)
        self.assertEqual(out[LX_SLOT], 0)
        self.assertEqual(out[RY_SLOT], 0)
        self.assertEqual(out[LT_SLOT], 0)
        self.assertEqual(out[RT_SLOT], 0)

    def test_heave_drives_ry_with_its_cap_and_sign(self):
        link, clock, logs = makeLink()
        engage(link, clock, heave = 1.0)
        out = link.merge_inputs(pilot())
        self.assertEqual(out[RY_SLOT], 8713) # heave cap 0.2
        self.assertEqual(link.build_status()['out']['ABS_RY'], 8713)
        link, clock, logs = makeLink(caps = dict(heave = 1.0), signs = dict(heave = -1))
        engage(link, clock, heave = 0.2)
        self.assertEqual(link.merge_inputs(pilot())[RY_SLOT], -8713)
        link, clock, logs = makeLink()
        engage(link, clock, heave = -0.1)
        self.assertEqual(link.merge_inputs(pilot())[RY_SLOT], stick(-0.1))

    def test_demand_without_heave_holds_depth(self):
        # An announced axis missing from the demand counts as 0, so RY stays 0
        link, clock, logs = makeLink()
        engage(link, clock, surge = 0.2)
        datagram = demandBytes(1, True, 0.2)
        self.assertNotIn(b'heave', datagram)
        [accepted, msg] = link.handle_demand(datagram, NEPI)
        self.assertTrue(accepted, msg)
        out = link.merge_inputs(pilot(ry = 2500))
        self.assertEqual((out[LY_SLOT], out[RY_SLOT]), (8713, 0))

    def test_unknown_axis_names_are_ignored(self):
        link, clock, logs = makeLink()
        seq = engage(link, clock, surge = 0.2)
        [accepted, msg] = link.handle_demand(demandBytes(seq, True, 0.2, pitch = 1.0, roll = 'x',
                                                         axes = dict(surge = 0.2, pitch = 5, roll = 'x')), NEPI)
        self.assertTrue(accepted, msg)
        self.assertEqual(link.merge_inputs(pilot())[LY_SLOT], 8713)


class TestNepiControl(unittest.TestCase):

    def setUp(self):
        self.link, self.clock, self.logs = makeLink()
        engage(self.link, self.clock, surge = 0.2, sway = 0.2, yaw = 0.1)

    def state(self):
        status = self.link.build_status()
        return status['control'], status['control_reason']

    def test_pilot_inputs_short_of_takeover_are_replaced(self):
        # Past the Teensy's dead zone but short of the takeover point: NEPI keeps every axis
        out = self.link.merge_inputs(pilot(ly = TAKEOVER_STICK - 1, lx = -5000, ry = 4000, lt = TAKEOVER_TRIGGER - 1))
        self.assertEqual(out[LY_SLOT], 8713)
        self.assertEqual(out[LX_SLOT], 8713)
        self.assertEqual(out[RY_SLOT], 0)
        self.assertEqual((out[LT_SLOT], out[RT_SLOT]), (32, 0))
        self.assertEqual(self.state(), (True, 'pilot_button'))

    def test_each_stick_past_takeover_ends_nepi_control(self):
        for [name, value] in [['ly', TAKEOVER_STICK], ['ly', -TAKEOVER_STICK], ['lx', TAKEOVER_STICK],
                              ['lx', -TAKEOVER_STICK], ['ry', TAKEOVER_STICK], ['ry', -TAKEOVER_STICK]]:
            link, clock, logs = makeLink()
            engage(link, clock, surge = 0.2)
            inputs = pilot(**{name: value})
            self.assertEqual(link.merge_inputs(inputs), tuple(inputs), name)
            status = link.build_status()
            self.assertEqual((status['control'], status['control_reason']), (False, 'pilot_stick'), name)
            self.assertIn('the pilot moved ' + name.upper(), logs[-1])

    def test_each_trigger_past_takeover_ends_nepi_control(self):
        for name in ['lt', 'rt']:
            link, clock, logs = makeLink()
            engage(link, clock, yaw = 0.1)
            inputs = pilot(**{name: TAKEOVER_TRIGGER})
            self.assertEqual(link.merge_inputs(inputs), tuple(inputs), name)
            self.assertEqual(link.build_status()['control_reason'], 'pilot_stick', name)

    def test_takeover_point_follows_the_trigger_max(self):
        link, clock, logs = makeLink(trigger = (40, 1023))
        engage(link, clock)
        link.merge_inputs(pilot(lt = 255)) # 0.25 of 1023 is 255.75, so 256 is the takeover point
        self.assertTrue(link.build_status()['control'])
        link.merge_inputs(pilot(lt = 256))
        self.assertEqual(link.build_status()['control_reason'], 'pilot_stick')

    def test_after_takeover_only_rb_turns_it_back_on(self):
        self.link.merge_inputs(pilot(ly = 20000))
        self.link.merge_inputs(pilot())
        self.clock.advance(0.1)
        self.link.handle_demand(demandBytes(10, True, 0.2), NEPI)
        self.link.merge_inputs(pilot())
        self.assertEqual(self.state(), (False, 'pilot_stick'))
        self.link.merge_inputs(pilot(rb = 1))
        self.assertEqual(self.state(), (True, 'pilot_button'))

    def test_other_inputs_pass_through_and_pilot_list_untouched(self):
        inputs = list(range(100, 118))
        inputs[LY_SLOT] = 0
        inputs[LX_SLOT] = 0
        inputs[LT_SLOT] = 0
        inputs[RT_SLOT] = 0
        inputs[LB_SLOT] = 0
        inputs[RB_SLOT] = 0
        before = list(inputs)
        out = self.link.merge_inputs(inputs)
        self.assertEqual(inputs, before)
        self.assertIsInstance(out, tuple)
        self.assertEqual(len(out), 18)
        changed = [i for i in range(18) if out[i] != inputs[i]]
        self.assertEqual(changed, [LX_SLOT, LY_SLOT, RY_SLOT, LT_SLOT])
        self.assertEqual(out[RY_SLOT], 0)
        self.assertEqual(out[RX_SLOT], inputs[RX_SLOT])

    def test_status_reports_pilot_flags_and_outputs(self):
        self.link.merge_inputs(pilot(ly = 5000, ry = -3000, rt = 10))
        status = self.link.build_status()
        self.assertTrue(status['pilot']['surge'])
        self.assertFalse(status['pilot']['sway'])
        self.assertTrue(status['pilot']['yaw'])
        self.assertTrue(status['pilot']['heave'])
        self.assertTrue(status['control'])
        out = status['out']
        self.assertEqual((out['ABS_Y'], out['ABS_X'], out['ABS_Z'], out['ABS_RZ']), (8713, 8713, 32, 0))


class TestTimeouts(unittest.TestCase):

    def setUp(self):
        self.link, self.clock, self.logs = makeLink()
        self.seq = engage(self.link, self.clock, surge = 0.2)

    def test_demand_older_than_250_ms_goes_neutral_with_wait(self):
        self.clock.advance(0.24)
        self.assertEqual(topsideTag(self.link), ',NEPI:ON')
        self.assertEqual(self.link.merge_inputs(pilot(ly = 1500))[LY_SLOT], 8713)
        self.clock.advance(0.02)
        out = self.link.merge_inputs(pilot(ly = 1500, lx = -100, ry = 2000))
        self.assertEqual((out[LY_SLOT], out[LX_SLOT], out[RY_SLOT]), (0, 0, 0))
        self.assertEqual(topsideTag(self.link), ',NEPI:WAIT')
        status = self.link.build_status()
        self.assertTrue(status['control'])
        self.assertEqual(status['out']['ABS_Y'], 0)
        self.assertEqual(status['demand_age_ms'], 260)
        # A fresh demand restores NEPI's axes
        self.link.handle_demand(demandBytes(self.seq, True, 0.2), NEPI)
        self.assertEqual(topsideTag(self.link), ',NEPI:ON')
        self.assertEqual(self.link.merge_inputs(pilot())[LY_SLOT], 8713)

    def test_two_seconds_without_a_demand_turns_nepi_control_off(self):
        self.clock.advance(1.98)
        self.assertEqual(topsideTag(self.link), ',NEPI:WAIT')
        self.assertTrue(self.link.build_status()['control'])
        self.clock.advance(0.04)
        self.assertEqual(topsideTag(self.link), ',NEPI:NOTREADY')
        status = self.link.build_status()
        self.assertFalse(status['control'])
        self.assertEqual(status['control_reason'], 'timeout')

    def test_timeout_seen_by_status_without_merge_calls(self):
        self.clock.advance(2.5)
        status = self.link.build_status()
        self.assertFalse(status['control'])
        self.assertEqual(status['control_reason'], 'timeout')


class TestToggle(unittest.TestCase):

    def setUp(self):
        self.link, self.clock, self.logs = makeLink()

    def state(self):
        status = self.link.build_status()
        return status['control'], status['control_reason']

    def test_starts_off_and_not_ready(self):
        self.assertEqual(self.state(), (False, 'start'))
        self.assertFalse(self.link.build_status()['nepi_ready'])
        self.assertEqual(topsideTag(self.link), ',NEPI:NOTREADY')
        inputs = pilot(ly = 100)
        self.assertEqual(self.link.merge_inputs(inputs), tuple(inputs))

    def test_active_true_alone_never_turns_nepi_control_on(self):
        for seq in range(5):
            [accepted, msg] = self.link.handle_demand(demandBytes(seq, True, 0.2), NEPI)
            self.assertTrue(accepted, msg)
            self.assertEqual(self.link.merge_inputs(pilot())[LY_SLOT], 0)
            self.clock.advance(0.1)
        self.assertEqual(self.state(), (False, 'start'))
        self.assertTrue(self.link.build_status()['nepi_ready'])
        self.assertEqual(topsideTag(self.link), ',NEPI:OFF')

    def test_rb_toggles_nepi_control(self):
        self.link.handle_demand(demandBytes(0, True, 0.2), NEPI)
        self.link.control_event.clear()
        out = self.link.merge_inputs(pilot(rb = 1))
        self.assertEqual(self.state(), (True, 'pilot_button'))
        self.assertTrue(self.link.control_event.is_set()) # The status goes out at once
        self.assertEqual(out[RB_SLOT], 1) # RB still reaches the Teensy
        self.assertEqual(out[LY_SLOT], 8713)
        self.assertEqual(topsideTag(self.link), ',NEPI:ON')
        self.link.merge_inputs(pilot())
        out = self.link.merge_inputs(pilot(rb = 1))
        self.assertEqual(self.state(), (False, 'pilot_button'))
        self.assertEqual(out[LY_SLOT], 0)
        self.assertEqual(topsideTag(self.link), ',NEPI:OFF')

    def test_rb_ignored_while_nepi_is_not_ready(self):
        # No demand yet
        self.link.merge_inputs(pilot(rb = 1))
        self.link.merge_inputs(pilot())
        self.assertEqual(self.state(), (False, 'start'))
        self.assertIn('NEPI is not connected', self.logs[-1])
        self.assertEqual(self.logs[-1], 'nepi_link: RB ignored: NEPI is not connected')
        # NEPI's newest demand is active false
        self.link.handle_demand(demandBytes(0, False), NEPI)
        self.link.merge_inputs(pilot(rb = 1))
        self.link.merge_inputs(pilot())
        self.assertEqual(self.state(), (False, 'start'))
        # NEPI's newest active true is older than 0.5 s
        self.link.handle_demand(demandBytes(1, True), NEPI)
        self.clock.advance(0.51)
        self.assertEqual(topsideTag(self.link), ',NEPI:NOTREADY')
        self.link.merge_inputs(pilot(rb = 1))
        self.assertEqual(self.state(), (False, 'start'))

    def test_rb_ignored_while_a_movement_input_is_past_takeover(self):
        self.link.handle_demand(demandBytes(0, True), NEPI)
        self.link.merge_inputs(pilot(rb = 1, rt = TAKEOVER_TRIGGER))
        self.assertEqual(self.state(), (False, 'start'))
        self.assertIn('release RT first', self.logs[-1])

    def test_held_button_counts_once(self):
        engage(self.link, self.clock, surge = 0.2)
        for i in range(3):
            self.link.merge_inputs(pilot(rb = 1))
        self.assertEqual(self.state(), (False, 'pilot_button'))
        self.link.merge_inputs(pilot(rb = 0))
        for i in range(3):
            self.link.merge_inputs(pilot(rb = 1))
        self.assertEqual(self.state(), (True, 'pilot_button'))

    def test_active_false_ends_nepi_control_and_rb_turns_it_back_on(self):
        seq = engage(self.link, self.clock, surge = 0.2)
        self.link.handle_demand(demandBytes(seq, False), NEPI)
        self.assertEqual(self.state(), (False, 'nepi_off'))
        self.link.handle_demand(demandBytes(seq + 1, True, 0.2), NEPI)
        self.link.merge_inputs(pilot())
        self.assertEqual(self.state(), (False, 'nepi_off'))
        self.link.merge_inputs(pilot(rb = 1))
        self.assertEqual(self.state(), (True, 'pilot_button'))

    def test_after_timeout_needs_rb(self):
        engage(self.link, self.clock, surge = 0.2)
        self.clock.advance(2.5)
        self.link.merge_inputs(pilot())
        self.assertEqual(self.state(), (False, 'timeout'))
        # NEPI comes back (seq 0 is accepted after the 1 s restart window)
        self.link.handle_demand(demandBytes(0, True, 0.2), NEPI)
        self.assertEqual(self.link.build_status()['demand_seq'], 0)
        self.link.merge_inputs(pilot())
        self.assertEqual(self.state(), (False, 'timeout'))
        self.link.merge_inputs(pilot(rb = 1))
        self.assertEqual(self.state(), (True, 'pilot_button'))

    def test_reserved_lb_passes_through_untouched(self):
        # Replaces version 1's slow-mode tracking: LB is reserved, never
        # written, never read, and the status carries no slow flag
        self.assertNotIn('slow', self.link.build_status())
        for lb in [1, 0, 1]:
            self.assertEqual(self.link.merge_inputs(pilot(lb = lb))[LB_SLOT], lb)
        engage(self.link, self.clock, surge = 0.2)
        for lb in [1, 0]:
            out = self.link.merge_inputs(pilot(lb = lb))
            self.assertEqual((out[LB_SLOT], out[LY_SLOT]), (lb, 8713))
        self.assertEqual(self.state(), (True, 'pilot_button'))


class TestDemandValidation(unittest.TestCase):

    def setUp(self):
        self.link, self.clock, self.logs = makeLink()
        self.seq = engage(self.link, self.clock, surge = 0.2)
        self.clock.advance(0.1)

    def assertDropped(self, datagram, source = NEPI):
        before = self.link.build_status()
        [accepted, msg] = self.link.handle_demand(datagram, source)
        self.assertFalse(accepted, msg)
        after = self.link.build_status()
        self.assertEqual(after['demand_bad'], before['demand_bad'] + 1)
        for key in ['control', 'control_reason', 'demand_seq', 'demand_age_ms']:
            self.assertEqual(after[key], before[key], key)
        self.assertEqual(self.link.merge_inputs(pilot())[LY_SLOT], 8713)

    def test_not_json(self):
        self.assertDropped(b'not json')

    def test_not_utf8(self):
        self.assertDropped(b'\xff\xfe{}')

    def test_not_an_object(self):
        self.assertDropped(b'[1, 2, 3]')

    def test_deeply_nested(self):
        self.assertDropped(b'[' * 600 + b']' * 600)

    def test_version_1(self):
        self.assertDropped(demandBytes(self.seq, True, 0.1, v = 1))

    def test_version_as_bool(self):
        self.assertDropped(demandBytes(self.seq, True, 0.1, v = True))

    def test_wrong_type(self):
        self.assertDropped(demandBytes(self.seq, True, 0.1, type = 'status'))

    def test_missing_field(self):
        for field in ['active', 't']:
            message = json.loads(demandBytes(self.seq, True, 0.1).decode('utf-8'))
            del message[field]
            self.assertDropped(json.dumps(message).encode('utf-8'))

    def test_missing_axes_and_servos_mean_none(self):
        # A robot need not have every part, so a demand with no axes or servos
        # field (or a null one) is every axis 0 and no servo values
        for [axes, servos] in [[OMIT, OMIT], [None, None]]:
            message = json.loads(demandBytes(self.seq, True, 0.1).decode('utf-8'))
            for [field, value] in [['axes', axes], ['servos', servos]]:
                if value is OMIT:
                    message.pop(field, None)
                else:
                    message[field] = value
            [accepted, msg] = self.link.handle_demand(json.dumps(message).encode('utf-8'), NEPI)
            self.assertTrue(accepted, msg)
            self.assertEqual(self.link.demand['axes'], dict(surge = 0.0, sway = 0.0, heave = 0.0, yaw = 0.0))
            self.assertEqual(self.link.demand['servos'], dict())
            self.seq += 1

    def test_axes_not_an_object(self):
        for bad in [[0.1, 0.0], 'surge', 0.1]:
            self.assertDropped(demandBytes(self.seq, True, axes = bad))

    def test_bool_as_number(self):
        self.assertDropped(demandBytes(self.seq, True, surge = True))

    def test_number_as_bool(self):
        self.assertDropped(demandBytes(self.seq, 1, 0.1))

    def test_string_as_number(self):
        self.assertDropped(demandBytes(self.seq, True, surge = '0.1'))

    def test_nan(self):
        self.assertDropped(demandBytes(self.seq, True, surge = float('nan')))

    def test_infinity(self):
        self.assertDropped(demandBytes(self.seq, True, yaw = float('inf')))
        self.assertDropped(demandBytes(self.seq, True, 0.1, t = float('-inf')))

    def test_bad_heave(self):
        for bad in [1.5, -1.01, None, True, '0.1', float('nan')]:
            self.assertDropped(demandBytes(self.seq, True, 0.1, heave = bad))

    def test_out_of_range(self):
        self.assertDropped(demandBytes(self.seq, True, surge = 1.01))
        self.assertDropped(demandBytes(self.seq, True, sway = -1.5))
        self.assertDropped(demandBytes(self.seq, True, 0.1, heave = 2))
        self.assertDropped(demandBytes(-1, True, 0.1))
        self.assertDropped(demandBytes(4294967296, True, 0.1))

    def test_float_seq(self):
        self.assertDropped(demandBytes(float(self.seq), True, 0.1))

    def test_over_1200_bytes(self):
        datagram = demandBytes(self.seq, True, 0.1)
        padding = 1201 - len(datagram) - len(b', "pad": ""')
        self.assertDropped(demandBytes(self.seq, True, 0.1, pad = 'x' * padding))

    def test_exactly_1200_bytes_and_extra_fields_accepted(self):
        datagram = demandBytes(self.seq, True, 0.1)
        padding = 1200 - len(datagram) - len(b', "pad": ""')
        datagram = demandBytes(self.seq, True, 0.1, pad = 'x' * padding)
        self.assertEqual(len(datagram), 1200)
        [accepted, msg] = self.link.handle_demand(datagram, NEPI)
        self.assertTrue(accepted, msg)

    def test_wrong_source(self):
        self.assertDropped(demandBytes(self.seq, True, 0.1), '192.168.68.12')
        self.assertDropped(demandBytes(self.seq, True, 0.1), '127.0.0.1')


class TestSeq(unittest.TestCase):

    def setUp(self):
        self.link, self.clock, self.logs = makeLink()

    def send(self, seq):
        return self.link.handle_demand(demandBytes(seq, False), NEPI)[0]

    def test_order(self):
        self.assertTrue(self.send(5))
        self.clock.advance(0.1)
        self.assertFalse(self.send(5))
        self.assertFalse(self.send(4))
        self.assertTrue(self.send(6))
        self.assertTrue(self.send(100))
        status = self.link.build_status()
        self.assertEqual(status['demand_bad'], 2)
        self.assertEqual(status['demand_seq'], 100)

    def test_restart_after_one_second(self):
        self.assertTrue(self.send(100))
        self.clock.advance(1.0)
        self.assertFalse(self.send(0))
        self.clock.advance(0.01)
        self.assertTrue(self.send(0))
        self.assertEqual(self.link.build_status()['demand_seq'], 0)


class TestTelemetry(unittest.TestCase):

    def setUp(self):
        self.link, self.clock, self.logs = makeLink()

    def parse(self, line):
        return telemetry.parse_line(line, TEAM['telemetry'])

    def test_bench_line(self):
        values = self.parse(BENCH_LINE)
        self.assertEqual(list(values.keys()), TN_NAMES)
        self.assertEqual([values[name] for name in TN_NAMES[:8]], [0, 0, 0, 0, 100, 100, 100, 100])
        for name in TN_NAMES[:8]:
            self.assertIsInstance(values[name], int)
        for name in TN_NAMES[8:]:
            self.assertIsInstance(values[name], float)
            self.assertEqual(values[name], 0.0)
        self.link.on_telemetry_line(BENCH_LINE)
        self.clock.advance(0.3)
        status = self.link.build_status()
        nav_fields = dict(TEAM['telemetry']['nav'])
        self.assertEqual(status['nav'], dict((standard, values[field]) for (standard, field) in nav_fields.items()))
        self.assertEqual(status['raw'], dict((name, values[name]) for name in TEAM['telemetry']['raw']))
        # nav and raw together still carry every value version 1's tn did
        self.assertEqual(sorted(set(nav_fields.values()) | set(status['raw'])), sorted(TN_NAMES))
        self.assertEqual((status['nav_count'], status['nav_bad'], status['nav_age_ms']), (1, 0, 300))

    def test_gyro_columns_map_to_board_axes(self):
        line = BENCH_LINE.split(',')
        line[14:17] = ['1.00', '2.00', '3.00'] # yaw_deg_s, pitch_deg_s, roll_deg_s
        self.link.on_telemetry_line(','.join(line))
        nav = self.link.build_status()['nav']
        self.assertEqual((nav['gyro_x_deg_s'], nav['gyro_y_deg_s'], nav['gyro_z_deg_s']), (1.0, 2.0, 3.0))

    def test_line_with_line_ending_and_signs(self):
        values = self.parse('$TNCTL,-90,109,109,-90,1,2,3,4,0.12,-0.50,179.99,-3.00,1.00,'
                            '2.00,3.00,4.00,0.50,0.25,-7.04,0.01,*FF\r\n')
        self.assertEqual(values['vfl_pwr'], -90)
        self.assertEqual(values['yaw_deg'], 179.99)
        self.assertEqual(values['yaw_rate_target_deg_s'], -7.04)

    def test_19_and_25_values_count_as_bad(self):
        parts = BENCH_LINE.split(',')
        line_19 = ','.join(parts[:20] + ['*FF'])
        line_25 = ','.join(parts[:21] + ['0.00'] * 5 + ['*FF'])
        self.assertEqual(len(line_19.split(',')) - 2, 19)
        self.assertEqual(len(line_25.split(',')) - 2, 25)
        self.link.on_telemetry_line(line_19)
        self.link.on_telemetry_line(line_25)
        status = self.link.build_status()
        self.assertEqual((status['nav_count'], status['nav_bad'], status['nav']), (0, 2, None))

    def test_other_lines_count_as_bad(self):
        for line in ['ERRCHK', 'Failed to initialize bar02', 'unicode error', '',
                     BENCH_LINE.replace('$TNCTL', '$TNCTX'), BENCH_LINE.replace(',*FF', ',FF'),
                     BENCH_LINE.replace('0.00,*FF', 'nan,*FF'), BENCH_LINE.replace('0.00,*FF', 'ovf,*FF'),
                     BENCH_LINE.replace(',100,', ',100.5,', 1), BENCH_LINE.replace(',100,', ',99999999999,', 1)]:
            self.assertIsNone(self.parse(line), line)
        self.link.on_telemetry_line('ERRCHK')
        self.link.on_telemetry_line(None)
        self.assertEqual(self.link.build_status()['nav_bad'], 2)


class TestStatus(unittest.TestCase):

    def test_fields_before_any_demand(self):
        link, clock, logs = makeLink()
        status = link.build_status()
        self.assertEqual(sorted(status.keys()), sorted(STATUS_FIELDS))
        self.assertEqual((status['v'], status['type'], status['seq']), (2, 'status', 0))
        self.assertEqual((status['demand_seq'], status['demand_age_ms'], status['nav_age_ms']), (-1, -1, -1))
        self.assertIsNone(status['nav'])
        self.assertIsNone(status['raw'])
        self.assertEqual(status['control_reason'], 'start')
        for key in ['control', 'nepi_ready']:
            self.assertIsInstance(status[key], bool)
        self.assertEqual(status['pilot'], dict(surge = False, sway = False, heave = False, yaw = False))
        self.assertEqual(status['out'], dict(ABS_Y = 0, ABS_X = 0, ABS_RY = 0, ABS_Z = 0, ABS_RZ = 0))

    def test_seq_advances_per_datagram(self):
        link, clock, logs = makeLink()
        first = json.loads(link.nextStatusDatagram().decode('utf-8'))
        second = json.loads(link.nextStatusDatagram().decode('utf-8'))
        self.assertEqual((first['seq'], second['seq']), (0, 1))

    def test_vehicle_message_has_its_own_seq(self):
        link, clock, logs = makeLink(team = True)
        link.nextStatusDatagram()
        first = json.loads(link.nextVehicleDatagram().decode('utf-8'))
        second = json.loads(link.nextVehicleDatagram().decode('utf-8'))
        self.assertEqual((first['seq'], second['seq']), (0, 1))
        self.assertEqual(sorted(first.keys()), sorted(VEHICLE_FIELDS))
        self.assertEqual((first['v'], first['type'], first['name'], first['kind'], first['demand_port']),
                         (2, 'vehicle', 'mate_coral_crusaders', 'ROV', 5701))
        self.assertEqual(first['axes'], dict(surge = 0.2, sway = 0.2, heave = 0.2, yaw = 0.1))
        self.assertEqual(first['servos'], [])
        self.assertEqual(first['nav'], ['depth_m', 'depth_m_s', 'roll_deg', 'pitch_deg', 'yaw_deg', 'gyro_x_deg_s',
                                        'gyro_y_deg_s', 'gyro_z_deg_s'])
        self.assertEqual(len(first['raw']), 12)
        self.assertEqual((first['cameras'], first['imu_mount']), ([], 'X_FORWARD_Y_LEFT'))
        self.assertEqual(link.nextStatusDatagram() is not None, True)

    def test_worst_case_size_under_1200_bytes(self):
        # The most servos, with the longest names
        servos = dict()
        for [number, event] in zip(range(1, vehicle.MAX_SERVOS + 1), ['ABS_RX', 'ABS_HAT0X', 'ABS_HAT0Y', 'BTN_WEST']):
            servos['servo' + str(number) + 'x' * 10] = dict(input = event)
        link, clock, logs = makeLink(trigger = (40, 1023), servos = servos)
        self.assertEqual([len(servo['name']) for servo in link.vehicle['control']['servos']], [16] * vehicle.MAX_SERVOS)
        self.assertEqual(link.vehicle['sizes'], [581, 1134]) # LINK_PROTOCOL.md "Size" quotes these
        link.wall_clock = lambda: 4102444800.123 # 2100-01-01
        worst_line = '$TNCTL,' + ','.join(['-2147483648'] * 8 + ['-9999999999.999999'] * 12) + ',*FF'
        link.on_telemetry_line(worst_line)
        self.assertEqual(link.build_status()['nav_count'], 1)
        huge = 2 ** 53
        link.status_seq = nc_link.SEQ_MAX
        link.demand_seq = nc_link.SEQ_MAX
        link.demand_time = clock() - huge / 1000.0
        link.nav_time = clock() - huge / 1000.0
        link.demand_bad = huge
        link.nav_count = huge
        link.nav_bad = huge
        link.control_reason = 'pilot_button'
        for name in ['ABS_X', 'ABS_Y', 'ABS_RY']:
            link.out[name] = -32768
        link.out['ABS_Z'] = 1023
        link.out['ABS_RZ'] = 1023
        datagram = link.nextStatusDatagram()
        self.assertLessEqual(len(datagram), 1200, str(len(datagram)) + ' bytes')
        # The bound the vehicle file check computed holds this status too
        self.assertLessEqual(len(datagram), link.vehicle['sizes'][1])

    def test_out_values_clamped_to_contract(self):
        # The contract range is each input's own: version 1 reported a stick
        # down to -32768, the team file's ABS_RY min is -32767
        link, clock, logs = makeLink()
        link.merge_inputs(pilot(lt = 1023, ly = 40000, ry = -40000))
        status = link.build_status()
        out = status['out']
        self.assertEqual((out['ABS_Z'], out['ABS_Y'], out['ABS_RY']), (255, 32767, -32767))

    def test_counters_and_ages_stop_at_their_limit(self):
        link, clock, logs = makeLink()
        link.demand_bad = nc_link.COUNT_MAX + 5
        link.demand_time = clock() - 10 ** 9
        status = link.build_status()
        self.assertEqual((status['demand_bad'], status['demand_age_ms']), (nc_link.COUNT_MAX, nc_link.COUNT_MAX))


class TestServos(unittest.TestCase):

    def withServos(self, inputs, pitch = 0, grip = 0):
        inputs[HAT0Y_SLOT] = pitch
        inputs[SOUTH_SLOT] = grip
        return inputs

    def test_value_table(self):
        # D-pad: pressed from 0.5 either way
        self.assertEqual([mapping.servo_value(d, -1, 0, 1) for d in [-1.0, -0.5, -0.49, 0.0, 0.49, 0.5, 1.0]],
                         [-1, -1, 0, 0, 0, 1, 1])
        # Button: any negative demand is released
        self.assertEqual([mapping.servo_value(d, 0, 0, 1) for d in [-1.0, 0.0, 0.4, 0.5, 1.0]], [0, 0, 0, 1, 1])
        # Stick: each half scales on its own
        self.assertEqual([mapping.servo_value(d, -32767, 0, 32767) for d in [-1.0, -0.5, 0.0, 0.25, 1.0]],
                         [-32767, -16384, 0, 8192, 32767])
        # An off-center neutral
        self.assertEqual([mapping.servo_value(d, 0, 100, 1100) for d in [-1.0, -0.5, 0.0, 0.5, 1.0]],
                         [0, 50, 100, 600, 1100])

    def test_config_defaults_by_input(self):
        # A servo takes its input's range and dead zone. The dead zones are
        # version 1's plus one, under the one >= rule (VEHICLE_FILE.md "Inputs")
        link, clock, logs = makeLink(servos = dict(claw_pitch = dict(input = 'ABS_HAT0Y'),
                                                   claw_roll = dict(input = 'ABS_RX'),
                                                   grip = dict(input = 'BTN_SOUTH', max = 2)))
        self.assertEqual([(s['name'], s['slot'], s['min'], s['neutral'], s['max'], s['deadzone'])
                          for s in link.vehicle['control']['servos']],
                         [('claw_pitch', HAT0Y_SLOT, -1, 0, 1, 1), ('claw_roll', RX_SLOT, -32767, 0, 32767, 3000),
                          ('grip', SOUTH_SLOT, 0, 0, 2, 1)])
        # No servos at all
        self.assertEqual(makeLink(servos = dict())[0].vehicle['control']['servos'], [])
        self.assertEqual(makeLink(servos = None)[0].vehicle['control']['servos'], [])
        [settings, msg] = validate(makeRaw(servos = CLAW_SERVOS))
        self.assertIn('servos claw_pitch on ABS_HAT0Y, claw_grip on BTN_SOUTH', msg)

    def test_allowed_inputs_match_version_1(self):
        self.assertEqual(sorted(TEAM['control']['servo_inputs']),
                         ['ABS_HAT0X', 'ABS_HAT0Y', 'ABS_RX', 'BTN_EAST', 'BTN_NORTH', 'BTN_SELECT', 'BTN_SOUTH',
                          'BTN_START', 'BTN_THUMBL', 'BTN_THUMBR', 'BTN_WEST'])

    def test_bad_servo_config_turns_the_link_off(self):
        good = dict(claw = dict(input = 'ABS_HAT0Y'))
        self.assertIsNotNone(validate(makeRaw(servos = good))[0])
        for servos in [dict(Claw = dict(input = 'ABS_HAT0Y')), {'1claw': dict(input = 'ABS_HAT0Y')},
                       {'c' * 17: dict(input = 'ABS_HAT0Y')}, {True: dict(input = 'ABS_HAT0Y')},
                       dict(claw = dict(input = None)), dict(claw = dict(input = 'ABS_HAT1X')),
                       # NEPI's own movement inputs, slow mode and the NEPI control button
                       dict(claw = dict(input = 'ABS_Y')), dict(claw = dict(input = 'ABS_RY')),
                       dict(claw = dict(input = 'ABS_Z')), dict(claw = dict(input = 'BTN_TL')),
                       dict(claw = dict(input = 'BTN_TR')),
                       dict(claw = dict(input = 'ABS_HAT0Y', min = 1)),
                       dict(claw = dict(input = 'ABS_HAT0Y', neutral = 2)),
                       dict(claw = dict(input = 'ABS_HAT0Y', max = -1)),
                       dict(claw = dict(input = 'ABS_HAT0Y', min = 0, neutral = 0, max = 0)),
                       dict(claw = dict(input = 'ABS_HAT0Y', deadzone = -1)),
                       dict(claw = dict(input = 'ABS_HAT0Y', deadzone = 0)),
                       dict(claw = dict(input = 'ABS_HAT0Y', max = 1.5)),
                       dict(claw = dict(input = 'ABS_HAT0Y', min = False)),
                       dict(claw = dict(input = 'ABS_HAT0Y', speed = 1)),
                       dict(claw = 'ABS_HAT0Y'),
                       dict(claw = dict(input = 'ABS_HAT0Y'), other = dict(input = 'ABS_HAT0Y')),
                       dict((name, dict(input = event)) for (name, event) in
                            zip(['a', 'b', 'c', 'd', 'e'], ['ABS_RX', 'ABS_HAT0X', 'ABS_HAT0Y', 'BTN_WEST', 'BTN_EAST']))]:
            [settings, msg] = validate(makeRaw(servos = servos))
            self.assertIsNone(settings, servos)
            self.assertIn('link off', msg)
            self.assertIn('control.servos', msg)

    def test_servos_from_yaml_text(self):
        # Replaces version 1's flat-file servo keys: a servo written in YAML
        raw = makeRaw()
        raw['control']['servos'] = yaml.safe_load('claw_pitch: {input: ABS_HAT0Y, deadzone: 1}   # D-pad\n')
        [settings, msg] = validate(raw)
        self.assertIsNotNone(settings, msg)
        self.assertEqual(settings['control']['servos'][0]['name'], 'claw_pitch')

    def test_pilot_mode_ignores_nepi_servo_values(self):
        link, clock, logs = makeLink(servos = CLAW_SERVOS)
        link.handle_demand(demandBytes(0, True, servos = dict(claw_pitch = 1.0, claw_grip = 1.0)), NEPI)
        inputs = self.withServos(pilot(), pitch = -1)
        self.assertEqual(link.merge_inputs(inputs), tuple(inputs))
        self.assertEqual(link.merge_inputs(pilot())[HAT0Y_SLOT], 0)
        self.assertFalse(link.build_status()['control'])

    def test_nepi_control_applies_servo_values(self):
        link, clock, logs = makeLink(servos = CLAW_SERVOS)
        seq = engage(link, clock)
        link.handle_demand(demandBytes(seq, True, surge = 0.1, servos = dict(claw_pitch = -1.0, claw_grip = 1.0)), NEPI)
        merged = link.merge_inputs(pilot())
        self.assertEqual((merged[HAT0Y_SLOT], merged[SOUTH_SLOT]), (-1, 1))
        self.assertEqual(merged[LY_SLOT], stick(0.1))
        # Names this robot has no servo for are ignored; a servo with no value is neutral
        link.handle_demand(demandBytes(seq + 1, True, servos = dict(claw_pitch = 1.0, wrist = 1.0)), NEPI)
        merged = link.merge_inputs(pilot())
        self.assertEqual((merged[HAT0Y_SLOT], merged[SOUTH_SLOT]), (1, 0))

    def test_pilot_wins_that_input_only(self):
        link, clock, logs = makeLink(servos = CLAW_SERVOS)
        seq = engage(link, clock)
        link.handle_demand(demandBytes(seq, True, surge = 0.1, servos = dict(claw_pitch = 1.0, claw_grip = 1.0)), NEPI)
        merged = link.merge_inputs(self.withServos(pilot(), pitch = -1))
        # The pilot's D-pad wins; the button and the movement stay NEPI's
        self.assertEqual((merged[HAT0Y_SLOT], merged[SOUTH_SLOT]), (-1, 1))
        self.assertEqual(merged[LY_SLOT], stick(0.1))
        status = link.build_status()
        self.assertEqual((status['control'], status['servos']), (True, dict(claw_pitch = True, claw_grip = False)))
        # Released, NEPI's value is back
        merged = link.merge_inputs(pilot())
        self.assertEqual(merged[HAT0Y_SLOT], 1)
        self.assertEqual(link.build_status()['servos'], dict(claw_pitch = False, claw_grip = False))

    def test_stick_servo_pilot_dead_zone(self):
        link, clock, logs = makeLink(servos = dict(claw_roll = dict(input = 'ABS_RX')))
        seq = engage(link, clock)
        link.handle_demand(demandBytes(seq, True, servos = dict(claw_roll = 0.5)), NEPI)
        # Resting stick noise inside the Teensy's dead zone is not the pilot
        self.assertEqual(link.merge_inputs(pilot(rx = 2999))[RX_SLOT], 16384)
        self.assertEqual(link.merge_inputs(pilot(rx = -3000))[RX_SLOT], -3000)
        # Unlike a movement stick, a servo stick never ends NEPI control
        self.assertEqual(link.merge_inputs(pilot(rx = 32767))[RX_SLOT], 32767)
        self.assertTrue(link.build_status()['control'])

    def test_one_dead_zone_rule_matches_version_1(self):
        # Version 1: a servo input was the pilot's while |v - neutral| > d,
        # with d 2999 for RX and 0 for the D-pad and buttons. Now every input
        # uses |v - neutral| >= deadzone, with 3000 and 1. For whole numbers
        # the two pick the same values.
        for [old_deadzone, new_deadzone, values] in [[2999, 3000, range(-33000, 33000)], [0, 1, range(-3, 4)]]:
            for value in values:
                self.assertEqual(abs(value) > old_deadzone, abs(value) >= new_deadzone, value)
        # And through the merge, at the edges, for each kind of servo input
        link, clock, logs = makeLink(servos = dict(roll = dict(input = 'ABS_RX'), pitch = dict(input = 'ABS_HAT0Y'),
                                                   grip = dict(input = 'BTN_SOUTH')))
        seq = engage(link, clock)
        link.handle_demand(demandBytes(seq, True, servos = dict(roll = 1.0, pitch = 1.0, grip = 1.0)), NEPI)
        for [rx, hat, south, held] in [[2999, 0, 0, [False, False, False]], [3000, -1, 1, [True, True, True]],
                                       [-2999, 1, 0, [False, True, False]], [-3000, 0, 1, [True, False, True]]]:
            inputs = pilot(rx = rx)
            inputs[HAT0Y_SLOT] = hat
            inputs[SOUTH_SLOT] = south
            link.merge_inputs(inputs)
            servos = link.build_status()['servos']
            self.assertEqual([servos['roll'], servos['pitch'], servos['grip']], held, (rx, hat, south))

    def test_stale_demand_goes_neutral(self):
        link, clock, logs = makeLink(servos = CLAW_SERVOS)
        seq = engage(link, clock)
        link.handle_demand(demandBytes(seq, True, servos = dict(claw_pitch = 1.0, claw_grip = 1.0)), NEPI)
        clock.advance(0.3)
        merged = link.merge_inputs(pilot())
        self.assertEqual((merged[HAT0Y_SLOT], merged[SOUTH_SLOT]), (0, 0))

    def test_demand_servos_validation(self):
        for bad in ['claw', [1.0], dict(claw_pitch = 1.5), dict(claw_pitch = True), dict(claw_pitch = '1'),
                    dict(claw_pitch = None)]:
            [demand, msg] = nc_link.parse_demand(demandBytes(0, True, servos = bad), NEPI, NEPI, AXES)
            self.assertIsNone(demand, bad)
        for bad in [float('nan'), float('inf')]:
            datagram = json.dumps(dict(v = 2, type = 'demand', seq = 0, t = 1.0, active = True,
                                       axes = dict(surge = 0.0, sway = 0.0, yaw = 0.0),
                                       servos = dict(claw_pitch = bad))).encode('utf-8')
            self.assertIsNone(nc_link.parse_demand(datagram, NEPI, NEPI, AXES)[0])
        # No servos field: no servo values
        self.assertEqual(nc_link.parse_demand(demandBytes(0, True), NEPI, NEPI, AXES)[0]['servos'], dict())
        demand = nc_link.parse_demand(demandBytes(0, True, servos = dict(claw_pitch = -1, grip = 0.25)),
                                      NEPI, NEPI, AXES)[0]
        self.assertEqual(demand['servos'], dict(claw_pitch = -1.0, grip = 0.25))

    def test_status_lists_servos_even_before_a_merge(self):
        link, clock, logs = makeLink(servos = CLAW_SERVOS)
        self.assertEqual(link.build_status()['servos'], dict(claw_pitch = False, claw_grip = False))
        self.assertEqual(makeLink()[0].build_status()['servos'], dict())


class TestLinkOff(unittest.TestCase):

    def test_empty_ip_passes_through_with_no_threads(self):
        logs = []
        link = control.NepiControl(log = logs.append)
        [on, msg] = link.start(makeRaw(nepi_ip = ''))
        self.assertFalse(on)
        self.assertEqual(link.threads, [])
        self.assertIsNone(link.stop_event)
        inputs = pilot(ly = 100, rb = 1)
        self.assertEqual(link.merge_inputs(inputs), tuple(inputs))
        self.assertEqual(topsideTag(link), '')
        link.on_telemetry_line(BENCH_LINE)
        self.assertIsNone(link.build_status())
        self.assertIsNone(link.build_vehicle_message())
        self.assertFalse(link.handle_demand(demandBytes(0, False), NEPI)[0])
        self.assertEqual(len(logs), 1)
        self.assertIn('link off', logs[0])
        self.assertIn('network.nepi_ip', logs[0])

    def test_module_functions_before_start(self):
        inputs = pilot(ly = 123)
        self.assertEqual(nepi_link.merge_inputs(inputs), tuple(inputs))
        self.assertEqual(nepi_link.topside_tag(), '')
        nepi_link.on_teensy_line(BENCH_LINE)
        nepi_link.stop()

    def test_missing_or_bad_values_turn_the_link_off(self):
        # The team's own config.yaml, which has no vehicle file sections
        team_config = yaml.safe_load('TOPSIDE_LAN_IP: 192.168.68.12 \nONBOARD_LAN_IP: 192.168.68.22\n'
                                     'VIDEO0_UDP_PORT: 5600\nNMEA_TCP_PORT: 10110\n')
        self.assertIsNone(validate(team_config)[0])
        self.assertIsNone(validate(None)[0])
        cases = [[dict(nepi_ip = 'nepi.local'), 'network.nepi_ip'],
                 [dict(caps = dict(surge = 1.5)), 'control.axes.surge.cap'],
                 [dict(signs = dict(yaw = 0)), 'control.axes.yaw.sign'],
                 [dict(caps = dict(heave = -0.1)), 'control.axes.heave.cap'],
                 [dict(signs = dict(heave = 2)), 'control.axes.heave.sign'],
                 [dict(trigger = (300, 255)), 'ABS_Z'],
                 [dict(caps = dict(yaw = True)), 'control.axes.yaw.cap'],
                 [dict(takeover = 0), 'control.takeover_fraction'],
                 [dict(takeover = 1.5), 'control.takeover_fraction'],
                 [dict(takeover = True), 'control.takeover_fraction'],
                 # 0.09 of 32767 is 2949, inside the Teensy's 3000 stick dead zone
                 [dict(takeover = 0.09), 'control.takeover_fraction']]
        for [changes, key] in cases:
            [settings, msg] = validate(makeRaw(**changes))
            self.assertIsNone(settings, changes)
            self.assertIn(key, msg, changes)
            self.assertTrue(msg.endswith(': link off'), msg)
        for [toggle, key] in [['BTN_TL', 'reserved'], [['BTN_TR'], 'control.toggle_input'],
                              ['ABS_Y', 'axis input'], ['BTN_NONE', 'control.toggle_input']]:
            raw = makeRaw()
            raw['control']['toggle_input'] = toggle
            [settings, msg] = validate(raw)
            self.assertIsNone(settings, toggle)
            self.assertIn(key, msg, toggle)
        raw = makeRaw()
        raw['network']['demand_port'] = 70000
        self.assertIn('network.demand_port', validate(raw)[1])


class TestVehicleFile(unittest.TestCase):

    def test_lookup_order(self):
        with tempfile.TemporaryDirectory() as root:
            module_dir = os.path.join(root, 'rpi', 'modes')
            home = os.path.join(root, 'home')
            os.makedirs(module_dir)
            os.makedirs(os.path.join(home, 'nepi_connect', 'config'))
            home_file = os.path.join(home, 'nepi_connect', 'config', 'nepi_connect_vehicle.yaml')
            beside = os.path.join(module_dir, 'nepi_connect_vehicle.yaml')
            env_file = os.path.join(root, 'env.yaml')
            env = {'NEPI_CONNECT_VEHICLE': env_file}
            self.assertIsNone(vehicle.find_vehicle_file(module_dir, {}, home))
            open(home_file, 'w').close()
            self.assertEqual(vehicle.find_vehicle_file(module_dir, {}, home), home_file)
            open(beside, 'w').close()
            self.assertEqual(vehicle.find_vehicle_file(module_dir, {}, home), beside)
            self.assertEqual(vehicle.find_vehicle_file(module_dir, env, home), beside)
            open(env_file, 'w').close()
            self.assertEqual(vehicle.find_vehicle_file(module_dir, env, home), env_file)

    def test_default_lookup_folder_is_beside_the_package(self):
        candidates = vehicle.vehicle_candidates(environ = {}, home = '/nowhere')
        self.assertEqual(candidates[0], os.path.join(HERE, 'nepi_connect_vehicle.yaml'))

    def test_old_config_without_vehicle_file_says_convert(self):
        with tempfile.TemporaryDirectory() as root:
            open(os.path.join(root, 'nepi_link.yaml'), 'w').close()
            [raw, path, msg] = vehicle.load_vehicle(root, {}, root)
            self.assertIsNone(raw)
            self.assertIn('nepi_link.yaml is a version 1 config', msg)
            self.assertIn('VEHICLE_FILE.md', msg)
            self.assertTrue(msg.endswith('link off'))
            logs = []
            link = control.NepiControl(log = logs.append, name = 'nepi_link')
            [on, msg] = link.start(module_dir = root)
            self.assertFalse(on)
            self.assertEqual(len(logs), 1)
            self.assertIn('convert it to a vehicle file', logs[0])
            self.assertEqual(link.threads, [])
            # NEPI_LINK_CONFIG names one too
            os.remove(os.path.join(root, 'nepi_link.yaml'))
            legacy = os.path.join(root, 'old.yaml')
            open(legacy, 'w').close()
            self.assertIn('old.yaml is a version 1 config', vehicle.load_vehicle(root, {'NEPI_LINK_CONFIG': legacy},
                                                                                 root)[2])
            os.remove(legacy)
            self.assertEqual(vehicle.load_vehicle(root, {}, root)[2], 'No vehicle file found: link off')

    def test_missing_pyyaml_leaves_the_link_off(self):
        with tempfile.TemporaryDirectory() as root:
            path = os.path.join(root, 'nepi_connect_vehicle.yaml')
            with open(TEAM_FILE) as source, open(path, 'w') as target:
                target.write(source.read())
            saved = vehicle.importYaml
            vehicle.importYaml = lambda: None
            try:
                logs = []
                link = control.NepiControl(log = logs.append, name = 'nepi_link')
                [on, msg] = link.start(module_dir = root)
            finally:
                vehicle.importYaml = saved
            self.assertFalse(on)
            self.assertEqual(msg, 'PyYAML missing: sudo apt install python3-yaml')
            self.assertEqual(logs, ['nepi_link: PyYAML missing: sudo apt install python3-yaml'])
            self.assertEqual(link.threads, [])
            inputs = pilot(ly = 9000, rb = 1)
            self.assertEqual(link.merge_inputs(inputs), tuple(inputs))

    def test_unreadable_or_bad_yaml_leaves_the_link_off(self):
        with tempfile.TemporaryDirectory() as root:
            path = os.path.join(root, 'bad.yaml')
            for text in ['vehicle: [unclosed\n', '- a list\n', 'just text\n']:
                with open(path, 'w') as bad_file:
                    bad_file.write(text)
                [raw, msg] = vehicle.load_vehicle_file(path)
                self.assertIsNone(raw, text)
                self.assertIn('link off', msg)
            self.assertIsNone(vehicle.load_vehicle_file(os.path.join(root, 'missing.yaml'))[0])

    def test_team_file(self):
        self.assertEqual(TEAM['name'], 'mate_coral_crusaders')
        self.assertEqual(TEAM['network'], dict(nepi_ip = '192.168.68.103', status_port = 5700, demand_port = 5701))
        control_section = TEAM['control']
        self.assertEqual((control_section['adapter'], control_section['toggle']['slot'], control_section['reserved']),
                         ('hooks', 13, ['BTN_TL']))
        self.assertEqual([(axis['name'], axis['kind'], [entry['name'] for entry in axis['inputs']], axis['sign'],
                           axis['cap']) for axis in control_section['axes']],
                         [('surge', 'stick', ['ABS_Y'], -1, 0.2), ('sway', 'stick', ['ABS_X'], 1, 0.2),
                          ('heave', 'stick', ['ABS_RY'], 1, 0.2), ('yaw', 'trigger_pair', ['ABS_Z', 'ABS_RZ'], 1, 0.1)])
        inputs = control_section['inputs']
        self.assertEqual((inputs['ABS_Z']['deadzone'], inputs['ABS_Z']['max']), (40, 1023))
        self.assertEqual((inputs['ABS_Y']['takeover_high'], inputs['ABS_Y']['takeover_low']), (8192, 8192))
        self.assertEqual((inputs['ABS_Z']['takeover_high'], inputs['ABS_Z']['takeover_low']), (256, None))
        self.assertEqual(control_section['servos'], [])
        self.assertEqual(sorted(SLOTS.values()), list(range(18)))
        self.assertEqual(TEAM['cameras'], [])
        self.assertEqual(TEAM['imu_mount'], 'X_FORWARD_Y_LEFT')

    def test_bench_file(self):
        with open(BENCH_FILE) as bench_file:
            [bench, msg] = validate(yaml.safe_load(bench_file))
        self.assertIsNotNone(bench, msg)
        self.assertEqual(bench['network']['nepi_ip'], '192.168.68.23')
        # Everything but the name, the address and the camera matches the team file
        self.assertEqual(bench['control'], TEAM['control'])
        self.assertEqual(bench['telemetry']['fields'], TEAM['telemetry']['fields'])
        self.assertEqual(bench['telemetry']['nav'], TEAM['telemetry']['nav'])
        self.assertEqual(bench['control']['axes'][0]['sign'], -1) # Measured on the bench 2026-10-22: LY up reads negative
        self.assertEqual(bench['cameras'], [dict(name = 'main', transport = 'http', port = 8080, device = '',
                                                 resolution = '1280x720', fps = 30)])

    def test_template_is_valid_and_off(self):
        with open(TEMPLATE_FILE) as template_file:
            [template, msg] = validate(yaml.safe_load(template_file))
        self.assertIsNotNone(template, msg)
        self.assertEqual(msg, 'network.nepi_ip is empty: link off')
        self.assertEqual(len(template['cameras']), 1)
        with open(TEMPLATE_FILE) as template_file:
            self.assertFalse(control.NepiControl(log = lambda msg: None).configure(yaml.safe_load(template_file))[0])

    def test_worst_case_sizes_of_the_team_file(self):
        # LINK_PROTOCOL.md "Size" and VEHICLE_FILE.md quote these
        self.assertEqual(TEAM['sizes'], [506, 1035])
        self.assertEqual(vehicle.worst_case_sizes(TEAM), [506, 1035])

    def test_worst_case_float_is_the_longest_a_field_can_parse_to(self):
        self.assertEqual(len(json.dumps(telemetry.WORST_FLOAT)), 19)
        for text in ['-9999999999.999999', '-1234567890.123456', '-8754283433.514171', '-0.000001', '-1']:
            self.assertLessEqual(len(json.dumps(float(text))), 19, text)
        self.assertEqual(len(json.dumps(telemetry.WORST_INT)), 11)

    def test_oversize_file_is_invalid(self):
        raw = makeRaw(team = True)
        fields = raw['telemetry']['fields']
        for number in range(30):
            fields.append(dict(name = 'extra_field_number_' + str(number).zfill(4), type = 'float'))
        raw['telemetry']['raw'] = [field['name'] for field in fields][-30:]
        [settings, msg] = validate(raw)
        self.assertIsNone(settings)
        self.assertIn('each must fit in 1200', msg)

    def test_each_class_of_bad_value_names_its_key(self):
        def changed(edit):
            raw = makeRaw(team = True)
            edit(raw)
            return validate(raw)
        cases = [
            [lambda raw: raw.pop('vehicle'), 'vehicle is missing'],
            [lambda raw: raw.update(extra = 1), 'extra is not a known key'],
            [lambda raw: raw['vehicle'].update(name = 'Mate'), 'vehicle.name'],
            [lambda raw: raw['vehicle'].update(name = 'm' * 25), 'vehicle.name'],
            [lambda raw: raw['vehicle'].update(kind = 'k' * 33), 'vehicle.kind'],
            [lambda raw: raw['network'].update(status_port = 0), 'network.status_port'],
            [lambda raw: raw['network'].update(nepi_port = 5700), 'network.nepi_port is not a known key'],
            [lambda raw: raw['control'].update(adapter = 'standalone'), 'control.adapter'],
            [lambda raw: raw['control'].pop('inputs'), 'control.inputs is missing'],
            [lambda raw: raw['control']['inputs']['ABS_X'].update(slot = 1), 'ABS_Y.slot 1 is already used by ABS_X'],
            [lambda raw: raw['control']['inputs']['ABS_X'].update(slot = 256), 'control.inputs.ABS_X.slot'],
            [lambda raw: raw['control']['inputs']['ABS_X'].update(min = 40000), 'control.inputs.ABS_X needs'],
            [lambda raw: raw['control']['inputs']['ABS_X'].update(deadzone = 0), 'control.inputs.ABS_X.deadzone'],
            [lambda raw: raw['control']['inputs']['ABS_X'].update(deadzon = 3000), 'ABS_X.deadzon is not a known key'],
            [lambda raw: raw['control']['inputs']['ABS_X'].update(label = 'x' * 25), 'control.inputs.ABS_X.label'],
            [lambda raw: raw['control']['inputs']['ABS_X'].pop('max'), 'control.inputs.ABS_X.max'],
            [lambda raw: raw['control']['axes'].update(pitch = dict(kind = 'stick', input = 'ABS_RX', sign = 1,
                                                                    cap = 0.1)), 'control.axes.pitch is not an axis'],
            [lambda raw: raw['control']['axes']['sway'].update(kind = 'wheel'), 'control.axes.sway.kind'],
            [lambda raw: raw['control']['axes']['sway'].update(input = 'ABS_Z'), 'needs min < neutral < max'],
            [lambda raw: raw['control']['axes']['sway'].update(input = 'ABS_Y'), 'already used by axis surge'],
            [lambda raw: raw['control']['axes']['sway'].update(input = 'BTN_TL'), 'control.axes.sway'],
            [lambda raw: raw['control']['axes']['yaw'].update(inputs = ['ABS_Z', 'ABS_Z']), 'control.axes.yaw.inputs'],
            [lambda raw: raw['control']['axes']['yaw'].update(inputs = ['ABS_Z']), 'control.axes.yaw.inputs'],
            [lambda raw: raw['control']['axes']['yaw'].update(input = 'ABS_Z'), 'control.axes.yaw.input is not'],
            [lambda raw: raw['control']['inputs']['ABS_Z'].update(deadzone = 1023), 'more travel than its dead zone'],
            [lambda raw: [raw['control'].update(axes = dict(), servos = dict()), raw.pop('telemetry'),
                          raw.pop('cameras', None)], 'describes nothing to connect'],
            [lambda raw: raw['control'].update(reserved_inputs = ['BTN_NONE']), 'control.reserved_inputs'],
            [lambda raw: raw['telemetry'].update(format = 'binary'), 'telemetry.format'],
            [lambda raw: raw['telemetry'].update(prefix = '$A,B'), 'telemetry.prefix'],
            [lambda raw: raw['telemetry'].update(trailer = '[unclosed'), 'telemetry.trailer'],
            [lambda raw: raw['telemetry']['fields'].append(dict(name = 'depth_m', type = 'float')), 'already used'],
            [lambda raw: raw['telemetry']['fields'][0].update(type = 'double'), 'telemetry.fields[0].type'],
            [lambda raw: raw['telemetry']['nav']['depth'].pop('depth_m_s'), 'telemetry.nav.depth'],
            [lambda raw: raw['telemetry']['nav']['depth'].update(depth_m = 'nowhere'), 'telemetry.nav.depth.depth_m'],
            [lambda raw: raw['telemetry']['nav'].update(position = dict()), 'telemetry.nav.position is not a group'],
            [lambda raw: raw['telemetry'].update(raw = ['nowhere']), 'telemetry.raw'],
            [lambda raw: raw['telemetry'].update(imu_mount = 'X_UP_Y_UP'), 'telemetry.imu_mount'],
            [lambda raw: raw.update(cameras = [dict(name = 'a', transport = 'rtsp', port = 1)]), 'cameras[0].transport'],
            [lambda raw: raw.update(cameras = [dict(name = 'a', transport = 'http', port = 1),
                                               dict(name = 'b', transport = 'http', port = 1)]), 'cameras[1].port'],
            [lambda raw: raw.update(cameras = [dict(name = 'a', transport = 'fastmjpg', port = 5600, fps = 30)]),
             'cameras[0].fps applies to http cameras only'],
            [lambda raw: raw.update(cameras = [dict(name = 'a', transport = 'http', port = 1, resolution = 'hd')]),
             'cameras[0].resolution'],
            [lambda raw: raw.update(cameras = [dict(name = 'c' + str(n), transport = 'fastmjpg', port = 5600 + n)
                                               for n in range(5)]), 'at most 4 cameras']]
        for [edit, key] in cases:
            [settings, msg] = changed(edit)
            self.assertIsNone(settings, key)
            self.assertIn(key, msg)
            self.assertTrue(msg.endswith(': link off'), msg)
            # The link stays off with that message, and passes inputs through
            logs = []
            link = control.NepiControl(log = logs.append)
            raw = makeRaw(team = True)
            edit(raw)
            self.assertFalse(link.start(raw)[0])
            self.assertEqual(len(logs), 1, key)
            self.assertIn(key, logs[0])
            self.assertEqual(link.threads, [])


class TestPartialVehicle(unittest.TestCase):
    # A robot need not have every part: motors without servos or telemetry,
    # servos without motors, or only sensors all work, each without the parts
    # it does not have

    def configure(self, raw):
        self.clock = FakeClock()
        self.logs = []
        self.link = control.NepiControl(clock = self.clock, wall_clock = lambda: 1792200000.25,
                                        log = self.logs.append, name = 'nepi_link')
        [on, msg] = self.link.configure(raw)
        self.assertTrue(on, msg)

    def test_motors_only(self):
        raw = makeRaw()
        raw.pop('telemetry')
        self.configure(raw)
        message = self.link.build_vehicle_message()
        self.assertEqual((message['servos'], message['nav'], message['raw'], message['cameras']), ([], [], [], []))
        engage(self.link, self.clock, surge = 0.2)
        self.assertEqual(self.link.merge_inputs(pilot())[LY_SLOT], 8713)
        # A telemetry line from a robot with no telemetry section is ignored, not an error
        self.link.on_telemetry_line(BENCH_LINE)
        status = self.link.build_status()
        self.assertIsNone(status['nav'])
        self.assertEqual(status['servos'], dict())
        self.assertNotIn('raw', status)

    def test_servos_only(self):
        raw = makeRaw(team = True, servos = CLAW_SERVOS)
        raw['control'].pop('axes')
        self.configure(raw)
        message = self.link.build_vehicle_message()
        self.assertEqual((message['axes'], sorted(message['servos'])), (dict(), ['claw_grip', 'claw_pitch']))
        # NEPI control still toggles with RB, from a demand that has no axes field
        demand = dict(v = 2, type = 'demand', seq = 0, t = 1.0, active = True, servos = dict(claw_pitch = 1.0))
        self.assertTrue(self.link.handle_demand(json.dumps(demand).encode('utf-8'), NEPI)[0])
        self.link.merge_inputs(pilot(rb = 1))
        self.link.merge_inputs(pilot())
        self.assertTrue(self.link.build_status()['control'])
        # The servo moves; the sticks are not NEPI's, so the pilot's pass through
        # and a full stick does not end NEPI control
        out = self.link.merge_inputs(pilot(ly = 32767, lt = 1023))
        self.assertEqual((out[HAT0Y_SLOT], out[LY_SLOT], out[LT_SLOT]), (1, 32767, 1023))
        status = self.link.build_status()
        self.assertTrue(status['control'])
        self.assertEqual((status['pilot'], status['out']), (dict(), dict()))

    def test_sensors_only(self):
        raw = makeRaw(team = True)
        raw['control'].pop('axes')
        raw['control'].pop('servos', None)
        self.configure(raw)
        self.assertEqual(self.link.build_vehicle_message()['axes'], dict())
        self.link.on_telemetry_line('$TNCTL,0,0,0,0,0,0,0,0,1.25,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,'
                                    '0.00,*FF')
        self.assertEqual(self.link.build_status()['nav']['depth_m'], 1.25)
        self.assertEqual(self.link.merge_inputs(pilot(ly = 20000)), tuple(pilot(ly = 20000)))


class TestSecondVehicle(unittest.TestCase):
    # A made-up surface boat: surge and yaw only, an 8-slot RC input map with
    # pulse widths around 1500, no telemetry and one servo. Nothing about it
    # comes from the MATE ROV, so these tests show the core holds nothing
    # MATE-specific.

    BOAT_YAML = '''
vehicle: {name: harbor_boat, kind: USV}
network: {nepi_ip: 10.0.0.5, status_port: 6700, demand_port: 6701}
control:
  toggle_input: AUX1
  takeover_fraction: 0.5
  reserved_inputs: [KILL]
  inputs:
    THROTTLE: {slot: 2, min: 1000, neutral: 1500, max: 2000, deadzone: 25}
    RUDDER:   {slot: 0, min: 1000, neutral: 1500, max: 2000, deadzone: 25}
    AUX1:     {slot: 4, min: 1000, neutral: 1000, max: 2000, deadzone: 500, label: SWITCH-A}
    WINCH:    {slot: 5, min: 1000, neutral: 1500, max: 2000, deadzone: 100}
    KILL:     {slot: 6, min: 1000, neutral: 1000, max: 2000, deadzone: 500}
    SPARE1:   {slot: 1, min: 1000, neutral: 1500, max: 2000, deadzone: 25}
    SPARE2:   {slot: 3, min: 1000, neutral: 1500, max: 2000, deadzone: 25}
    SPARE3:   {slot: 7, min: 1000, neutral: 1000, max: 2000, deadzone: 500}
  axes:
    surge: {kind: stick, input: THROTTLE, sign: 1, cap: 0.5}
    yaw:   {kind: stick, input: RUDDER, sign: -1, cap: 0.3}
  servos:
    winch: {input: WINCH}
cameras:
  - {name: mast, transport: fastmjpg, port: 5620}
'''
    BOAT_NEPI = '10.0.0.5'

    def setUp(self):
        self.raw = yaml.safe_load(self.BOAT_YAML)
        self.clock = FakeClock()
        self.logs = []
        self.link = control.NepiControl(clock = self.clock, wall_clock = lambda: 1792200000.25, log = self.logs.append)
        [on, msg] = self.link.configure(self.raw)
        self.assertTrue(on, msg)
        self.seq = 0

    def rc(self, throttle = 1500, rudder = 1500, aux1 = 1000, winch = 1500, kill = 1000):
        inputs = [1500, 1500, 1500, 1500, 1000, 1500, 1000, 1000]
        [inputs[2], inputs[0], inputs[4], inputs[5], inputs[6]] = [throttle, rudder, aux1, winch, kill]
        return inputs

    def demand(self, active = True, axes = None, servos = None):
        message = dict(v = 2, type = 'demand', seq = self.seq, t = 1.0, active = active,
                       axes = axes if axes is not None else dict(), servos = servos or dict())
        self.seq += 1
        return self.link.handle_demand(json.dumps(message).encode('utf-8'), self.BOAT_NEPI)

    def engage(self):
        self.assertTrue(self.demand()[0])
        self.link.merge_inputs(self.rc(aux1 = 2000))
        self.link.merge_inputs(self.rc())
        self.assertTrue(self.link.build_status()['control'])

    def test_vehicle_message(self):
        message = self.link.build_vehicle_message()
        self.assertEqual((message['name'], message['kind'], message['demand_port']), ('harbor_boat', 'USV', 6701))
        self.assertEqual(message['axes'], dict(surge = 0.5, yaw = 0.3))
        self.assertEqual((message['servos'], message['nav'], message['raw']), (['winch'], [], []))
        self.assertEqual(message['cameras'], [dict(name = 'mast', transport = 'fastmjpg', port = 5620)])
        self.assertEqual(message['imu_mount'], 'X_FORWARD_Y_LEFT')
        self.assertEqual(sorted(self.link.vehicle['control']['servo_inputs']), ['SPARE1', 'SPARE2', 'SPARE3', 'WINCH'])

    def test_merge_writes_only_announced_axes_around_their_neutral(self):
        self.engage()
        # sway and heave are not this boat's: ignored, whatever their values
        self.assertTrue(self.demand(axes = dict(surge = 0.2, yaw = 0.1, sway = 1.0, heave = -1.0),
                                    servos = dict(winch = 1.0))[0])
        out = self.link.merge_inputs(self.rc())
        # surge 0.2: 1500 + round(25 + 0.19 / 0.99 * 475) = 1616
        self.assertEqual(out[2], 1616)
        # yaw 0.1 with sign -1: 1500 - round(25 + 0.09 / 0.99 * 475) = 1432
        self.assertEqual(out[0], 1432)
        self.assertEqual(out[5], 2000) # winch 1.0
        self.assertEqual([out[1], out[3], out[6], out[7]], [1500, 1500, 1000, 1000])
        status = self.link.build_status()
        self.assertEqual(status['out'], dict(THROTTLE = 1616, RUDDER = 1432))
        self.assertEqual(status['pilot'], dict(surge = False, yaw = False))
        self.assertEqual(sorted(status.keys()), sorted(set(STATUS_FIELDS) - set(['raw'])))
        self.assertIsNone(status['nav'])
        # A stale demand leaves both axes at their neutral, not at 0
        self.clock.advance(0.3)
        out = self.link.merge_inputs(self.rc(throttle = 1520))
        self.assertEqual((out[2], out[0], out[5]), (1500, 1500, 1500))

    def test_takeover_and_toggle_follow_the_file(self):
        self.engage()
        # takeover_fraction 0.5 of 500 is 250 from neutral, either side
        self.link.merge_inputs(self.rc(rudder = 1749))
        self.assertTrue(self.link.build_status()['control'])
        self.link.merge_inputs(self.rc(rudder = 1250))
        status = self.link.build_status()
        self.assertEqual((status['control'], status['control_reason']), (False, 'pilot_stick'))
        self.assertIn('the pilot moved RUDDER', self.logs[-1])
        # The reserved KILL input passes through and never counts as the toggle
        self.assertTrue(self.demand()[0])
        self.assertEqual(self.link.merge_inputs(self.rc(kill = 2000))[6], 2000)
        self.assertFalse(self.link.build_status()['control'])
        self.link.merge_inputs(self.rc(aux1 = 1600))
        self.assertTrue(self.link.build_status()['control'])
        self.link.merge_inputs(self.rc())
        self.assertIsNone(self.link.build_status()['nav'])
        self.link.control = False
        self.link.toggle_btn_avl = True
        self.link.merge_inputs(self.rc(aux1 = 2000, throttle = 1750))
        self.assertIn('nepi_connect: SWITCH-A ignored: release THROTTLE first', self.logs[-1])

    def test_no_telemetry_means_no_nav(self):
        for line in ['$TNCTL,1,2', 'anything', None]:
            self.link.on_telemetry_line(line)
        status = self.link.build_status()
        self.assertEqual((status['nav'], status['nav_count'], status['nav_bad'], status['nav_age_ms']),
                         (None, 0, 0, -1))
        self.assertNotIn('raw', status)

    def test_sizes(self):
        [vehicle_bytes, status_bytes] = self.link.vehicle['sizes']
        self.assertLess(status_bytes, TEAM['sizes'][1])
        self.assertEqual(len(nc_link.encode_message(self.link.build_status())) <= status_bytes, True)

    def test_core_has_no_mate_names(self):
        forbidden = ['Teensy', 'TNCTL', 'Coral', 'gamepad', 'MATE', '32767', '3000', '1023', 'BTN_', 'ABS_',
                     ' LX', ' LY', ' RB', ' LB', ' LT', ' RT', ' RX', ' RY']
        for [folder, names, files] in os.walk(NEPI_CONNECT_DIR):
            for name in files:
                if not name.endswith('.py'):
                    continue
                with open(os.path.join(folder, name)) as source:
                    text = source.read()
                for token in forbidden:
                    self.assertNotIn(token, text, name + ' holds ' + token)


class TestAdapters(unittest.TestCase):

    def test_hooks_adapter_provides_the_adapter_interface(self):
        link, clock, logs = makeLink()
        adapter = hooks.HooksAdapter(link)
        self.assertIsInstance(adapter, Adapter)
        self.assertIsNone(adapter.pilot_inputs())
        engage(link, clock, surge = 0.2)
        inputs = pilot(lx = 10)
        self.assertEqual(adapter.apply_demand(inputs)[LY_SLOT], 8713)
        self.assertEqual(adapter.pilot_inputs(), tuple(inputs))
        adapter.take_telemetry(BENCH_LINE)
        self.assertEqual(link.build_status()['nav_count'], 1)
        self.assertEqual(adapter.status_tag(), 'ON')

    def test_base_adapter_methods_must_be_provided(self):
        adapter = Adapter(control.NepiControl(log = lambda msg: None))
        for call in [lambda: adapter.apply_demand([]), lambda: adapter.take_telemetry(''),
                     lambda: adapter.pilot_inputs()]:
            self.assertRaises(NotImplementedError, call)

    def test_nepi_link_formats_the_tag_for_the_topside_line(self):
        link, clock, logs = makeLink()
        self.assertEqual(topsideTag(link), ',NEPI:NOTREADY')
        engage(link, clock, surge = 0.2)
        self.assertEqual(topsideTag(link), ',NEPI:ON')

    def test_nepi_link_without_the_package_passes_through(self):
        saved = nepi_link._hooks
        nepi_link._hooks = None
        printed = []
        try:
            import builtins
            saved_print = builtins.print
            builtins.print = lambda *args: printed.append(' '.join(str(arg) for arg in args))
            try:
                nepi_link.start()
            finally:
                builtins.print = saved_print
            inputs = pilot(ly = 9000, rb = 1)
            self.assertEqual(nepi_link.merge_inputs(inputs), tuple(inputs))
            self.assertEqual(nepi_link.topside_tag(), '')
            nepi_link.on_teensy_line(BENCH_LINE)
            nepi_link.stop()
        finally:
            nepi_link._hooks = saved
        self.assertEqual(len(printed), 1)
        self.assertIn('nepi_connect folder must sit beside nepi_link.py', printed[0])


class TestErrorFallback(unittest.TestCase):

    def test_exception_inside_merge_returns_inputs_unchanged(self):
        link, clock, logs = makeLink()
        engage(link, clock, surge = 0.2)
        def broken(*args):
            raise RuntimeError('forced')
        link.updatePilotLocked = broken
        inputs = pilot(ly = 50, rb = 1)
        self.assertEqual(link.merge_inputs(inputs), tuple(inputs))
        self.assertEqual(len(logs), 1)
        self.assertIn('forced', logs[0])
        # Printed at most once per 10 s
        link.merge_inputs(inputs)
        clock.advance(9.9)
        link.merge_inputs(inputs)
        self.assertEqual(len(logs), 1)
        clock.advance(0.2)
        link.merge_inputs(inputs)
        self.assertEqual(len(logs), 2)
        self.assertIn('2 more not shown', logs[1])

    def test_short_input_list_returns_unchanged(self):
        link, clock, logs = makeLink()
        engage(link, clock, surge = 0.2)
        self.assertEqual(link.merge_inputs([1, 2, 3]), (1, 2, 3))

    def test_module_functions_never_raise(self):
        self.assertIsNone(nepi_link.merge_inputs(None))
        nepi_link.on_teensy_line(12345)
        self.assertIsNone(hooks.merge_inputs(None))
        hooks.on_telemetry_line(12345)

    def test_broken_clock_gives_empty_tag(self):
        link, clock, logs = makeLink()
        def broken():
            raise RuntimeError('clock')
        link.clock = broken
        self.assertEqual(topsideTag(link), '')
        inputs = pilot(ly = 7)
        self.assertEqual(link.merge_inputs(inputs), tuple(inputs))


class TestFakeNepi(unittest.TestCase):

    def test_fake_demand_is_accepted(self):
        datagram = json.dumps(fake_nepi.make_demand(0, False, 0.1, -0.1, 0.05, heave = 0.15)).encode('utf-8')
        [demand, msg] = nc_link.parse_demand(datagram, NEPI, NEPI, AXES)
        self.assertIsNotNone(demand, msg)
        axes = demand['axes']
        self.assertEqual((axes['surge'], axes['sway'], axes['heave'], axes['yaw']), (0.1, -0.1, 0.15, 0.05))

    def test_fake_demand_sends_only_announced_axes(self):
        message = fake_nepi.make_demand(0, True, 0.1, -0.1, 0.05, heave = 0.15, axis_names = ['surge', 'yaw'])
        self.assertEqual(message['axes'], dict(surge = 0.1, yaw = 0.05))


class TestLoopback(unittest.TestCase):

    def setUp(self):
        self.status_port = freeUdpPort()
        self.demand_port = freeUdpPort()
        while self.demand_port == self.status_port:
            self.demand_port = freeUdpPort()
        self.logs = []
        self.link = control.NepiControl(log = self.logs.append, name = 'nepi_link')
        self.receiver = fake_nepi.StatusReceiver('127.0.0.1', self.status_port, bind_ip = '127.0.0.1')
        self.sender = fake_nepi.DemandSender('127.0.0.1', self.demand_port)

    def tearDown(self):
        self.link.stop()
        self.receiver.stop()
        self.sender.close()

    def test_demand_reaches_the_merge_over_udp(self):
        [success, msg] = self.receiver.start()
        self.assertTrue(success, msg)
        raw = makeRaw(nepi_ip = '127.0.0.1')
        raw['network'].update(status_port = self.status_port, demand_port = self.demand_port)
        [on, msg] = self.link.start(raw, 'test')
        self.assertTrue(on, msg)
        # The vehicle message comes first, and names the demand port
        announced = self.receiver.get_vehicle(2.0)
        self.assertIsNotNone(announced)
        self.assertEqual((announced['name'], announced['demand_port']), ('mate_coral_crusaders', self.demand_port))
        # NEPI ready, until the link has a demand (its thread may still be binding)
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline and self.link.build_status()['demand_seq'] < 0:
            self.sender.send(True, 0.2, 0.0, 0.0)
            time.sleep(0.1)
        self.assertGreaterEqual(self.link.build_status()['demand_seq'], 0)
        self.assertEqual(topsideTag(self.link), ',NEPI:OFF')
        # The pilot presses RB
        merged = self.link.merge_inputs(pilot(rb = 1))
        self.link.merge_inputs(pilot())
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            self.sender.send(True, 0.2, 0.0, 0.0)
            time.sleep(0.1)
            merged = self.link.merge_inputs([0] * 18)
            if merged[LY_SLOT] == 8713:
                break
        self.assertEqual(merged[LY_SLOT], 8713)
        self.assertEqual(topsideTag(self.link), ',NEPI:ON')
        # Statuses from after the merge: drop the queue, then collect for 0.35 s
        self.receiver.get_statuses(0)
        statuses = []
        end = time.monotonic() + 0.35
        while time.monotonic() < end:
            statuses += self.receiver.get_statuses(0.05)
        self.assertTrue(len(statuses) >= 2)
        newest = statuses[-1]
        self.assertTrue(newest['control'])
        self.assertTrue(newest['nepi_ready'])
        self.assertEqual((newest['control_reason'], newest['out']['ABS_Y'], newest['demand_bad']),
                         ('pilot_button', 8713, 0))
        # Vehicle messages keep coming, about every 0.5 s
        self.assertGreaterEqual(self.receiver.get_health()['vehicle_count'], 2)
        # NEPI stops: after 250 ms every movement input is neutral and the tag reads WAIT
        time.sleep(0.5)
        self.assertEqual(self.link.merge_inputs(pilot(ly = 1500))[LY_SLOT], 0)
        self.assertEqual(topsideTag(self.link), ',NEPI:WAIT')
        # stop() closes the demand port
        self.link.stop()
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.bind(('', self.demand_port))
        finally:
            sock.close()


if __name__ == '__main__':
    unittest.main()
