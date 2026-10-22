#!/usr/bin/env python3
##
## Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
##
## This file is part of nepi-engine
## (see https://github.com/nepi-engine).
##
## License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
##

# Tests for nepi_link.py. Standard library only. Run from this folder:
#   python3 -B -m unittest -v test_nepi_link
#
# Every timing test uses an injected clock. Only TestLoopback opens sockets,
# and it sends to 127.0.0.1 alone, with free ports and real time; it drives
# the bridge with tools/fake_nepi.py's DemandSender and StatusReceiver.

import json
import math
import os
import socket
import sys
import tempfile
import time
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..', 'tools'))

import nepi_link
import fake_nepi


NEPI = '192.168.68.23'
BENCH_LINE = '$TNCTL,0,0,0,0,100,100,100,100,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,*FF'
STATUS_FIELDS = ['v', 'type', 'seq', 't', 'assist', 'pilot_surge', 'pilot_sway', 'pilot_yaw', 'pilot_depth',
                 'slow', 'out_lx', 'out_ly', 'out_lt', 'out_rt', 'demand_seq', 'demand_age_ms', 'demand_bad',
                 'tn', 'tn_age_ms', 'tn_count', 'tn_bad', 'assist_reason']


class FakeClock(object):

    def __init__(self, now = 1000.0):
        self.now = now

    def __call__(self):
        return self.now

    def advance(self, seconds):
        self.now += seconds


def makeLink(**overrides):
    clock = FakeClock()
    logs = []
    link = nepi_link.NepiLink(clock = clock, wall_clock = lambda: 1792200000.25, log = logs.append)
    config = dict(NEPI_LAN_IP = NEPI)
    config.update(overrides)
    [on, msg] = link.configure(config)
    if not on:
        raise AssertionError('link did not configure: ' + msg)
    return link, clock, logs

def demandBytes(seq, active, surge = 0.0, sway = 0.0, yaw = 0.0, **changes):
    message = dict(v = 1, type = 'demand', seq = seq, t = 1792200000.25, active = active,
                   surge = surge, sway = sway, yaw = yaw, pitch = 0.0, roll = 0.0)
    message.update(changes)
    return json.dumps(message).encode('utf-8')

def pilot(lx = 0, ly = 0, rx = 0, ry = 0, lt = 0, rt = 0, lb = 0, rb = 0):
    inputs = [0] * 18
    inputs[nepi_link.LX_SLOT] = lx
    inputs[nepi_link.LY_SLOT] = ly
    inputs[nepi_link.RX_SLOT] = rx
    inputs[nepi_link.RY_SLOT] = ry
    inputs[nepi_link.LT_SLOT] = lt
    inputs[nepi_link.RT_SLOT] = rt
    inputs[nepi_link.LB_SLOT] = lb
    inputs[nepi_link.RB_SLOT] = rb
    return inputs

def engage(link, clock, surge = 0.0, sway = 0.0, yaw = 0.0, seq = 0):
    # The D3 option (c) rising edge: active false, then active true
    link.handle_demand(demandBytes(seq, False), NEPI)
    clock.advance(0.1)
    [accepted, msg] = link.handle_demand(demandBytes(seq + 1, True, surge, sway, yaw), NEPI)
    if not accepted:
        raise AssertionError('engage demand dropped: ' + msg)
    return seq + 2

def freeUdpPort():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('127.0.0.1', 0))
    port = sock.getsockname()[1]
    sock.close()
    return port


class TestMapping(unittest.TestCase):

    def test_stick_table(self):
        self.assertEqual(nepi_link.stick_value(0.01), 3000)
        self.assertEqual(nepi_link.stick_value(0.2), 8713)
        self.assertEqual(nepi_link.stick_value(1.0), 32767)
        self.assertEqual(nepi_link.stick_value(-0.2), -8713)
        self.assertEqual(nepi_link.stick_value(0.0099), 0)
        self.assertEqual(nepi_link.stick_value(0.0), 0)

    def test_trigger_table_10_255(self):
        self.assertEqual(nepi_link.trigger_value(0.01, 10, 255), 10)
        self.assertEqual(nepi_link.trigger_value(0.1, 10, 255), 32)
        self.assertEqual(nepi_link.trigger_value(1.0, 10, 255), 255)
        self.assertEqual(nepi_link.trigger_value(0.0099, 10, 255), 0)

    def test_trigger_table_40_1023(self):
        self.assertEqual(nepi_link.trigger_value(0.01, 40, 1023), 40)
        self.assertEqual(nepi_link.trigger_value(1.0, 40, 1023), 1023)

    def test_merge_uses_table_with_caps_at_one(self):
        link, clock, logs = makeLink(NEPI_MAX_SURGE = 1.0, NEPI_MAX_SWAY = 1.0, NEPI_MAX_YAW = 1.0)
        engage(link, clock, surge = 1.0, sway = 0.2, yaw = 0.1)
        out = link.merge_inputs(pilot())
        self.assertEqual(out[nepi_link.LY_SLOT], 32767)
        self.assertEqual(out[nepi_link.LX_SLOT], 8713)
        self.assertEqual(out[nepi_link.LT_SLOT], 32)
        self.assertEqual(out[nepi_link.RT_SLOT], 0)

    def test_negative_yaw_uses_right_trigger(self):
        link, clock, logs = makeLink(NEPI_MAX_YAW = 1.0)
        engage(link, clock, yaw = -1.0)
        out = link.merge_inputs(pilot())
        self.assertEqual(out[nepi_link.LT_SLOT], 0)
        self.assertEqual(out[nepi_link.RT_SLOT], 255)

    def test_caps_clamp_before_mapping(self):
        link, clock, logs = makeLink(NEPI_TRIGGER_DEADZONE = 40, NEPI_TRIGGER_MAX = 1023)
        engage(link, clock, surge = 1.0, sway = -1.0, yaw = 1.0)
        out = link.merge_inputs(pilot())
        self.assertEqual(out[nepi_link.LY_SLOT], 8713) # NEPI_MAX_SURGE 0.2
        self.assertEqual(out[nepi_link.LX_SLOT], -8713) # NEPI_MAX_SWAY 0.2
        self.assertEqual(out[nepi_link.LT_SLOT], 129) # NEPI_MAX_YAW 0.1: 40 + 0.09 / 0.99 * 983 = 129.4

    def test_signs(self):
        link, clock, logs = makeLink(NEPI_SURGE_SIGN = -1, NEPI_SWAY_SIGN = -1, NEPI_YAW_SIGN = -1)
        engage(link, clock, surge = 0.2, sway = 0.2, yaw = 0.1)
        out = link.merge_inputs(pilot())
        self.assertEqual(out[nepi_link.LY_SLOT], -8713)
        self.assertEqual(out[nepi_link.LX_SLOT], -8713)
        self.assertEqual(out[nepi_link.LT_SLOT], 0)
        self.assertEqual(out[nepi_link.RT_SLOT], 32)

    def test_tiny_and_zero_demands_send_zero(self):
        link, clock, logs = makeLink()
        engage(link, clock, surge = 0.005, sway = 0.0, yaw = 0.0)
        out = link.merge_inputs(pilot(ly = 1500, lx = -2000, lt = 5, rt = 3))
        self.assertEqual(out[nepi_link.LY_SLOT], 0)
        self.assertEqual(out[nepi_link.LX_SLOT], 0)
        self.assertEqual(out[nepi_link.LT_SLOT], 0)
        self.assertEqual(out[nepi_link.RT_SLOT], 0)


class TestOverride(unittest.TestCase):

    def setUp(self):
        self.link, self.clock, self.logs = makeLink()
        engage(self.link, self.clock, surge = 0.2, sway = 0.2, yaw = 0.1)

    def test_pilot_ly_wins_surge_only(self):
        out = self.link.merge_inputs(pilot(ly = 5000))
        self.assertEqual(out[nepi_link.LY_SLOT], 5000)
        self.assertEqual(out[nepi_link.LX_SLOT], 8713)
        self.assertEqual(out[nepi_link.LT_SLOT], 32)
        # abs(LY) >= 3000 is outside the dead zone, on both sides
        self.assertEqual(self.link.merge_inputs(pilot(ly = 3000))[nepi_link.LY_SLOT], 3000)
        self.assertEqual(self.link.merge_inputs(pilot(ly = -3000))[nepi_link.LY_SLOT], -3000)

    def test_pilot_lx_wins_sway_only(self):
        out = self.link.merge_inputs(pilot(lx = -3000))
        self.assertEqual(out[nepi_link.LX_SLOT], -3000)
        self.assertEqual(out[nepi_link.LY_SLOT], 8713)
        self.assertEqual(out[nepi_link.LT_SLOT], 32)

    def test_pilot_trigger_wins_yaw_only(self):
        out = self.link.merge_inputs(pilot(lt = 10))
        self.assertEqual((out[nepi_link.LT_SLOT], out[nepi_link.RT_SLOT]), (10, 0))
        self.assertEqual(out[nepi_link.LY_SLOT], 8713)
        out = self.link.merge_inputs(pilot(rt = 200))
        self.assertEqual((out[nepi_link.LT_SLOT], out[nepi_link.RT_SLOT]), (0, 200))
        self.assertEqual(out[nepi_link.LX_SLOT], 8713)

    def test_inside_dead_zone_nepi_wins(self):
        out = self.link.merge_inputs(pilot(ly = -2999, lx = 2999, lt = 9, rt = 9))
        self.assertEqual(out[nepi_link.LY_SLOT], 8713)
        self.assertEqual(out[nepi_link.LX_SLOT], 8713)
        self.assertEqual((out[nepi_link.LT_SLOT], out[nepi_link.RT_SLOT]), (32, 0))

    def test_other_inputs_pass_through_and_pilot_list_untouched(self):
        inputs = list(range(100, 118))
        inputs[nepi_link.LY_SLOT] = 0
        inputs[nepi_link.LX_SLOT] = 0
        inputs[nepi_link.LT_SLOT] = 0
        inputs[nepi_link.RT_SLOT] = 0
        inputs[nepi_link.LB_SLOT] = 0
        inputs[nepi_link.RB_SLOT] = 0
        before = list(inputs)
        out = self.link.merge_inputs(inputs)
        self.assertEqual(inputs, before)
        self.assertIsInstance(out, tuple)
        self.assertEqual(len(out), 18)
        changed = [i for i in range(18) if out[i] != inputs[i]]
        self.assertEqual(changed, [nepi_link.LX_SLOT, nepi_link.LY_SLOT, nepi_link.LT_SLOT])
        self.assertEqual(out[nepi_link.RY_SLOT], inputs[nepi_link.RY_SLOT])
        self.assertEqual(out[nepi_link.RX_SLOT], inputs[nepi_link.RX_SLOT])

    def test_status_reports_pilot_flags_and_outputs(self):
        self.link.merge_inputs(pilot(ly = 5000, ry = -3000, rt = 10))
        status = self.link.build_status()
        self.assertTrue(status['pilot_surge'])
        self.assertFalse(status['pilot_sway'])
        self.assertTrue(status['pilot_yaw'])
        self.assertTrue(status['pilot_depth'])
        self.assertEqual((status['out_ly'], status['out_lx'], status['out_lt'], status['out_rt']), (5000, 8713, 0, 10))


class TestTimeouts(unittest.TestCase):

    def setUp(self):
        self.link, self.clock, self.logs = makeLink()
        self.seq = engage(self.link, self.clock, surge = 0.2)

    def test_demand_older_than_250_ms_passes_pilot_through_with_wait(self):
        self.clock.advance(0.24)
        self.assertEqual(self.link.topside_tag(), ',NEPI:ON')
        self.assertEqual(self.link.merge_inputs(pilot(ly = 1500))[nepi_link.LY_SLOT], 8713)
        self.clock.advance(0.02)
        inputs = pilot(ly = 1500, lx = -100)
        self.assertEqual(self.link.merge_inputs(inputs), tuple(inputs))
        self.assertEqual(self.link.topside_tag(), ',NEPI:WAIT')
        status = self.link.build_status()
        self.assertTrue(status['assist'])
        self.assertEqual(status['out_ly'], 1500)
        self.assertEqual(status['demand_age_ms'], 260)
        # A fresh demand restores NEPI's axes
        self.link.handle_demand(demandBytes(self.seq, True, 0.2), NEPI)
        self.assertEqual(self.link.topside_tag(), ',NEPI:ON')
        self.assertEqual(self.link.merge_inputs(pilot())[nepi_link.LY_SLOT], 8713)

    def test_two_seconds_without_a_demand_turns_assist_off(self):
        self.clock.advance(1.98)
        self.assertEqual(self.link.topside_tag(), ',NEPI:WAIT')
        self.assertTrue(self.link.build_status()['assist'])
        self.clock.advance(0.04)
        self.assertEqual(self.link.topside_tag(), ',NEPI:OFF')
        status = self.link.build_status()
        self.assertFalse(status['assist'])
        self.assertEqual(status['assist_reason'], 'timeout')

    def test_timeout_seen_by_status_without_merge_calls(self):
        self.clock.advance(2.5)
        status = self.link.build_status()
        self.assertFalse(status['assist'])
        self.assertEqual(status['assist_reason'], 'timeout')


class TestToggle(unittest.TestCase):

    def setUp(self):
        self.link, self.clock, self.logs = makeLink()

    def state(self):
        status = self.link.build_status()
        return status['assist'], status['assist_reason']

    def test_assist_starts_off(self):
        self.assertEqual(self.state(), (False, 'start'))
        self.assertEqual(self.link.topside_tag(), ',NEPI:OFF')
        inputs = pilot(ly = 100)
        self.assertEqual(self.link.merge_inputs(inputs), tuple(inputs))

    def test_first_demand_active_true_does_not_turn_assist_on(self):
        [accepted, msg] = self.link.handle_demand(demandBytes(0, True, 0.2), NEPI)
        self.assertTrue(accepted)
        self.assertEqual(self.state(), (False, 'start'))
        self.assertEqual(self.link.merge_inputs(pilot())[nepi_link.LY_SLOT], 0)
        self.clock.advance(0.1)
        self.link.handle_demand(demandBytes(1, True, 0.2), NEPI)
        self.assertEqual(self.state(), (False, 'start'))

    def test_rising_edge_turns_on_and_active_false_turns_off(self):
        self.link.handle_demand(demandBytes(0, False), NEPI)
        self.assertEqual(self.state(), (False, 'start'))
        self.link.assist_event.clear()
        self.link.handle_demand(demandBytes(1, True, 0.2), NEPI)
        self.assertEqual(self.state(), (True, 'nepi_on'))
        self.assertTrue(self.link.assist_event.is_set()) # The status goes out at once
        self.assertEqual(self.link.topside_tag(), ',NEPI:ON')
        self.link.handle_demand(demandBytes(2, False), NEPI)
        self.assertEqual(self.state(), (False, 'nepi_off'))

    def test_pilot_button_turns_off_and_needs_a_fresh_edge(self):
        seq = engage(self.link, self.clock, surge = 0.2)
        out = self.link.merge_inputs(pilot(rb = 1))
        self.assertEqual(out[nepi_link.RB_SLOT], 1) # RB still reaches the Teensy
        self.assertEqual(out[nepi_link.LY_SLOT], 0)
        self.assertEqual(self.state(), (False, 'pilot_button'))
        self.assertEqual(self.link.topside_tag(), ',NEPI:OFF')
        # NEPI keeps sending active true: assist stays off
        for i in range(5):
            self.clock.advance(0.1)
            self.link.handle_demand(demandBytes(seq + i, True, 0.2), NEPI)
            self.link.merge_inputs(pilot())
        self.assertEqual(self.state(), (False, 'pilot_button'))
        # A fresh false-then-true edge turns it back on
        engage(self.link, self.clock, surge = 0.2, seq = seq + 5)
        self.assertEqual(self.state(), (True, 'nepi_on'))

    def test_held_button_counts_once(self):
        engage(self.link, self.clock, surge = 0.2)
        self.link.merge_inputs(pilot(rb = 1))
        self.assertEqual(self.state(), (False, 'pilot_button'))
        # Still held while NEPI re-engages: no new press, so assist stays on
        engage(self.link, self.clock, surge = 0.2, seq = 10)
        self.link.merge_inputs(pilot(rb = 1))
        self.assertEqual(self.state(), (True, 'nepi_on'))
        # Release, press again: off
        self.link.merge_inputs(pilot(rb = 0))
        self.link.merge_inputs(pilot(rb = 1))
        self.assertEqual(self.state(), (False, 'pilot_button'))

    def test_button_does_not_turn_assist_on(self):
        self.link.merge_inputs(pilot(rb = 1))
        self.link.merge_inputs(pilot(rb = 0))
        self.link.merge_inputs(pilot(rb = 1))
        self.assertEqual(self.state(), (False, 'start'))

    def test_after_timeout_needs_a_fresh_edge(self):
        seq = engage(self.link, self.clock, surge = 0.2)
        self.clock.advance(2.5)
        self.link.merge_inputs(pilot())
        self.assertEqual(self.state(), (False, 'timeout'))
        # NEPI comes back still sending active true (accepted after the 1 s restart window)
        self.link.handle_demand(demandBytes(0, True, 0.2), NEPI)
        self.assertEqual(self.link.build_status()['demand_seq'], 0)
        self.assertEqual(self.state(), (False, 'timeout'))
        engage(self.link, self.clock, surge = 0.2, seq = 1)
        self.assertEqual(self.state(), (True, 'nepi_on'))

    def test_slow_tracks_lb_like_the_teensy(self):
        self.assertFalse(self.link.build_status()['slow'])
        self.link.merge_inputs(pilot(lb = 1))
        self.assertTrue(self.link.build_status()['slow'])
        self.link.merge_inputs(pilot(lb = 1))
        self.assertTrue(self.link.build_status()['slow'])
        self.link.merge_inputs(pilot(lb = 0))
        self.link.merge_inputs(pilot(lb = 1))
        self.assertFalse(self.link.build_status()['slow'])


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
        for key in ['assist', 'assist_reason', 'demand_seq', 'demand_age_ms']:
            self.assertEqual(after[key], before[key], key)
        self.assertEqual(self.link.merge_inputs(pilot())[nepi_link.LY_SLOT], 8713)

    def test_not_json(self):
        self.assertDropped(b'not json')

    def test_not_utf8(self):
        self.assertDropped(b'\xff\xfe{}')

    def test_not_an_object(self):
        self.assertDropped(b'[1, 2, 3]')

    def test_deeply_nested(self):
        self.assertDropped(b'[' * 600 + b']' * 600)

    def test_version_2(self):
        self.assertDropped(demandBytes(self.seq, True, 0.1, v = 2))

    def test_version_as_bool(self):
        self.assertDropped(demandBytes(self.seq, True, 0.1, v = True))

    def test_wrong_type(self):
        self.assertDropped(demandBytes(self.seq, True, 0.1, type = 'status'))

    def test_missing_field(self):
        message = json.loads(demandBytes(self.seq, True, 0.1).decode('utf-8'))
        del message['roll']
        self.assertDropped(json.dumps(message).encode('utf-8'))

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

    def test_out_of_range(self):
        self.assertDropped(demandBytes(self.seq, True, surge = 1.01))
        self.assertDropped(demandBytes(self.seq, True, sway = -1.5))
        self.assertDropped(demandBytes(self.seq, True, 0.1, pitch = 2))
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

    def test_bench_line(self):
        values = nepi_link.parse_teensy_line(BENCH_LINE)
        self.assertEqual(list(values.keys()), nepi_link.TN_NAMES)
        self.assertEqual([values[name] for name in nepi_link.TN_NAMES[:8]], [0, 0, 0, 0, 100, 100, 100, 100])
        for name in nepi_link.TN_NAMES[:8]:
            self.assertIsInstance(values[name], int)
        for name in nepi_link.TN_NAMES[8:]:
            self.assertIsInstance(values[name], float)
            self.assertEqual(values[name], 0.0)
        self.link.on_teensy_line(BENCH_LINE)
        self.clock.advance(0.3)
        status = self.link.build_status()
        self.assertEqual(status['tn'], values)
        self.assertEqual((status['tn_count'], status['tn_bad'], status['tn_age_ms']), (1, 0, 300))

    def test_line_with_line_ending_and_signs(self):
        values = nepi_link.parse_teensy_line('$TNCTL,-90,109,109,-90,1,2,3,4,0.12,-0.50,179.99,-3.00,1.00,'
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
        self.link.on_teensy_line(line_19)
        self.link.on_teensy_line(line_25)
        status = self.link.build_status()
        self.assertEqual((status['tn_count'], status['tn_bad'], status['tn']), (0, 2, None))

    def test_other_lines_count_as_bad(self):
        for line in ['ERRCHK', 'Failed to initialize bar02', 'unicode error', '',
                     BENCH_LINE.replace('$TNCTL', '$TNCTX'), BENCH_LINE.replace(',*FF', ',FF'),
                     BENCH_LINE.replace('0.00,*FF', 'nan,*FF'), BENCH_LINE.replace('0.00,*FF', 'ovf,*FF'),
                     BENCH_LINE.replace(',100,', ',100.5,', 1), BENCH_LINE.replace(',100,', ',99999999999,', 1)]:
            self.assertIsNone(nepi_link.parse_teensy_line(line), line)
        self.link.on_teensy_line('ERRCHK')
        self.link.on_teensy_line(None)
        self.assertEqual(self.link.build_status()['tn_bad'], 2)


class TestStatus(unittest.TestCase):

    def test_fields_before_any_demand(self):
        link, clock, logs = makeLink()
        status = link.build_status()
        self.assertEqual(sorted(status.keys()), sorted(STATUS_FIELDS))
        self.assertEqual((status['v'], status['type'], status['seq']), (1, 'status', 0))
        self.assertEqual((status['demand_seq'], status['demand_age_ms'], status['tn_age_ms']), (-1, -1, -1))
        self.assertIsNone(status['tn'])
        self.assertEqual(status['assist_reason'], 'start')
        for key in ['assist', 'pilot_surge', 'pilot_sway', 'pilot_yaw', 'pilot_depth', 'slow']:
            self.assertIsInstance(status[key], bool)

    def test_seq_advances_per_datagram(self):
        link, clock, logs = makeLink()
        first = json.loads(link.nextStatusDatagram().decode('utf-8'))
        second = json.loads(link.nextStatusDatagram().decode('utf-8'))
        self.assertEqual((first['seq'], second['seq']), (0, 1))

    def test_worst_case_size_under_1200_bytes(self):
        link, clock, logs = makeLink(NEPI_TRIGGER_DEADZONE = 40, NEPI_TRIGGER_MAX = 1023)
        link.wall_clock = lambda: 4102444800.123 # 2100-01-01
        worst_line = '$TNCTL,' + ','.join(['-2147483648'] * 8 + ['-9999999999.999999'] * 12) + ',*FF'
        link.on_teensy_line(worst_line)
        self.assertEqual(link.build_status()['tn_count'], 1)
        huge = 2 ** 53
        link.status_seq = nepi_link.SEQ_MAX
        link.demand_seq = nepi_link.SEQ_MAX
        link.demand_time = clock() - huge / 1000.0
        link.tn_time = clock() - huge / 1000.0
        link.demand_bad = huge
        link.tn_count = huge
        link.tn_bad = huge
        link.assist_reason = 'pilot_button'
        link.out_lx = -32768
        link.out_ly = -32768
        link.out_lt = 1023
        link.out_rt = 1023
        datagram = link.nextStatusDatagram()
        self.assertLessEqual(len(datagram), 1200, str(len(datagram)) + ' bytes')

    def test_out_values_clamped_to_contract(self):
        link, clock, logs = makeLink()
        link.merge_inputs(pilot(lt = 1023, ly = 40000))
        status = link.build_status()
        self.assertEqual((status['out_lt'], status['out_ly']), (255, 32767))


class TestLinkOff(unittest.TestCase):

    def test_empty_ip_passes_through_with_no_threads(self):
        logs = []
        link = nepi_link.NepiLink(log = logs.append)
        [on, msg] = link.start(dict(NEPI_LAN_IP = ''))
        self.assertFalse(on)
        self.assertEqual(link.threads, [])
        self.assertIsNone(link.stop_event)
        inputs = pilot(ly = 100, rb = 1)
        self.assertEqual(link.merge_inputs(inputs), tuple(inputs))
        self.assertEqual(link.topside_tag(), '')
        link.on_teensy_line(BENCH_LINE)
        self.assertIsNone(link.build_status())
        self.assertFalse(link.handle_demand(demandBytes(0, False), NEPI)[0])
        self.assertEqual(len(logs), 1)
        self.assertIn('link off', logs[0])

    def test_module_functions_before_start(self):
        inputs = pilot(ly = 123)
        self.assertEqual(nepi_link.merge_inputs(inputs), tuple(inputs))
        self.assertEqual(nepi_link.topside_tag(), '')
        nepi_link.on_teensy_line(BENCH_LINE)
        nepi_link.stop()

    def test_missing_or_bad_values_turn_the_link_off(self):
        team_config = nepi_link.parse_flat_yaml('TOPSIDE_LAN_IP: 192.168.68.12 \nONBOARD_LAN_IP: 192.168.68.22\n'
                                                'VIDEO0_UDP_PORT: 5600\nNMEA_TCP_PORT: 10110\n')
        self.assertIsNone(nepi_link.validate_config(team_config)[0])
        self.assertIsNone(nepi_link.validate_config(None)[0])
        for bad in [dict(NEPI_LAN_IP = 'nepi.local'), dict(NEPI_LAN_IP = NEPI, NEPI_MAX_SURGE = 1.5),
                    dict(NEPI_LAN_IP = NEPI, NEPI_YAW_SIGN = 0), dict(NEPI_LAN_IP = NEPI, NEPI_ASSIST_BUTTON = 'BTN_TL'),
                    dict(NEPI_LAN_IP = NEPI, NEPI_ASSIST_BUTTON = ['BTN_TR']),
                    dict(NEPI_LAN_IP = NEPI, NEPI_TRIGGER_DEADZONE = 300),
                    dict(NEPI_LAN_IP = NEPI, NEPI_DEMAND_UDP_PORT = 70000),
                    dict(NEPI_LAN_IP = NEPI, NEPI_MAX_YAW = True)]:
            self.assertIsNone(nepi_link.validate_config(bad)[0], bad)


class TestConfig(unittest.TestCase):

    def test_lookup_order(self):
        with tempfile.TemporaryDirectory() as root:
            module_dir = os.path.join(root, 'rpi', 'modes')
            os.makedirs(module_dir)
            repo_config = os.path.join(root, 'config.yaml')
            beside = os.path.join(module_dir, 'nepi_link.yaml')
            env_file = os.path.join(root, 'env.yaml')
            self.assertIsNone(nepi_link.find_config_file(module_dir, {}))
            open(repo_config, 'w').close()
            self.assertEqual(nepi_link.find_config_file(module_dir, {}), repo_config)
            open(beside, 'w').close()
            self.assertEqual(nepi_link.find_config_file(module_dir, {}), beside)
            self.assertEqual(nepi_link.find_config_file(module_dir, {'NEPI_LINK_CONFIG': env_file}), beside)
            open(env_file, 'w').close()
            self.assertEqual(nepi_link.find_config_file(module_dir, {'NEPI_LINK_CONFIG': env_file}), env_file)

    def test_bench_file(self):
        path = os.path.join(HERE, 'nepi_link_bench.yaml')
        [config, msg] = nepi_link.load_config_file(path)
        [settings, msg] = nepi_link.validate_config(config)
        self.assertIsNotNone(settings, msg)
        self.assertEqual(settings['NEPI_LAN_IP'], '192.168.68.23')
        self.assertEqual((settings['NEPI_TRIGGER_DEADZONE'], settings['NEPI_TRIGGER_MAX']), (40, 1023))
        self.assertEqual(settings['assist_slot'], nepi_link.RB_SLOT)
        for key in ['NEPI_STATUS_UDP_PORT', 'NEPI_DEMAND_UDP_PORT', 'NEPI_ASSIST_BUTTON', 'NEPI_MAX_SURGE',
                    'NEPI_MAX_SWAY', 'NEPI_MAX_YAW', 'NEPI_SURGE_SIGN', 'NEPI_SWAY_SIGN', 'NEPI_YAW_SIGN']:
            self.assertEqual(settings[key], nepi_link.DEFAULT_CONFIG[key], key)

    def test_builtin_parser_matches_pyyaml(self):
        with open(os.path.join(HERE, 'nepi_link_bench.yaml')) as bench_file:
            text = bench_file.read()
        text += "QUOTED: 'a # b'\nDOUBLE: \"x\"\nEMPTY:\nNEG: -1\nFLOAT: .5\nWORD: BTN_TR # RB\n  NESTED: 1\n"
        parsed = nepi_link.parse_flat_yaml(text)
        self.assertEqual(parsed['QUOTED'], 'a # b')
        self.assertEqual(parsed['DOUBLE'], 'x')
        self.assertIsNone(parsed['EMPTY'])
        self.assertEqual((parsed['NEG'], parsed['FLOAT'], parsed['WORD']), (-1, 0.5, 'BTN_TR'))
        self.assertNotIn('NESTED', parsed)
        self.assertEqual(parsed['NEPI_LAN_IP'], '192.168.68.23')
        yaml_module = nepi_link.importYaml()
        if yaml_module is not None:
            flat_text = text.replace('  NESTED: 1\n', '')
            self.assertEqual(nepi_link.parse_flat_yaml(flat_text), yaml_module.safe_load(flat_text))

    def test_load_without_pyyaml(self):
        saved = nepi_link.importYaml
        nepi_link.importYaml = lambda: None
        try:
            [config, msg] = nepi_link.load_config_file(os.path.join(HERE, 'nepi_link_bench.yaml'))
        finally:
            nepi_link.importYaml = saved
        self.assertEqual(config['NEPI_TRIGGER_MAX'], 1023)


class TestErrorFallback(unittest.TestCase):

    def test_exception_inside_merge_returns_inputs_unchanged(self):
        link, clock, logs = makeLink()
        engage(link, clock, surge = 0.2)
        def broken(pilot_inputs):
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

    def test_broken_clock_gives_empty_tag(self):
        link, clock, logs = makeLink()
        def broken():
            raise RuntimeError('clock')
        link.clock = broken
        self.assertEqual(link.topside_tag(), '')
        inputs = pilot(ly = 7)
        self.assertEqual(link.merge_inputs(inputs), tuple(inputs))


class TestFakeNepi(unittest.TestCase):

    def test_fake_demand_is_accepted(self):
        link, clock, logs = makeLink()
        datagram = json.dumps(fake_nepi.make_demand(0, False, 0.1, -0.1, 0.05)).encode('utf-8')
        [demand, msg] = nepi_link.parse_demand(datagram, NEPI, NEPI)
        self.assertIsNotNone(demand, msg)
        self.assertEqual((demand['surge'], demand['sway'], demand['yaw']), (0.1, -0.1, 0.05))


class TestLoopback(unittest.TestCase):

    def setUp(self):
        self.status_port = freeUdpPort()
        self.demand_port = freeUdpPort()
        while self.demand_port == self.status_port:
            self.demand_port = freeUdpPort()
        self.logs = []
        self.link = nepi_link.NepiLink(log = self.logs.append)
        self.receiver = fake_nepi.StatusReceiver('127.0.0.1', self.status_port, bind_ip = '127.0.0.1')
        self.sender = fake_nepi.DemandSender('127.0.0.1', self.demand_port)

    def tearDown(self):
        self.link.stop()
        self.receiver.stop()
        self.sender.close()

    def test_demand_reaches_the_merge_over_udp(self):
        [success, msg] = self.receiver.start()
        self.assertTrue(success, msg)
        config = dict(NEPI_LAN_IP = '127.0.0.1', NEPI_STATUS_UDP_PORT = self.status_port,
                      NEPI_DEMAND_UDP_PORT = self.demand_port)
        [on, msg] = self.link.start(config, 'test')
        self.assertTrue(on, msg)
        # active false first, until the bridge has one (its thread may still be binding)
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline and self.link.build_status()['demand_seq'] < 0:
            self.sender.send(False)
            time.sleep(0.1)
        self.assertGreaterEqual(self.link.build_status()['demand_seq'], 0)
        merged = None
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            self.sender.send(True, 0.2, 0.0, 0.0)
            time.sleep(0.1)
            merged = self.link.merge_inputs([0] * 18)
            if merged[nepi_link.LY_SLOT] == 8713:
                break
        self.assertEqual(merged[nepi_link.LY_SLOT], 8713)
        self.assertEqual(self.link.topside_tag(), ',NEPI:ON')
        # Statuses from after the merge: drop the queue, then collect for 0.35 s
        self.receiver.get_statuses(0)
        statuses = []
        end = time.monotonic() + 0.35
        while time.monotonic() < end:
            statuses += self.receiver.get_statuses(0.05)
        self.assertTrue(len(statuses) >= 2)
        newest = statuses[-1]
        self.assertTrue(newest['assist'])
        self.assertEqual((newest['assist_reason'], newest['out_ly'], newest['demand_bad']), ('nepi_on', 8713, 0))
        # NEPI stops: after 250 ms the pilot's inputs pass through and the tag reads WAIT
        time.sleep(0.5)
        self.assertEqual(self.link.merge_inputs([0] * 18), tuple([0] * 18))
        self.assertEqual(self.link.topside_tag(), ',NEPI:WAIT')
        # stop() closes the demand port
        self.link.stop()
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.bind(('', self.demand_port))
        finally:
            sock.close()


if __name__ == '__main__':
    unittest.main()
