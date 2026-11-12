##
## Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
##
## This file is part of nepi-engine
## (see https://github.com/nepi-engine).
##
## License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
##

# The link's messages, version 2 (Nepi_Connect/LINK_PROTOCOL.md): the rules
# every message follows, the demand parser and the status builder. Nothing
# here holds state or opens a socket; control.py does both.

import json
import math


PROTOCOL_VERSION = 2
MAX_DATAGRAM_BYTES = 1200 # LINK_PROTOCOL "Rules for Every Message", rule 1
SEQ_MAX = 4294967295
SEQ_RESTART_SEC = 1.0 # After this long, any seq is accepted (sender restarted)
COUNT_MAX = SEQ_MAX # Ages and counters in the status stop here, which bounds its size

# The axes, in the order every list of them follows
AXIS_NAMES = ['surge', 'sway', 'heave', 'yaw']
# Every reason the status can give for the last NEPI control change
CONTROL_REASONS = ['start', 'pilot_button', 'pilot_stick', 'timeout', 'nepi_off']
# The standard navigation names, by group; a robot sends a group whole or not at all
NAV_GROUPS = [['depth', ['depth_m', 'depth_m_s']],
              ['orientation', ['roll_deg', 'pitch_deg', 'yaw_deg', 'gyro_x_deg_s', 'gyro_y_deg_s', 'gyro_z_deg_s']]]


def isInteger(value):
    return isinstance(value, int) and not isinstance(value, bool)

def isNumber(value):
    if isinstance(value, bool):
        return False
    if isinstance(value, int):
        return True
    return isinstance(value, float) and math.isfinite(value)

def clampInt(value, low, high):
    return max(low, min(high, int(value)))

def clampCount(value):
    return min(value, COUNT_MAX)


def encode_message(message):
    """Encode a message dict as one compact JSON datagram.

    Args:
        message (dict): Any link message.

    Returns:
        bytes: UTF-8 JSON. Raises ValueError on a NaN or infinite number.
    """
    return json.dumps(message, separators = (',', ':'), allow_nan = False).encode('utf-8')

def next_seq(seq):
    """Return the seq after this one, wrapping from SEQ_MAX to 0.

    Args:
        seq (int): The current seq.

    Returns:
        int: seq + 1, or 0 after SEQ_MAX.
    """
    return (seq + 1) & SEQ_MAX

def seq_out_of_order(seq, last_seq, last_time, now):
    """Apply LINK_PROTOCOL "Rules for Every Message" rule 4 to one message type.

    Args:
        seq (int): The new message's seq.
        last_seq (int): The last accepted seq of this type, -1 before the first.
        last_time (float): When it was accepted, on the receiver's clock.
        now (float): The receiver's clock now.

    Returns:
        bool: True when the message must be dropped: its seq is not after the
            last accepted one, which arrived no more than SEQ_RESTART_SEC ago.
    """
    return last_seq >= 0 and seq <= last_seq and now - last_time <= SEQ_RESTART_SEC

def age_ms(now, then):
    """Return the age of something accepted at `then`, in ms.

    Args:
        now (float): The clock now, in seconds.
        then (float): When it was accepted, or None.

    Returns:
        int: The age, from 0 to COUNT_MAX, or -1 when then is None.
    """
    if then is None:
        return -1
    return clampCount(max(0, int(round((now - then) * 1000))))


def parse_demand(datagram, source_ip, nepi_ip, axis_names):
    """Apply LINK_PROTOCOL "Rules for Every Message" 1, 2, 3 and 5 to a demand.

    Rule 4 (seq order) needs the previous demand, so the caller applies it
    with seq_out_of_order().

    Args:
        datagram (bytes): The UDP payload.
        source_ip (str): The sender's address.
        nepi_ip (str): The vehicle file's network.nepi_ip, the only accepted sender.
        axis_names (list): The announced axes. Others in the demand are ignored.

    Returns:
        list: [demand (dict or None), msg (str)]. demand holds seq, active,
            axes (every announced axis to its value, 0.0 when missing) and
            servos (name to value), or is None when the datagram must be
            dropped, with the reason in msg.
    """
    if source_ip != nepi_ip:
        return None, 'Demand from ' + str(source_ip) + ' dropped: only ' + nepi_ip + ' may send demands'
    if len(datagram) > MAX_DATAGRAM_BYTES:
        return None, 'Demand of ' + str(len(datagram)) + ' bytes dropped: over ' + str(MAX_DATAGRAM_BYTES)
    try:
        message = json.loads(datagram.decode('utf-8'))
    except Exception as e:
        return None, 'Demand dropped: not UTF-8 JSON (' + str(e) + ')'
    if not isinstance(message, dict):
        return None, 'Demand dropped: not a JSON object'
    if not isInteger(message.get('v')) or message['v'] != PROTOCOL_VERSION:
        return None, 'Demand dropped: v must be ' + str(PROTOCOL_VERSION)
    if message.get('type') != 'demand':
        return None, 'Demand dropped: type must be "demand"'
    seq = message.get('seq')
    if not isInteger(seq) or seq < 0 or seq > SEQ_MAX:
        return None, 'Demand dropped: seq must be an integer from 0 to ' + str(SEQ_MAX)
    if not isNumber(message.get('t')):
        return None, 'Demand dropped: t must be a finite number'
    if not isinstance(message.get('active'), bool):
        return None, 'Demand dropped: active must be true or false'
    # No axes field means every axis is 0, as a missing axis is
    axes = message.get('axes')
    if axes is None:
        axes = dict()
    if not isinstance(axes, dict):
        return None, 'Demand dropped: axes must be an object'
    values = dict()
    for name in axis_names:
        value = axes.get(name, 0.0)
        if not isNumber(value) or value < -1 or value > 1:
            return None, 'Demand dropped: axes ' + name + ' must be a finite number from -1 to 1'
        values[name] = float(value)
    # No servos field means no servo values. Names this robot has no servo for
    # are kept and ignored by the merge.
    servos = message.get('servos')
    if servos is None:
        servos = dict()
    if not isinstance(servos, dict):
        return None, 'Demand dropped: servos must be an object'
    for value in servos.values():
        if not isNumber(value) or value < -1 or value > 1:
            return None, 'Demand dropped: each servos value must be a finite number from -1 to 1'
    demand = dict(seq = seq, active = message['active'], axes = values,
                  servos = dict((name, float(value)) for (name, value) in servos.items()))
    return demand, 'Accepted'


def build_status(seq, t, control, control_reason, nepi_ready, pilot, servos, out, demand_seq, demand_age_ms,
                 demand_bad, nav, nav_age_ms, nav_count, nav_bad, raw = None, has_raw = False):
    """Build a status message (LINK_PROTOCOL "Status") from its field values.

    Both the status sender and the worst-case size check build through here,
    so the two always agree on the fields.

    Args:
        seq (int), t (float): The common fields.
        control (bool), control_reason (str), nepi_ready (bool): NEPI control.
        pilot (dict): Axis name to whether the pilot holds one of its inputs.
        servos (dict): Servo name to whether the pilot holds its input.
        out (dict): Axis input name to the value last handed on.
        demand_seq, demand_age_ms, demand_bad (int): The demand counters.
        nav (dict): Standard name to value, or None.
        nav_age_ms, nav_count, nav_bad (int): The telemetry counters.
        raw (dict): Raw field name to value, or None.
        has_raw (bool): The vehicle file lists raw fields; without it, the
            status has no raw field at all.

    Returns:
        dict: The status message.
    """
    status = dict(v = PROTOCOL_VERSION, type = 'status', seq = seq, t = t, control = control,
                  control_reason = control_reason, nepi_ready = nepi_ready, pilot = pilot, servos = servos,
                  out = out, demand_seq = demand_seq, demand_age_ms = demand_age_ms, demand_bad = demand_bad,
                  nav = nav, nav_age_ms = nav_age_ms, nav_count = nav_count, nav_bad = nav_bad)
    if has_raw:
        status['raw'] = raw
    return status
