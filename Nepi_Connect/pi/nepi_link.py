#!/usr/bin/env python3
##
## Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
##
## This file is part of nepi-engine
## (see https://github.com/nepi-engine).
##
## License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
##

# The control Pi's side of the NEPI link: the NEPI control toggle and the merge.
#
# The Coral Crusaders' Pi script (rov-26 rpi/modes/main.py) imports this file
# and calls it at four places (Nepi_Connect/README.md Section 3):
#   nepi_link.start()                       once, before the main loop
#   nepi_link.merge_inputs(gamepad_inputs)  in place of tuple(gamepad_inputs)
#   nepi_link.on_teensy_line(onboard_data)  after each Teensy line is read
#   nepi_link.topside_tag()                 inside the $RPCTL line to the topside
# stop() exists for tests. main_py_nepi_link.patch in this folder adds the hooks.
#
# The wire contract is Nepi_Connect/LINK_PROTOCOL.md. NEPI sends a demand
# (surge, sway, heave, yaw) to UDP 5701 at 10 Hz; this file merges it into a
# COPY of the pilot's inputs just before nmea_encode, and sends NEPI a status
# to UDP 5700 every 100 ms. Every demand is capped.
#
# NEPI control (BENCH_SETUP_PLAN.md D3, decided 2026-10-22): the pilot flies,
# and pressing NEPI_ASSIST_BUTTON (RB) hands the ROV to NEPI. In NEPI control,
# NEPI owns every movement input (LY, LX, RY, LT, RT): the ROV follows NEPI's
# demand, whether that comes from NEPI's stabilization process or an operator
# on NEPI's Robots page, and a demand older than 250 ms means neutral. RY
# carries NEPI's heave; while heave is 0, RY is 0 and the Teensy's own depth
# hold keeps the depth, the same as when the pilot centers the stick. Buttons,
# the D-pad, RX and LB pass through, so the pilot can still work tools, except
# the inputs named as NEPI servos (NEPI_SERVO_n_*, e.g. the claw's): in NEPI
# control those carry the demand's servo values, and the pilot's own input
# wins whenever it is off neutral, for that input only. NEPI
# control ends when the pilot presses RB again, when the pilot moves any
# movement input past NEPI_TAKEOVER_FRACTION of its travel (the safety exit),
# when NEPI sends active false, or after 2 s with no accepted demand. NEPI
# never turns it on: `active` true only means NEPI is connected and ready, and
# RB is ignored while it is not.
#
# Config: a flat "KEY: value" file with the LINK_PROTOCOL "Pi Configuration"
# keys. The first file that exists wins: $NEPI_LINK_CONFIG, nepi_link.yaml
# beside this file, then ../../config.yaml (the team's repo root when this
# file sits in rpi/modes). With no file, or an empty NEPI_LAN_IP, the link is
# off: no sockets, no threads, and the pilot's inputs and topside line pass
# through unchanged.
#
# Nothing here may stop the team's script. Every public function catches every
# exception and falls back to passing the pilot's inputs through untouched.
# Errors print at most once per 10 s. Standard library only, Python 3.8 or
# newer; PyYAML is used for the config only if it imports.

import ipaddress
import json
import math
import os
import re
import socket
import threading
import time


# Input slots in the 18-value tuple the Pi hands to nmea_encode, from
# gamepad_map in rov-26 rpi/modes/main.py:39-58. They match the Teensy's
# input_data_t order in rov-26 mcu/modes/new_ctl/new_ctl.ino:130-149. This
# file must not import main.py, which opens the serial port at import.
LX_SLOT = 0  # ABS_X, main.py:40; ABS_LX, new_ctl.ino:131
LY_SLOT = 1  # ABS_Y, main.py:41; ABS_LY, new_ctl.ino:132
RX_SLOT = 2  # ABS_RX, main.py:42; ABS_RX, new_ctl.ino:133
RY_SLOT = 3  # ABS_RY, main.py:43; ABS_RY, new_ctl.ino:134
LB_SLOT = 12 # BTN_TL, main.py:52; BTN_LB, new_ctl.ino:143
RB_SLOT = 13 # BTN_TR, main.py:53; BTN_RB, new_ctl.ino:144
LT_SLOT = 14 # ABS_Z, main.py:54; ABS_LT, new_ctl.ino:145
RT_SLOT = 15 # ABS_RZ, main.py:55; ABS_RT, new_ctl.ino:146

# Event names for the slots above, so NEPI_ASSIST_BUTTON can name its button
EVENT_SLOTS = {'ABS_X': LX_SLOT, 'ABS_Y': LY_SLOT, 'ABS_RX': RX_SLOT, 'ABS_RY': RY_SLOT,
               'BTN_TL': LB_SLOT, 'BTN_TR': RB_SLOT, 'ABS_Z': LT_SLOT, 'ABS_RZ': RT_SLOT}

STICK_DEADZONE = 3000 # JOYSTICK_DEADZONE_CONST, new_ctl.ino:29
STICK_MAX = 32767 # JOYSTICK_MAGNITUDE, new_ctl.ino:27
MIN_DEMAND = 0.01 # Smaller demands map to 0 (LINK_PROTOCOL "Dead-zone mapping")

# Every input slot by its gamepad_map event name (main.py:39-58)
GAMEPAD_SLOTS = {'ABS_X': 0, 'ABS_Y': 1, 'ABS_RX': 2, 'ABS_RY': 3, 'BTN_THUMBL': 4, 'BTN_THUMBR': 5,
                 'ABS_HAT0X': 6, 'ABS_HAT0Y': 7, 'BTN_SOUTH': 8, 'BTN_EAST': 9, 'BTN_NORTH': 10, 'BTN_WEST': 11,
                 'BTN_TL': 12, 'BTN_TR': 13, 'ABS_Z': 14, 'ABS_RZ': 15, 'BTN_START': 16, 'BTN_SELECT': 17}

# Inputs a NEPI servo may use (NEPI_SERVO_n_INPUT), each with its default
# [min, neutral, max, dead zone]. Not offered: the movement inputs NEPI
# already owns, LB (slow mode) and RB (the NEPI control button). The RX dead
# zone is 2999 because the Teensy zeroes |RX| < 3000 (new_ctl.ino:570).
SERVO_INPUTS = {'ABS_RX': [-STICK_MAX, 0, STICK_MAX, STICK_DEADZONE - 1],
                'ABS_HAT0X': [-1, 0, 1, 0], 'ABS_HAT0Y': [-1, 0, 1, 0],
                'BTN_THUMBL': [0, 0, 1, 0], 'BTN_THUMBR': [0, 0, 1, 0],
                'BTN_SOUTH': [0, 0, 1, 0], 'BTN_EAST': [0, 0, 1, 0], 'BTN_NORTH': [0, 0, 1, 0], 'BTN_WEST': [0, 0, 1, 0],
                'BTN_START': [0, 0, 1, 0], 'BTN_SELECT': [0, 0, 1, 0]}
MAX_SERVOS = 4 # NEPI_SERVO_1_* to NEPI_SERVO_4_*; more would not fit the status
SERVO_NAME = re.compile(r'^[a-z][a-z0-9_]{0,15}$')

# LINK_PROTOCOL "Pi Configuration" defaults
DEFAULT_CONFIG = {
    'NEPI_LAN_IP': '',
    'NEPI_STATUS_UDP_PORT': 5700,
    'NEPI_DEMAND_UDP_PORT': 5701,
    'NEPI_ASSIST_BUTTON': 'BTN_TR',
    'NEPI_MAX_SURGE': 0.2,
    'NEPI_MAX_SWAY': 0.2,
    'NEPI_MAX_HEAVE': 0.2,
    'NEPI_MAX_YAW': 0.1,
    'NEPI_SURGE_SIGN': 1,
    'NEPI_SWAY_SIGN': 1,
    'NEPI_HEAVE_SIGN': 1,
    'NEPI_YAW_SIGN': 1,
    'NEPI_TRIGGER_DEADZONE': 10,
    'NEPI_TRIGGER_MAX': 255,
    'NEPI_TAKEOVER_FRACTION': 0.25,
}

# Names for the 20 values of a $TNCTL line (LINK_PROTOCOL "Teensy telemetry").
# The first 8 are integers, the rest decimals.
TN_NAMES = ['vfl_pwr', 'vfr_pwr', 'vbl_pwr', 'vbr_pwr', 'hfl_pwr', 'hfr_pwr', 'hbl_pwr', 'hbr_pwr',
            'depth_m', 'depth_m_s', 'yaw_deg', 'pitch_deg', 'roll_deg',
            'yaw_deg_s', 'pitch_deg_s', 'roll_deg_s',
            'depth_rate_target_m_s', 'depth_pwr_out', 'yaw_rate_target_deg_s', 'yaw_pwr_out']
TN_INT_COUNT = 8
# The firmware prints int32 values and 2-decimal doubles; Arduino prints
# "nan", "inf" or "ovf" for doubles it cannot show, and those lines count as
# tn_bad. Bounded lengths keep the status under its size limit.
TN_INT = re.compile(r'^-?[0-9]{1,10}$')
TN_FLOAT = re.compile(r'^-?[0-9]{1,10}(\.[0-9]{1,6})?$')
TN_TRAILER = re.compile(r'^\*[0-9A-Fa-f]{2}$')
INT32_MIN = -2147483648
INT32_MAX = 2147483647

MAX_DATAGRAM_BYTES = 1200 # LINK_PROTOCOL "Rules for Every Message", rule 1
SEQ_MAX = 4294967295
STATUS_PERIOD_SEC = 0.1
DEMAND_STALE_SEC = 0.25 # An older demand counts as 0 on every axis
NEPI_READY_SEC = 0.5 # NEPI is ready while an active true demand was accepted within this
ASSIST_TIMEOUT_SEC = 2.0 # Assist turns off after this long with no accepted demand
SEQ_RESTART_SEC = 1.0 # After this long, any seq is accepted (sender restarted)
ERROR_PRINT_SEC = 10.0 # Each kind of message prints at most once per this period
SOCKET_TIMEOUT_SEC = 0.2 # How often the receive thread checks for stop
SOCKET_RETRY_SEC = 1.0 # Wait before retrying a socket that failed

FLAT_LINE = re.compile(r'^([A-Za-z_][A-Za-z0-9_]*)[ \t]*:(.*)$')
INT_VALUE = re.compile(r'^[-+]?[0-9]+$')
FLOAT_VALUE = re.compile(r'^[-+]?([0-9]+\.[0-9]*|\.[0-9]+|[0-9]+)([eE][-+]?[0-9]+)?$')
COMMENT_START = re.compile(r'(^|\s)#')


#**********************
# Mapping

def stick_value(demand):
    """Map a demand fraction onto the stick range the Teensy acts on.

    The Teensy zeroes sticks inside +/-3000 and passes the rest through
    without rescaling (new_ctl.ino:563-582), so 0.01 to 1 maps evenly onto
    3000 to 32767 and anything smaller becomes 0.

    Args:
        demand (float): Fraction of full stick travel, -1 to 1.

    Returns:
        int: Stick value, -32767 to 32767.
    """
    magnitude = abs(demand)
    if magnitude < MIN_DEMAND:
        return 0
    value = int(STICK_DEADZONE + (magnitude - MIN_DEMAND) / (1.0 - MIN_DEMAND) * (STICK_MAX - STICK_DEADZONE) + 0.5)
    value = min(value, STICK_MAX)
    if demand < 0:
        return -value
    return value

def trigger_value(demand, deadzone, maximum):
    """Map a demand fraction onto the trigger range the Teensy acts on.

    Args:
        demand (float): Fraction of a full trigger pull, 0 to 1.
        deadzone (int): The sketch's TRIGGER_DEADZONE_CONST.
        maximum (int): The sketch's TRIGER_MAGNITUDE.

    Returns:
        int: Trigger value, 0 or deadzone to maximum.
    """
    if demand < MIN_DEMAND:
        return 0
    value = int(deadzone + (demand - MIN_DEMAND) / (1.0 - MIN_DEMAND) * (maximum - deadzone) + 0.5)
    return min(value, maximum)

def servo_value(demand, low, neutral, high):
    """Map a servo demand onto its input's raw range.

    0 is the input's neutral, 1 its high end and -1 its low end, each half
    scaled on its own, so a button (0, 0, 1) reads any negative demand as
    released. Rounds half away from neutral, so a D-pad (-1, 0, 1) is pressed
    for any demand of at least 0.5 either way.

    Args:
        demand (float): -1 to 1.
        low (int): NEPI_SERVO_n_MIN.
        neutral (int): NEPI_SERVO_n_NEUTRAL.
        high (int): NEPI_SERVO_n_MAX.

    Returns:
        int: Input value, low to high.
    """
    if demand >= 0:
        offset = demand * (high - neutral)
    else:
        offset = demand * (neutral - low)
    steps = int(math.floor(abs(offset) + 0.5))
    if offset < 0:
        steps = -steps
    return max(low, min(high, neutral + steps))


#**********************
# Config

def config_candidates(module_dir = None, environ = None):
    """List the config file paths to try, in lookup order.

    Args:
        module_dir (str): Folder to treat as this file's folder (default: the real one).
        environ (dict): Environment to read NEPI_LINK_CONFIG from (default: os.environ).

    Returns:
        list: Paths, first match wins.
    """
    if module_dir is None:
        module_dir = os.path.dirname(os.path.abspath(__file__))
    if environ is None:
        environ = os.environ
    candidates = []
    env_path = environ.get('NEPI_LINK_CONFIG', '')
    if env_path != '':
        candidates.append(env_path)
    candidates.append(os.path.join(module_dir, 'nepi_link.yaml'))
    candidates.append(os.path.normpath(os.path.join(module_dir, '..', '..', 'config.yaml')))
    return candidates

def find_config_file(module_dir = None, environ = None):
    """Return the first config file that exists.

    Args:
        module_dir (str): As in config_candidates().
        environ (dict): As in config_candidates().

    Returns:
        str: The path, or None when no candidate exists.
    """
    for path in config_candidates(module_dir, environ):
        if os.path.isfile(path):
            return path
    return None

def parse_flat_yaml(text):
    """Parse a flat "KEY: value" file without PyYAML.

    Handles # comments, integers, floats, and quoted or bare strings. Indented
    lines and lines without a top-level key are skipped. An empty value is None.

    Args:
        text (str): The file's contents.

    Returns:
        dict: Key to value.
    """
    config = dict()
    for raw_line in text.splitlines():
        if raw_line[:1] in (' ', '\t'):
            continue
        line = raw_line.strip()
        if line == '' or line.startswith('#'):
            continue
        match = FLAT_LINE.match(line)
        if match is None:
            continue
        config[match.group(1)] = parseScalar(match.group(2).strip())
    return config

def parseScalar(value):
    if value[:1] in ('"', "'"):
        end = value.find(value[0], 1)
        if end > 0:
            return value[1:end]
    comment = COMMENT_START.search(value)
    if comment is not None:
        value = value[:comment.start()].rstrip()
    if value in ('', '~', 'null'):
        return None
    if INT_VALUE.match(value):
        return int(value)
    if FLOAT_VALUE.match(value):
        return float(value)
    return value

def importYaml():
    try:
        import yaml
        return yaml
    except Exception:
        return None

def load_config_file(path):
    """Read a config file, with PyYAML if it imports, else parse_flat_yaml().

    Args:
        path (str): The file.

    Returns:
        list: [config (dict or None), msg (str)]. config is None when the file
            cannot be read or is not a flat key-value file.
    """
    try:
        with open(path, 'r') as config_file:
            text = config_file.read()
    except (OSError, UnicodeDecodeError) as e:
        return None, 'Cannot read ' + path + ': ' + str(e)
    yaml_module = importYaml()
    if yaml_module is None:
        config = parse_flat_yaml(text)
    else:
        try:
            config = yaml_module.safe_load(text)
        except Exception as e:
            return None, 'Cannot parse ' + path + ': ' + str(e)
        if config is None:
            config = dict()
    if not isinstance(config, dict):
        return None, path + ' is not a flat KEY: value file'
    return config, 'Read ' + path

def validate_config(config):
    """Check a raw config, fill in the LINK_PROTOCOL defaults, and decide on or off.

    Args:
        config (dict): Raw keys and values, e.g. from load_config_file().

    Returns:
        list: [settings (dict or None), msg (str)]. settings is None when the
            link is off: NEPI_LAN_IP missing or empty, or any value invalid.
            Otherwise it holds every DEFAULT_CONFIG key plus assist_slot.
    """
    if not isinstance(config, dict):
        return None, 'No config: link off'
    settings = dict(DEFAULT_CONFIG)
    for key in DEFAULT_CONFIG:
        if key in config and config[key] is not None:
            settings[key] = config[key]
    lan_ip = settings['NEPI_LAN_IP']
    if not isinstance(lan_ip, str) or lan_ip.strip() == '':
        return None, 'NEPI_LAN_IP is empty: link off'
    try:
        settings['NEPI_LAN_IP'] = str(ipaddress.IPv4Address(lan_ip.strip()))
    except ValueError:
        return None, 'NEPI_LAN_IP ' + repr(lan_ip) + ' is not an IPv4 address: link off'
    for key in ['NEPI_STATUS_UDP_PORT', 'NEPI_DEMAND_UDP_PORT']:
        if not isInteger(settings[key]) or settings[key] < 1 or settings[key] > 65535:
            return None, key + ' must be a port number from 1 to 65535: link off'
    button = settings['NEPI_ASSIST_BUTTON']
    if not isinstance(button, str) or button not in EVENT_SLOTS or not button.startswith('BTN_') or button == 'BTN_TL':
        return None, 'NEPI_ASSIST_BUTTON must be BTN_TR (RB); BTN_TL is slow mode: link off'
    settings['assist_slot'] = EVENT_SLOTS[button]
    for key in ['NEPI_MAX_SURGE', 'NEPI_MAX_SWAY', 'NEPI_MAX_HEAVE', 'NEPI_MAX_YAW']:
        if not isNumber(settings[key]) or settings[key] < 0 or settings[key] > 1:
            return None, key + ' must be a number from 0 to 1: link off'
        settings[key] = float(settings[key])
    for key in ['NEPI_SURGE_SIGN', 'NEPI_SWAY_SIGN', 'NEPI_HEAVE_SIGN', 'NEPI_YAW_SIGN']:
        if not isInteger(settings[key]) or settings[key] not in (1, -1):
            return None, key + ' must be 1 or -1: link off'
    deadzone = settings['NEPI_TRIGGER_DEADZONE']
    maximum = settings['NEPI_TRIGGER_MAX']
    if not isInteger(deadzone) or not isInteger(maximum) or deadzone < 1 or maximum <= deadzone:
        return None, 'NEPI_TRIGGER_DEADZONE and NEPI_TRIGGER_MAX must be integers with 1 <= dead zone < max: link off'
    fraction = settings['NEPI_TAKEOVER_FRACTION']
    if not isNumber(fraction) or fraction <= 0 or fraction > 1:
        return None, 'NEPI_TAKEOVER_FRACTION must be a number above 0 and at most 1: link off'
    settings['NEPI_TAKEOVER_FRACTION'] = float(fraction)
    # A movement input at or past these ends NEPI control
    settings['takeover_stick'] = int(math.ceil(fraction * STICK_MAX))
    settings['takeover_trigger'] = int(math.ceil(fraction * maximum))
    if settings['takeover_stick'] <= STICK_DEADZONE or settings['takeover_trigger'] <= deadzone:
        return None, ('NEPI_TAKEOVER_FRACTION must put the takeover point past the stick and trigger '
                      'dead zones: link off')
    [servos, msg] = validateServos(config)
    if servos is None:
        return None, msg
    settings['servos'] = servos
    servos_msg = ', no servos'
    if len(servos) > 0:
        servos_msg = ', servos ' + ', '.join(servo['name'] + ' on ' + servo['input'] for servo in servos)
    return settings, ('On: NEPI ' + settings['NEPI_LAN_IP'] + ', status to UDP ' + str(settings['NEPI_STATUS_UDP_PORT']) +
                      ', demands on UDP ' + str(settings['NEPI_DEMAND_UDP_PORT']) +
                      ', triggers ' + str(deadzone) + ' to ' + str(maximum) +
                      ', takeover at stick ' + str(settings['takeover_stick']) +
                      ' and trigger ' + str(settings['takeover_trigger']) + servos_msg)

def validateServos(config):
    # NEPI_SERVO_1_* to NEPI_SERVO_<MAX_SERVOS>_*, as [servos (list) or None, msg].
    # A number whose NAME is missing or empty is unused.
    servos = []
    for number in range(1, MAX_SERVOS + 1):
        prefix = 'NEPI_SERVO_' + str(number) + '_'
        name = config.get(prefix + 'NAME')
        if name is None or name == '':
            continue
        if not isinstance(name, str) or not SERVO_NAME.match(name):
            return None, (prefix + 'NAME must be 1 to 16 lowercase letters, digits or _, starting with a letter: '
                          'link off')
        event = config.get(prefix + 'INPUT')
        if not isinstance(event, str) or event not in SERVO_INPUTS:
            return None, prefix + 'INPUT must be one of ' + ', '.join(sorted(SERVO_INPUTS)) + ': link off'
        values = dict()
        for [key, default] in zip(['MIN', 'NEUTRAL', 'MAX', 'DEADZONE'], SERVO_INPUTS[event]):
            value = config.get(prefix + key)
            if value is None:
                value = default
            if not isInteger(value):
                return None, prefix + key + ' must be an integer: link off'
            values[key] = value
        if (not values['MIN'] <= values['NEUTRAL'] <= values['MAX'] or values['MIN'] == values['MAX'] or
                values['DEADZONE'] < 0):
            return None, (prefix + 'MIN, NEUTRAL, MAX and DEADZONE need MIN <= NEUTRAL <= MAX, MIN < MAX and '
                          'DEADZONE >= 0: link off')
        for other in servos:
            if other['name'] == name:
                return None, prefix + 'NAME ' + name + ' is already used: link off'
            if other['input'] == event:
                return None, prefix + 'INPUT ' + event + ' is already used: link off'
        servos.append(dict(name = name, input = event, slot = GAMEPAD_SLOTS[event], min = values['MIN'],
                           neutral = values['NEUTRAL'], max = values['MAX'], deadzone = values['DEADZONE']))
    return servos, 'Servos valid'


#**********************
# Messages

def isInteger(value):
    return isinstance(value, int) and not isinstance(value, bool)

def isNumber(value):
    if isinstance(value, bool):
        return False
    if isinstance(value, int):
        return True
    return isinstance(value, float) and math.isfinite(value)

def parse_demand(datagram, source_ip, nepi_ip):
    """Apply LINK_PROTOCOL "Rules for Every Message" 1, 2, 3 and 5 to a demand.

    Rule 4 (seq order) needs the previous demand, so NepiLink applies it.

    Args:
        datagram (bytes): The UDP payload.
        source_ip (str): The sender's address.
        nepi_ip (str): NEPI_LAN_IP, the only accepted sender.

    Returns:
        list: [demand (dict or None), msg (str)]. demand holds seq, active,
            surge, sway, heave, yaw and servos (name to value), or is None
            when the datagram must be dropped, with the reason in msg.
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
    if not isInteger(message.get('v')) or message['v'] != 1:
        return None, 'Demand dropped: v must be 1'
    if message.get('type') != 'demand':
        return None, 'Demand dropped: type must be "demand"'
    seq = message.get('seq')
    if not isInteger(seq) or seq < 0 or seq > SEQ_MAX:
        return None, 'Demand dropped: seq must be an integer from 0 to ' + str(SEQ_MAX)
    if not isNumber(message.get('t')):
        return None, 'Demand dropped: t must be a finite number'
    if not isinstance(message.get('active'), bool):
        return None, 'Demand dropped: active must be true or false'
    for name in ['surge', 'sway', 'yaw', 'pitch', 'roll']:
        value = message.get(name)
        if not isNumber(value) or value < -1 or value > 1:
            return None, 'Demand dropped: ' + name + ' must be a finite number from -1 to 1'
    # heave joined version 1 after the other axes, so a demand without it is
    # still accepted and means 0 (rule 3: version 1 can gain fields)
    heave = message.get('heave', 0.0)
    if not isNumber(heave) or heave < -1 or heave > 1:
        return None, 'Demand dropped: heave must be a finite number from -1 to 1'
    # servos joined version 1 later too: no field means no servo values. Names
    # this Pi has no servo for are kept and ignored by the merge.
    servos = message.get('servos', dict())
    if not isinstance(servos, dict):
        return None, 'Demand dropped: servos must be an object'
    for value in servos.values():
        if not isNumber(value) or value < -1 or value > 1:
            return None, 'Demand dropped: each servos value must be a finite number from -1 to 1'
    demand = dict(seq = seq, active = message['active'], surge = float(message['surge']),
                  sway = float(message['sway']), heave = float(heave), yaw = float(message['yaw']),
                  servos = dict((name, float(value)) for (name, value) in servos.items()))
    return demand, 'Accepted'

def parse_teensy_line(line):
    """Parse one 20-value $TNCTL line (LINK_PROTOCOL "Teensy telemetry").

    Args:
        line (str): A line from the Teensy, with or without its line ending.

    Returns:
        dict: The 20 values by TN_NAMES name, ints then floats, or None when
            the line is not exactly $TNCTL, 20 values and a *XX trailer.
    """
    if not isinstance(line, str):
        return None
    parts = line.strip().split(',')
    if len(parts) != len(TN_NAMES) + 2 or parts[0] != '$TNCTL' or not TN_TRAILER.match(parts[-1]):
        return None
    values = dict()
    for i, text in enumerate(parts[1:-1]):
        if i < TN_INT_COUNT:
            if not TN_INT.match(text):
                return None
            value = int(text)
            if value < INT32_MIN or value > INT32_MAX:
                return None
        else:
            if not TN_FLOAT.match(text):
                return None
            value = float(text)
        values[TN_NAMES[i]] = value
    return values

def encode_status(status):
    """Encode a status dict as one compact JSON datagram.

    Args:
        status (dict): From NepiLink.build_status().

    Returns:
        bytes: UTF-8 JSON. Raises ValueError on a NaN or infinite number.
    """
    return json.dumps(status, separators = (',', ':'), allow_nan = False).encode('utf-8')

def clampDemand(value, cap):
    return max(-cap, min(cap, value))

def clampInt(value, low, high):
    return max(low, min(high, int(value)))

def ageMs(now, then):
    if then is None:
        return -1
    return max(0, int(round((now - then) * 1000)))

def safeTuple(gamepad_inputs):
    try:
        return tuple(gamepad_inputs)
    except Exception:
        return gamepad_inputs


#**********************
# Bridge

class NepiLink(object):
    """The Pi side of the NEPI link: toggle, merge, status and demands.

    The team's main loop, the demand receiver thread and the status sender
    thread all touch the same state, so all of it sits behind one lock.
    Methods named ...Locked expect the caller to hold it; messages they want
    printed go in pending_msgs, printed once the lock is released. clock is a
    monotonic seconds function, injectable so tests can control time;
    wall_clock fills the status `t` field; log prints messages.
    """

    def __init__(self, clock = time.monotonic, wall_clock = time.time, log = print):
        self.lock = threading.Lock()
        self.clock = clock
        self.wall_clock = wall_clock
        self.log = log
        self.settings = None # None means the link is off
        self.config_path = None
        self.stop_event = None
        self.threads = []
        self.assist_event = threading.Event() # Wakes the status sender early
        self.last_print = dict()
        self.suppressed = dict()
        self.resetStateLocked()

    def resetStateLocked(self):
        self.assist = False # NEPI control
        self.assist_reason = 'start'
        self.assist_btn_avl = True
        self.pending_msgs = [] # [key, msg] pairs for printLimited, once the lock is released
        self.slow = False
        self.slow_btn_avl = True
        self.pilot_surge = False
        self.pilot_sway = False
        self.pilot_yaw = False
        self.pilot_depth = False
        self.servo_pilot = dict() # Servo name to whether the pilot's own input is off neutral
        self.out_lx = 0
        self.out_ly = 0
        self.out_ry = 0
        self.out_lt = 0
        self.out_rt = 0
        self.demand = None
        self.demand_seq = -1
        self.demand_time = None
        self.demand_bad = 0
        self.tn = None
        self.tn_time = None
        self.tn_count = 0
        self.tn_bad = 0
        self.status_seq = 0


    def configure(self, config, config_path = None):
        """Validate a raw config and reset all link state. Opens no sockets.

        Args:
            config (dict): Raw keys and values, or None for link off.
            config_path (str): Where config came from, for messages.

        Returns:
            list: [on (bool), msg (str)]. on is False when the link is off.
        """
        try:
            [settings, msg] = validate_config(config)
            with self.lock:
                self.settings = settings
                self.config_path = config_path
                self.resetStateLocked()
            if config_path is not None:
                msg += ' (' + config_path + ')'
            return settings is not None, msg
        except Exception as e:
            self.reportError('configure', e)
            return False, 'Config error: ' + str(e)

    def start(self, config = None, config_path = None):
        """Read the config and, if the link is on, start the socket threads.

        With no config given, the config file is found as described at the
        top of this file. Prints one line saying whether the link is on.

        Args:
            config (dict): Raw config to use instead of a file (tests).
            config_path (str): Name of that config, for messages.

        Returns:
            list: [on (bool), msg (str)].
        """
        try:
            return self.startLink(config, config_path)
        except Exception as e:
            self.reportError('start', e)
            return False, 'nepi_link did not start: ' + str(e)

    def startLink(self, config, config_path):
        with self.lock:
            if self.stop_event is not None:
                return self.settings is not None, 'Already started'
        load_msg = ''
        if config is None:
            config_path = find_config_file()
            if config_path is None:
                load_msg = 'No config file found'
            else:
                [config, load_msg] = load_config_file(config_path)
        [on, msg] = self.configure(config, config_path)
        if config is None:
            msg = load_msg + ': link off'
        self.printMessage('nepi_link: ' + msg)
        if not on:
            return False, msg
        with self.lock:
            demand_port = self.settings['NEPI_DEMAND_UDP_PORT']
            status_address = (self.settings['NEPI_LAN_IP'], self.settings['NEPI_STATUS_UDP_PORT'])
            stop_event = threading.Event()
            self.stop_event = stop_event
            self.assist_event.clear()
            self.threads = [threading.Thread(target = self.runDemandThread, args = [stop_event, demand_port],
                                             name = 'nepi_link_demand'),
                            threading.Thread(target = self.runStatusThread, args = [stop_event, status_address],
                                             name = 'nepi_link_status')]
            threads = list(self.threads)
        for thread in threads:
            thread.daemon = True
            thread.start()
        return True, msg

    def stop(self):
        """Stop the socket threads and close their sockets. For tests.

        Returns:
            list: [success (bool), msg (str)].
        """
        try:
            with self.lock:
                stop_event = self.stop_event
                threads = self.threads
                self.stop_event = None
                self.threads = []
            if stop_event is None:
                return True, 'Not started'
            stop_event.set()
            self.assist_event.set()
            for thread in threads:
                thread.join(SOCKET_TIMEOUT_SEC * 2 + 1.0)
            return True, 'Stopped'
        except Exception as e:
            self.reportError('stop', e)
            return False, 'Stop error: ' + str(e)

    def merge_inputs(self, gamepad_inputs):
        """Merge NEPI's newest demand into a copy of the pilot's inputs.

        Args:
            gamepad_inputs (list): The 18 pilot inputs, in gamepad_map order.
                Never modified.

        Returns:
            tuple: The inputs to encode for the Teensy. tuple(gamepad_inputs)
                unchanged while the link or NEPI control is off, and on any
                error. In NEPI control, the movement inputs carry NEPI's
                demand, or neutral while the demand is older than 250 ms.
        """
        try:
            with self.lock:
                merged = self.mergeLocked(gamepad_inputs, self.clock())
                pending = self.pending_msgs
                self.pending_msgs = []
            for [key, msg] in pending:
                self.printLimited(key, msg)
            return merged
        except Exception as e:
            self.reportError('merge_inputs', e)
            return safeTuple(gamepad_inputs)

    def mergeLocked(self, gamepad_inputs, now):
        pilot = tuple(gamepad_inputs)
        settings = self.settings
        if settings is None:
            return pilot
        self.updatePilotLocked(pilot, now)
        self.checkTimeoutLocked(now)
        merged = pilot
        if self.assist:
            # NEPI owns every movement input. The pilot's own are all short of
            # the takeover point here (past it, updatePilotLocked ended NEPI
            # control), so they are replaced, not mixed in. A stale demand
            # leaves every axis neutral, and RY at 0 lets the Teensy hold depth.
            out = list(pilot)
            for slot in [LY_SLOT, LX_SLOT, RY_SLOT, LT_SLOT, RT_SLOT]:
                out[slot] = 0
            fresh = self.demandFreshLocked(now)
            # A servo input the pilot holds off neutral stays the pilot's.
            # Otherwise it carries NEPI's value, or neutral while the demand
            # is stale or has no value for it.
            for servo in settings['servos']:
                if self.servo_pilot[servo['name']]:
                    continue
                value = None
                if fresh:
                    value = self.demand['servos'].get(servo['name'])
                if value is None:
                    out[servo['slot']] = servo['neutral']
                else:
                    out[servo['slot']] = servo_value(value, servo['min'], servo['neutral'], servo['max'])
            if fresh:
                demand = self.demand
                out[LY_SLOT] = stick_value(clampDemand(demand['surge'], settings['NEPI_MAX_SURGE']) *
                                           settings['NEPI_SURGE_SIGN'])
                out[LX_SLOT] = stick_value(clampDemand(demand['sway'], settings['NEPI_MAX_SWAY']) *
                                           settings['NEPI_SWAY_SIGN'])
                out[RY_SLOT] = stick_value(clampDemand(demand['heave'], settings['NEPI_MAX_HEAVE']) *
                                           settings['NEPI_HEAVE_SIGN'])
                deadzone = settings['NEPI_TRIGGER_DEADZONE']
                maximum = settings['NEPI_TRIGGER_MAX']
                yaw = clampDemand(demand['yaw'], settings['NEPI_MAX_YAW']) * settings['NEPI_YAW_SIGN']
                if yaw > 0:
                    out[LT_SLOT] = trigger_value(yaw, deadzone, maximum)
                elif yaw < 0:
                    out[RT_SLOT] = trigger_value(-yaw, deadzone, maximum)
            merged = tuple(out)
        self.out_lx = merged[LX_SLOT]
        self.out_ly = merged[LY_SLOT]
        self.out_ry = merged[RY_SLOT]
        self.out_lt = merged[LT_SLOT]
        self.out_rt = merged[RT_SLOT]
        return merged

    def updatePilotLocked(self, pilot, now):
        settings = self.settings
        deadzone = settings['NEPI_TRIGGER_DEADZONE']
        self.pilot_surge = abs(pilot[LY_SLOT]) >= STICK_DEADZONE
        self.pilot_sway = abs(pilot[LX_SLOT]) >= STICK_DEADZONE
        self.pilot_yaw = pilot[LT_SLOT] >= deadzone or pilot[RT_SLOT] >= deadzone
        self.pilot_depth = abs(pilot[RY_SLOT]) >= STICK_DEADZONE
        for servo in settings['servos']:
            self.servo_pilot[servo['name']] = abs(pilot[servo['slot']] - servo['neutral']) > servo['deadzone']
        # Slow mode, with the Teensy's "button available" rule (new_ctl.ino:364-379)
        if pilot[LB_SLOT]:
            if self.slow_btn_avl:
                self.slow = not self.slow
            self.slow_btn_avl = False
        else:
            self.slow_btn_avl = True
        # The safety exit: any movement input past the takeover point ends NEPI control
        takeover = self.takeoverInputLocked(pilot)
        if self.assist and takeover is not None:
            self.setAssistLocked(False, 'pilot_stick')
            self.pending_msgs.append(['takeover', 'nepi_link: NEPI control off: the pilot moved ' + takeover])
        # The assist button toggles NEPI control, same "button available" pattern
        if pilot[settings['assist_slot']]:
            if self.assist_btn_avl:
                self.toggleAssistLocked(takeover, now)
            self.assist_btn_avl = False
        else:
            self.assist_btn_avl = True

    def takeoverInputLocked(self, pilot):
        # The name of the first movement input at or past the takeover point, or None
        stick = self.settings['takeover_stick']
        trigger = self.settings['takeover_trigger']
        for [name, slot] in [['LY', LY_SLOT], ['LX', LX_SLOT], ['RY', RY_SLOT]]:
            if abs(pilot[slot]) >= stick:
                return name
        for [name, slot] in [['LT', LT_SLOT], ['RT', RT_SLOT]]:
            if pilot[slot] >= trigger:
                return name
        return None

    def toggleAssistLocked(self, takeover, now):
        if self.assist:
            self.setAssistLocked(False, 'pilot_button')
        elif not self.nepiReadyLocked(now):
            self.pending_msgs.append(['button', 'nepi_link: RB ignored: NEPI is not connected'])
        elif takeover is not None:
            self.pending_msgs.append(['button', 'nepi_link: RB ignored: release ' + takeover + ' first'])
        else:
            self.setAssistLocked(True, 'pilot_button')

    def setAssistLocked(self, on, reason):
        if self.assist == on:
            return
        self.assist = on
        self.assist_reason = reason
        self.assist_event.set()

    def checkTimeoutLocked(self, now):
        if self.assist and (self.demand_time is None or now - self.demand_time >= ASSIST_TIMEOUT_SEC):
            self.setAssistLocked(False, 'timeout')

    def demandFreshLocked(self, now):
        return self.demand_time is not None and now - self.demand_time <= DEMAND_STALE_SEC

    def nepiReadyLocked(self, now):
        return (self.demand is not None and self.demand['active'] and
                now - self.demand_time <= NEPI_READY_SEC)

    def handle_demand(self, datagram, source_ip):
        """Validate one demand datagram and apply it. Active false ends NEPI control.

        Args:
            datagram (bytes): The UDP payload.
            source_ip (str): The sender's address.

        Returns:
            list: [accepted (bool), msg (str)]. A dropped datagram adds 1 to
                demand_bad and changes nothing else.
        """
        try:
            with self.lock:
                return self.handleDemandLocked(datagram, source_ip, self.clock())
        except Exception as e:
            self.reportError('handle_demand', e)
            return False, 'Demand error: ' + str(e)

    def handleDemandLocked(self, datagram, source_ip, now):
        if self.settings is None:
            return False, 'Link off'
        [demand, msg] = parse_demand(datagram, source_ip, self.settings['NEPI_LAN_IP'])
        if demand is None:
            self.demand_bad += 1
            return False, msg
        if (self.demand_seq >= 0 and demand['seq'] <= self.demand_seq and
                now - self.demand_time <= SEQ_RESTART_SEC):
            self.demand_bad += 1
            return False, ('Demand dropped: seq ' + str(demand['seq']) + ' is not after ' +
                           str(self.demand_seq))
        self.demand = demand
        self.demand_seq = demand['seq']
        self.demand_time = now
        # Active true only marks NEPI ready; only the pilot's button turns NEPI control on
        if not demand['active']:
            self.setAssistLocked(False, 'nepi_off')
        return True, msg

    def on_teensy_line(self, line):
        """Record one line read from the Teensy. Never changes the line.

        Args:
            line (str): The decoded line. Anything but a 20-value $TNCTL line
                adds 1 to tn_bad.
        """
        try:
            with self.lock:
                if self.settings is None:
                    return
            values = parse_teensy_line(line)
            with self.lock:
                if values is None:
                    self.tn_bad += 1
                else:
                    self.tn = values
                    self.tn_time = self.clock()
                    self.tn_count += 1
        except Exception as e:
            self.reportError('on_teensy_line', e)

    def topside_tag(self):
        """Return the NEPI control state for the pilot's $RPCTL line.

        Returns:
            str: ',NEPI:ON', ',NEPI:WAIT' (NEPI control on, demand older than
                250 ms), ',NEPI:OFF' (off, NEPI ready, so RB hands over) or
                ',NEPI:NOTREADY' (off, NEPI not connected, so RB does nothing).
                An empty string when the link is off or on any error, so the
                pilot's line is unchanged.
        """
        try:
            with self.lock:
                if self.settings is None:
                    return ''
                now = self.clock()
                self.checkTimeoutLocked(now)
                if not self.assist:
                    if self.nepiReadyLocked(now):
                        return ',NEPI:OFF'
                    return ',NEPI:NOTREADY'
                if self.demandFreshLocked(now):
                    return ',NEPI:ON'
                return ',NEPI:WAIT'
        except Exception as e:
            self.reportError('topside_tag', e)
            return ''

    def build_status(self):
        """Build the status that would be sent now, without advancing seq.

        Returns:
            dict: Every LINK_PROTOCOL "Status" field plus assist_reason, or
                None when the link is off or on any error.
        """
        try:
            with self.lock:
                if self.settings is None:
                    return None
                return self.buildStatusLocked(self.clock())
        except Exception as e:
            self.reportError('build_status', e)
            return None

    def buildStatusLocked(self, now):
        self.checkTimeoutLocked(now)
        trigger_max = self.settings['NEPI_TRIGGER_MAX']
        tn = None
        if self.tn is not None:
            tn = dict(self.tn)
        # Each NEPI servo by name: whether the pilot holds its input
        servos = dict((servo['name'], self.servo_pilot.get(servo['name'], False))
                      for servo in self.settings['servos'])
        # The out_* ranges are part of the contract, so a pilot value past
        # them (a controller or config mismatch) is reported at the limit.
        return dict(v = 1, type = 'status', seq = self.status_seq, t = round(self.wall_clock(), 3),
                    assist = self.assist, assist_reason = self.assist_reason, nepi_ready = self.nepiReadyLocked(now),
                    pilot_surge = self.pilot_surge, pilot_sway = self.pilot_sway,
                    pilot_yaw = self.pilot_yaw, pilot_depth = self.pilot_depth, slow = self.slow,
                    out_lx = clampInt(self.out_lx, -32768, 32767), out_ly = clampInt(self.out_ly, -32768, 32767),
                    out_ry = clampInt(self.out_ry, -32768, 32767),
                    out_lt = clampInt(self.out_lt, 0, trigger_max), out_rt = clampInt(self.out_rt, 0, trigger_max),
                    demand_seq = self.demand_seq, demand_age_ms = ageMs(now, self.demand_time),
                    demand_bad = self.demand_bad,
                    tn = tn, tn_age_ms = ageMs(now, self.tn_time), tn_count = self.tn_count, tn_bad = self.tn_bad,
                    servos = servos)

    def nextStatusDatagram(self):
        with self.lock:
            if self.settings is None:
                return None
            status = self.buildStatusLocked(self.clock())
            self.status_seq = (self.status_seq + 1) & SEQ_MAX
        datagram = encode_status(status)
        if len(datagram) > MAX_DATAGRAM_BYTES:
            raise ValueError('status of ' + str(len(datagram)) + ' bytes is over ' + str(MAX_DATAGRAM_BYTES))
        return datagram


    def runDemandThread(self, stop_event, port):
        sock = None
        while not stop_event.is_set():
            try:
                if sock is None:
                    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                    sock.bind(('', port))
                    sock.settimeout(SOCKET_TIMEOUT_SEC)
                try:
                    [datagram, sender] = sock.recvfrom(65536)
                except socket.timeout:
                    continue
                [accepted, msg] = self.handle_demand(datagram, sender[0])
                if not accepted:
                    self.printLimited('demand', 'nepi_link: ' + msg)
            except Exception as e:
                if sock is not None:
                    sock.close()
                    sock = None
                if stop_event.is_set():
                    break
                self.reportError('demand receiver on UDP ' + str(port), e)
                stop_event.wait(SOCKET_RETRY_SEC)
        if sock is not None:
            sock.close()

    def runStatusThread(self, stop_event, address):
        sock = None
        next_send = time.monotonic()
        while not stop_event.is_set():
            wait_sec = next_send - time.monotonic()
            if wait_sec > 0:
                self.assist_event.wait(wait_sec)
            if stop_event.is_set():
                break
            self.assist_event.clear()
            next_send = time.monotonic() + STATUS_PERIOD_SEC
            try:
                if sock is None:
                    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                datagram = self.nextStatusDatagram()
                if datagram is not None:
                    sock.sendto(datagram, address)
            except Exception as e:
                self.reportError('status sender to ' + address[0] + ':' + str(address[1]), e)
        if sock is not None:
            sock.close()


    def reportError(self, where, error):
        self.printLimited('error', 'nepi_link: ' + where + ' error: ' + str(error))

    def printLimited(self, key, msg):
        # Never call this while holding the lock
        try:
            with self.lock:
                now = self.clock()
                last = self.last_print.get(key)
                if last is not None and now - last < ERROR_PRINT_SEC:
                    self.suppressed[key] = self.suppressed.get(key, 0) + 1
                    return
                self.last_print[key] = now
                skipped = self.suppressed.pop(key, 0)
            if skipped > 0:
                msg += ' (' + str(skipped) + ' more not shown)'
            self.log(msg)
        except Exception:
            pass

    def printMessage(self, msg):
        try:
            self.log(msg)
        except Exception:
            pass


#**********************
# The team's script calls these

_bridge = NepiLink()

def start():
    """Read the config and start the link. Does nothing when the link is off."""
    try:
        _bridge.start()
    except Exception:
        pass

def stop():
    """Stop the link's threads and close its sockets. For tests."""
    try:
        _bridge.stop()
    except Exception:
        pass

def merge_inputs(gamepad_inputs):
    """Return the inputs to encode for the Teensy, with NEPI's demand merged in.

    Args:
        gamepad_inputs (list): The pilot's 18 inputs. Never modified.

    Returns:
        tuple: The merged inputs, or tuple(gamepad_inputs) unchanged when the
            link or NEPI control is off, or on any error.
    """
    try:
        return _bridge.merge_inputs(gamepad_inputs)
    except Exception:
        return safeTuple(gamepad_inputs)

def on_teensy_line(line):
    """Record a line read from the Teensy for NEPI's status. Never changes it.

    Args:
        line (str): The decoded Teensy line.
    """
    try:
        _bridge.on_teensy_line(line)
    except Exception:
        pass

def topside_tag():
    """Return ',NEPI:ON', ',NEPI:WAIT', ',NEPI:OFF' or ',NEPI:NOTREADY' for the pilot's line.

    Returns:
        str: The tag, or an empty string when the link is off or on any error.
    """
    try:
        return _bridge.topside_tag()
    except Exception:
        return ''
