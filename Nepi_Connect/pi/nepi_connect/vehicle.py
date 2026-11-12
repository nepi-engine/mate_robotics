##
## Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
##
## This file is part of nepi-engine
## (see https://github.com/nepi-engine).
##
## License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
##

# The vehicle file: finding it, loading it, checking every value, and the
# vehicle message built from it (Nepi_Connect/VEHICLE_FILE.md).
#
# validate_vehicle() turns the raw YAML into the validated vehicle every
# other module reads. Any invalid value gives None and one message naming the
# key, and the link stays off. Inputs, axes and servos come out as dicts that
# carry everything the merge needs, so the merge never looks a name up again.

import ipaddress
import math
import os
import re

from . import link
from . import telemetry as telemetry_mod


VEHICLE_FILE_NAME = 'nepi_connect_vehicle.yaml'
VEHICLE_ENV = 'NEPI_CONNECT_VEHICLE'
HOME_VEHICLE_PATH = os.path.join('nepi_connect', 'config', VEHICLE_FILE_NAME)
# Version 1 config files, found only to say they need converting
LEGACY_ENV = 'NEPI_LINK_CONFIG'
LEGACY_FILE_NAME = 'nepi_link.yaml'
CONVERT_HINT = 'Nepi_Connect/VEHICLE_FILE.md, "Converting a Version 1 File"'
PYYAML_MISSING = 'PyYAML missing: sudo apt install python3-yaml'

VEHICLE_NAME = re.compile(r'^[a-z][a-z0-9_]{0,23}$')
SERVO_NAME = re.compile(r'^[a-z][a-z0-9_]{0,15}$')
CAMERA_NAME = re.compile(r'^[a-z][a-z0-9_]{0,15}$')
INPUT_NAME = re.compile(r'^[A-Za-z][A-Za-z0-9_]{0,23}$')
FIELD_NAME = re.compile(r'^[a-z][a-z0-9_]{0,23}$')
RESOLUTION = re.compile(r'^[1-9][0-9]{1,4}x[1-9][0-9]{1,4}$')
PRINTABLE = re.compile(r'^[\x20-\x7e]*$')

MAX_KIND_CHARS = 32
MAX_LABEL_CHARS = 24
MAX_INPUTS = 64
MAX_SLOT = 255
MAX_SERVOS = 4 # More would not fit the status
MAX_FIELDS = 64
MAX_CAMERAS = 4
MAX_PREFIX_CHARS = 16
MAX_TRAILER_CHARS = 64
MAX_FPS = 120

ADAPTERS = ['hooks']
AXIS_KINDS = ['stick', 'trigger_pair']
FIELD_TYPES = ['int', 'float']
CAMERA_TRANSPORTS = ['http', 'fastmjpg']

DEFAULT_STATUS_PORT = 5700
DEFAULT_DEMAND_PORT = 5701
DEFAULT_TAKEOVER_FRACTION = 0.25
DEFAULT_RESOLUTION = '1280x720'
DEFAULT_FPS = 30

# The keys each section may hold
TOP_KEYS = ['vehicle', 'network', 'control', 'telemetry', 'cameras']
VEHICLE_KEYS = ['name', 'kind']
NETWORK_KEYS = ['nepi_ip', 'status_port', 'demand_port']
CONTROL_KEYS = ['adapter', 'toggle_input', 'takeover_fraction', 'reserved_inputs', 'inputs', 'axes', 'servos']
INPUT_KEYS = ['slot', 'min', 'neutral', 'max', 'deadzone', 'label']
STICK_KEYS = ['kind', 'input', 'sign', 'cap']
TRIGGER_PAIR_KEYS = ['kind', 'inputs', 'sign', 'cap']
SERVO_KEYS = ['input', 'min', 'neutral', 'max', 'deadzone']
TELEMETRY_KEYS = ['format', 'prefix', 'trailer', 'fields', 'nav', 'raw', 'imu_mount']
FIELD_KEYS = ['name', 'type']
CAMERA_KEYS = ['name', 'transport', 'port', 'device', 'resolution', 'fps']
HTTP_ONLY_KEYS = ['device', 'resolution', 'fps']

# IMU board mounts: where the board's printed X and Y arrows point in the
# robot, which must be perpendicular. The NEPI driver's ImuMount uses the same
# names.
IMU_DIRECTIONS = [('FORWARD', (1, 0, 0)), ('BACK', (-1, 0, 0)), ('LEFT', (0, 1, 0)),
                  ('RIGHT', (0, -1, 0)), ('UP', (0, 0, 1)), ('DOWN', (0, 0, -1))]
IMU_MOUNT_OPTIONS = ['X_' + x_name + '_Y_' + y_name
                     for (x_name, x_dir) in IMU_DIRECTIONS for (y_name, y_dir) in IMU_DIRECTIONS
                     if sum(a * b for (a, b) in zip(x_dir, y_dir)) == 0]
IMU_MOUNT_FLAT = 'X_FORWARD_Y_LEFT'

WORST_T = 9999999999.999 # The longest status `t` until the year 2286


class InvalidVehicle(Exception):
    pass


#**********************
# Finding and loading the file

def vehicle_candidates(module_dir = None, environ = None, home = None):
    """List the vehicle file paths to try, in lookup order.

    Args:
        module_dir (str): The folder that holds nepi_link.py (default: the
            folder that holds this package).
        environ (dict): Environment to read NEPI_CONNECT_VEHICLE from (default: os.environ).
        home (str): Home folder (default: the user's).

    Returns:
        list: Paths, first match wins.
    """
    if module_dir is None:
        module_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    if environ is None:
        environ = os.environ
    if home is None:
        home = os.path.expanduser('~')
    candidates = []
    env_path = environ.get(VEHICLE_ENV, '')
    if env_path != '':
        candidates.append(env_path)
    candidates.append(os.path.join(module_dir, VEHICLE_FILE_NAME))
    candidates.append(os.path.join(home, HOME_VEHICLE_PATH))
    return candidates

def find_vehicle_file(module_dir = None, environ = None, home = None):
    """Return the first vehicle file that exists.

    Args:
        module_dir, environ, home: As in vehicle_candidates().

    Returns:
        str: The path, or None when no candidate exists.
    """
    for path in vehicle_candidates(module_dir, environ, home):
        if os.path.isfile(path):
            return path
    return None

def find_legacy_config(module_dir = None, environ = None):
    """Return a version 1 nepi_link config file, if one exists.

    Args:
        module_dir, environ: As in vehicle_candidates().

    Returns:
        str: The path, or None.
    """
    if module_dir is None:
        module_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    if environ is None:
        environ = os.environ
    candidates = []
    env_path = environ.get(LEGACY_ENV, '')
    if env_path != '':
        candidates.append(env_path)
    candidates.append(os.path.join(module_dir, LEGACY_FILE_NAME))
    for path in candidates:
        if os.path.isfile(path):
            return path
    return None

def importYaml():
    try:
        import yaml
        return yaml
    except Exception:
        return None

def load_vehicle_file(path, yaml_module = None):
    """Read one vehicle file with PyYAML.

    Args:
        path (str): The file.
        yaml_module (module): PyYAML, or None to import it (tests pass a stand-in).

    Returns:
        list: [raw (dict or None), msg (str)]. raw is None when PyYAML is
            missing or the file cannot be read or is not a YAML mapping.
    """
    if yaml_module is None:
        yaml_module = importYaml()
    if yaml_module is None:
        return None, PYYAML_MISSING
    try:
        with open(path, 'r') as vehicle_file:
            text = vehicle_file.read()
    except (OSError, UnicodeDecodeError) as e:
        return None, 'Cannot read ' + path + ': ' + str(e) + ': link off'
    try:
        raw = yaml_module.safe_load(text)
    except Exception as e:
        return None, 'Cannot parse ' + path + ': ' + str(e) + ': link off'
    if not isinstance(raw, dict):
        return None, path + ' is not a YAML mapping: link off'
    return raw, 'Read ' + path

def load_vehicle(module_dir = None, environ = None, home = None, yaml_module = None):
    """Find and read the vehicle file, as the link does at start.

    Args:
        module_dir, environ, home: As in vehicle_candidates().
        yaml_module (module): As in load_vehicle_file().

    Returns:
        list: [raw (dict or None), path (str or None), msg (str)]. raw is None
            when no file is found, PyYAML is missing, or the file cannot be
            read; msg then says why.
    """
    path = find_vehicle_file(module_dir, environ, home)
    if path is None:
        legacy = find_legacy_config(module_dir, environ)
        if legacy is not None:
            return None, None, ('No vehicle file found, but ' + legacy + ' is a version 1 config: convert it to a '
                                'vehicle file (' + CONVERT_HINT + '): link off')
        return None, None, 'No vehicle file found: link off'
    [raw, msg] = load_vehicle_file(path, yaml_module)
    return raw, path, msg


#**********************
# Validation

def validate_vehicle(raw):
    """Check a raw vehicle file and build the validated vehicle.

    Args:
        raw (dict): The file's contents, e.g. from load_vehicle_file().

    Returns:
        list: [vehicle (dict or None), msg (str)]. vehicle is None when any
            value is invalid, with the key in msg. A valid file with an empty
            network.nepi_ip still gives the vehicle (its cameras are still
            useful), but the link stays off, and msg says so.
    """
    try:
        vehicle = buildVehicle(raw)
    except InvalidVehicle as e:
        return None, str(e) + ': link off'
    network = vehicle['network']
    if network['nepi_ip'] == '':
        return vehicle, 'network.nepi_ip is empty: link off'
    control = vehicle['control']
    servos_msg = ', no servos'
    if len(control['servos']) > 0:
        servos_msg = ', servos ' + ', '.join(servo['name'] + ' on ' + servo['input'] for servo in control['servos'])
    telemetry_msg = ', no telemetry'
    if vehicle['telemetry'] is not None:
        telemetry_msg = ', telemetry ' + str(len(vehicle['telemetry']['fields'])) + ' fields'
    return vehicle, ('On: vehicle ' + vehicle['name'] + ', NEPI ' + network['nepi_ip'] +
                     ', status to UDP ' + str(network['status_port']) +
                     ', demands on UDP ' + str(network['demand_port']) +
                     ', axes ' + ', '.join(axis['name'] for axis in control['axes']) +
                     ', takeover at ' + str(control['takeover_fraction']) + ' of travel' + servos_msg +
                     telemetry_msg)

def buildVehicle(raw):
    if not isinstance(raw, dict):
        raise InvalidVehicle('The vehicle file must be a YAML mapping')
    checkKeys(raw, TOP_KEYS, '')
    vehicle = dict()
    section = getMapping(raw, 'vehicle', 'vehicle', True)
    checkKeys(section, VEHICLE_KEYS, 'vehicle')
    name = section.get('name')
    if not isinstance(name, str) or not VEHICLE_NAME.match(name):
        raise InvalidVehicle('vehicle.name must be 1 to 24 lowercase letters, digits or _, starting with a letter')
    kind = section.get('kind', '')
    if kind is None:
        kind = ''
    if not isinstance(kind, str) or len(kind) > MAX_KIND_CHARS or not PRINTABLE.match(kind):
        raise InvalidVehicle('vehicle.kind must be text of up to ' + str(MAX_KIND_CHARS) + ' printable characters')
    vehicle['name'] = name
    vehicle['kind'] = kind
    vehicle['network'] = buildNetwork(getMapping(raw, 'network', 'network', True))
    vehicle['control'] = buildControl(getMapping(raw, 'control', 'control', True))
    vehicle['telemetry'] = None
    vehicle['imu_mount'] = IMU_MOUNT_FLAT
    if raw.get('telemetry') is not None:
        vehicle['telemetry'] = buildTelemetry(getMapping(raw, 'telemetry', 'telemetry', True))
        vehicle['imu_mount'] = vehicle['telemetry']['imu_mount']
    vehicle['cameras'] = buildCameras(raw.get('cameras'))
    # Every part is optional, but a robot with none of them has nothing to connect
    if (len(vehicle['control']['axes']) == 0 and len(vehicle['control']['servos']) == 0 and
            vehicle['telemetry'] is None and len(vehicle['cameras']) == 0):
        raise InvalidVehicle('The vehicle file describes nothing to connect: give it at least one of control.axes, '
                             'control.servos, telemetry or cameras')
    [vehicle_bytes, status_bytes] = worst_case_sizes(vehicle)
    vehicle['sizes'] = [vehicle_bytes, status_bytes]
    if vehicle_bytes > link.MAX_DATAGRAM_BYTES or status_bytes > link.MAX_DATAGRAM_BYTES:
        raise InvalidVehicle('The vehicle file allows a vehicle message of up to ' + str(vehicle_bytes) +
                             ' bytes and a status of up to ' + str(status_bytes) + ' bytes; each must fit in ' +
                             str(link.MAX_DATAGRAM_BYTES) + ' (use shorter names, fewer raw fields or fewer servos)')
    return vehicle

def buildNetwork(section):
    checkKeys(section, NETWORK_KEYS, 'network')
    nepi_ip = section.get('nepi_ip', '')
    if nepi_ip is None:
        nepi_ip = ''
    if not isinstance(nepi_ip, str):
        raise InvalidVehicle('network.nepi_ip must be an IPv4 address, or empty')
    nepi_ip = nepi_ip.strip()
    if nepi_ip != '':
        try:
            nepi_ip = str(ipaddress.IPv4Address(nepi_ip))
        except ValueError:
            raise InvalidVehicle('network.nepi_ip ' + repr(nepi_ip) + ' is not an IPv4 address')
    network = dict(nepi_ip = nepi_ip)
    for [key, default] in [['status_port', DEFAULT_STATUS_PORT], ['demand_port', DEFAULT_DEMAND_PORT]]:
        port = section.get(key, default)
        if not link.isInteger(port) or port < 1 or port > 65535:
            raise InvalidVehicle('network.' + key + ' must be a port number from 1 to 65535')
        network[key] = port
    return network

def buildControl(section):
    checkKeys(section, CONTROL_KEYS, 'control')
    adapter = section.get('adapter', 'hooks')
    if adapter not in ADAPTERS:
        raise InvalidVehicle('control.adapter must be ' + ' or '.join(ADAPTERS))
    inputs = buildInputs(getMapping(section, 'inputs', 'control.inputs', True))
    reserved = section.get('reserved_inputs', [])
    if reserved is None:
        reserved = []
    if (not isinstance(reserved, list) or any(name not in inputs for name in reserved) or
            len(set(reserved)) != len(reserved)):
        raise InvalidVehicle('control.reserved_inputs must be a list of different inputs in control.inputs')
    axes = buildAxes(getMapping(section, 'axes', 'control.axes', False), inputs, reserved)
    axis_inputs = dict((stick['name'], axis['name']) for axis in axes for stick in axis['inputs'])
    fraction = section.get('takeover_fraction', DEFAULT_TAKEOVER_FRACTION)
    if not link.isNumber(fraction) or fraction <= 0 or fraction > 1:
        raise InvalidVehicle('control.takeover_fraction must be a number above 0 and at most 1')
    fraction = float(fraction)
    for axis in axes:
        for axis_input in axis['inputs']:
            setTakeover(axis_input, fraction)
    toggle_name = section.get('toggle_input')
    if not isinstance(toggle_name, str) or toggle_name not in inputs:
        raise InvalidVehicle('control.toggle_input must name an input in control.inputs')
    if toggle_name in axis_inputs:
        raise InvalidVehicle('control.toggle_input ' + toggle_name + ' is already the ' + axis_inputs[toggle_name] +
                             ' axis input')
    if toggle_name in reserved:
        raise InvalidVehicle('control.toggle_input ' + toggle_name + ' is in control.reserved_inputs')
    toggle = inputs[toggle_name]
    if max(toggle['max'] - toggle['neutral'], toggle['neutral'] - toggle['min']) < toggle['deadzone']:
        raise InvalidVehicle('control.toggle_input ' + toggle_name + ' cannot reach its dead zone')
    # A servo may use any input no axis, the toggle or the reserved list claims
    allowed = [name for name in inputs if name not in axis_inputs and name != toggle_name and name not in reserved]
    servos = buildServos(section.get('servos'), inputs, allowed)
    return dict(adapter = adapter, toggle = toggle, takeover_fraction = fraction, reserved = list(reserved),
                inputs = inputs, axes = axes, servos = servos, servo_inputs = allowed)

def buildInputs(section):
    if len(section) < 1 or len(section) > MAX_INPUTS:
        raise InvalidVehicle('control.inputs must list 1 to ' + str(MAX_INPUTS) + ' inputs')
    inputs = dict()
    slots = dict()
    for [name, entry] in section.items():
        path = 'control.inputs.' + str(name)
        if not isinstance(name, str) or not INPUT_NAME.match(name):
            raise InvalidVehicle(path + ': an input name must be 1 to 24 letters, digits or _, starting with a letter')
        if not isinstance(entry, dict):
            raise InvalidVehicle(path + ' must be a mapping of slot, min, neutral, max and deadzone')
        checkKeys(entry, INPUT_KEYS, path)
        values = dict()
        for key in ['slot', 'min', 'neutral', 'max', 'deadzone']:
            if not link.isInteger(entry.get(key)):
                raise InvalidVehicle(path + '.' + key + ' must be an integer')
            values[key] = entry[key]
        if values['slot'] < 0 or values['slot'] > MAX_SLOT:
            raise InvalidVehicle(path + '.slot must be an integer from 0 to ' + str(MAX_SLOT))
        if values['slot'] in slots:
            raise InvalidVehicle(path + '.slot ' + str(values['slot']) + ' is already used by ' + slots[values['slot']])
        slots[values['slot']] = name
        if not values['min'] <= values['neutral'] <= values['max'] or values['min'] == values['max']:
            raise InvalidVehicle(path + ' needs min <= neutral <= max and min < max')
        if values['deadzone'] < 1:
            raise InvalidVehicle(path + '.deadzone must be an integer, 1 or more')
        label = entry.get('label', name)
        if label is None:
            label = name
        if (not isinstance(label, str) or len(label) < 1 or len(label) > MAX_LABEL_CHARS or
                not PRINTABLE.match(label)):
            raise InvalidVehicle(path + '.label must be 1 to ' + str(MAX_LABEL_CHARS) + ' printable characters')
        inputs[name] = dict(name = name, slot = values['slot'], min = values['min'], neutral = values['neutral'],
                            max = values['max'], deadzone = values['deadzone'], label = label,
                            takeover_high = None, takeover_low = None)
    return inputs

def buildAxes(section, inputs, reserved):
    # No axes is allowed: a robot with only servos or sensors has none
    for name in section:
        if name not in link.AXIS_NAMES:
            raise InvalidVehicle('control.axes.' + str(name) + ' is not an axis; axes are ' +
                                 ', '.join(link.AXIS_NAMES))
    axes = []
    used = dict()
    # Always in AXIS_NAMES order, which decides which input a takeover names
    for name in link.AXIS_NAMES:
        if name not in section:
            continue
        path = 'control.axes.' + name
        entry = section[name]
        if not isinstance(entry, dict):
            raise InvalidVehicle(path + ' must be a mapping of kind, input or inputs, sign and cap')
        kind = entry.get('kind')
        if kind not in AXIS_KINDS:
            raise InvalidVehicle(path + '.kind must be ' + ' or '.join(AXIS_KINDS))
        if kind == 'stick':
            checkKeys(entry, STICK_KEYS, path)
            input_name = entry.get('input')
            if not isinstance(input_name, str) or input_name not in inputs:
                raise InvalidVehicle(path + '.input must name an input in control.inputs')
            stick = inputs[input_name]
            if not stick['min'] < stick['neutral'] < stick['max']:
                raise InvalidVehicle(path + '.input ' + input_name + ' needs min < neutral < max for a stick')
            axis_inputs = [stick]
        else:
            checkKeys(entry, TRIGGER_PAIR_KEYS, path)
            names = entry.get('inputs')
            if (not isinstance(names, list) or len(names) != 2 or
                    any(not isinstance(item, str) or item not in inputs for item in names) or names[0] == names[1]):
                raise InvalidVehicle(path + '.inputs must list two different inputs in control.inputs, '
                                     'positive first')
            axis_inputs = [inputs[names[0]], inputs[names[1]]]
            for trigger in axis_inputs:
                if trigger['max'] <= trigger['neutral']:
                    raise InvalidVehicle(path + '.inputs ' + trigger['name'] + ' needs max > neutral for a trigger')
        for axis_input in axis_inputs:
            if axis_input['name'] in used:
                raise InvalidVehicle(path + ': input ' + axis_input['name'] + ' is already used by axis ' +
                                     used[axis_input['name']])
            if axis_input['name'] in reserved:
                raise InvalidVehicle(path + ': input ' + axis_input['name'] + ' is in control.reserved_inputs')
            used[axis_input['name']] = name
            # Each side the axis moves toward needs travel past the dead zone
            sides = [axis_input['max'] - axis_input['neutral']]
            if kind == 'stick':
                sides.append(axis_input['neutral'] - axis_input['min'])
            if min(sides) <= axis_input['deadzone']:
                raise InvalidVehicle(path + ': input ' + axis_input['name'] + ' needs more travel than its dead zone')
        sign = entry.get('sign')
        if not link.isInteger(sign) or sign not in (1, -1):
            raise InvalidVehicle(path + '.sign must be 1 or -1')
        cap = entry.get('cap')
        if not link.isNumber(cap) or cap < 0 or cap > 1:
            raise InvalidVehicle(path + '.cap must be a number from 0 to 1')
        axes.append(dict(name = name, kind = kind, inputs = axis_inputs, sign = sign, cap = float(cap)))
    return axes

def setTakeover(axis_input, fraction):
    # A movement input at or past these, either side of neutral, ends NEPI
    # control. A side with no travel never takes over.
    path = 'control.takeover_fraction'
    for [key, travel] in [['takeover_high', axis_input['max'] - axis_input['neutral']],
                          ['takeover_low', axis_input['neutral'] - axis_input['min']]]:
        if travel <= 0:
            continue
        point = int(math.ceil(fraction * travel))
        if point <= axis_input['deadzone']:
            raise InvalidVehicle(path + ' must put the takeover point past the dead zone of every axis input; ' +
                                 axis_input['name'] + "'s is " + str(point) + ' from neutral, inside its dead zone ' +
                                 str(axis_input['deadzone']))
        axis_input[key] = point

def buildServos(section, inputs, allowed):
    if section is None:
        section = dict()
    if not isinstance(section, dict):
        raise InvalidVehicle('control.servos must be a mapping of servo names')
    if len(section) > MAX_SERVOS:
        raise InvalidVehicle('control.servos may name at most ' + str(MAX_SERVOS) + ' servos')
    servos = []
    used = dict()
    for [name, entry] in section.items():
        path = 'control.servos.' + str(name)
        if not isinstance(name, str) or not SERVO_NAME.match(name):
            raise InvalidVehicle(path + ': a servo name must be 1 to 16 lowercase letters, digits or _, starting with '
                                 'a letter')
        if not isinstance(entry, dict):
            raise InvalidVehicle(path + ' must be a mapping with an input')
        checkKeys(entry, SERVO_KEYS, path)
        input_name = entry.get('input')
        if not isinstance(input_name, str) or input_name not in allowed:
            raise InvalidVehicle(path + '.input must be one of ' + ', '.join(sorted(allowed)))
        if input_name in used:
            raise InvalidVehicle(path + '.input ' + input_name + ' is already used by servo ' + used[input_name])
        used[input_name] = name
        servo_input = inputs[input_name]
        values = dict()
        for key in ['min', 'neutral', 'max', 'deadzone']:
            value = entry.get(key)
            if value is None:
                value = servo_input[key]
            if not link.isInteger(value):
                raise InvalidVehicle(path + '.' + key + ' must be an integer')
            values[key] = value
        if not values['min'] <= values['neutral'] <= values['max'] or values['min'] == values['max']:
            raise InvalidVehicle(path + ' needs min <= neutral <= max and min < max')
        if values['deadzone'] < 1:
            raise InvalidVehicle(path + '.deadzone must be an integer, 1 or more')
        servos.append(dict(name = name, input = input_name, slot = servo_input['slot'], min = values['min'],
                           neutral = values['neutral'], max = values['max'], deadzone = values['deadzone']))
    return servos

def buildTelemetry(section):
    checkKeys(section, TELEMETRY_KEYS, 'telemetry')
    if section.get('format') != 'csv_line':
        raise InvalidVehicle('telemetry.format must be csv_line')
    prefix = section.get('prefix', '')
    if prefix is None:
        prefix = ''
    if (not isinstance(prefix, str) or len(prefix) > MAX_PREFIX_CHARS or ',' in prefix or
            prefix != prefix.strip() or not PRINTABLE.match(prefix)):
        raise InvalidVehicle('telemetry.prefix must be up to ' + str(MAX_PREFIX_CHARS) +
                             ' printable characters with no comma or outer spaces')
    trailer_text = section.get('trailer', '')
    if trailer_text is None:
        trailer_text = ''
    if not isinstance(trailer_text, str) or len(trailer_text) > MAX_TRAILER_CHARS:
        raise InvalidVehicle('telemetry.trailer must be a regular expression of up to ' + str(MAX_TRAILER_CHARS) +
                             ' characters, or empty')
    trailer = None
    if trailer_text != '':
        try:
            trailer = re.compile(trailer_text)
        except re.error as e:
            raise InvalidVehicle('telemetry.trailer is not a valid regular expression: ' + str(e))
    raw_fields = section.get('fields')
    if not isinstance(raw_fields, list) or len(raw_fields) < 1 or len(raw_fields) > MAX_FIELDS:
        raise InvalidVehicle('telemetry.fields must list 1 to ' + str(MAX_FIELDS) + ' fields')
    fields = []
    field_types = dict()
    for [index, entry] in enumerate(raw_fields):
        path = 'telemetry.fields[' + str(index) + ']'
        if not isinstance(entry, dict):
            raise InvalidVehicle(path + ' must be a mapping of name and type')
        checkKeys(entry, FIELD_KEYS, path)
        name = entry.get('name')
        if not isinstance(name, str) or not FIELD_NAME.match(name):
            raise InvalidVehicle(path + '.name must be 1 to 24 lowercase letters, digits or _, starting with a letter')
        if name in field_types:
            raise InvalidVehicle(path + '.name ' + name + ' is already used')
        if entry.get('type') not in FIELD_TYPES:
            raise InvalidVehicle(path + '.type must be ' + ' or '.join(FIELD_TYPES))
        field_types[name] = entry['type']
        fields.append(dict(name = name, type = entry['type']))
    nav = buildNav(section.get('nav'), field_types)
    raw = section.get('raw', [])
    if raw is None:
        raw = []
    if (not isinstance(raw, list) or any(not isinstance(name, str) or name not in field_types for name in raw) or
            len(set(raw)) != len(raw)):
        raise InvalidVehicle('telemetry.raw must be a list of different field names from telemetry.fields')
    imu_mount = section.get('imu_mount', IMU_MOUNT_FLAT)
    if imu_mount not in IMU_MOUNT_OPTIONS:
        raise InvalidVehicle('telemetry.imu_mount must be X_<direction>_Y_<direction>, two perpendicular directions '
                             'out of FORWARD, BACK, LEFT, RIGHT, UP and DOWN, such as ' + IMU_MOUNT_FLAT)
    return dict(prefix = prefix, trailer = trailer, trailer_text = trailer_text, fields = fields,
                field_types = field_types, nav = nav, raw = list(raw), imu_mount = imu_mount)

def buildNav(section, field_types):
    # Returns [[standard name, field name]] in NAV_GROUPS order, each group whole
    if section is None:
        return []
    if not isinstance(section, dict):
        raise InvalidVehicle('telemetry.nav must be a mapping of groups')
    groups = dict(link.NAV_GROUPS)
    for group in section:
        if group not in groups:
            raise InvalidVehicle('telemetry.nav.' + str(group) + ' is not a group; groups are ' +
                                 ', '.join(groups))
    nav = []
    for [group, names] in link.NAV_GROUPS:
        if section.get(group) is None:
            continue
        path = 'telemetry.nav.' + group
        entry = section[group]
        if not isinstance(entry, dict) or sorted(entry) != sorted(names):
            raise InvalidVehicle(path + ' must map every one of ' + ', '.join(names) + ' to a field, and nothing else')
        for standard in names:
            field = entry[standard]
            if not isinstance(field, str) or field not in field_types:
                raise InvalidVehicle(path + '.' + standard + ' must name a field in telemetry.fields')
            nav.append([standard, field])
    return nav

def buildCameras(cameras):
    if cameras is None:
        return []
    if not isinstance(cameras, list) or len(cameras) > MAX_CAMERAS:
        raise InvalidVehicle('cameras must be a list of at most ' + str(MAX_CAMERAS) + ' cameras')
    result = []
    names = set()
    ports = set()
    for [index, entry] in enumerate(cameras):
        path = 'cameras[' + str(index) + ']'
        if not isinstance(entry, dict):
            raise InvalidVehicle(path + ' must be a mapping of name, transport and port')
        checkKeys(entry, CAMERA_KEYS, path)
        name = entry.get('name')
        if not isinstance(name, str) or not CAMERA_NAME.match(name):
            raise InvalidVehicle(path + '.name must be 1 to 16 lowercase letters, digits or _, starting with a letter')
        if name in names:
            raise InvalidVehicle(path + '.name ' + name + ' is already used')
        names.add(name)
        transport = entry.get('transport')
        if transport not in CAMERA_TRANSPORTS:
            raise InvalidVehicle(path + '.transport must be ' + ' or '.join(CAMERA_TRANSPORTS))
        port = entry.get('port')
        if not link.isInteger(port) or port < 1 or port > 65535:
            raise InvalidVehicle(path + '.port must be a port number from 1 to 65535')
        if port in ports:
            raise InvalidVehicle(path + '.port ' + str(port) + ' is already used')
        ports.add(port)
        camera = dict(name = name, transport = transport, port = port)
        if transport == 'http':
            device = entry.get('device', '')
            if device is None:
                device = ''
            if not isinstance(device, str) or not PRINTABLE.match(device) or ' ' in device:
                raise InvalidVehicle(path + '.device must be a device path with no spaces, or empty')
            resolution = entry.get('resolution', DEFAULT_RESOLUTION)
            if not isinstance(resolution, str) or not RESOLUTION.match(resolution):
                raise InvalidVehicle(path + '.resolution must be <width>x<height>, such as ' + DEFAULT_RESOLUTION)
            fps = entry.get('fps', DEFAULT_FPS)
            if not link.isInteger(fps) or fps < 1 or fps > MAX_FPS:
                raise InvalidVehicle(path + '.fps must be an integer from 1 to ' + str(MAX_FPS))
            camera.update(device = device, resolution = resolution, fps = fps)
        else:
            for key in HTTP_ONLY_KEYS:
                if key in entry:
                    raise InvalidVehicle(path + '.' + key + ' applies to http cameras only')
        result.append(camera)
    return result

def checkKeys(section, allowed, path):
    for key in section:
        if key not in allowed:
            where = str(key) if path == '' else path + '.' + str(key)
            raise InvalidVehicle(where + ' is not a known key (VEHICLE_FILE.md lists every key)')

def getMapping(section, key, path, required):
    value = section.get(key)
    if value is None:
        if required:
            raise InvalidVehicle(path + ' is missing')
        return dict()
    if not isinstance(value, dict):
        raise InvalidVehicle(path + ' must be a mapping')
    return value


#**********************
# The vehicle message and the size check

def build_vehicle_message(vehicle, seq, t):
    """Build the vehicle message (LINK_PROTOCOL "Vehicle") for a validated vehicle.

    Args:
        vehicle (dict): From validate_vehicle().
        seq (int), t (float): The common fields.

    Returns:
        dict: The vehicle message.
    """
    control = vehicle['control']
    telemetry = vehicle['telemetry']
    nav = []
    raw = []
    if telemetry is not None:
        nav = [standard for [standard, field] in telemetry['nav']]
        raw = list(telemetry['raw'])
    return dict(v = link.PROTOCOL_VERSION, type = 'vehicle', seq = seq, t = t, name = vehicle['name'],
                kind = vehicle['kind'], demand_port = vehicle['network']['demand_port'],
                axes = dict((axis['name'], axis['cap']) for axis in control['axes']),
                servos = [servo['name'] for servo in control['servos']], nav = nav, raw = raw,
                cameras = [dict(name = camera['name'], transport = camera['transport'], port = camera['port'])
                           for camera in vehicle['cameras']],
                imu_mount = vehicle['imu_mount'])

def worst_case_sizes(vehicle):
    """Return the longest vehicle message and status this vehicle can produce.

    Every counter, age and seq is at its longest, every flag false, every
    axis input at the longer end of its range, and every telemetry value at
    the longest its type can parse to.

    Args:
        vehicle (dict): A vehicle as validate_vehicle() builds it.

    Returns:
        list: [vehicle_bytes (int), status_bytes (int)].
    """
    vehicle_bytes = len(link.encode_message(build_vehicle_message(vehicle, link.SEQ_MAX, WORST_T)))
    control = vehicle['control']
    out = dict()
    for axis in control['axes']:
        for axis_input in axis['inputs']:
            out[axis_input['name']] = max([axis_input['min'], axis_input['max']], key = lambda value: len(str(value)))
    telemetry = vehicle['telemetry']
    nav = None
    raw = None
    has_raw = False
    if telemetry is not None:
        worst = dict()
        for field in telemetry['fields']:
            worst[field['name']] = telemetry_mod.WORST_INT if field['type'] == 'int' else telemetry_mod.WORST_FLOAT
        nav = telemetry_mod.nav_values(worst, telemetry)
        raw = telemetry_mod.raw_values(worst, telemetry)
        has_raw = len(telemetry['raw']) > 0
    status = link.build_status(seq = link.SEQ_MAX, t = WORST_T, control = False,
                               control_reason = max(link.CONTROL_REASONS, key = len), nepi_ready = False,
                               pilot = dict((axis['name'], False) for axis in control['axes']),
                               servos = dict((servo['name'], False) for servo in control['servos']), out = out,
                               demand_seq = link.SEQ_MAX, demand_age_ms = link.COUNT_MAX,
                               demand_bad = link.COUNT_MAX, nav = nav, nav_age_ms = link.COUNT_MAX,
                               nav_count = link.COUNT_MAX, nav_bad = link.COUNT_MAX, raw = raw, has_raw = has_raw)
    return [vehicle_bytes, len(link.encode_message(status))]
