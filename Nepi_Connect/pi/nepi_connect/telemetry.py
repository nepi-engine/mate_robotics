##
## Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
##
## This file is part of nepi-engine
## (see https://github.com/nepi-engine).
##
## License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
##

# The robot's telemetry line, parsed from the vehicle file's field list
# (Nepi_Connect/VEHICLE_FILE.md "telemetry"). The only format is csv_line:
# an optional prefix value, one value per field, and an optional trailer.

import re


# Values are bounded so the status stays under its size limit. Arduino prints
# "nan", "inf" or "ovf" for a number it cannot show, and those lines count as
# bad.
INT_TEXT = re.compile(r'^-?[0-9]{1,10}$')
FLOAT_TEXT = re.compile(r'^-?[0-9]{1,10}(\.[0-9]{1,6})?$')
INT32_MIN = -2147483648
INT32_MAX = 2147483647
# The longest JSON a parsed value can take: an int at INT32_MIN, and a float
# with 17 significant digits, 10 of them before the point. The size check
# builds its worst-case status from these.
WORST_INT = INT32_MIN
WORST_FLOAT = -1234567890.1234567


def parse_line(line, telemetry):
    """Parse one telemetry line.

    Args:
        line (str): A line from the robot, with or without its line ending.
        telemetry (dict): The vehicle's validated telemetry section.

    Returns:
        dict: Each field's value by name, as its type, or None when the line
            is not exactly the prefix, one value per field and the trailer.
    """
    if not isinstance(line, str):
        return None
    parts = line.strip().split(',')
    fields = telemetry['fields']
    first = 0
    end = len(parts)
    if telemetry['prefix'] != '':
        first = 1
    if telemetry['trailer'] is not None:
        end -= 1
    if end - first != len(fields):
        return None
    if first == 1 and parts[0] != telemetry['prefix']:
        return None
    if telemetry['trailer'] is not None and not telemetry['trailer'].match(parts[-1]):
        return None
    values = dict()
    for [field, text] in zip(fields, parts[first:end]):
        value = parseValue(text, field['type'])
        if value is None:
            return None
        values[field['name']] = value
    return values

def parseValue(text, field_type):
    if field_type == 'int':
        if not INT_TEXT.match(text):
            return None
        value = int(text)
        if value < INT32_MIN or value > INT32_MAX:
            return None
        return value
    if not FLOAT_TEXT.match(text):
        return None
    return float(text)

def nav_values(values, telemetry):
    """Pick the standard navigation values out of a parsed line.

    Args:
        values (dict): From parse_line().
        telemetry (dict): The vehicle's validated telemetry section.

    Returns:
        dict: Standard name to value, in announcement order.
    """
    return dict((standard, values[field]) for [standard, field] in telemetry['nav'])

def raw_values(values, telemetry):
    """Pick the raw fields out of a parsed line.

    Args:
        values (dict): From parse_line().
        telemetry (dict): The vehicle's validated telemetry section.

    Returns:
        dict: Field name to value, in the vehicle file's raw order.
    """
    return dict((name, values[name]) for name in telemetry['raw'])
