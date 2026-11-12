##
## Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
##
## This file is part of nepi-engine
## (see https://github.com/nepi-engine).
##
## License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
##

# Demand to input value: the stick, trigger_pair and servo mappings
# (Nepi_Connect/VEHICLE_FILE.md "Mapping", LINK_PROTOCOL.md "Servos"). Every
# range, neutral and dead zone comes from the vehicle file.

import math


MIN_DEMAND = 0.01 # Smaller demands map to the input's neutral


def stick_value(demand, low, neutral, high, deadzone):
    """Map a demand fraction onto the part of a stick's range the robot acts on.

    The robot's code ignores the dead zone around neutral, so 0.01 to 1 maps
    evenly from just past the dead zone to the end of the range on the
    demand's side, and anything smaller becomes neutral.

    Args:
        demand (float): Fraction of full travel, -1 to 1.
        low (int), neutral (int), high (int): The input's raw range.
        deadzone (int): The input's dead zone.

    Returns:
        int: Input value, low to high.
    """
    magnitude = abs(demand)
    if magnitude < MIN_DEMAND:
        return neutral
    if demand < 0:
        travel = neutral - low
    else:
        travel = high - neutral
    offset = int(deadzone + (magnitude - MIN_DEMAND) / (1.0 - MIN_DEMAND) * (travel - deadzone) + 0.5)
    offset = min(offset, travel)
    if demand < 0:
        return neutral - offset
    return neutral + offset

def trigger_value(demand, neutral, high, deadzone):
    """Map a demand fraction onto the part of a trigger's range the robot acts on.

    Args:
        demand (float): Fraction of a full pull, 0 to 1.
        neutral (int): The trigger's resting value.
        high (int): Its value at a full pull.
        deadzone (int): Its dead zone.

    Returns:
        int: Input value, neutral or neutral + deadzone to high.
    """
    if demand < MIN_DEMAND:
        return neutral
    travel = high - neutral
    offset = int(deadzone + (demand - MIN_DEMAND) / (1.0 - MIN_DEMAND) * (travel - deadzone) + 0.5)
    return neutral + min(offset, travel)

def servo_value(demand, low, neutral, high):
    """Map a servo demand onto its input's raw range.

    0 is the input's neutral, 1 its high end and -1 its low end, each half
    scaled on its own, so a button (0, 0, 1) reads any negative demand as
    released. Rounds half away from neutral, so a three-way input (-1, 0, 1)
    is pressed for any demand of at least 0.5 either way.

    Args:
        demand (float): -1 to 1.
        low (int): The servo's min.
        neutral (int): The servo's neutral.
        high (int): The servo's max.

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

def clamp_demand(value, cap):
    """Limit a demand to +/-cap.

    Args:
        value (float): The demand.
        cap (float): The axis's cap, 0 to 1.

    Returns:
        float: value limited to -cap to cap.
    """
    return max(-cap, min(cap, value))

def apply_axis(axis, value, out):
    """Write one axis's demand into the robot's inputs.

    The demand is capped, multiplied by the axis's sign, and mapped onto its
    input (stick) or onto one of its two inputs (trigger_pair: the first for
    positive values, the second for negative ones). Inputs it does not write
    keep what out already holds.

    Args:
        axis (dict): A validated axis from vehicle.validate_vehicle().
        value (float): NEPI's demand on that axis, -1 to 1.
        out (list): The robot's inputs, changed in place.
    """
    signed = clamp_demand(value, axis['cap']) * axis['sign']
    if axis['kind'] == 'stick':
        stick = axis['inputs'][0]
        out[stick['slot']] = stick_value(signed, stick['min'], stick['neutral'], stick['max'], stick['deadzone'])
        return
    [positive, negative] = axis['inputs']
    if signed > 0:
        out[positive['slot']] = trigger_value(signed, positive['neutral'], positive['max'], positive['deadzone'])
    elif signed < 0:
        out[negative['slot']] = trigger_value(-signed, negative['neutral'], negative['max'], negative['deadzone'])
