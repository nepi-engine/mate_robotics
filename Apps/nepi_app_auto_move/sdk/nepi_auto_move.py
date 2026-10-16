#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus <https://www.numurus.com>.
#
# This file is part of nepi engine (nepi_engine) repo
# (see https://github.com/nepi-engine/nepi_engine)
#
# License: NEPI Engine repo source-code and NEPI Images that use this source-code
# are licensed under the "Numurus Software License",
# which can be found at: <https://numurus.com/wp-content/uploads/Numurus-Software-License-Terms.pdf>
#
# Redistributions in source code must retain this top-level comment block.
# Plagiarizing this software to sidestep the license obligations is illegal.
#
# Contact Information:
# ====================
# - mailto:nepi@numurus.com
#

import copy
import math

from nepi_sdk import nepi_controls

from nepi_sdk.nepi_sdk import logger as Logger
logger = Logger()


########################
## Library Data

# Control definitions in the nepi_controls init-dict form. A node deep-copies
# this and hands it to a ControlsIF, which turns it into a live controls dict
# and passes that dict back to plan_move on every goto.
#
# Every default should sit inside its own bounds. nepi_controls.create_controls_dict
# CLAMPS an out-of-bounds Int or Float default to the nearer bound and registers
# the control -- it does not drop it, and it does not log. A default of 99.0 under
# bounds [0.0, 10.0] comes back as 10.0, so the operator sees a starting value
# nothing in this file asked for. Keep the defaults inside the bounds and the
# question does not arise.
#
# These are the controls a real planner needs. The placeholder planner below
# reads only max_step_m; the rest are declared now so the operator-facing
# surface does not move when the planner is filled in.
MOVE_CONTROLS_DICT = {

    'max_step_m': {
        'type': 'Float', 'default': 2.0, 'bounds': [0.1, 50.0], 'round_value': 2,
        'display_name': 'Max Step (m)',
        'description': 'Longest single move the plan may contain. A longer '
                       'request is split into several steps.',
        'display_hidden': False},

    'standoff_m': {
        'type': 'Float', 'default': 0.5, 'bounds': [0.0, 20.0], 'round_value': 2,
        'display_name': 'Standoff (m)',
        'description': 'Stop this far short of the clicked point.',
        'display_hidden': False},

    'clearance_m': {
        'type': 'Float', 'default': 0.5, 'bounds': [0.0, 20.0], 'round_value': 2,
        'display_name': 'Obstacle Clearance (m)',
        'description': 'Closest the planned path may pass to a known obstacle.',
        'display_hidden': False},

    'hold_altitude': {
        'type': 'Toggle', 'default': True,
        'display_name': 'Hold Altitude',
        'description': 'Keep the current height, ignoring the vertical part of '
                       'the clicked offset.',
        'display_hidden': False},

    'face_target': {
        'type': 'Toggle', 'default': False,
        'display_name': 'Face Target',
        'description': 'Turn to face the clicked bearing before moving.',
        'display_hidden': False},

    'avoid_obstacles': {
        'type': 'Toggle', 'default': False,
        'display_name': 'Avoid Obstacles',
        'description': 'Plan around known obstacles instead of moving straight '
                       'to the clicked point.',
        'display_hidden': False},

    }


#########################
# Move Planning
#########################

##############################################################################
# EXTENSION POINT
#
# plan_move() below is the placeholder. It is the ONLY thing a developer adding
# real move logic has to replace.
#
# WHAT TO CHANGE
#   The body of plan_move(). Read the inputs, return a list of move steps.
#   Everything a planner needs is already handed in -- the requested offset, the
#   current depth map, the object and target lists, the robot's reported state,
#   and the live control values. A planner that needs something not on that list
#   is a signal to widen this signature deliberately, not to reach for ROS from
#   inside this module.
#
# WHAT NOT TO CHANGE
#   The signature and the return shape. AutoMoveIF.runPlanning() calls this with
#   exactly these six arguments and hands each returned step straight to
#   ConnectRBXDeviceIF.goto_position(). Changing either without changing that
#   caller breaks the goto process silently -- a step with an unexpected key
#   simply moves zero on that axis.
#
#   Do not import ROS, nepi_sdk transport, or any interface class here. This
#   module is a pure function of its arguments so a test can call it with
#   fixtures and any other node can call it with the same six arguments.
#
#   Do not block. plan_move() runs on the goto process timer thread; a planner
#   that needs to iterate should return the steps it has and let the process
#   call it again on the next goto.
#
# UNITS
#   goto_dict, and every value in every returned step, are METRES and DEGREES.
#   np_depth_map is MILLIMETRES -- that is the platform contract for a depth map
#   array, and the *_range_m bounds on its status message are metres. Divide by
#   1000.0 at the point of use and comment the conversion; mixing the two
#   produces a plan that looks reasonable and is a thousand times wrong.
#
#   The frame is the robot body frame from nepi_interfaces/GotoPosition:
#   x forward, y LEFT, z up, yaw positive to port, all relative to where the
#   robot is when the step is issued -- steps compose, they are not absolute.
#
# STATUS FIELDS THE PROCESS UPDATES
#   The process owns every status field; a planner writes none of them. It sets
#   goto_state through idle -> planning -> moving -> complete (or cancelled),
#   and goto_step / goto_step_count from the length of the list returned here,
#   advancing one step per completed move. A step's 'description' is copied into
#   goto_msg while that step runs, so it is worth writing for an operator.
#
#   Returning an empty list is a legitimate answer -- "no move needed". The
#   process reports COMPLETE for it rather than treating it as a failure.
##############################################################################

def _control_value(controls_dict, name, default):
    """Read a value from either a live ControlsIF dict or the factory dict.

    Live dicts go through nepi_controls.get_value(), which is the only thing
    that returns a control's NATIVE value. Reading controls_dict[name]['value']
    directly does not: nepi_controls stores every value as a list of strings, so
    a Float came back as ['7.5'] -- float() of a list raises and _as_float fell
    back to the hardcoded default, discarding every edit the operator made --
    and a Toggle came back as ['False'], which is neither a bool, a str nor
    None, so _as_bool ended at bool(['False']) and a NON-EMPTY LIST IS TRUTHY.
    A Toggle switched off read as True. avoid_obstacles and face_target are both
    declared False, so the planner took paths nobody asked for and hold_altitude
    could not be turned off.

    The factory-dict path stays: MOVE_CONTROLS_DICT above is a plain init dict
    whose entries carry 'default' and no 'value', and plan_move() is documented
    to accept either form.
    """
    if controls_dict is None or name not in controls_dict:
        return default
    value = None
    try:
        value = nepi_controls.get_value(controls_dict, name)
    except Exception:
        value = None
    if value is not None:
        return value
    entry = controls_dict.get(name)
    if not isinstance(entry, dict):
        return entry
    # 'length' is set on every entry create_controls_dict() builds and on no
    # entry an init dict carries, so it is what tells the two apart. A LIVE
    # control that nepi_controls could not resolve has nothing readable here --
    # its 'default' is the same list of strings its 'value' is -- so the caller's
    # default stands rather than handing a list to _as_float / _as_bool.
    if 'length' in entry:
        return default
    # An init dict entry: 'default' is the authored native value.
    if 'default' in entry:
        return entry.get('default')
    if 'data' in entry:
        return entry.get('data')
    if 'value' in entry:
        return entry.get('value')
    return default


def _as_float(value, default):
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _as_bool(value, default):
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ['true', '1', 'yes', 'on']
    if value is None:
        return default
    return bool(value)


def _distance_3d(x_m, y_m, z_m):
    return math.sqrt((x_m * x_m) + (y_m * y_m) + (z_m * z_m))


def _make_step(x_m, y_m, z_m, yaw_deg, description):
    return {
        'x_m': x_m,
        'y_m': y_m,
        'z_m': z_m,
        'yaw_deg': yaw_deg,
        'description': description,
    }


def _split_translation_step(x_m, y_m, z_m, max_step_m, description):
    distance_m = _distance_3d(x_m, y_m, z_m)
    if distance_m <= 0.0:
        return []
    if max_step_m <= 0.0:
        max_step_m = distance_m
    step_count = int(math.ceil(distance_m / max_step_m))
    steps = []
    for i in range(step_count):
        ratio = 1.0 / float(step_count)
        step_description = description
        if step_count > 1:
            step_description = description + ' part ' + str(i + 1) + '/' + str(step_count)
        steps.append(_make_step(x_m * ratio, y_m * ratio, z_m * ratio, 0.0, step_description))
    return steps


def _entry_to_xy(entry):
    """Convert a range/bearing perception entry into body-frame x/y metres."""
    if not isinstance(entry, dict):
        return None
    range_m = _as_float(entry.get('range_m', None), -999.0)
    azimuth_deg = _as_float(entry.get('azimuth_deg', None), -999.0)
    if range_m <= 0.0 or range_m == -999.0 or azimuth_deg == -999.0:
        return None
    azimuth_rad = math.radians(azimuth_deg)
    x_m = range_m * math.cos(azimuth_rad)
    # Positive azimuth is to the camera/right side; body y is positive left.
    y_m = -range_m * math.sin(azimuth_rad)
    return {
        'name': str(entry.get('name', 'obstacle')),
        'x_m': x_m,
        'y_m': y_m,
        'range_m': range_m,
    }


def _collect_obstacles(obstacles_list, objects_list):
    obstacles = []
    for entry in obstacles_list or []:
        obstacle = _entry_to_xy(entry)
        if obstacle is not None:
            obstacle['source'] = 'obstacle'
            obstacles.append(obstacle)
    for entry in objects_list or []:
        obstacle = _entry_to_xy(entry)
        if obstacle is not None:
            obstacle['source'] = 'object'
            obstacles.append(obstacle)
    return obstacles


def _blocking_obstacles(dest_x, dest_y, obstacles, clearance_m):
    path_len = math.sqrt((dest_x * dest_x) + (dest_y * dest_y))
    if path_len <= 0.0:
        return []

    ux = dest_x / path_len
    uy = dest_y / path_len
    nx = -uy
    ny = ux

    blocked = []
    for obstacle in obstacles:
        ox = obstacle['x_m']
        oy = obstacle['y_m']
        along_m = (ox * ux) + (oy * uy)
        if along_m <= 0.0 or along_m >= path_len:
            continue
        lateral_m = (ox * nx) + (oy * ny)
        miss_m = abs(lateral_m)
        if miss_m < clearance_m:
            item = dict(obstacle)
            item['along_m'] = along_m
            item['lateral_m'] = lateral_m
            item['miss_m'] = miss_m
            blocked.append(item)
    blocked.sort(key = lambda item: item['along_m'])
    return blocked


def _choose_avoidance_side(blocked, obstacles, dest_x, dest_y, clearance_m):
    path_len = math.sqrt((dest_x * dest_x) + (dest_y * dest_y))
    ux = dest_x / path_len
    uy = dest_y / path_len
    nx = -uy
    ny = ux

    best_side = 1.0
    best_score = None
    for side in [-1.0, 1.0]:
        offset_m = side * (clearance_m + 0.5)
        score = None
        for obstacle in obstacles:
            along_m = (obstacle['x_m'] * ux) + (obstacle['y_m'] * uy)
            if along_m <= 0.0 or along_m >= path_len:
                continue
            lateral_m = (obstacle['x_m'] * nx) + (obstacle['y_m'] * ny)
            separation_m = abs(lateral_m - offset_m)
            if score is None or separation_m < score:
                score = separation_m
        if score is None:
            score = clearance_m + 0.5
        if best_score is None or score > best_score:
            best_score = score
            best_side = side

    if len(blocked) > 0:
        first_lateral = blocked[0].get('lateral_m', 0.0)
        if abs(first_lateral) > 0.01:
            best_side = -1.0 if first_lateral > 0.0 else 1.0
    return best_side


def _build_avoidance_waypoints(dest_x, dest_y, clearance_m, blocked, obstacles):
    path_len = math.sqrt((dest_x * dest_x) + (dest_y * dest_y))
    if path_len <= 0.0 or len(blocked) == 0:
        return [(dest_x, dest_y)]

    ux = dest_x / path_len
    uy = dest_y / path_len
    nx = -uy
    ny = ux
    side = _choose_avoidance_side(blocked, obstacles, dest_x, dest_y, clearance_m)
    lateral_offset_m = side * (clearance_m + 0.5)

    first_block = blocked[0]
    last_block = blocked[-1]
    before_m = max(first_block['along_m'] - clearance_m, 0.0)
    after_m = min(last_block['along_m'] + clearance_m, path_len)

    return [
        ((ux * before_m) + (nx * lateral_offset_m),
         (uy * before_m) + (ny * lateral_offset_m)),
        ((ux * after_m) + (nx * lateral_offset_m),
         (uy * after_m) + (ny * lateral_offset_m)),
        (dest_x, dest_y),
    ]


def plan_move(goto_dict, np_depth_map, objects_list, targets_list, robot_dict, controls_dict, obstacles_list = None):
    """Plan the sequence of relative moves that satisfies a goto request.

    Builds a conservative RBX goto_position plan. The planner can stop short of
    the clicked point, hold altitude, split long moves into smaller steps, turn
    toward the target first, and route laterally around range/bearing obstacles.
    Obstacles from the obstacle app and ranged detected objects are both used
    when avoid_obstacles is enabled.
    """
    x_m = _as_float(goto_dict.get('x_m', 0.0), 0.0)
    y_m = _as_float(goto_dict.get('y_m', 0.0), 0.0)
    z_m = _as_float(goto_dict.get('z_m', 0.0), 0.0)

    max_step_m = _as_float(_control_value(controls_dict, 'max_step_m', 2.0), 2.0)
    standoff_m = _as_float(_control_value(controls_dict, 'standoff_m', 0.5), 0.5)
    clearance_m = _as_float(_control_value(controls_dict, 'clearance_m', 0.5), 0.5)
    hold_altitude = _as_bool(_control_value(controls_dict, 'hold_altitude', True), True)
    face_target = _as_bool(_control_value(controls_dict, 'face_target', False), False)
    avoid_obstacles = _as_bool(_control_value(controls_dict, 'avoid_obstacles', False), False)

    if max_step_m < 0.1:
        max_step_m = 0.1
    if standoff_m < 0.0:
        standoff_m = 0.0
    if clearance_m < 0.0:
        clearance_m = 0.0
    if hold_altitude == True:
        z_m = 0.0

    distance_m = _distance_3d(x_m, y_m, z_m)
    if distance_m <= 0.0:
        logger.log_info('Goto request is a zero move, no steps planned')
        return []

    if standoff_m > 0.0:
        if distance_m <= standoff_m:
            logger.log_info('Goto request is inside standoff distance, no steps planned')
            return []
        scale = (distance_m - standoff_m) / distance_m
        x_m = x_m * scale
        y_m = y_m * scale
        z_m = z_m * scale

    waypoints = [(x_m, y_m)]
    obstacles = []
    blocked = []
    if avoid_obstacles == True and clearance_m > 0.0:
        obstacles = _collect_obstacles(obstacles_list, objects_list)
        blocked = _blocking_obstacles(x_m, y_m, obstacles, clearance_m)
        if len(blocked) > 0:
            waypoints = _build_avoidance_waypoints(x_m, y_m, clearance_m, blocked, obstacles)

    steps = []
    current_x = 0.0
    current_y = 0.0

    if face_target == True:
        yaw_deg = math.degrees(math.atan2(y_m, x_m))
        if abs(yaw_deg) > 1.0:
            steps.append(_make_step(0.0, 0.0, 0.0, yaw_deg,
                                    'Turn toward destination ' + str(round(yaw_deg, 1)) + 'deg'))
            if len(blocked) == 0:
                waypoints = [(math.sqrt((x_m * x_m) + (y_m * y_m)), 0.0)]

    for i, waypoint in enumerate(waypoints):
        next_x = waypoint[0]
        next_y = waypoint[1]
        step_x = next_x - current_x
        step_y = next_y - current_y
        step_z = z_m if i == len(waypoints) - 1 else 0.0
        description = 'Move to destination'
        if len(blocked) > 0:
            if i == 0:
                description = 'Avoid obstacle: offset from path'
            elif i == len(waypoints) - 1:
                description = 'Return to destination path'
            else:
                description = 'Avoid obstacle: pass obstruction'
        steps.extend(_split_translation_step(step_x, step_y, step_z, max_step_m, description))
        current_x = next_x
        current_y = next_y

    if len(steps) == 0:
        logger.log_info('Planner produced no non-zero RBX moves')
    return steps

##############################################################################
# EXTENSION POINT
#
# plan_velocity_move() below is the velocity-mode twin of plan_move(). It is the
# ONLY thing a developer adding real velocity logic has to replace.
#
# WHAT TO CHANGE
#   The body of plan_velocity_move(). For the MVP it returns exactly ONE step,
#   derived straight from goto_dict, with no obstacle awareness and no
#   splitting. A real implementation would read the same inputs plan_move()
#   reads -- the depth map, the object, target and obstacle lists, the robot
#   state and the live controls -- and return several timed velocity legs.
#
# WHAT NOT TO CHANGE
#   The signature and the return shape. AutoMoveIF.runPlanning() calls this with
#   exactly the same arguments plan_move() takes, and hands each returned step
#   straight to ConnectRBXDeviceIF.goto_velocity(). A step with an unexpected
#   key simply commands zero on that axis -- no error, just a robot that does
#   not move the way anyone asked.
#
#   Do not import ROS, nepi_sdk transport, or any interface class here, and do
#   not block, for the same reasons stated on plan_move() above.
#
# UNITS -- DIFFERENT FROM plan_move()
#   goto_dict still arrives in METRES and DEGREES, exactly as plan_move() gets
#   it, and it additionally carries 'move_speed_mps' -- the speed that turned
#   the clicked distance into a duration: a fraction of the robot's reported
#   max speed when it reports one, else the operator's hand entered value. It
#   also carries the already-derived velocity values.
#
#   Every value in every RETURNED step is METRES PER SECOND and DEGREES PER
#   SECOND, plus seconds for the duration. The keys are:
#       x_mps, y_mps, z_mps, yaw_degps, duration_s, description
#
#   The frame is the robot body frame from nepi_interfaces/GotoVelocity:
#   x forward, y LEFT, z up, yaw positive to port.
#
# OPEN LOOP
#   A velocity step is not a target. Nothing converges on it and nothing
#   measures the result: the robot holds the commanded velocity for duration_s
#   and stops. A planner that assumes a step "arrives" somewhere is assuming
#   something this command cannot deliver.
#
#   Returning an empty list is a legitimate answer -- "no move needed". The
#   process reports COMPLETE for it rather than treating it as a failure.
##############################################################################

def plan_velocity_move(goto_dict, np_depth_map, objects_list, targets_list, robot_dict, controls_dict, obstacles_list = None):
    """Plan the sequence of timed velocity moves that satisfies a goto request.

    MVP: returns exactly one step, taken from the velocity values the app
    already derived from the click. No obstacle awareness, no splitting, no
    turn-to-face. A step with a non-positive duration is no move at all and
    comes back as an empty list.
    """
    x_mps = _as_float(goto_dict.get('x_mps', 0.0), 0.0)
    y_mps = _as_float(goto_dict.get('y_mps', 0.0), 0.0)
    z_mps = _as_float(goto_dict.get('z_mps', 0.0), 0.0)
    yaw_degps = _as_float(goto_dict.get('yaw_degps', 0.0), 0.0)
    duration_s = _as_float(goto_dict.get('duration_s', 0.0), 0.0)

    if duration_s <= 0.0:
        logger.log_info('Velocity goto request has no duration, no steps planned')
        return []

    if x_mps == 0.0 and y_mps == 0.0 and z_mps == 0.0 and yaw_degps == 0.0:
        logger.log_info('Velocity goto request is all zeros, no steps planned')
        return []

    speed_mps = math.sqrt((x_mps * x_mps) + (y_mps * y_mps) + (z_mps * z_mps))
    description = ('Hold ' + str(round(speed_mps, 2)) + 'm/s for ' +
                   str(round(duration_s, 2)) + 's')

    return [{
        'x_mps': x_mps,
        'y_mps': y_mps,
        'z_mps': z_mps,
        'yaw_degps': yaw_degps,
        'duration_s': duration_s,
        'description': description,
    }]


def get_controls_dict():
    """Return a copy of this module's control definition dictionary.

    Returns:
        dict: The nepi_controls init-dict form of MOVE_CONTROLS_DICT, safe for
            a caller to hand to a ControlsIF and modify.
    """
    return copy.deepcopy(MOVE_CONTROLS_DICT)
