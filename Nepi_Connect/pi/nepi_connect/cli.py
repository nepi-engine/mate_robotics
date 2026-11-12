##
## Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
##
## This file is part of nepi-engine
## (see https://github.com/nepi-engine).
##
## License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
##

# The vehicle file from the command line, for the NEPI Connect service
# (resources/config/nepi_connect.sh), which reads it through here rather than
# parsing YAML in bash. Run with this package's folder on PYTHONPATH:
#
#   python3 -B -m nepi_connect.cli check
#       Finds and checks the vehicle file; prints where it is and whether the
#       link would be on.
#   python3 -B -m nepi_connect.cli cameras
#       Prints one line per http camera for the service to stream:
#       name, port, resolution, fps and device, separated by tabs (device
#       last, empty for the first USB camera found). fastmjpg cameras are
#       announced only, so they are noted on stderr, not printed.
#
# Both find the file as VEHICLE_FILE.md "Where the Robot Looks" says;
# --vehicle PATH reads that file instead. On any problem (no vehicle file,
# PyYAML missing, an invalid value) they print why on stderr and exit 1.

import argparse
import sys

from . import vehicle as vehicle_mod


def load(path = None, module_dir = None):
    """Find, read and check the vehicle file.

    Args:
        path (str): The file to read, or None to look it up.
        module_dir (str): As in vehicle.vehicle_candidates().

    Returns:
        list: [vehicle (dict or None), path (str or None), msg (str)]. vehicle
            is None when there is no usable file, with the reason in msg.
    """
    if path is None:
        [raw, path, msg] = vehicle_mod.load_vehicle(module_dir)
    else:
        [raw, msg] = vehicle_mod.load_vehicle_file(path)
    if raw is None:
        return None, path, msg
    [vehicle, msg] = vehicle_mod.validate_vehicle(raw)
    return vehicle, path, msg

def camera_lines(vehicle):
    """Format a vehicle's http cameras for the service.

    Args:
        vehicle (dict): From validate_vehicle().

    Returns:
        list: [lines, notes]. lines holds one tab-separated line per http
            camera (name, port, resolution, fps, device); notes holds one line
            per fastmjpg camera, which the service does not stream.
    """
    lines = []
    notes = []
    for camera in vehicle['cameras']:
        if camera['transport'] == 'http':
            lines.append('\t'.join([camera['name'], str(camera['port']), camera['resolution'], str(camera['fps']),
                                    camera['device']]))
        else:
            notes.append('Camera ' + camera['name'] + ': fastmjpg to NEPI port ' + str(camera['port']) +
                         ', sent by the robot\'s own FastMJPG command; announced only')
    return lines, notes

def main(argv = None):
    """Run one command; see the top of this file.

    Args:
        argv (list): The arguments, without the program name (default: sys.argv).

    Returns:
        int: 0 on success, 1 when the vehicle file cannot be used.
    """
    parser = argparse.ArgumentParser(prog = 'python3 -m nepi_connect.cli',
                                     description = 'Read the NEPI Connect vehicle file for the service.')
    parser.add_argument('--vehicle', default = None, help = 'The vehicle file to read (default: look it up)')
    parser.add_argument('command', choices = ['check', 'cameras'])
    args = parser.parse_args(argv)
    try:
        [vehicle, path, msg] = load(args.vehicle)
    except Exception as e:
        sys.stderr.write('Vehicle file error: ' + str(e) + '\n')
        return 1
    if vehicle is None:
        if path is not None:
            msg += ' (' + path + ')'
        sys.stderr.write(msg + '\n')
        return 1
    if args.command == 'check':
        print('Vehicle file: ' + path)
        print(msg)
        return 0
    [lines, notes] = camera_lines(vehicle)
    for note in notes:
        sys.stderr.write(note + '\n')
    for line in lines:
        print(line)
    return 0


if __name__ == '__main__':
    sys.exit(main())
