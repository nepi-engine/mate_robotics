#!/usr/bin/env python3
##
## Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
##
## This file is part of nepi-engine
## (see https://github.com/nepi-engine).
##
## License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
##

# The Coral Crusaders' hook file for NEPI Connect (Nepi_Connect/README.md).
#
# The team's Pi script (rov-26 rpi/modes/main.py) imports this file and calls
# it at four places, as main_py_nepi_link.patch in this folder adds:
#   nepi_link.start()                       once, before the main loop
#   nepi_link.merge_inputs(gamepad_inputs)  in place of tuple(gamepad_inputs)
#   nepi_link.on_teensy_line(onboard_data)  after each Teensy line is read
#   nepi_link.topside_tag()                 inside the $RPCTL line to the topside
# stop() exists for tests.
#
# Everything else lives in the nepi_connect folder beside this file: these
# four names only hand over to its hooks adapter, so the patch stays valid.
# The vehicle file, nepi_connect_vehicle.yaml beside this file, describes the
# ROV (Nepi_Connect/VEHICLE_FILE.md); vehicles/mate_coral_crusaders.yaml is
# the team's. With no vehicle file, an empty nepi_ip, no PyYAML, or no
# nepi_connect folder, the link is off: no sockets, no threads, and the
# pilot's inputs and topside line pass through unchanged.
#
# Nothing here may stop the team's script. Every function catches every
# exception and falls back to passing the pilot's inputs through untouched.

import os
import sys

MODULE_DIR = os.path.dirname(os.path.abspath(__file__))

_hooks = None
_import_error = None
try:
    if MODULE_DIR not in sys.path:
        sys.path.insert(0, MODULE_DIR)
    from nepi_connect.adapters import hooks as _hooks
except Exception as e:
    _hooks = None
    _import_error = e


def safeTuple(gamepad_inputs):
    try:
        return tuple(gamepad_inputs)
    except Exception:
        return gamepad_inputs

def start():
    """Read the vehicle file and start the link. Does nothing when the link is off."""
    try:
        if _hooks is None:
            print('nepi_link: the nepi_connect folder must sit beside nepi_link.py (' + str(_import_error) +
                  '): link off')
            return
        _hooks.configure(name = 'nepi_link')
        _hooks.start(module_dir = MODULE_DIR)
    except Exception:
        pass

def stop():
    """Stop the link's threads and close its sockets. For tests."""
    try:
        if _hooks is not None:
            _hooks.stop()
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
        if _hooks is None:
            return safeTuple(gamepad_inputs)
        return _hooks.merge_inputs(gamepad_inputs)
    except Exception:
        return safeTuple(gamepad_inputs)

def on_teensy_line(line):
    """Record a line read from the Teensy for NEPI's status. Never changes it.

    Args:
        line (str): The decoded Teensy line.
    """
    try:
        if _hooks is not None:
            _hooks.on_telemetry_line(line)
    except Exception:
        pass

def topside_tag():
    """Return ',NEPI:ON', ',NEPI:WAIT', ',NEPI:OFF' or ',NEPI:NOTREADY' for the pilot's line.

    Returns:
        str: The tag, or an empty string when the link is off or on any error.
    """
    try:
        if _hooks is None:
            return ''
        tag = _hooks.status_tag()
        if tag == '':
            return ''
        return ',NEPI:' + tag
    except Exception:
        return ''
