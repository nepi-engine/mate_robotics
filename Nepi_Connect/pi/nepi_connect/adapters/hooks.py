##
## Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
##
## This file is part of nepi-engine
## (see https://github.com/nepi-engine).
##
## License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
##

# The hooks adapter (control.adapter: hooks). The robot's own control loop
# calls these module functions at four places:
#   start()                         once, before the loop
#   merge_inputs(pilot_inputs)      in place of the pilot's inputs, just before they are used
#   on_telemetry_line(line)         after each line the robot prints
#   status_tag()                    wherever the pilot can see NEPI's state
# stop() exists for tests. Nothing here may stop the robot's code: every
# function catches every exception and falls back to passing the pilot's
# inputs through untouched.

from . import Adapter
from ..control import NepiControl
from ..control import safeTuple


class HooksAdapter(Adapter):
    """The adapter the robot's own loop calls, one hook at a time."""

    def __init__(self, core = None):
        """Builds the adapter around a core.

        Args:
            core (NepiControl): The core to use (default: a new one).
        """
        Adapter.__init__(self, core)
        self.last_pilot = None

    def apply_demand(self, pilot_inputs):
        """Apply NEPI's demand to a copy of the pilot's inputs.

        Args:
            pilot_inputs (list): The pilot's inputs, indexed by slot. Never modified.

        Returns:
            tuple: The merged inputs, or tuple(pilot_inputs) unchanged when the
                link or NEPI control is off, or on any error.
        """
        try:
            self.last_pilot = safeTuple(pilot_inputs)
            return self.core.merge_inputs(pilot_inputs)
        except Exception:
            return safeTuple(pilot_inputs)

    def take_telemetry(self, line):
        """Record a telemetry line for NEPI's status. Never changes it.

        Args:
            line (str): The decoded line.
        """
        try:
            self.core.on_telemetry_line(line)
        except Exception:
            pass

    def pilot_inputs(self):
        """Report the pilot's inputs from the latest merge_inputs() call.

        Returns:
            tuple: The pilot's inputs, or None before the first call.
        """
        return self.last_pilot

    def status_tag(self):
        """Return 'ON', 'WAIT', 'OFF' or 'NOTREADY', or '' while the link is off.

        Returns:
            str: As NepiControl.status_tag(), '' on any error.
        """
        try:
            return self.core.status_tag()
        except Exception:
            return ''


#**********************
# The robot's code calls these

_adapter = HooksAdapter()

def configure(name = None, log = None):
    """Set the name that starts each printed message, and where messages go.

    Call it before start(). nepi_link.py names messages after itself.

    Args:
        name (str): The prefix, such as 'nepi_link' (default: unchanged).
        log (function): Prints one message (default: unchanged).
    """
    try:
        if name is not None:
            _adapter.core.name = name
        if log is not None:
            _adapter.core.log = log
    except Exception:
        pass

def start(module_dir = None):
    """Read the vehicle file and start the link. Does nothing when the link is off.

    Args:
        module_dir (str): The folder that holds the hooks file, where the
            vehicle file may sit (default: beside the nepi_connect package).
    """
    try:
        _adapter.start(module_dir = module_dir)
    except Exception:
        pass

def stop():
    """Stop the link's threads and close its sockets. For tests."""
    try:
        _adapter.stop()
    except Exception:
        pass

def merge_inputs(pilot_inputs):
    """Return the inputs for the robot, with NEPI's demand merged in.

    Args:
        pilot_inputs (list): The pilot's inputs. Never modified.

    Returns:
        tuple: The merged inputs, or tuple(pilot_inputs) unchanged when the
            link or NEPI control is off, or on any error.
    """
    try:
        return _adapter.apply_demand(pilot_inputs)
    except Exception:
        return safeTuple(pilot_inputs)

def on_telemetry_line(line):
    """Record a line the robot printed, for NEPI's status. Never changes it.

    Args:
        line (str): The decoded line.
    """
    try:
        _adapter.take_telemetry(line)
    except Exception:
        pass

def status_tag():
    """Return 'ON', 'WAIT', 'OFF' or 'NOTREADY' for the pilot's display.

    Returns:
        str: The state, or an empty string when the link is off or on any error.
    """
    try:
        return _adapter.status_tag()
    except Exception:
        return ''
