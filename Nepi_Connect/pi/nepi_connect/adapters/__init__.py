##
## Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
##
## This file is part of nepi-engine
## (see https://github.com/nepi-engine).
##
## License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
##

# Adapters: how NEPI Connect reaches one kind of robot's inputs.
#
# The core (control.NepiControl) owns the link to NEPI and the NEPI control
# rules, and knows nothing about how a robot reads its controller or prints
# its telemetry. An adapter does that part. Every adapter provides the three
# operations of the Adapter class below, plus start and stop.
#
# Only the hooks adapter exists (hooks.py): the robot's own control loop
# calls it. A standalone adapter, in which the NEPI Connect service owns the
# robot's connection and runs the loop itself, would be one more file here
# that implements the same Adapter class and selects itself with
# control.adapter in the vehicle file.

from ..control import NepiControl


class Adapter(object):
    """The interface every adapter provides.

    Each method is safe to call from the robot's code: it catches every
    exception, and on any error the robot gets the pilot's own inputs back.
    """

    def __init__(self, core = None):
        """Builds the adapter around a core.

        Args:
            core (NepiControl): The core to use (default: a new one).
        """
        if core is None:
            core = NepiControl()
        self.core = core

    def start(self, raw = None, vehicle_path = None, module_dir = None):
        """Read the vehicle file and start the link. As NepiControl.start()."""
        return self.core.start(raw, vehicle_path, module_dir)

    def stop(self):
        """Stop the link. As NepiControl.stop()."""
        return self.core.stop()

    def apply_demand(self, pilot_inputs):
        """Apply NEPI's demand to the robot's inputs.

        Args:
            pilot_inputs (list): The pilot's inputs, indexed by slot.

        Returns:
            tuple: The inputs the robot should use.
        """
        raise NotImplementedError

    def take_telemetry(self, line):
        """Take one telemetry line from the robot in.

        Args:
            line (str): The line, as the robot printed it.
        """
        raise NotImplementedError

    def pilot_inputs(self):
        """Report the pilot's inputs, as last seen.

        Returns:
            tuple: The pilot's inputs, or None before the first.
        """
        raise NotImplementedError
