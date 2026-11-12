##
## Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
##
## This file is part of nepi-engine
## (see https://github.com/nepi-engine).
##
## License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
##

# NEPI Connect, the robot's side of the NEPI link (Nepi_Connect/LINK_PROTOCOL.md),
# for any robot a vehicle file describes (Nepi_Connect/VEHICLE_FILE.md).
#
#   vehicle.py     find, load and validate the vehicle file; the vehicle message
#   link.py        the version 2 messages and the rules every message follows
#   control.py     the NEPI control state machine, the merge and the sockets
#   mapping.py     demand to input value: stick, trigger_pair and servo
#   telemetry.py   the robot's telemetry line, from the file's field list
#   adapters/      how the robot's inputs are reached; hooks.py is the only one
#
# Python 3.8 or newer, standard library plus PyYAML.
