# This message details the RobotStab App Status Msg
#
# Status is what the node REPORTS. Everything the operator ADJUSTS is rendered
# by the shared control renderer from the app's ControlsStatus on
# <node>/controls/status, so it does not need a field here. The adjustable
# values below are kept anyway because the migrated apps kept theirs: editing a
# status message ripples into the connect API and the RUI for no gain.

# App enabled state
bool enabled
# Available options (published so a programmatic consumer sees the same list
# the 'selected_option' control offers)
string[] options
# Currently selected option
string selected_option
# Current value setting
float32 value

# Device connection state, one per device connect. Each comes from that
# Connect*IF's check_connection(). Which device is selected, and the list of
# candidates, is NOT repeated here: it lives on each connect's own
# nepi_interfaces/ConnectIFStatus at <node>/<connect_name>/status.
bool servo_connected
bool light_connected
bool motor_connected
bool npx_connected
bool idx_connected
