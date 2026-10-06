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

# Connection state, one per connect. rbx_connected and targets_connected come
# from that Connect*IF's check_connection(); which device or process is
# selected, and the list of candidates, is NOT repeated here: it lives on each
# connect's own nepi_interfaces/ConnectIFStatus at <node>/<connect_name>/status.
bool rbx_connected
bool targets_connected

# The obstacles connect is not a ConnectNodeIF and publishes no ConnectIFStatus,
# so its selection is reported here. obstacles_connected is ConnectObstaclesIF's
# check_connection(); selected_obstacles_namespace is the obstacles app
# namespace the node is pointed at, or 'None'.
bool obstacles_connected
string selected_obstacles_namespace
