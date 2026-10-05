# This message details the Template App Status Msg
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
