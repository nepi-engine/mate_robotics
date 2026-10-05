#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus <https://www.numurus.com>.
#
# This file is part of nepi applications (nepi_drivers) repo
# (see https://https://github.com/nepi-engine/nepi_drivers)
#
# License: nepi applications are licensed under the "Numurus Software License",
# which can be found at: <https://numurus.com/wp-content/uploads/Numurus-Software-License-Terms.pdf>
#
# Redistributions in source code must retain this top-level comment bstab.
# Plagiarizing this software to sidestep the license obligations is illegal.
#
# Contact Information:
# ====================
# - mailto:nepi@numurus.com

##############################################################################
# RBX (Robot) DRIVER TEMPLATE -- NODE
# ---------------------------------------------------------------------------
# Registers an RBXRobotIF -- the widest callback surface of all device types.
# An RBX driver models a controllable vehicle as:
#   - STATES   (e.g. DISARM / ARM)          -> get/setStateInd
#   - MODES    (e.g. STABILIZE / GUIDED..)  -> get/setModeInd
#   - SETUP actions (TAKEOFF, LAUNCH)       -> setSetupActionInd
#   - GO actions                            -> setGoActionInd
#   - goto commands (pose / position / location) for autonomous control
#   - a battery %, a home location, a NavPose, and an AxisControls DOF mask
# You supply an ordered list for each of states/modes/setup_actions/go_actions;
# the IF passes back the selected INDEX into that list.
#
# An EMPTY list is legitimate, not a placeholder to be filled: a differential-
# drive ground robot has no arm/disarm or flight modes, and RBXRobotIF handles
# empty lists correctly (its bounds check rejects any set_state/set_mode index
# and status shows "Not Set"). Do NOT invent dummy entries. The matching
# get*Ind / set*Ind functions must still be real callables either way --
# RBXRobotIF calls the getters unconditionally.
#
# The first thirteen constructor arguments (axisControls, getBatteryPercent-
# Function, states, get/setStateIndFunction, modes, get/setModeIndFunction,
# checkStopFunction, setup_actions, setSetupActionIndFunction, go_actions,
# setGoActionIndFunction) are REQUIRED and positional in the IF's signature --
# pass them by keyword, as below, and none of them may be omitted.
#
# If the robot's protocol is bridged by a companion node (mavros etc.) launched
# by discovery, this node typically attaches to that node's topics/services.
# Here the hardware calls are stubbed with TODOs.
#
# SETTINGS CONTRACT (identical in every NEPI driver -- idx_v4l2_node.py is the
# reference implementation):
#   initSettingsDict() / refreshSettingsDict() / getSettingsFunction() /
#   setSettingFunction(name, value) -> [success, msg, settings_dict]
# The retired capSettings= / factorySettings= / settingUpdateFunction=
# arguments no longer exist on ANY device IF -- passing them raises TypeError
# at construction.
##############################################################################

import copy

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_nav
from nepi_sdk import nepi_controls    # controls dict: create/get/set value, bounds, options

from nepi_api.messages_if import MsgIF
from nepi_api.device_if_rbx import RBXRobotIF

from nepi_interfaces.msg import AxisControls

PKG_NAME = 'RBX_TEMPLATE'
FILE_TYPE = 'NODE'


class RbxTemplateNode:

    # ---- Capability tables. The IF returns the INDEX into each list.
    RBX_STATES = ["DISARM", "ARM"]
    RBX_MODES = ["STABILIZE", "LAND", "RTL", "LOITER", "GUIDED"]
    RBX_SETUP_ACTIONS = ["TAKEOFF", "LAUNCH"]
    RBX_GO_ACTIONS = []

    # ---- Runtime-tunable settings (see lsx_template_node.py for the fully
    #      annotated pattern). 'type' must be a nepi_controls CONTROL_TYPE --
    #      'Discrete' is not one, and a setting declared that way is silently
    #      dropped and never appears in the RUI.
    CAP_SETTINGS = dict(
        takeoff_height_m={"type": "Float", "name": "takeoff_height_m", "options": ["0.0", "100.0"]},
    )
    FACTORY_SETTINGS = dict(
        takeoff_height_m={"type": "Float", "name": "takeoff_height_m", "value": "10.0"},
    )
    FACTORY_SETTINGS_OVERRIDES = dict()
    settingFunctions = dict(
        takeoff_height_m={'get': 'getTakeoffHeight', 'set': 'setTakeoffHeight'},
    )

    device_info_dict = dict(device_name="", path="", serial_number="",
                            hw_version="", sw_version="")

    POSITION_UPDATE_RATE = 10

    # runtime state
    state_ind = 0
    mode_ind = 0
    battery_percent = 0.0
    takeoff_height_m = 10.0
    takeoff_complete = False
    rbx_if = None

    init_settings_dict = dict()
    settings_dict = dict()

    # Per-motor speed ratios, reported and driven through the optional motor
    # control callbacks below. Leave both callbacks None for a robot with no
    # direct motor control.
    motor_control_ratios = [0.0, 0.0]
    DEFAULT_NODE_NAME = PKG_NAME.lower() + "_node"

    ##########################################################################
    def __init__(self):
        nepi_sdk.init_node(name=self.DEFAULT_NODE_NAME)
        self.class_name = type(self).__name__
        self.base_namespace = nepi_sdk.get_base_namespace()
        self.node_name = nepi_sdk.get_node_name()
        self.node_namespace = nepi_sdk.get_node_namespace()
        self.msg_if = MsgIF(log_name=self.class_name)
        self.msg_if.pub_info("Starting Node Initialization Processes")

        try:
            self.drv_dict = nepi_sdk.get_param('~drv_dict', dict())
            self.device_name = self.drv_dict['DEVICE_DICT']['device_name']
            self.device_path = self.drv_dict['DEVICE_DICT']['device_path']
            # self.companion_node_name = self.drv_dict['DEVICE_DICT'].get('companion_node_name')
        except Exception as e:
            self.msg_if.pub_warn("Failed to load Device Dict " + str(e))
            nepi_sdk.signal_shutdown(self.node_name + ": no valid Device Dict")
            return

        # ---- If discovery launched a companion protocol node, attach to it:
        #   nepi_sdk.wait_for_node(self.companion_node_name)
        #   ... subscribe to its state/battery/position topics,
        #   ... connect its arm/mode/takeoff services via nepi_sdk.connect_service.
        # TODO: connect to your robot here and confirm the link is up.

        self.navpose_dict = copy.deepcopy(nepi_nav.BLANK_NAVPOSE_DICT)

        # ---- 6-DOF axis capability mask.
        self.axis_controls = AxisControls()
        self.axis_controls.x = True
        self.axis_controls.y = True
        self.axis_controls.z = True
        self.axis_controls.roll = True
        self.axis_controls.pitch = True
        self.axis_controls.yaw = True

        self.settings_dict = self.initSettingsDict()
        self.settings_dict = self.refreshSettingsDict()
        self.device_info_dict["device_name"] = self.device_name
        self.device_info_dict["path"] = self.device_path
        self.device_info_dict["serial_number"] = ""
        self.device_info_dict["hw_version"] = ""
        self.device_info_dict["sw_version"] = ""

        # ---- Register the robot with NEPI.
        self.rbx_if = RBXRobotIF(
            device_info=self.device_info_dict,
            data_source_description='control_system',
            data_ref_description='control_system',
            getSettingsFunction=self.getSettingsFunction,
            setSettingFunction=self.setSettingFunction,
            axisControls=self.axis_controls,
            getBatteryPercentFunction=self.getBatteryPercent,
            states=self.RBX_STATES,
            getStateIndFunction=self.getStateInd,
            setStateIndFunction=self.setStateInd,
            modes=self.RBX_MODES,
            getModeIndFunction=self.getModeInd,
            setModeIndFunction=self.setModeInd,
            checkStopFunction=self.checkStopFunction,
            setup_actions=self.RBX_SETUP_ACTIONS,
            setSetupActionIndFunction=self.setSetupActionInd,
            go_actions=self.RBX_GO_ACTIONS,
            setGoActionIndFunction=self.setGoActionInd,
            # ---- optional manual-control callbacks ----
            manualControlsReadyFunction=None,
            getMotorControlRatios=self.getMotorControlRatios,   # -> [ratio, ...]
            setMotorControlRatio=self.setMotorControlRatio,     # (motor_ind, ratio)
            # ---- optional autonomous-control callbacks ----
            autonomousControlsReadyFunction=self.autonomousControlsReady,
            getHomeFunction=self.getHomeLocation,
            setHomeFunction=self.setHomeLocation,
            goHomeFunction=self.goHome,
            goStopFunction=self.goStop,
            gotoPoseFunction=self.gotoPose,
            gotoPositionFunction=self.gotoPosition,
            gotoLocationFunction=self.gotoLocation,
            gotoVelocityFunction=None,      # TODO: add for a velocity-setpoint robot
            getNavPoseCb=self.getNavPoseCb,
            navpose_update_rate=self.POSITION_UPDATE_RATE,
            msg_if=self.msg_if,
        )

        nepi_sdk.on_shutdown(self.cleanup_actions)
        self.msg_if.pub_info("Initialization complete")
        nepi_sdk.spin()

    #########################################################################
    # SETTINGS PLUMBING
    #
    # CAP_SETTINGS' numeric "options" pair becomes 'bounds' in the controls
    # dict: the retired cap-settings form carried an Int/Float control's min
    # and max in an 'options' pair, and 'options' on a numeric control is
    # ignored now.
    #########################################################################
    def initSettingsDict(self):
        init_settings_dict = dict()
        for name, cap_setting in self.CAP_SETTINGS.items():
            setting_type = cap_setting['type']
            setting_dict = {'type': setting_type}
            if 'options' in cap_setting:
                try:
                    if setting_type == 'Int':
                        setting_dict['bounds'] = [int(cap_setting['options'][0]),
                                                  int(cap_setting['options'][1])]
                    elif setting_type == 'Float':
                        setting_dict['bounds'] = [float(cap_setting['options'][0]),
                                                  float(cap_setting['options'][1])]
                    else:
                        setting_dict['options'] = [str(o) for o in cap_setting['options']]
                except Exception as e:
                    self.msg_if.pub_warn("Invalid bounds for setting: " + name + " : " + str(e))

            default = None
            if name in self.FACTORY_SETTINGS:
                default = self.FACTORY_SETTINGS[name].get('value')
            if name in self.FACTORY_SETTINGS_OVERRIDES:
                default = self.FACTORY_SETTINGS_OVERRIDES[name]
            if default is None:
                continue
            try:
                if setting_type == 'Int':
                    default = int(float(default))
                elif setting_type == 'Float':
                    default = float(default)
                elif setting_type == 'Toggle':
                    default = (str(default) in ('True', 'true'))
                else:
                    default = str(default)
            except Exception as e:
                self.msg_if.pub_warn("Invalid factory value for setting: " + name + " : " + str(e))
                continue
            setting_dict['default'] = default
            init_settings_dict[name] = setting_dict

        self.init_settings_dict = init_settings_dict
        settings_dict = nepi_controls.create_controls_dict(init_settings_dict)
        self.msg_if.pub_info("Initialized Settings: " +
                             str(nepi_controls.get_values_dict(settings_dict)))
        return settings_dict

    def refreshSettingsDict(self):
        # These are software settings this node holds and pushes to the robot;
        # there is nothing to read back off the vehicle, so this only returns a
        # copy. Kept for contract consistency with the other drivers. A driver
        # whose device DOES report its settings re-reads them here instead.
        return copy.deepcopy(self.settings_dict)

    def getSettingsFunction(self):
        return self.settings_dict

    def setSettingFunction(self, setting_name, setting_value):
        # SettingsIF replaces its own dict with the THIRD element, so all three
        # must always be returned.
        setting_str = setting_name + ":" + str(setting_value)
        if setting_name not in self.settings_dict.keys():
            return False, (self.node_name + " Setting name " + setting_str +
                           " is not supported"), self.settings_dict
        # get_clean_value() returns None for an invalid value. Test 'is None',
        # not falsiness -- a Toggle cleans to a legitimate False.
        if nepi_controls.get_clean_value(self.settings_dict, setting_name, setting_value) is None:
            return False, (self.node_name + " Setting data " + setting_str +
                           " is not valid"), self.settings_dict

        self.settings_dict = nepi_controls.set_value(self.settings_dict,
                                                     setting_name, setting_value)
        if setting_name in self.settingFunctions:
            try:
                getattr(self, self.settingFunctions[setting_name]['set'])(
                    nepi_controls.get_value(self.settings_dict, setting_name))
            except Exception as e:
                msg = "Failed to apply " + setting_str + " : " + str(e)
                self.msg_if.pub_warn(msg)
                return False, msg, self.settings_dict
        return True, (self.node_name + " UPDATED SETTINGS " + setting_str), self.settings_dict

    def getTakeoffHeight(self):
        return self.takeoff_height_m

    def setTakeoffHeight(self, val):
        self.takeoff_height_m = float(val)
        return True

    #########################################################################
    # RBX CAPABILITY CALLBACKS  (TODO: implement against your robot/protocol)
    #########################################################################
    def getStateInd(self):
        return self.state_ind

    def setStateInd(self, state_ind):
        # e.g. state_ind 1 == "ARM" -> send arm command to the vehicle.
        if 0 <= state_ind < len(self.RBX_STATES):
            self.state_ind = state_ind
            # TODO: send arm/disarm to hardware.

    def getModeInd(self):
        return self.mode_ind

    def setModeInd(self, mode_ind):
        if 0 <= mode_ind < len(self.RBX_MODES):
            self.mode_ind = mode_ind
            # TODO: command the flight/drive mode on hardware.

    def checkStopFunction(self):
        # Return True if the vehicle should be considered stopped/safe.
        return True

    def getBatteryPercent(self):
        return self.battery_percent

    def getMotorControlRatios(self):
        # One 0.0-1.0 ratio per motor, in a fixed order. Return an empty list
        # (and pass None for both motor callbacks) for a robot with no direct
        # motor control.
        return self.motor_control_ratios

    def setMotorControlRatio(self, motor_ind, speed_ratio):
        # TODO: drive one motor directly. Bounds-check the index -- the IF does
        # not know how many motors the robot has.
        if 0 <= motor_ind < len(self.motor_control_ratios):
            self.motor_control_ratios[motor_ind] = max(0.0, min(1.0, float(speed_ratio)))

    def setSetupActionInd(self, action_ind):
        # e.g. RBX_SETUP_ACTIONS[action_ind] == "TAKEOFF"
        if 0 <= action_ind < len(self.RBX_SETUP_ACTIONS):
            action = self.RBX_SETUP_ACTIONS[action_ind]
            # TODO: perform the named setup action (takeoff to takeoff_height_m).
            if action == "TAKEOFF":
                self.takeoff_complete = True

    def setGoActionInd(self, action_ind):
        # TODO: perform the named go action (only if RBX_GO_ACTIONS is non-empty).
        pass

    def autonomousControlsReady(self):
        # goto* commands are only honored when this returns True.
        return (self.RBX_STATES[self.state_ind] == "ARM"
                and self.RBX_MODES[self.mode_ind] == "GUIDED"
                and self.takeoff_complete)

    def getHomeLocation(self):
        # TODO: return [lat, lon, alt].
        return [self.navpose_dict['latitude'], self.navpose_dict['longitude'],
                self.navpose_dict['altitude_m']]

    def setHomeLocation(self, geopoint):
        # TODO: command the vehicle's home location.
        pass

    def goHome(self):
        pass    # TODO: command return-to-home

    def goStop(self):
        pass    # TODO: command an immediate stop/hold

    def gotoPose(self, attitude_enu_degs):
        pass    # TODO: command an attitude setpoint (roll,pitch,yaw)

    def gotoPosition(self, point_enu_m, orientation_enu_deg):
        pass    # TODO: command a local ENU position setpoint

    def gotoLocation(self, geopoint_amsl, orientation_ned_deg):
        pass    # TODO: command a global lat/lon/alt setpoint

    def getNavPoseCb(self):
        # TODO: fill from live vehicle telemetry (see npx_template for the dict
        # and the full has_* flag list). Always deepcopy -- never hand out a
        # reference to a dict the telemetry thread is still writing.
        return copy.deepcopy(self.navpose_dict)

    #########################################################################
    def cleanup_actions(self):
        # TODO: disarm / close links / stop the companion node if this node owns it.
        pass


if __name__ == '__main__':
    node = RbxTemplateNode()
