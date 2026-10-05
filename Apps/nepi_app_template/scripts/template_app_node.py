#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus <https://www.numurus.com>.
#
# This file is part of nepi applications (nepi_apps) repo
# (see https://https://github.com/nepi-engine/nepi_apps)
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
# NEPI APP TEMPLATE -- NODE
# ---------------------------------------------------------------------------
# apps_mgr launches this file with a plain rosrun, using the pkg_name/app_file/
# node_name from params/template_app_params.yaml. The node then:
#   1. init_node() and create a MsgIF for logging.
#   2. Build a NodeClassIF from four plain dicts (configs, params, pubs, subs).
#      NodeClassIF creates every ROS interface and the standard
#      save_config / reset_config / factory_reset_config topics.
#   3. Mount a ControlsIF holding everything an operator adjusts from the RUI.
#   4. initCb(), start the work and status timers, spin().
#
# This node never touches rospy. rospy lives only in nepi_sdk/nepi_ros.py; use
# the nepi_sdk wrappers (init_node, start_timer_process, on_shutdown, spin).
#
# CONTROLS CONTRACT (src/nepi_apps/CONTROLS_MIGRATION_PATTERN.md is the
# authority; nepi_app_file_pub_img is the worked example):
#   Anything an operator ADJUSTS from the RUI is a control. Anything read once
#   at startup, written by the NODE, or owned by a sub-interface is not.
#     * value an operator types or drags -> CONTROLS_INIT_DICT
#     * command carrying a compound value or a traversal verb (goto_location,
#       select_folder, home_folder) -> stays a topic; a Button control calls
#       the SAME private method, never a second implementation
#     * node-written state that merely persists (current_folder, running)
#       -> stays in PARAMS_DICT
#   PARAMS_DICT does NOT have to end up empty. The test is not "does this
#   value persist" but "does the operator type it".
#   What must never survive: a control AND a param both writing one piece of
#   state.
#
# THREE TRAPS WORTH KNOWING BEFORE YOU EDIT THE CONTROL SET
#   * controls_updated_callback is called with TWO arguments,
#     (control_name, update_value) -- see nepi_api/system_if.py. A one-argument
#     callback raises TypeError on every update.
#   * 'Discrete' is NOT a control type. nepi_controls.CONTROL_TYPES is
#     Menu, Button, Buttons, Toggle, Toggles, String, Selection, Selections,
#     Int, Ints, IntSlider, Float, Floats, FloatSlider, RangeSlider, ColorRGB.
#     create_controls_dict() wraps each entry in a bare except, so a bad type
#     is silently dropped and the widget never appears. checkControlsInitDict()
#     below turns that into a named warning.
#   * Prefer Selection over Menu whenever the stored value is meaningful text.
#     A Menu value is the INDEX into its option list, and that index silently
#     re-points at a different entry as soon as the list is rebuilt.
#
# ControlsIF is given NO node_if, so it builds and owns its own NodeClassIF.
# Sharing this node's would merge both registries -- register_pubs/register_subs
# /add_params do a keyed dict.update() -- and a generic key ('status_pub',
# 'reset', 'enable') would silently orphan a sibling's publisher: the topic
# stays advertised and never publishes again, with nothing in the log
# (2026-07 DECISION LOG in the workspace CLAUDE.md). The same applies to every
# other sub-interface you mount here (NavPoseIF, ColorImageIF, SettingsIF):
# leave node_if unset.
#
# TODO: run setup_new_app.sh, then replace the example control set, the status
# msg fields and the work in updaterCb with your own.
##############################################################################

import time

from std_msgs.msg import Empty

from nepi_app_template.msg import NepiAppTemplateStatus

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_controls    # controls dict: types, create/get/set value

from nepi_api.node_if import NodeClassIF
from nepi_api.messages_if import MsgIF
from nepi_api.system_if import ControlsIF


#########################################
# Factory Control Values
FACTORY_ENABLED = False
FACTORY_SELECTED_OPTION = "None"
FACTORY_VALUE = 0.0
FACTORY_OPTIONS = ["None", "Option_A", "Option_B", "Option_C"]

MIN_VALUE = 0.0
MAX_VALUE = 100.0

STATUS_PUBLISH_RATE_HZ = 1.0
UPDATE_RATE_HZ = 1.0


#########################################
# Controls
#########################################

# A ControlsIF is ALWAYS a direct child of the node namespace: its __init__
# builds create_namespace(node_namespace, controls_name) and there is no
# namespace argument. The RUI page must derive the same name independently --
# see CONTROLS_NAME in rui/NepiAppTemplate.js.
#
# Before naming a set, check the leaf namespace of every sub-interface this
# node mounts, not just the other sets. NavPoseIF handed a node namespace
# occupies <node>/navpose, so a set named 'navpose' would collide with it and
# advertise a second <node>/navpose/status of a different message type.
CONTROLS_NAME         = 'controls'
CONTROLS_DISPLAY_NAME = 'Template Controls'
CONTROLS_DESCRIPTION  = 'Example controls for the template app'

# Button controls, keyed here so the updated callback can tell a command press
# from a value edit without restating the names inline.
BUTTON_CONTROLS = ['trigger_action']

# Key order is display order: create_controls_dict iterates this dict, Python
# preserves insertion order, and the RUI renders controls_msg_list in that
# order. There is no separate ordering field.
#
# Controls sharing a NON-EMPTY display_group render on ONE horizontal line, in
# declaration order, and grouping applies to CONSECUTIVE runs only. Only
# String, Toggle, Toggles, Int, Ints, Float, Floats, Button and Buttons can be
# grouped; Menu, Selection, Selections, both sliders and ColorRGB arrive from
# the renderer already wrapped in their own Label and will not line up.
# display_width is a pixel hint -- a row needs fixed widths, because "100%" of
# a flex child collapses -- and bounds are not drawn inside a row, so a control
# that must show its min/max stays ungrouped.
CONTROLS_INIT_DICT = {

    'enabled': {
        'type': 'Toggle', 'default': FACTORY_ENABLED,
        'display_name': 'Enabled',
        'description': 'Run the app work cycle'},

    # Selection, not Menu: the stored value is the option text. Where the
    # option list is discovered at runtime the node owns it and pushes it with
    # set_control_options() -- ControlsIF carries whatever list it is given and
    # discovers nothing. Set the options BEFORE the value: a Selection value
    # that is not in the current list is rejected and falls back to the first
    # option.
    'selected_option': {
        'type': 'Selection', 'default': FACTORY_SELECTED_OPTION,
        'options': FACTORY_OPTIONS,
        'display_name': 'Option',
        'description': 'Which option the app work cycle uses'},

    # Reuse the constant the node already clamps with, so the control and the
    # node cannot drift apart. Do not leave a numeric control unbounded just
    # because the old parameter was.
    'value': {
        'type': 'Float', 'default': FACTORY_VALUE,
        'bounds': [MIN_VALUE, MAX_VALUE],
        'round': 2, 'display_round': 2,
        'display_name': 'Value',
        'description': 'Example numeric setting'},

    # Buttons need no default: create_controls_dict seeds every trigger type
    # with a "never fired" value. A press arrives as the sentinel 'TRIGGER'.
    # Avoid the substring 'Trigger' in a control NAME -- create_controls_dict
    # rewrites it to 'Button'.
    'trigger_action': {
        'type': 'Button',
        'display_name': 'Trigger Action',
        'description': 'Run the one-shot action once'},
}


class ControlValue:
    """Stand-in for the std_msgs message this app's setter callbacks expect.

    Routing a control to the callback that already owns the value keeps that
    callback's clamping, parsing and side effects exactly where they were,
    instead of reimplementing them in a control handler.
    """

    def __init__(self, data):
        self.data = data


#########################################
# Node Class
#########################################

class NepiTemplateApp(object):

    enabled = FACTORY_ENABLED
    selected_option = FACTORY_SELECTED_OPTION
    value = FACTORY_VALUE
    options = FACTORY_OPTIONS

    # Node-WRITTEN state, persisted through PARAMS_DICT rather than a control.
    last_run_count = 0

    node_if = None
    # Must be None before NodeClassIF is constructed: with init_configs set,
    # NodeClassIF calls initCb during construction, which is BEFORE ControlsIF
    # exists. initCb therefore runs twice at startup and every control read
    # goes through getControlValue(), which falls back.
    controls_if = None
    controls_routes = dict()

    DEFAULT_NODE_NAME = "app_template"

    def __init__(self):
        #### APP NODE INIT SETUP ####
        # DEFAULT_NODE_NAME is only a fallback. apps_mgr launches the node with
        # the node_name from params/template_app_params.yaml, and the yaml
        # wins -- keep the two in step.
        nepi_sdk.init_node(name=self.DEFAULT_NODE_NAME)
        self.class_name = type(self).__name__
        self.base_namespace = nepi_sdk.get_base_namespace()
        self.node_name = nepi_sdk.get_node_name()
        self.node_namespace = nepi_sdk.get_node_namespace()

        ##############################
        # Create Msg Class
        self.msg_if = MsgIF(log_name=self.class_name)
        self.msg_if.pub_info("Starting IF Initialization Processes")

        ##############################
        # Initialize Class Variables

        ##############################
        ### Setup Node

        # Configs Config Dict ####################
        self.CFGS_DICT = {
            'init_callback': self.initCb,
            'reset_callback': self.resetCb,
            'factory_reset_callback': self.factoryResetCb,
            'init_configs': True,
            'namespace': self.node_namespace
        }

        # Params Config Dict ####################
        # Everything an operator adjusts moved to ControlsIF, which registers
        # and persists its own param under the controls namespace. What belongs
        # here instead is node-WRITTEN state: a value the node itself sets that
        # has to survive a restart. The example below is a placeholder; delete
        # it and set PARAMS_DICT = None if your app has no such value.
        #
        # NOTE: unlike pubs/subs/services, a param's KEY is part of its wire
        # name (namespace + key), so renaming a param key renames the ROS
        # param.
        self.PARAMS_DICT = {
            'last_run_count': {
                'namespace': self.node_namespace,
                'factory_val': 0
            }
        }

        # Publishers Config Dict ####################
        self.PUBS_DICT = {
            'status_pub': {
                'namespace': self.node_namespace,
                'topic': 'status',
                'msg': NepiAppTemplateStatus,
                'qsize': 1,
                'latch': True
            }
        }

        # Subscribers Config Dict ####################
        # What stays on the wire are the COMMANDS: a message carrying a
        # compound value or a traversal verb that a control set, where each
        # value is written independently, cannot express atomically. The
        # trigger_action Button control calls the SAME private method this
        # callback does -- there is exactly one implementation per command.
        #
        # Removed from here and now driven over <node>/controls/update_control:
        # set_enabled, set_option and set_value.
        self.SUBS_DICT = {
            'trigger_action': {
                'namespace': self.node_namespace,
                'topic': 'trigger_action',
                'msg': Empty,
                'qsize': 10,
                'callback': self.triggerActionCb,
                'callback_args': ()
            }
        }

        # Create Node Class ####################
        self.node_if = NodeClassIF(
            configs_dict=self.CFGS_DICT,
            params_dict=self.PARAMS_DICT,
            pubs_dict=self.PUBS_DICT,
            subs_dict=self.SUBS_DICT,
            msg_if=self.msg_if
        )

        self.node_if.wait_for_ready()

        ##############################
        # Controls. Mounted AFTER the node's own NodeClassIF is ready and
        # BEFORE anything reads app state, because initCb sources every
        # adjustable value from it.
        self.setupControls()

        ##############################
        self.initCb(do_updates=True)

        time.sleep(1)
        nepi_sdk.start_timer_process(float(1) / UPDATE_RATE_HZ, self.updaterCb, oneshot=True)
        nepi_sdk.start_timer_process(float(1) / STATUS_PUBLISH_RATE_HZ, self.statusPublishCb)

        time.sleep(1)
        self.msg_if.pub_info("Initialization Complete")

        nepi_sdk.on_shutdown(self.cleanup_actions)
        nepi_sdk.spin()


    #######################
    ### Controls

    def setupControls(self):
        self.controls_routes = self.controlRoutes()
        self.checkControlsInitDict()
        try:
            self.controls_if = ControlsIF(
                controls_name=CONTROLS_NAME,
                controls_display_name=CONTROLS_DISPLAY_NAME,
                controls_description=CONTROLS_DESCRIPTION,
                controls_init_dict=CONTROLS_INIT_DICT,
                controls_updated_callback=self.controlsUpdatedCb,
                pub_status=True,
                save_params=True,
                msg_if=self.msg_if,
            )
            # ControlsIF sets its ready flag at the end of its own __init__,
            # after its NodeClassIF is up and its saved params are applied.
            self.controls_if.wait_for_controls_ready(timeout=10)
        except Exception as e:
            # Degrade to None rather than failing to start: every read goes
            # through getControlValue, which falls back to the factory value,
            # so the app still runs at factory settings.
            self.msg_if.pub_warn("Template App: controls unavailable: " + str(e))
            self.controls_if = None

    def checkControlsInitDict(self):
        # create_controls_dict drops a malformed control with a log warning
        # rather than raising, so a typo above costs one widget and nothing
        # else says so. Run it here first and name what went missing.
        try:
            controls_dict = nepi_controls.create_controls_dict(CONTROLS_INIT_DICT)
        except Exception as e:
            self.msg_if.pub_warn("Template App: could not validate controls init dict: " + str(e))
            return
        missing = [name for name in CONTROLS_INIT_DICT.keys() if name not in controls_dict.keys()]
        if len(missing) > 0:
            self.msg_if.pub_warn("Template App: controls dropped at registration: " + str(missing))

    def controlRoutes(self):
        # control name -> the app callback that already owns that value.
        # Routing rather than reimplementing is what keeps a callback's clamp,
        # parse or side effect exactly where it was. A control with no
        # per-field logic needs no route: applyControls() reads it directly.
        return {
            'value': self.setValueCb,
        }

    def getControlValue(self, control_name, fallback=None):
        if self.controls_if is None:
            return fallback
        value = None
        try:
            value = self.controls_if.get_control_value(control_name)
        except Exception as e:
            self.msg_if.pub_warn("Template App: failed to read control " +
                                 str(control_name) + ": " + str(e))
        if value is None:
            return fallback
        return value

    def setControlOptions(self, control_name, options):
        # Discovered option lists are the NODE's responsibility. Push the list
        # before setting a value into it.
        if self.controls_if is None:
            return
        try:
            if self.controls_if.get_control_options(control_name) != options:
                self.controls_if.set_control_options(control_name, options)
        except Exception as e:
            self.msg_if.pub_warn("Template App: failed to set options for control " +
                                 str(control_name) + ": " + str(e))

    def applyControls(self):
        # The single point where a control value becomes running app state.
        # Cheap enough to re-run on every update, which keeps the updated
        # callback from having to know which control feeds which attribute.
        self.enabled = bool(self.getControlValue('enabled', FACTORY_ENABLED))
        self.selected_option = str(self.getControlValue('selected_option', FACTORY_SELECTED_OPTION))
        self.value = float(self.getControlValue('value', FACTORY_VALUE))

    def controlsUpdatedCb(self, control_name, control_value):
        # Called by ControlsIF AFTER its dict is updated and its status
        # published. TWO arguments -- a one-argument version raises TypeError
        # on every update.
        if control_name == 'trigger_action':
            self.triggerAction()
        else:
            route = self.controls_routes.get(control_name, None)
            if route is not None:
                value = self.getControlValue(control_name)
                if value is not None:
                    route(ControlValue(value))

        # applyControls runs AFTER the route so an in-callback clamp is what
        # lands in app state, not the raw control value.
        self.applyControls()

        # Matches what the removed set_param calls did: persist on change. The
        # config IF debounces this onto its own timer, so a slider drag does
        # not write a file per frame. A Button press is not state, so it is not
        # persisted.
        if control_name not in BUTTON_CONTROLS and self.node_if is not None:
            self.node_if.save_config()

        self.publish_status()


    ###################
    ## App Callbacks

    def updaterCb(self, timer):
        # TODO: Add periodic background work here (e.g. discover resources,
        # poll hardware). Where the work discovers an option list, push it with
        # self.setControlOptions(name, ['None'] + discovered).
        #
        if self.enabled:
            self.last_run_count = self.last_run_count + 1
            if self.node_if is not None:
                self.node_if.set_param('last_run_count', self.last_run_count)

        # Self-rescheduling oneshot rather than a periodic timer: the next
        # cycle is armed at the end, so cycles never overlap if one runs long.
        nepi_sdk.start_timer_process(float(1) / UPDATE_RATE_HZ, self.updaterCb, oneshot=True)

    def setValueCb(self, msg):
        # Reached from the 'value' control route. A callback like this is where
        # per-field clamping, parsing or a side effect belongs. It no longer
        # calls set_param: the control persists the value, and leaving both in
        # place is the "two things writing one piece of state" failure the
        # controls migration exists to remove.
        value = msg.data
        if value < MIN_VALUE:
            value = MIN_VALUE
        if value > MAX_VALUE:
            value = MAX_VALUE
        self.value = value

    def triggerActionCb(self, msg):
        # Topic path into the one-shot action.
        self.triggerAction()

    def triggerAction(self):
        # The single implementation. The topic callback above and the Button
        # control both delegate here; neither reimplements the other.
        self.msg_if.pub_info("Action triggered")
        if self.enabled:
            # TODO: Replace with real action logic.
            pass


    #######################
    ### Config Functions

    def initCb(self, do_updates=False):
        # Runs twice at startup: once from NodeClassIF's init_configs, before
        # ControlsIF exists, and once explicitly after setupControls. The first
        # pass falls back to the factory values, the second picks up whatever
        # the config manager restored.
        if self.node_if is not None:
            self.last_run_count = self.node_if.get_param('last_run_count')
        self.applyControls()
        if do_updates:
            pass
        self.publish_status()

    def resetCb(self, do_updates=True):
        self.msg_if.pub_warn("Resetting")
        # ControlsIF owns its own config tier under its own namespace, so an
        # app-level reset has to hand the reset down or the controls keep their
        # current values while the rest of the app resets.
        if self.controls_if is not None:
            try:
                self.controls_if.reset()
            except Exception as e:
                self.msg_if.pub_warn("Template App: controls reset failed: " + str(e))
        if do_updates:
            pass
        self.initCb(do_updates=do_updates)

    def factoryResetCb(self, do_updates=True):
        self.msg_if.pub_warn("Factory Resetting")
        if self.controls_if is not None:
            try:
                self.controls_if.factory_reset()
            except Exception as e:
                self.msg_if.pub_warn("Template App: controls factory reset failed: " + str(e))
        if do_updates:
            pass
        self.initCb(do_updates=do_updates)


    ###################
    ## Status Publishers

    def statusPublishCb(self, timer):
        self.publish_status()

    def publish_status(self):
        # Status is what the node REPORTS. Everything the operator ADJUSTS is
        # rendered by the shared control renderer from ControlsStatus, so it
        # does not need to be duplicated here -- but leaving the values in
        # status is harmless and is what the migrated apps did, because editing
        # a status message ripples into the connect API and the RUI for no gain.
        status_msg = NepiAppTemplateStatus()
        status_msg.enabled = self.enabled
        status_msg.options = self.options
        status_msg.selected_option = self.selected_option
        status_msg.value = self.value
        if self.node_if is not None:
            self.node_if.publish_pub('status_pub', status_msg)


    #######################
    # Utility Functions
    #######################

    def cleanup_actions(self):
        self.msg_if.pub_info("TEMPLATE_APP: Shutting down: Executing script cleanup actions")
        if self.controls_if is not None:
            try:
                self.controls_if.unregister()
            except Exception:
                pass
            self.controls_if = None


#########################################
# Main
#########################################
if __name__ == '__main__':
    NepiTemplateApp()
