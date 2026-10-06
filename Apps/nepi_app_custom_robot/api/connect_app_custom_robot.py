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
# Redistributions in source code must retain this top-level comment block.
# Plagiarizing this software to sidestep the license obligations is illegal.
#
# Contact Information:
# ====================
# - mailto:nepi@numurus.com

##############################################################################
# NEPI APP CUSTOM_ROBOT -- CONNECT API
# ---------------------------------------------------------------------------
# A thin client class other nodes import to command this app without knowing
# its topic layout. It mirrors the node's interface through ConnectNodeClassIF:
# the node's SUBSCRIBERS become this class's PUBLISHERS, and the node's status
# topic becomes a subscription cached into status_msg.
#
# This file installs INTO the shared nepi_api package (CMakeLists copies api/
# into .../dist-packages/nepi_api), so consumers write
#     from nepi_api.connect_app_custom_robot import ConnectAppCustomRobot
#
# TWO CONSEQUENCES OF THAT INSTALL, BOTH LEARNED THE HARD WAY:
#   * api/ is NOT live-synced. deploy_app.sh syncs only scripts/ to the running
#     device; a change here needs a catkin build before it takes effect.
#   * the install has no --delete, so a file you REMOVE from api/ stays on the
#     device forever and keeps shadowing the engine's own copy of that name.
#     Never name a file here after something that exists in nepi_api.
#
# THE CONTROLS HALF OF THE API
# The app's adjustable state lives in a ControlsIF, so every state setter below
# publishes ONE UpdateControl to <app>/controls/update_control instead of its
# own typed topic. Public method signatures are kept unchanged across that move
# so existing callers do not break. Commands (trigger_action) stay as they are.
#
# GOTCHA: every pub/sub entry must use the TARGET APP's namespace
# (self.namespace, or self.controls_namespace below it), never the calling
# node's self.node_namespace. Getting this wrong publishes into the caller's
# own namespace and silently does nothing. Several shipped connect_app_* files
# have exactly this bug.
##############################################################################

import time

from std_msgs.msg import Empty

from nepi_app_custom_robot.msg import NepiAppCustomRobotStatus
from nepi_interfaces.msg import ControlsStatus, UpdateControl

from nepi_sdk import nepi_sdk

from nepi_api.messages_if import MsgIF
from nepi_api.connect_node_if import ConnectNodeClassIF

APP_NODE_NAME = 'app_custom_robot'

# Must match CONTROLS_NAME in scripts/custom_robot_app_node.py. A ControlsIF is
# always a direct child of the node namespace.
CONTROLS_NAME = 'controls'


class ConnectAppCustomRobot:
    msg_if = None
    ready = False
    namespace = '~'
    controls_namespace = ''

    con_node_if = None

    connected = False
    status_msg = None
    status_connected = False
    controls_status_msg = None

    #######################
    ### IF Initialization

    def __init__(self, namespace=None):
        self.class_name = type(self).__name__
        self.base_namespace = nepi_sdk.get_base_namespace()
        self.node_name = nepi_sdk.get_node_name()
        self.node_namespace = nepi_sdk.get_node_namespace()

        self.msg_if = MsgIF(log_name=self.class_name)
        self.msg_if.pub_info("Starting IF Initialization Processes")

        if namespace is None:
            namespace = nepi_sdk.create_namespace(self.base_namespace, APP_NODE_NAME)
        self.namespace = nepi_sdk.get_full_namespace(namespace)
        self.controls_namespace = nepi_sdk.create_namespace(self.namespace, CONTROLS_NAME)

        # Configs Config Dict ####################
        self.CFGS_DICT = {
            'namespace': self.namespace
        }

        # Services Config Dict ####################
        self.SRVS_DICT = None

        # Publishers Config Dict ####################
        self.PUBS_DICT = {
            'update_control': {
                'namespace': self.controls_namespace,
                'topic': 'update_control',
                'msg': UpdateControl,
                'qsize': 1,
                'latch': False
            },
            'trigger_action': {
                'namespace': self.namespace,
                'topic': 'trigger_action',
                'msg': Empty,
                'qsize': 1
            },
            'save_config': {
                'namespace': self.namespace,
                'topic': 'save_config',
                'msg': Empty,
                'qsize': None,
                'latch': False
            },
            'reset_config': {
                'namespace': self.namespace,
                'topic': 'reset_config',
                'msg': Empty,
                'qsize': None,
                'latch': False
            },
            'factory_reset_config': {
                'namespace': self.namespace,
                'topic': 'factory_reset_config',
                'msg': Empty,
                'qsize': None,
                'latch': False
            }
        }

        # Subscribers Config Dict ####################
        self.SUBS_DICT = {
            'status_sub': {
                'namespace': self.namespace,
                'topic': 'status',
                'msg': NepiAppCustomRobotStatus,
                'qsize': 1,
                'callback': self._statusCb
            },
            'controls_status_sub': {
                'namespace': self.controls_namespace,
                'topic': 'status',
                'msg': ControlsStatus,
                'qsize': 1,
                'callback': self._controlsStatusCb
            }
        }

        # Create Node Class ####################
        # NOTE: ConnectNodeClassIF takes no 'namespace' and no
        # 'log_class_name' kwarg -- verified against
        # nepi_api/connect_node_if.py. The target app's namespace is carried by
        # the per-entry 'namespace' fields in the dicts above (and
        # CFGS_DICT['namespace']). Several shipped connect_app_* files pass
        # both anyway; those would TypeError if ever instantiated.
        self.con_node_if = ConnectNodeClassIF(
            configs_dict=self.CFGS_DICT,
            services_dict=self.SRVS_DICT,
            pubs_dict=self.PUBS_DICT,
            subs_dict=self.SUBS_DICT,
            msg_if=self.msg_if
        )

        self.con_node_if.wait_for_ready()

        self.ready = True
        self.msg_if.pub_info("IF Initialization Complete")


    #######################
    # Class Public Methods
    #######################

    def get_ready_state(self):
        return self.ready

    def get_namespace(self):
        return self.namespace

    def check_connection(self):
        return self.connected

    def check_status_connection(self):
        return self.status_connected

    def get_status_dict(self):
        if self.status_msg is not None:
            return nepi_sdk.convert_msg2dict(self.status_msg)
        return None

    def get_controls_namespace(self):
        """Return the namespace the app's controls status and update_control topics sit on."""
        return self.controls_namespace

    def get_controls_status_dict(self):
        """Return the app's last received ControlsStatus message as a dict.

        Returns:
            dict: The controls status as a dict, or None if no status has arrived yet.
        """
        if self.controls_status_msg is not None:
            return nepi_sdk.convert_msg2dict(self.controls_status_msg)
        return None

    def set_control_value(self, control_name, value, index=None):
        """Update one of the app's controls.

        Args:
            control_name (str): Name of the control, as it appears in the app's ControlsStatus.
            value: New value. Lists are sent entry by entry; anything else is sent as one value.
            index (int, optional): Component index for a multi-value control. Defaults to None,
                which replaces the whole value.
        """
        msg = UpdateControl()
        msg.name = str(control_name)
        if isinstance(value, (list, tuple)):
            msg.value = [str(item) for item in value]
        else:
            msg.value = [str(value)]
        msg.index = '' if index is None else str(index)
        self.con_node_if.publish_pub('update_control', msg)

    def set_enabled(self, enabled):
        """Enable or disable the app."""
        self.set_control_value('enabled', bool(enabled))

    def set_option(self, option):
        """Set the selected option string."""
        self.set_control_value('selected_option', str(option))

    def set_value(self, value):
        """Set the float value."""
        self.set_control_value('value', float(value))

    def trigger_action(self):
        """Trigger the one-shot action.

        The app implements this once; the Button control and this topic both
        reach the same method.
        """
        self.con_node_if.publish_pub('trigger_action', Empty())

    def save_config(self):
        self.con_node_if.publish_pub('save_config', Empty())

    def reset_config(self):
        self.con_node_if.publish_pub('reset_config', Empty())

    def factory_reset_config(self):
        self.con_node_if.publish_pub('factory_reset_config', Empty())

    def unregister(self):
        self._unregisterNode()


    ###############################
    # Class Private Methods
    ###############################

    def _unregisterNode(self):
        self.connected = False
        if self.con_node_if is not None:
            self.msg_if.pub_warn("Unregistering: " + str(self.namespace))
            try:
                self.con_node_if.unregister_class()
                time.sleep(1)
                self.con_node_if = None
                self.namespace = None
                self.status_connected = False
            except Exception as e:
                self.msg_if.pub_warn("Failed to unregister: " + str(e))

    def _statusCb(self, status_msg):
        self.status_connected = True
        self.connected = True
        self.status_msg = status_msg

    def _controlsStatusCb(self, status_msg):
        self.controls_status_msg = status_msg
