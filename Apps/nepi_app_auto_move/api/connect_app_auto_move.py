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

import time

from std_msgs.msg import Bool, Empty, String, Float32

from nepi_app_auto_move.msg import NepiAppAutoMoveStatus

from nepi_sdk import nepi_sdk

from nepi_api.messages_if import MsgIF
from nepi_api.connect_node_if import ConnectNodeClassIF

APP_NODE_NAME = 'app_auto_move'

# Connect names, matching the connect_name each connect IF in AutoMoveIF is
# constructed with. Selecting a robot or an image means publishing a String to
# <app>/<connect_name>/select_topic, which is what the RUI selectors do.
RBX_CONNECT_NAME = 'rbx_connect'
IMAGE_CONNECT_NAME = 'image_connect'


class ConnectAppAutoMove:
    msg_if = None
    ready = False
    namespace = '~'

    con_node_if = None

    connected = False
    status_msg = None
    status_connected = False

    # Optional consumer callback, invoked from _statusCb with the status dict.
    # The auto move app publishes no data product this interface subscribes to,
    # so the status dict IS the data dict here.
    dataCB = None

    #######################
    ### IF Initialization

    def __init__(self, namespace = None, dataCB = None):
        self.class_name = type(self).__name__
        self.base_namespace = nepi_sdk.get_base_namespace()
        self.node_name = nepi_sdk.get_node_name()
        self.node_namespace = nepi_sdk.get_node_namespace()

        self.msg_if = MsgIF()
        self.msg_if.pub_info("Starting IF Initialization Processes")

        if namespace is None:
            namespace = nepi_sdk.create_namespace(self.base_namespace, APP_NODE_NAME)
        # The app node namespace carries the status topic and every command
        # topic; the two connect namespaces hang off it.
        self.namespace = nepi_sdk.get_full_namespace(namespace)
        self.rbx_connect_namespace = nepi_sdk.create_namespace(self.namespace, RBX_CONNECT_NAME)
        self.image_connect_namespace = nepi_sdk.create_namespace(self.namespace, IMAGE_CONNECT_NAME)

        self.dataCB = dataCB

        # Configs Config Dict ####################
        self.CFGS_DICT = {
            'namespace': self.namespace
        }

        # Services Config Dict ####################
        self.SRVS_DICT = None

        # Publishers Config Dict ####################
        self.PUBS_DICT = {
            'select_rbx': {
                'namespace': self.rbx_connect_namespace,
                'topic': 'select_topic',
                'msg': String,
                'qsize': 1
            },
            'select_image': {
                'namespace': self.image_connect_namespace,
                'topic': 'select_topic',
                'msg': String,
                'qsize': 1
            },
            'set_goto_x': {
                'namespace': self.namespace,
                'topic': 'set_goto_x',
                'msg': Float32,
                'qsize': 1
            },
            'set_goto_y': {
                'namespace': self.namespace,
                'topic': 'set_goto_y',
                'msg': Float32,
                'qsize': 1
            },
            'set_goto_z': {
                'namespace': self.namespace,
                'topic': 'set_goto_z',
                'msg': Float32,
                'qsize': 1
            },
            'set_max_move': {
                'namespace': self.namespace,
                'topic': 'set_max_move',
                'msg': Float32,
                'qsize': 1
            },
            'set_goto_velocity_enabled': {
                'namespace': self.namespace,
                'topic': 'set_goto_velocity_enabled',
                'msg': Bool,
                'qsize': 1
            },
            'set_move_speed': {
                'namespace': self.namespace,
                'topic': 'set_move_speed',
                'msg': Float32,
                'qsize': 1
            },
            'set_move_speed_ratio': {
                'namespace': self.namespace,
                'topic': 'set_move_speed_ratio',
                'msg': Float32,
                'qsize': 1
            },
            'set_goto_vx': {
                'namespace': self.namespace,
                'topic': 'set_goto_vx',
                'msg': Float32,
                'qsize': 1
            },
            'set_goto_vy': {
                'namespace': self.namespace,
                'topic': 'set_goto_vy',
                'msg': Float32,
                'qsize': 1
            },
            'set_goto_yaw_rate': {
                'namespace': self.namespace,
                'topic': 'set_goto_yaw_rate',
                'msg': Float32,
                'qsize': 1
            },
            'set_goto_duration': {
                'namespace': self.namespace,
                'topic': 'set_goto_duration',
                'msg': Float32,
                'qsize': 1
            },
            'set_depth_map_transparency': {
                'namespace': self.namespace,
                'topic': 'set_depth_map_transparency',
                'msg': Float32,
                'qsize': 1
            },
            'set_show_depth_map': {
                'namespace': self.namespace,
                'topic': 'set_show_depth_map',
                'msg': Bool,
                'qsize': 1
            },
            'set_show_objects': {
                'namespace': self.namespace,
                'topic': 'set_show_objects',
                'msg': Bool,
                'qsize': 1
            },
            'set_show_targets': {
                'namespace': self.namespace,
                'topic': 'set_show_targets',
                'msg': Bool,
                'qsize': 1
            },
            'set_show_obstacles': {
                'namespace': self.namespace,
                'topic': 'set_show_obstacles',
                'msg': Bool,
                'qsize': 1
            },
            'set_show_crosshair': {
                'namespace': self.namespace,
                'topic': 'set_show_crosshair',
                'msg': Bool,
                'qsize': 1
            },
            'goto_trigger': {
                'namespace': self.namespace,
                'topic': 'goto_trigger',
                'msg': Empty,
                'qsize': 1
            },
            'goto_cancel': {
                'namespace': self.namespace,
                'topic': 'goto_cancel',
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
                'msg': NepiAppAutoMoveStatus,
                'qsize': 1,
                'callback': self._statusCb
            }
        }

        # Create Node Class ####################
        # NOTE: ConnectNodeClassIF takes no 'namespace' kwarg -- the target
        # app's namespace is carried by the per-entry 'namespace' fields in
        # the dicts above (and CFGS_DICT['namespace']). Some shipped
        # connect_app_* files pass namespace=/log_class_name= anyway; those
        # kwargs don't exist and would TypeError if ever instantiated.
        self.con_node_if = ConnectNodeClassIF(
            configs_dict = self.CFGS_DICT,
            services_dict = self.SRVS_DICT,
            pubs_dict = self.PUBS_DICT,
            subs_dict = self.SUBS_DICT,
            msg_if = self.msg_if
        )

        self.con_node_if.wait_for_ready()

        self.ready = True
        self.msg_if.pub_info("IF Initialization Complete")


    #######################
    # Class Public Methods
    #######################

    def get_ready_state(self):
        """Return True once this connect interface has finished initializing."""
        return self.ready

    def get_namespace(self):
        """Return the app namespace this interface talks to."""
        return self.namespace

    def check_connection(self):
        """Return True if any status message has been received."""
        return self.connected

    def check_status_connection(self):
        """Return True if the app status topic is publishing."""
        return self.status_connected

    def get_status_dict(self):
        """Return the last received NepiAppAutoMoveStatus message as a dictionary.

        Returns:
            dict: The status message converted to a dict, or None if no status
                message has arrived yet.
        """
        if self.status_msg is not None:
            return nepi_sdk.convert_msg2dict(self.status_msg)
        return None

    def get_status_msg(self):
        """Return the last received NepiAppAutoMoveStatus message.

        Returns:
            NepiAppAutoMoveStatus: The most recent status message, or None if no
                status message has arrived yet.
        """
        return self.status_msg

    def select_rbx_device(self, device_namespace):
        """Select which RBX robot the app drives.

        Args:
            device_namespace (str): Fully-qualified RBX device namespace, or
                'None' to clear the selection. A namespace the app has not
                discovered is ignored by the connect interface.
        """
        msg = String()
        msg.data = device_namespace
        self.con_node_if.publish_pub('select_rbx', msg)

    def select_image_topic(self, image_topic):
        """Select which image the app displays and takes clicks on.

        The app derives its depth map, depth map image, objects and targets from
        whatever is selected here.

        Args:
            image_topic (str): Fully-qualified image topic, or 'None' to clear
                the selection.
        """
        msg = String()
        msg.data = image_topic
        self.con_node_if.publish_pub('select_image', msg)

    def set_goto_x(self, x_meters):
        """Set the forward component of the requested move.

        Args:
            x_meters (float): Forward offset in METERS, robot body frame.
                Overwritten by the next click on the image.
        """
        msg = Float32()
        msg.data = x_meters
        self.con_node_if.publish_pub('set_goto_x', msg)

    def set_goto_y(self, y_meters):
        """Set the left component of the requested move.

        Args:
            y_meters (float): Left offset in METERS, robot body frame.
                Overwritten by the next click on the image.
        """
        msg = Float32()
        msg.data = y_meters
        self.con_node_if.publish_pub('set_goto_y', msg)

    def set_goto_z(self, z_meters):
        """Set the up component of the requested move.

        Args:
            z_meters (float): Up offset in METERS, robot body frame.
                Overwritten by the next click on the image.
        """
        msg = Float32()
        msg.data = z_meters
        self.con_node_if.publish_pub('set_goto_z', msg)

    def set_max_move(self, max_move_meters):
        """Set the largest forward distance one click may request.

        A click farther than this is scaled back along the same bearing rather
        than truncated, so the commanded direction is preserved.

        Args:
            max_move_meters (float): Requested limit in METERS. The app clamps
                to 0.1-100 m and persists the value.
        """
        msg = Float32()
        msg.data = max_move_meters
        self.con_node_if.publish_pub('set_max_move', msg)

    def set_goto_velocity_enabled(self, enabled):
        """Switch the app between position mode and timed velocity mode.

        Velocity mode does not change how a click is resolved. It changes what
        a goto issues: a body-frame velocity held for a duration, open loop,
        instead of a distance the robot closes on.

        Args:
            enabled (bool): True for velocity mode, False for position mode.
                Persisted by the app.
        """
        msg = Bool()
        msg.data = enabled
        self.con_node_if.publish_pub('set_goto_velocity_enabled', msg)

    def set_move_speed(self, move_speed_mps):
        """Set the fallback speed used while the robot reports no max speed.

        A clicked distance is divided by the move speed to get a velocity move's
        duration. While the selected robot reports a max speed the app uses
        set_move_speed_ratio() of it instead, and this value is ignored.

        Args:
            move_speed_mps (float): Speed in METERS PER SECOND. The app clamps
                to 0.1-20 m/s and persists the value, per app rather than per
                robot.
        """
        msg = Float32()
        msg.data = move_speed_mps
        self.con_node_if.publish_pub('set_move_speed', msg)

    def set_move_speed_ratio(self, move_speed_ratio):
        """Set the fraction of the robot's reported max speed a move runs at.

        Used whenever the selected robot reports a max speed, in place of
        set_move_speed().

        Args:
            move_speed_ratio (float): 0.0-1.0. The app clamps to 0.05-1.0 and
                persists the value, per app rather than per robot.
        """
        msg = Float32()
        msg.data = move_speed_ratio
        self.con_node_if.publish_pub('set_move_speed_ratio', msg)

    def set_goto_vx(self, x_mps):
        """Set the forward velocity of the requested timed move.

        Args:
            x_mps (float): Forward velocity in METERS PER SECOND, robot body
                frame. Overwritten by the next click while velocity mode is on.
        """
        msg = Float32()
        msg.data = x_mps
        self.con_node_if.publish_pub('set_goto_vx', msg)

    def set_goto_vy(self, y_mps):
        """Set the left velocity of the requested timed move.

        Args:
            y_mps (float): Left velocity in METERS PER SECOND, robot body
                frame. Overwritten by the next click while velocity mode is on.
        """
        msg = Float32()
        msg.data = y_mps
        self.con_node_if.publish_pub('set_goto_vy', msg)

    def set_goto_yaw_rate(self, yaw_degps):
        """Set the yaw rate of the requested timed move.

        A click sets this to zero -- a click gives a translation, not a
        rotation -- so this is the only way to command a turn.

        Args:
            yaw_degps (float): Yaw rate in DEGREES PER SECOND, positive to
                port. Converted to rad/s before it reaches the robot.
        """
        msg = Float32()
        msg.data = yaw_degps
        self.con_node_if.publish_pub('set_goto_yaw_rate', msg)

    def set_goto_duration(self, duration_s):
        """Set how long the robot holds the requested velocity.

        Args:
            duration_s (float): Duration in SECONDS. The app clamps to at most
                60 s. Overwritten by the next click while velocity mode is on.
        """
        msg = Float32()
        msg.data = duration_s
        self.con_node_if.publish_pub('set_goto_duration', msg)

    def set_depth_map_transparency(self, transparency):
        """Set how transparent the depth map overlay is drawn.

        Args:
            transparency (float): 0.0 fully opaque through 1.0 invisible.
                Clamped by the app.
        """
        msg = Float32()
        msg.data = transparency
        self.con_node_if.publish_pub('set_depth_map_transparency', msg)

    def set_show_depth_map(self, enabled):
        """Enable or disable the depth map overlay layer.

        Args:
            enabled (bool): True to blend the depth map render over the frame.
        """
        msg = Bool()
        msg.data = enabled
        self.con_node_if.publish_pub('set_show_depth_map', msg)

    def set_show_objects(self, enabled):
        """Enable or disable the object detection overlay layer.

        Args:
            enabled (bool): True to draw detection boxes and labels.
        """
        msg = Bool()
        msg.data = enabled
        self.con_node_if.publish_pub('set_show_objects', msg)

    def set_show_targets(self, enabled):
        """Enable or disable the target marker overlay layer.

        Args:
            enabled (bool): True to draw target markers.
        """
        msg = Bool()
        msg.data = enabled
        self.con_node_if.publish_pub('set_show_targets', msg)

    def set_show_obstacles(self, enabled):
        """Enable or disable the obstacle overlay layer.

        Args:
            enabled (bool): True to draw obstacle boxes. Has no effect where the
                obstacles app is not installed.
        """
        msg = Bool()
        msg.data = enabled
        self.con_node_if.publish_pub('set_show_obstacles', msg)

    def set_show_crosshair(self, enabled):
        """Enable or disable the clicked-point crosshair overlay layer.

        Args:
            enabled (bool): True to draw a crosshair at the last clicked pixel.
        """
        msg = Bool()
        msg.data = enabled
        self.con_node_if.publish_pub('set_show_crosshair', msg)

    def goto(self):
        """Run the goto process for the current x/y/z offset.

        Plans a move, then hands each step of the plan to the selected robot.
        Ignored while a goto is already planning or moving, and while no robot
        is connected.
        """
        self.con_node_if.publish_pub('goto_trigger', Empty())

    def goto_cancel(self):
        """Stop the running goto process and return the app to idle.

        Sends a stop to the selected robot and discards the remaining plan.
        """
        self.con_node_if.publish_pub('goto_cancel', Empty())

    def save_config(self):
        """Persist the app's current configuration."""
        self.con_node_if.publish_pub('save_config', Empty())

    def reset_config(self):
        """Reset the app's configuration to the last saved values."""
        self.con_node_if.publish_pub('reset_config', Empty())

    def factory_reset_config(self):
        """Reset the app's configuration to factory values."""
        self.con_node_if.publish_pub('factory_reset_config', Empty())

    def unregister(self):
        """Unregister every publisher and subscriber this interface created."""
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

        if self.dataCB is not None:
            self.dataCB(self.get_status_dict())
