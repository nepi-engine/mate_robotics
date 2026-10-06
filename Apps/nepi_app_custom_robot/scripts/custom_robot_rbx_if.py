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
#
# The whole RBX surface for the Custom Robot app, in one self-contained module.
#
# One robot is one RBX device: this builds ONE RBXRobotIF representing the
# robot the operator composed from the app's connects, with the selected motor
# device's motors as that device's motor channels and the selected NPX device's
# navpose as its navpose. Modelled directly on WpilibRbxIF
# (first_robotics/nepi_app_wpilib_if/scripts/wpilib_rbx_if.py).
#
# TRANSPORT-BLIND. This module never imports a connect class or any transport.
# Its only inputs are injected callables, so the app node decides where motor
# commands go and where navpose comes from, and this file does not change when
# that does.
#
# FIXED CAPABILITIES. RBXRobotIF derives its has_* flags once, at construction,
# from which callbacks are non-None. Every callback below is therefore passed
# unconditionally (or is unconditionally None), and each one guards on its own
# inputs -- returning a safe value rather than raising when the motor device is
# not connected or the navpose is missing. The reported capabilities stay the
# same for the life of the instance; a different motor device means the node
# tears this down and builds a new one.
#
# AUTONOMY IS OUT OF SCOPE. No goto, go-home or home functions are passed.
# autonomousControlsReadyFunction is still a real callable that always returns
# False: RBXRobotIF's goto subscribers are registered regardless and call it
# unconditionally, so None there would raise on the first goto message.
#
# REGISTRY KEYS (2026-07 DECISION LOG). This module shares no node_if with
# anything. RBXRobotIF always builds its own NodeClassIF internally, and nothing
# here passes node_if= to any interface, so no domain prefix is needed. If a
# future change ever hands this module the app's node_if, every key it
# registers must be prefixed 'rbx_' first.

import threading

from nepi_interfaces.msg import AxisControls

from nepi_api.messages_if import MsgIF
from nepi_api.device_if_rbx import RBXRobotIF


#########################################
# Custom Robot RBX IF Class
#########################################

class CustomRobotRbxIF:

    ready = False

    rbx_if = None
    namespace = 'None'

    def __init__(self,
                 device_name,
                 getConnectedFunction,
                 getMotorNamesFunction,
                 getMotorSpeedRatioFunction,
                 setMotorSpeedFunction,
                 stopMotorFunction,
                 getNavPoseFunction,
                 log_name=None,
                 msg_if=None):
        """Build and own the RBX device for the custom robot.

        Args:
            device_name (str): Device name reported in RBX info and status.
            getConnectedFunction (callable): Returns True while the selected
                motor device is connected.
            getMotorNamesFunction (callable): Returns the selected motor
                device's motor names in motor-index order. Motor index here is
                the RBX motor index.
            getMotorSpeedRatioFunction (callable): Called as (motor_name).
                Returns that motor's reported speed ratio (0.0-1.0), or None.
            setMotorSpeedFunction (callable): Called as (motor_name,
                speed_ratio) to command one motor's speed.
            stopMotorFunction (callable): Called as (motor_name) to stop one
                motor.
            getNavPoseFunction (callable): Returns the robot's navpose dict, or
                None when no navpose source is connected.
            log_name (str): Optional log name for this interface.
            msg_if (MsgIF): Shared message interface, or None to create one.
        """
        self.class_name = type(self).__name__
        self.device_name = device_name

        self.getConnectedFunction = getConnectedFunction
        self.getMotorNamesFunction = getMotorNamesFunction
        self.getMotorSpeedRatioFunction = getMotorSpeedRatioFunction
        self.setMotorSpeedFunction = setMotorSpeedFunction
        self.stopMotorFunction = stopMotorFunction
        self.getNavPoseFunction = getNavPoseFunction

        if msg_if is not None:
            self.msg_if = msg_if
        else:
            self.msg_if = MsgIF(log_name=self.class_name)
        self.log_name = log_name

        ##############################
        # Stop tracking. goStop latches it, checkStopFunction reads and clears
        # it, the same latch WpilibRbxIF uses.
        self.stop_lock = threading.Lock()
        self.stop_triggered = False

        ##############################
        # Build the RBX device
        self.rbx_if = RBXRobotIF(
            device_info=dict(device_name=self.device_name,
                             path='',
                             serial_number='',
                             hw_version='',
                             sw_version=''),
            # No settings: the composed robot has nothing an operator sets on
            # the device itself (device selection is the app's own connects).
            # SettingsIF substitutes its NONE_* defaults for each None, so the
            # settings panel renders empty rather than fake.
            getSettingsFunction=None,
            setSettingFunction=None,
            axisControls=self.buildAxisControls(),
            # None: neither connect carries a battery field.
            getBatteryPercentFunction=None,
            # Empty: no robot state or mode enumeration. RBXRobotIF handles
            # empty lists (its bounds checks reject every set_state/set_mode
            # index and status reads "Not Set"), but it calls the get*Ind
            # functions unconditionally, so those must still be real callables.
            states=[],
            getStateIndFunction=self.getStateInd,
            setStateIndFunction=self.setStateInd,
            modes=[],
            getModeIndFunction=self.getModeInd,
            setModeIndFunction=self.setModeInd,
            checkStopFunction=self.checkStopFunction,
            # Empty: no named actions. The set functions are unreachable with
            # empty lists (RBXRobotIF bounds-checks first) and are passed anyway
            # so nothing about this device depends on a list being non-empty.
            setup_actions=[],
            setSetupActionIndFunction=self.setSetupActionInd,
            go_actions=[],
            setGoActionIndFunction=self.setGoActionInd,
            # Home is not readable or writable: there is no home pose source.
            getHomeFunction=None,
            setHomeFunction=None,
            manualControlsReadyFunction=self.manualControlsReady,
            getMotorControlRatios=self.getMotorControlRatios,
            setMotorControlRatio=self.setMotorControlRatio,
            autonomousControlsReadyFunction=self.autonomousControlsReady,
            goHomeFunction=None,
            goStopFunction=self.goStop,
            gotoPoseFunction=None,
            gotoPositionFunction=None,
            gotoLocationFunction=None,
            gotoVelocityFunction=None,
            getNavPoseCb=self.getNavPoseCb,
            navpose_update_rate=10,
            log_name=self.log_name,
            msg_if=self.msg_if)

        self.namespace = self.rbx_if.namespace

        self.ready = True
        self.msg_if.pub_info("Custom Robot RBX interface running at " + str(self.namespace))


    #######################
    # Class Public Methods
    #######################

    def get_ready_state(self):
        """Return the ready state of this interface.

        Returns:
            bool: True once the RBX device has been built.
        """
        return self.ready

    def get_namespace(self):
        """Return the namespace the RBX device is advertised at.

        Returns:
            str: The RBX device namespace, '<app node>/rbx'.
        """
        return self.namespace

    def get_device_ready_state(self):
        """Return the RBX device's own ready (not busy) state.

        Returns:
            bool: True when the device is idle and can accept a new command.
        """
        if self.rbx_if is None:
            return False
        try:
            return bool(self.rbx_if.status_msg.ready)
        except Exception:
            return False

    def shutdown(self):
        """Retract as much of the RBX device from the ROS graph as nepi_api allows.

        Unregisters the RBXRobotIF NodeClassIF (every rbx/* topic, service and
        param) and each child interface that exposes a public unregister. What
        CANNOT be retracted is logged rather than glossed over: RBXRobotIF has
        no teardown entry point of its own, its NPXDeviceIF exposes none, and
        nepi_sdk timers cannot be cancelled once started, so the npx status and
        navpose timers keep firing against unregistered publishers until the
        node restarts. A full retract needs an apps_mgr disable/enable cycle.

        Returns:
            bool: True if the RBX device's own NodeClassIF was unregistered.
        """
        if self.rbx_if is None:
            return False

        self.ready = False
        success = False

        # Child interfaces first, so their own topics go before the device's.
        for (attr_name, if_obj) in self.reachableChildIfs():
            try:
                if_obj.unregister()
            except Exception as e:
                self.msg_if.pub_warn("Could not unregister " + str(attr_name) +
                                     ": " + str(e))

        # The NPX navpose publisher is reachable through its own NavPoseIF.
        try:
            npx_if = getattr(self.rbx_if, 'npx_if', None)
            if npx_if is not None:
                navpose_if = getattr(npx_if, 'navpose_if', None)
                if navpose_if is not None:
                    navpose_if.unsubscribe()
                node_if = getattr(npx_if, 'node_if', None)
                if node_if is not None:
                    node_if.unregister_class()
        except Exception as e:
            self.msg_if.pub_warn("Could not unregister NPX interface: " + str(e))

        try:
            self.rbx_if.node_if.unregister_class()
            success = True
        except Exception as e:
            self.msg_if.pub_warn("Could not unregister RBX node class: " + str(e))

        self.msg_if.pub_warn("RBX device at " + str(self.namespace) + " torn down. "
                             "NPX status and navpose timers cannot be cancelled by "
                             "nepi_sdk, so a full retract needs an app restart.")
        self.rbx_if = None
        return success


    ###############################
    # Class Private Methods
    ###############################

    def reachableChildIfs(self):
        child_ifs = []
        for attr_name in ['image_if', 'settings_if', 'save_data_if', 'transform_if']:
            if_obj = getattr(self.rbx_if, attr_name, None)
            if if_obj is not None and hasattr(if_obj, 'unregister'):
                child_ifs.append((attr_name, if_obj))
        return child_ifs

    def buildAxisControls(self):
        # No goto function is passed, so no axis is commandable.
        axis_controls = AxisControls()
        axis_controls.x = False
        axis_controls.y = False
        axis_controls.z = False
        axis_controls.roll = False
        axis_controls.pitch = False
        axis_controls.yaw = False
        return axis_controls

    def isConnected(self):
        try:
            return bool(self.getConnectedFunction())
        except Exception:
            return False

    def getMotorNames(self):
        # Empty whenever the motor device is not connected, which makes every
        # motor index out of range in RBXRobotIF's own bounds check.
        if self.isConnected() is False:
            return []
        try:
            names = self.getMotorNamesFunction()
        except Exception as e:
            self.msg_if.pub_warn("Failed to read motor names: " + str(e), throttle_s=10.0)
            return []
        if names is None:
            return []
        return list(names)

    def logCommandEntry(self, command_name, detail):
        self.msg_if.pub_info("RBX command in: " + str(command_name) + "  " + str(detail))

    def stopAllMotors(self):
        motor_names = self.getMotorNames()
        success = len(motor_names) > 0
        for motor_name in motor_names:
            try:
                self.stopMotorFunction(motor_name)
            except Exception as e:
                success = False
                self.msg_if.pub_warn("Failed to stop motor " + str(motor_name) + ": " + str(e))
        return success

    ##########################
    # RBX Interface Functions

    def getStateInd(self):
        # No robot states. RBXRobotIF calls this unconditionally and displays
        # "Not Set" for the empty list.
        return 0

    def setStateInd(self, state_ind):
        # Unreachable with an empty states list (RBXRobotIF bounds-checks first)
        return False

    def getModeInd(self):
        return 0

    def setModeInd(self, mode_ind):
        # Unreachable with an empty modes list
        return False

    def setSetupActionInd(self, action_ind):
        # Unreachable with an empty setup_actions list
        return False

    def setGoActionInd(self, action_ind):
        # Unreachable with an empty go_actions list
        return False

    def checkStopFunction(self):
        # Polled by RBXRobotIF's goto loops; none are reachable here because no
        # goto function is passed. Read-and-clear, as in WpilibRbxIF. When a
        # stop is pending it re-issues stop_motor to every motor, which is
        # idempotent, so whichever path observes the stop also leaves the
        # motors stopped.
        with self.stop_lock:
            triggered = self.stop_triggered
            self.stop_triggered = False
        if triggered is True:
            self.stopAllMotors()
        return triggered

    def manualControlsReady(self):
        # Gates per-motor manual control on the motor device being connected.
        # Must stay a real callable -- RBXRobotIF.setMotorControl calls it
        # unconditionally.
        return self.isConnected()

    def autonomousControlsReady(self):
        # Autonomy is out of scope. Always False, so every goto is rejected
        # with RBXRobotIF's own "Autonomous Controls not Ready" error.
        return False

    def getMotorControlRatios(self):
        # One ratio per motor of the selected motor device, as that device
        # reports it. A motor that has not reported reads 0.0. Empty when the
        # device is not connected.
        ratios = []
        for motor_name in self.getMotorNames():
            ratio = None
            try:
                ratio = self.getMotorSpeedRatioFunction(motor_name)
            except Exception as e:
                self.msg_if.pub_warn("Failed to read speed ratio for motor " +
                                     str(motor_name) + ": " + str(e), throttle_s=10.0)
            if ratio is None:
                ratio = 0.0
            ratios.append(float(ratio))
        return ratios

    def setMotorControlRatio(self, motor_ind, speed_ratio):
        # Maps an RBX motor index to the motor device's motor name in index
        # order and commands that motor's speed. Logged on the way in, before
        # any rejection, so what RBXRobotIF asked for is on the record.
        self.logCommandEntry("set_motor_control_ratio", "motor_ind " + str(motor_ind) +
                             "  speed_ratio " + str(speed_ratio))
        motor_names = self.getMotorNames()
        if motor_ind < 0 or motor_ind >= len(motor_names):
            self.msg_if.pub_warn("Motor control ignored: motor " + str(motor_ind) +
                                 " is out of range for " + str(len(motor_names)) + " motors")
            return
        speed_ratio = max(0.0, min(1.0, float(speed_ratio)))
        try:
            self.setMotorSpeedFunction(motor_names[motor_ind], speed_ratio)
        except Exception as e:
            self.msg_if.pub_warn("Failed to write motor command for motor " +
                                 str(motor_ind) + ": " + str(e))

    def goStop(self):
        # Latches the stop for checkStopFunction and stops every motor now.
        # Returns False when there is no connected motor to stop, which
        # RBXRobotIF reports as cmd_success.
        motor_names = self.getMotorNames()
        self.logCommandEntry("go_stop", "stopping " + str(len(motor_names)) + " motors")
        with self.stop_lock:
            self.stop_triggered = True
        return self.stopAllMotors()

    def getNavPoseCb(self):
        try:
            return self.getNavPoseFunction()
        except Exception as e:
            self.msg_if.pub_warn("Failed to read navpose: " + str(e), throttle_s=10.0)
            return None
