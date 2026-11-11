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

import copy

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_auto_move

from nepi_api.messages_if import MsgIF
from nepi_api.auto_move_if import AutoMoveIF


#########################################
# Node Class
#########################################

class NepiAutoMoveApp(object):

    #######################
    ### Node Initialization
    DEFAULT_NODE_NAME = "app_auto_move"  # Can be overwritten by launch command

    APP_DESCRIPTION = 'Click to go move process'

    auto_move_if = None

    def __init__(self):
        #### APP NODE INIT SETUP ####
        nepi_sdk.init_node(name = self.DEFAULT_NODE_NAME)
        self.class_name = type(self).__name__
        self.base_namespace = nepi_sdk.get_base_namespace()
        self.node_name = nepi_sdk.get_node_name()
        self.node_namespace = nepi_sdk.get_node_namespace()

        ##############################
        # Create Msg Class
        self.msg_if = MsgIF()
        self.msg_if.pub_info("Starting Node Initialization Processes")

        ##############################
        # Create the AutoMove IF.
        #
        # controls_dict is the SDK module's control DEFINITION dict (the
        # nepi_controls init-dict form). AutoMoveIF hands it to a ControlsIF and
        # hands the ControlsIF's live controls_dict back to planMove on every
        # goto. The node never reads control values itself.
        controls_dict = copy.deepcopy(nepi_auto_move.MOVE_CONTROLS_DICT)

        self.auto_move_if = AutoMoveIF(
                            namespace = self.node_namespace,
                            description = self.APP_DESCRIPTION,
                            controls_dict = controls_dict,
                            planMoveFunction = self.planMove,
                            planVelocityMoveFunction = self.planVelocityMove,
                            msg_if = self.msg_if
                            )

        #########################################################
        ## Initiation Complete
        self.msg_if.pub_info("Initialization Complete")

        # Spin forever
        nepi_sdk.spin()
        #########################################################

    def planMove(self, goto_dict, np_depth_map, objects_list, targets_list, robot_dict, controls_dict, obstacles_list = None):
        # Called by AutoMoveIF once per goto trigger. Thin pass-through to the
        # SDK module so the planner ships in nepi_sdk and any other node can call
        # it with the same planner arguments.
        return nepi_auto_move.plan_move(goto_dict,
                                        np_depth_map,
                                        objects_list,
                                        targets_list,
                                        robot_dict,
                                        controls_dict,
                                        obstacles_list)

    def planVelocityMove(self, goto_dict, np_depth_map, objects_list, targets_list, robot_dict, controls_dict, obstacles_list = None):
        # The velocity-mode twin of planMove. Same arguments, same pass-through
        # shape; the returned steps carry m/s and deg/s plus a duration instead
        # of metres and degrees.
        return nepi_auto_move.plan_velocity_move(goto_dict,
                                                 np_depth_map,
                                                 objects_list,
                                                 targets_list,
                                                 robot_dict,
                                                 controls_dict,
                                                 obstacles_list)


#########################################
# Main
#########################################
if __name__ == '__main__':
    NepiAutoMoveApp()
