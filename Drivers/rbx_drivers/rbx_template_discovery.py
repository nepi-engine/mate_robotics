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
# RBX (Robot) DRIVER TEMPLATE -- DISCOVERY (CALL model)
# ---------------------------------------------------------------------------
# RBX drivers control a whole vehicle (drone, rover, boat). This discovery uses
# the CALL model (drivers_mgr calls discoveryFunction each cycle), like lsx/ptx.
#
# WHAT MAKES RBX SPECIAL: many robots speak a protocol carried by a SEPARATE
# ROS driver (e.g. ArduPilot over MAVLink is bridged by a `mavros` node). So an
# RBX discovery often launches TWO processes per device:
#   (a) a COMPANION protocol node (mavros_node, etc.) via subprocess.Popen, and
#   (b) the NEPI RBX node (this driver's node) via nepi_drvs.launchDriverNode.
# Both handles are tracked so purge tears BOTH down. This template shows the
# structure with the companion launch stubbed out -- delete it if your robot's
# protocol needs no companion node.
##############################################################################

import os
import subprocess

import serial   # TODO: drop if your link is TCP/UDP-only

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_drvs
from nepi_sdk import nepi_system
from nepi_sdk import nepi_serial

PKG_NAME = 'RBX_TEMPLATE'
FILE_TYPE = 'DISCOVERY'


class RbxTemplateDiscovery:

    NODE_LOAD_TIME_SEC = 10
    launch_time_dict = dict()
    retry = True
    dont_retry_list = []

    # path_str -> {'node_name','rbx_subproc','companion_subproc'}
    active_devices_dict = dict()
    node_launch_name = "rbx_template"

    excludedDevices = ['ttyTHS', 'ttyTCU']

    ##########################################################################
    def __init__(self):
        self.log_name = PKG_NAME.lower() + "_discovery"
        self.logger = nepi_sdk.logger(log_name=self.log_name)
        self.logger.log_info("Starting Initialization")
        self.logger.log_info("Initialization Complete")

    ##########################################################################
    def discoveryFunction(self, available_paths_list, active_paths_list,
                          base_namespace, drv_dict, retry_enabled=True):
        self.drv_dict = drv_dict
        self.available_paths_list = available_paths_list
        self.active_paths_list = active_paths_list
        self.base_namespace = base_namespace

        try:
            opts = drv_dict['DISCOVERY_DICT']['OPTIONS']
            self.baud_str = opts['baud_rate']['value']
        except Exception as e:
            self.logger.log_warn("Failed to load options " + str(e))
            return self.active_paths_list

        self.retry = retry_enabled
        if self.retry:
            self.dont_retry_list = []

        # Purge dead vehicles (companion OR rbx process gone / port dropped).
        for path_str in list(self.active_devices_dict.keys()):
            if self.checkOnDevice(path_str) is False:
                self._teardown(path_str)

        # Probe candidate serial ports for a live robot link.
        for path_str in nepi_serial.get_serial_ports_list():
            if any(x in path_str for x in self.excludedDevices):
                continue
            if path_str in self.active_paths_list:
                continue
            if self.checkForDevice(path_str) and path_str not in self.dont_retry_list:
                self.launchDeviceNode(path_str)

        return self.active_paths_list

    ##########################################################################
    # Confirm a robot is on this port (e.g. read a heartbeat packet).
    ##########################################################################
    def checkForDevice(self, path_str):
        try:
            ser = serial.Serial(path_str, int(self.baud_str), timeout=1)
        except Exception:
            return False
        found = False
        try:
            # TODO: read your protocol's heartbeat/handshake here.
            # e.g. for MAVLink, read_until(b'\xFD') then match a HEARTBEAT msg id.
            data = ser.read(64)
            found = len(data) > 0    # placeholder -- replace with a real match!
        except Exception:
            found = False
        finally:
            ser.close()
        return found

    ##########################################################################
    def checkOnDevice(self, path_str):
        if path_str not in self.available_paths_list:
            return False
        entry = self.active_devices_dict[path_str]
        if entry['rbx_subproc'].poll() is not None:
            return False
        if entry['companion_subproc'] is not None and entry['companion_subproc'].poll() is not None:
            return False
        return True

    ##########################################################################
    def launchDeviceNode(self, path_str):
        if path_str in self.launch_time_dict:
            if (nepi_sdk.get_time() - self.launch_time_dict[path_str]) < self.NODE_LOAD_TIME_SEC:
                return False

        device_name = self.node_launch_name + "_" + path_str.split('/')[-1]
        node_name = nepi_system.get_device_alias(device_name)
        self.launch_time_dict[path_str] = nepi_sdk.get_time()

        # ---- (a) OPTIONAL companion protocol node ---------------------------
        # Robots whose protocol is bridged by another ROS package launch it here
        # as a subprocess.Popen. Example shape (ArduPilot/MAVLink via mavros):
        #   companion_ns = nepi_sdk.create_namespace(self.base_namespace, "mavlink_" + device_name)
        #   subprocess.run(['rosparam','load', APM_PLUGINLISTS_PATH, companion_ns])
        #   companion = subprocess.Popen(['rosrun','mavros','mavros_node',
        #                                 '__name:=mavlink_' + device_name,
        #                                 '_fcu_url:=' + path_str + ':' + self.baud_str])
        # Set companion = None if your robot needs no companion node.
        companion = None
        companion_node_name = None

        # ---- (b) the NEPI RBX node -----------------------------------------
        self.drv_dict['DEVICE_DICT'] = {
            'device_name': device_name,
            'device_path': path_str,
            'baud_str': self.baud_str,
            # 'companion_node_name': companion_node_name,   # pass if you launched one
        }
        dict_param_name = nepi_sdk.create_namespace(self.base_namespace, node_name + "/drv_dict")
        nepi_sdk.set_param(dict_param_name, self.drv_dict)

        file_name = self.drv_dict['NODE_DICT']['file_name']
        [success, msg, rbx_subproc] = nepi_drvs.launchDriverNode(file_name, node_name)
        if success:
            self.active_devices_dict[path_str] = {
                'node_name': node_name,
                'rbx_subproc': rbx_subproc,
                'companion_subproc': companion,
            }
            if path_str not in self.active_paths_list:
                self.active_paths_list.append(path_str)
            self.logger.log_info("Launched RBX node " + node_name)
        else:
            # If the RBX node failed, tear down any companion we started.
            if companion is not None:
                nepi_drvs.killDriverNode(companion_node_name, companion)
            if self.retry is False:
                self.dont_retry_list.append(path_str)
        return success

    ##########################################################################
    def _teardown(self, path_str):
        entry = self.active_devices_dict.get(path_str)
        if entry is None:
            return
        nepi_drvs.killDriverNode(entry['node_name'], entry['rbx_subproc'])
        if entry['companion_subproc'] is not None:
            try:
                entry['companion_subproc'].terminate()
            except Exception:
                pass
        del self.active_devices_dict[path_str]
        if path_str in self.active_paths_list:
            self.active_paths_list.remove(path_str)

    ##########################################################################
    # Called by drivers_mgr when the driver is disabled/removed. Return the
    # CALLER'S list, not self.active_paths_list -- drivers_mgr may hand in a
    # different list object here than the one the last discoveryFunction() pass
    # cached, and every shipped driver removes from and returns the argument.
    ##########################################################################
    def killAllDevices(self, active_paths_list):
        for path_str in list(self.active_devices_dict.keys()):
            self._teardown(path_str)
            if path_str in active_paths_list:
                active_paths_list.remove(path_str)
        return active_paths_list


if __name__ == '__main__':
    # CALL-model discovery is normally instantiated by drivers_mgr, not run
    # standalone; this stub is only here for import checks.
    disc = RbxTemplateDiscovery()
