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
# NPX (Navigation / NavPose) DRIVER TEMPLATE -- DISCOVERY (CALL model)
# ---------------------------------------------------------------------------
# This template models a NETWORK navpose source (a TCP endpoint streaming
# position/heading, e.g. an NMEA-over-TCP GPS). For network devices there is
# no port to scan: "configuration IS discovery." We build a launch_key from
# the host:port OPTIONS and launch a node for it; if the user changes the
# endpoint we purge the old node and launch a new one.
#
# For a SERIAL navpose sensor instead, copy the serial probe logic from
# lsx_template/ptx_template discovery (checkForDevice opening a serial port).
##############################################################################

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_drvs
from nepi_sdk import nepi_system

PKG_NAME = 'NPX_TEMPLATE'
FILE_TYPE = 'DISCOVERY'


class NpxTemplateDiscovery:

    NODE_LOAD_TIME_SEC = 10
    launch_time_dict = dict()
    retry = True
    dont_retry_list = []

    active_devices_dict = dict()       # launch_key ("host:port") -> {'node_name','sub_process'}
    node_launch_name = "npx_template"

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
            host = opts['tcp_host']['value']
            port = int(opts['tcp_port']['value'])
        except Exception as e:
            self.logger.log_warn("Failed to load options " + str(e))
            return self.active_paths_list

        self.retry = retry_enabled
        if self.retry:
            self.dont_retry_list = []

        launch_key = host + ":" + str(port)

        # Purge any node whose configured endpoint no longer matches (or died).
        for path_str in list(self.active_devices_dict.keys()):
            still_valid = (path_str == launch_key) and \
                (self.active_devices_dict[path_str]['sub_process'].poll() is None)
            if not still_valid:
                node_name = self.active_devices_dict[path_str]['node_name']
                sub_process = self.active_devices_dict[path_str]['sub_process']
                nepi_drvs.killDriverNode(node_name, sub_process)
                del self.active_devices_dict[path_str]
                if path_str in self.active_paths_list:
                    self.active_paths_list.remove(path_str)

        # Launch the configured endpoint if it is not already running.
        if launch_key not in self.active_devices_dict and launch_key not in self.dont_retry_list:
            if self.checkForDevice(host, port):
                self.launchDeviceNode(launch_key, host, port)

        return self.active_paths_list

    ##########################################################################
    # For a TCP endpoint we treat configuration as discovery and return True.
    # OPTIONAL: do a real reachability probe (socket connect_ex) here if you
    # only want to launch when the endpoint is actually up.
    ##########################################################################
    def checkForDevice(self, host, port):
        return True

    ##########################################################################
    def launchDeviceNode(self, launch_key, host, port):
        if launch_key in self.launch_time_dict:
            if (nepi_sdk.get_time() - self.launch_time_dict[launch_key]) < self.NODE_LOAD_TIME_SEC:
                return False

        file_name = self.drv_dict['NODE_DICT']['file_name']
        device_name = self.node_launch_name + "_" + host.replace('.', '_') + "_" + str(port)
        node_name = nepi_system.get_device_alias(device_name)

        self.drv_dict['DEVICE_DICT'] = {
            'device_name': device_name,
            'device_path': launch_key,
            'tcp_host': host,
            'tcp_port': int(port),
        }
        dict_param_name = nepi_sdk.create_namespace(self.base_namespace, node_name + "/drv_dict")
        nepi_sdk.set_param(dict_param_name, self.drv_dict)

        self.launch_time_dict[launch_key] = nepi_sdk.get_time()
        [success, msg, sub_process] = nepi_drvs.launchDriverNode(file_name, node_name)
        if success:
            self.active_devices_dict[launch_key] = {'node_name': node_name, 'sub_process': sub_process}
            if launch_key not in self.active_paths_list:
                self.active_paths_list.append(launch_key)
            self.logger.log_info("Launched node " + node_name)
        elif self.retry is False:
            self.dont_retry_list.append(launch_key)
        return success

    ##########################################################################
    def killAllDevices(self, active_paths_list):
        for path_str in list(self.active_devices_dict.keys()):
            node_name = self.active_devices_dict[path_str]['node_name']
            sub_process = self.active_devices_dict[path_str]['sub_process']
            nepi_drvs.killDriverNode(node_name, sub_process)
            del self.active_devices_dict[path_str]
            if path_str in active_paths_list:
                active_paths_list.remove(path_str)
        return active_paths_list
