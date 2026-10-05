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
# LSX (Lighting) DRIVER TEMPLATE -- DISCOVERY (CALL model)
# ---------------------------------------------------------------------------
# WHAT THIS FILE IS
#   The discovery half of an LSX driver for a fictional serial LED light.
#   Copy this whole lsx_template/ folder, rename every "template"/"Template"
#   token to your device, and fill in the TODO markers.
#
# DISCOVERY MODEL: "CALL"  (see DRIVER_ARCHITECTURE.md)
#   drivers_mgr imports this class, instantiates it ONCE with no arguments,
#   and then calls discoveryFunction(...) on every poll cycle (~1-3 s).
#   Therefore:
#     - __init__ takes no args, must NOT call nepi_sdk.init_node(), must NOT
#       call nepi_sdk.spin(). This object lives inside the drivers_mgr process.
#     - discoveryFunction(...) returns the updated active_paths_list.
#     - killAllDevices(...) is called by drivers_mgr on teardown.
#   (The alternative "LAUNCH" model, used by IDX, runs discovery as its own
#    node -- see idx_template/idx_template_discovery.py.)
#
# WHAT DISCOVERY DOES
#   1. Read the user-configurable OPTIONS out of drv_dict['DISCOVERY_DICT'].
#   2. Enumerate candidate hardware (serial ports here).
#   3. Purge nodes whose hardware has disappeared / whose process has died.
#   4. Probe each new candidate with a cheap handshake (checkForDevice).
#   5. For each confirmed new device, write a per-node drv_dict to the ROS
#      param server and launch the driver node via nepi_drvs.launchDriverNode.
##############################################################################

import os
import time
import serial                       # TODO: drop if your transport is not serial
import serial.tools.list_ports

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_drvs       # launchDriverNode, killDriverNode, importDriverClass
from nepi_sdk import nepi_system     # get_device_alias
from nepi_sdk import nepi_serial     # get_serial_ports_list

PKG_NAME = 'LSX_TEMPLATE'            # TODO: must match params.yaml 'pkg_name'
FILE_TYPE = 'DISCOVERY'


class LsxTemplateDiscovery:

    # ---- backoff / retry bookkeeping (see DRIVER_ARCHITECTURE.md > Retry & backoff)
    NODE_LOAD_TIME_SEC = 10          # min seconds between relaunch attempts per device
    launch_time_dict = dict()        # launch_key -> last launch time
    retry = True
    dont_retry_list = []             # devices permanently skipped (only when retry disabled)

    active_devices_dict = dict()     # path_str -> {'node_name', 'sub_process'}
    node_launch_name = "lsx_template"

    baudrate_list = []
    baud_str = '9600'
    baud_int = 9600
    addr_str = "001"
    addr_search_list = []

    # Serial device basenames to ignore (autopilot/console ports, etc.)
    excludedDevices = ['ttyACM', 'ttyTCU', 'ttyTHS']

    ##########################################################################
    def __init__(self):
        # CALL-model discovery uses a lightweight logger, NOT MsgIF, and never
        # calls init_node()/spin() -- it is instantiated inside drivers_mgr.
        self.log_name = PKG_NAME.lower() + "_discovery"
        self.logger = nepi_sdk.logger(log_name=self.log_name)
        self.logger.log_info("Starting Initialization")
        self.logger.log_info("Initialization Complete")

    ##########################################################################
    # NEPI standard discovery entry point. drivers_mgr calls this every cycle.
    #   available_paths_list : paths the system currently sees (shared across drivers)
    #   active_paths_list    : paths that already have a running node (shared, returned)
    #   base_namespace       : e.g. "/nepi/device/"
    #   drv_dict             : the parsed params.yaml 'driver:' block + injected keys
    #   retry_enabled        : whether to keep retrying failed launches
    ##########################################################################
    def discoveryFunction(self, available_paths_list, active_paths_list,
                          base_namespace, drv_dict, retry_enabled=True):
        self.drv_dict = drv_dict
        self.available_paths_list = available_paths_list
        self.active_paths_list = active_paths_list
        self.base_namespace = base_namespace

        ######################
        # 1) Read discovery OPTIONS (defined in params.yaml DISCOVERY_DICT.OPTIONS)
        try:
            opts = drv_dict['DISCOVERY_DICT']['OPTIONS']

            baudrate_options = opts['baud_rate']['options']
            baudrate_sel = opts['baud_rate']['value']
            self.baudrate_list = [b for b in baudrate_options if b != "All"] \
                if baudrate_sel == "All" else [baudrate_sel]

            start_addr = int(opts['start_addr']['value'])
            stop_addr = int(opts['stop_addr']['value'])
            self.addr_search_list = list(range(start_addr, stop_addr + 1)) \
                if stop_addr > start_addr else [start_addr]
        except Exception as e:
            self.logger.log_warn("Failed to load options " + str(e))
            return self.active_paths_list

        self.retry = retry_enabled
        if self.retry:
            self.dont_retry_list = []

        ######################
        # 2) Enumerate candidate hardware.  TODO: swap for your transport
        #    (nepi_serial.get_serial_ports_list(), a network scan, a USB walk...)
        self.path_list = nepi_serial.get_serial_ports_list()

        ######################
        # 3) Purge devices whose hardware or process has gone away
        purge_list = []
        for path_str in self.active_devices_dict.keys():
            if self.checkOnDevice(path_str) is False:
                purge_list.append(path_str)
        for path_str in purge_list:
            node_name = self.active_devices_dict[path_str]['node_name']
            sub_process = self.active_devices_dict[path_str]['sub_process']
            self.logger.log_info("Purging disconnected device: " + node_name)
            nepi_drvs.killDriverNode(node_name, sub_process)
            del self.active_devices_dict[path_str]
            if path_str in self.active_paths_list:
                self.active_paths_list.remove(path_str)

        ######################
        # 4) Probe new candidates and launch a node for each confirmed device
        for path_str in self.path_list:
            if any(x in path_str for x in self.excludedDevices):
                continue
            if path_str in self.active_paths_list:
                continue
            if self.checkForDevice(path_str) and path_str not in self.dont_retry_list:
                self.launchDeviceNode(path_str)

        return self.active_paths_list

    ##########################################################################
    # Cheap handshake to confirm OUR device is really on this port.
    # Keep it fast -- it runs every cycle for every unclaimed port.
    ##########################################################################
    def checkForDevice(self, path_str):
        found_device = False
        for baud_str in self.baudrate_list:
            try:
                self.baud_int = int(baud_str)
                serial_port = serial.Serial(path_str, self.baud_int, timeout=1)
            except Exception:
                continue
            try:
                for addr in self.addr_search_list:
                    addr_str = str(addr).zfill(3)
                    # TODO: replace with your device's identity/handshake command.
                    ser_msg = ('!' + addr_str + ':INFO?\r\n')
                    serial_port.write(bytearray(ser_msg, 'utf-8'))
                    nepi_sdk.sleep(0.05, 5)
                    response = serial_port.readline().decode(errors='ignore')
                    # TODO: replace with your device's expected reply signature.
                    if response.startswith('INFO,'):
                        self.addr_str = addr_str
                        self.baud_str = baud_str
                        found_device = True
                        break
            except Exception as e:
                self.logger.log_warn("Probe error on " + path_str + ": " + str(e))
            finally:
                serial_port.close()
            if found_device:
                break
        return found_device

    ##########################################################################
    # Return False if a launched device is no longer valid (hardware gone or
    # node process dead), which triggers a purge above.
    ##########################################################################
    def checkOnDevice(self, path_str):
        if path_str not in self.available_paths_list:
            return False
        sub_process = self.active_devices_dict[path_str]['sub_process']
        if sub_process.poll() is not None:   # process exited
            return False
        return True

    ##########################################################################
    # Write the per-node drv_dict to the param server, then launch the node.
    # THIS IS THE CONFIG HANDSHAKE: the node reads '~drv_dict' back on startup.
    ##########################################################################
    def launchDeviceNode(self, path_str):
        # backoff: don't relaunch the same device faster than NODE_LOAD_TIME_SEC
        launch_id = path_str
        if launch_id in self.launch_time_dict:
            if (nepi_sdk.get_time() - self.launch_time_dict[launch_id]) < self.NODE_LOAD_TIME_SEC:
                return False

        file_name = self.drv_dict['NODE_DICT']['file_name']
        device_name = self.node_launch_name + "_" + path_str.split('/')[-1] + "_" + str(self.addr_str)
        node_name = nepi_system.get_device_alias(device_name)
        node_namespace = os.path.join(self.base_namespace, node_name)

        # DEVICE_DICT: per-instance data discovery learned that the node needs.
        # TODO: add whatever your node must know (addr, baud, ip, model, serial...).
        self.drv_dict['DEVICE_DICT'] = {
            'device_name': device_name,
            'device_path': path_str,
            'baud_str': self.baud_str,
            'addr_str': self.addr_str,
        }

        dict_param_name = nepi_sdk.create_namespace(self.base_namespace, node_name + "/drv_dict")
        nepi_sdk.set_param(dict_param_name, self.drv_dict)

        self.launch_time_dict[launch_id] = nepi_sdk.get_time()
        [success, msg, sub_process] = nepi_drvs.launchDriverNode(file_name, node_name, device_path=path_str)
        if success:
            self.active_devices_dict[path_str] = {'node_name': node_name, 'sub_process': sub_process}
            if path_str not in self.active_paths_list:
                self.active_paths_list.append(path_str)
            self.logger.log_info("Launched node " + node_name)
        elif self.retry is False:
            self.dont_retry_list.append(path_str)
        return success

    ##########################################################################
    # Called by drivers_mgr when the driver is disabled/removed.
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
