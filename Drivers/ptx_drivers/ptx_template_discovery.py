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
# PTX (Pan-Tilt) DRIVER TEMPLATE -- DISCOVERY (CALL model)
# ---------------------------------------------------------------------------
# Same "CALL" discovery contract as the LSX template (drivers_mgr instantiates
# this class with no args and calls discoveryFunction() each cycle). See
# DRIVER_ARCHITECTURE.md and lsx_template/ for the fully-annotated walkthrough;
# comments here focus on what differs for a pan-tilt device.
##############################################################################

import os
import time
import serial
import serial.tools.list_ports

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_drvs
from nepi_sdk import nepi_system
from nepi_sdk import nepi_serial

PKG_NAME = 'PTX_TEMPLATE'
FILE_TYPE = 'DISCOVERY'


class PtxTemplateDiscovery:

    NODE_LOAD_TIME_SEC = 10
    launch_time_dict = dict()
    retry = True
    dont_retry_list = []

    active_devices_dict = dict()
    node_launch_name = "ptx_template"

    baudrate_list = []
    baud_str = '9600'
    addr_str = "A"
    addr_search_list = []

    excludedDevices = ['ttyACM', 'ttyTCU', 'ttyTHS']

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
            baudrate_options = opts['baud_rate']['options']
            baudrate_sel = opts['baud_rate']['value']
            self.baudrate_list = [b for b in baudrate_options if b != "All"] \
                if baudrate_sel == "All" else [baudrate_sel]
            # PTX addresses here are letters (A..Z); adjust to your protocol.
            self.addr_search_list = self._letterRange(opts['start_addr']['value'],
                                                      opts['stop_addr']['value'])
        except Exception as e:
            self.logger.log_warn("Failed to load options " + str(e))
            return self.active_paths_list

        self.retry = retry_enabled
        if self.retry:
            self.dont_retry_list = []

        self.path_list = nepi_serial.get_serial_ports_list()

        # Purge disconnected / dead devices
        for path_str in list(self.active_devices_dict.keys()):
            if self.checkOnDevice(path_str) is False:
                node_name = self.active_devices_dict[path_str]['node_name']
                sub_process = self.active_devices_dict[path_str]['sub_process']
                nepi_drvs.killDriverNode(node_name, sub_process)
                del self.active_devices_dict[path_str]
                if path_str in self.active_paths_list:
                    self.active_paths_list.remove(path_str)

        # Probe + launch new devices
        for path_str in self.path_list:
            if any(x in path_str for x in self.excludedDevices):
                continue
            if path_str in self.active_paths_list:
                continue
            if self.checkForDevice(path_str) and path_str not in self.dont_retry_list:
                self.launchDeviceNode(path_str)

        return self.active_paths_list

    ##########################################################################
    def checkForDevice(self, path_str):
        found_device = False
        for baud_str in self.baudrate_list:
            try:
                serial_port = serial.Serial(path_str, int(baud_str), timeout=1)
            except Exception:
                continue
            try:
                for addr in self.addr_search_list:
                    # TODO: send your pan-tilt's identity/status query.
                    ser_msg = ('#' + addr + 'INFO\r')
                    serial_port.reset_input_buffer()
                    serial_port.write(bytearray(ser_msg, 'utf-8'))
                    nepi_sdk.sleep(0.05, 5)
                    response = serial_port.readline().decode(errors='ignore')
                    # TODO: match your device's echo/reply signature.
                    if response.startswith('#' + addr):
                        self.addr_str = addr
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
    def checkOnDevice(self, path_str):
        if path_str not in self.available_paths_list:
            return False
        if self.active_devices_dict[path_str]['sub_process'].poll() is not None:
            return False
        return True

    ##########################################################################
    def launchDeviceNode(self, path_str):
        launch_id = path_str
        if launch_id in self.launch_time_dict:
            if (nepi_sdk.get_time() - self.launch_time_dict[launch_id]) < self.NODE_LOAD_TIME_SEC:
                return False

        file_name = self.drv_dict['NODE_DICT']['file_name']
        device_name = self.node_launch_name + "_" + path_str.split('/')[-1] + "_" + str(self.addr_str)
        node_name = nepi_system.get_device_alias(device_name)

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
    def killAllDevices(self, active_paths_list):
        for path_str in list(self.active_devices_dict.keys()):
            node_name = self.active_devices_dict[path_str]['node_name']
            sub_process = self.active_devices_dict[path_str]['sub_process']
            nepi_drvs.killDriverNode(node_name, sub_process)
            del self.active_devices_dict[path_str]
            if path_str in active_paths_list:
                active_paths_list.remove(path_str)
        return active_paths_list

    ##########################################################################
    @staticmethod
    def _letterRange(start_char, stop_char):
        start, stop = ord(start_char), ord(stop_char)
        return [chr(c) for c in range(start, stop + 1)] if stop >= start else [start_char]
