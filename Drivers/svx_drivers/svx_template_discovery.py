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
# SVX (Servo) DRIVER TEMPLATE -- DISCOVERY (CALL model)
# ---------------------------------------------------------------------------
# Same "CALL" discovery contract as the LSX/PTX/NPX/RBX templates: drivers_mgr
# imports this class, instantiates it ONCE with no arguments, and calls
# discoveryFunction(...) on every poll cycle inside its own process. So
# __init__ takes no args, must NOT call nepi_sdk.init_node(), and must NOT call
# nepi_sdk.spin(). See DRIVER_ARCHITECTURE.md and lsx_template_discovery.py for
# the fully-annotated walkthrough; comments here focus on what is different for
# a servo controller.
#
# WHAT IS DIFFERENT: ONE PATH BACKS SEVERAL DEVICES.
# The other categories claim a device path once -- one port, one device, one
# node. A servo controller drives several channels over one transport, and each
# channel is its own SVX device with its own node. So this discovery does not
# claim a path; it RECONCILES A DESIRED SET of (path, channel) pairs against
# what is running, every pass:
#     desired  = every configured channel on every detected board
#     launch   = desired - running
#     kill     = running - desired
# That reconciliation is what lets the channel list be edited live in the RUI:
# adding a channel launches just that node, removing one kills just that node,
# and reordering the list changes nothing.
#
# The board's path is added to active_paths_list once ANY of its channels is
# live, and dropped only when the LAST one goes away.
#
# WHY THE CHANNEL LIST IS USER-DECLARED. An open-loop servo has no feedback
# wire, so a controller cannot tell whether a servo is physically attached to a
# channel. Which channels carry servos is therefore config, never auto-detected
# -- probing would invent phantom devices. Detect the BOARD; declare the
# CHANNELS.
#
# WHAT BELONGS IN OPTIONS. Board-level facts only (which protocol, which bus
# address, which baud rate). Everything describing an individual servo -- its
# travel range, its drive endpoints, its acceleration -- is a NODE setting,
# persisted per device. Discovery must never overwrite an operator's
# calibration when it relaunches a channel.
##############################################################################

import copy
import time

import serial                       # TODO: drop if your transport is not serial
from serial.tools import list_ports

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_drvs       # launchDriverNode, killDriverNode
from nepi_sdk import nepi_system     # get_device_alias

PKG_NAME = 'SVX_TEMPLATE'            # TODO: must match params.yaml 'pkg_name'
FILE_TYPE = 'DISCOVERY'


class SvxTemplateDiscovery:

    # TODO: identify YOUR controller. A USB board is best matched on its
    # vendor/product IDs (stable across reboots, unlike /dev paths); a TTL/UART
    # board needs a serial handshake instead.
    VENDOR_ID = None                 # e.g. 0x1FFB
    PRODUCT_CHANNEL_COUNTS = {}      # e.g. {0x0089: 6, 0x008A: 12}
    DEFAULT_CHANNEL_COUNT = 6        # used when a board reports no usable PID

    node_launch_name = "svx_template"

    # launch_id ("<path>:ch<N>") -> {node_name, sub_process, path, channel}
    active_devices_dict = dict()
    active_paths_list = []
    dont_retry_list = []
    # path -> last resolved channel list, so a resolution is logged on change
    # rather than every pass.
    last_channels_dict = dict()

    retry = True

    # Discovery options, refreshed from drv_dict each pass. All board-level.
    channels_str = 'all'
    baud_str = '9600'

    ##########################################################################
    def __init__(self):
        # CALL-model discovery uses a lightweight logger, NOT MsgIF, and never
        # calls init_node()/spin() -- it is instantiated inside drivers_mgr.
        self.log_name = PKG_NAME.lower() + "_discovery"
        self.logger = nepi_sdk.logger(log_name=self.log_name)
        time.sleep(1)
        self.logger.log_info("Starting Initialization")
        self.logger.log_info("Initialization Complete")

    ##########################################################################
    # NEPI standard discovery entry point. drivers_mgr calls this every cycle.
    ##########################################################################
    def discoveryFunction(self, available_paths_list, active_paths_list,
                          base_namespace, drv_dict, retry_enabled=True):
        self.drv_dict = drv_dict
        self.available_paths_list = available_paths_list
        self.active_paths_list = active_paths_list
        self.base_namespace = base_namespace

        ######################
        # 1) Read discovery OPTIONS (board-level only)
        try:
            options = drv_dict.get('DISCOVERY_DICT', {}).get('OPTIONS', {})
            self.channels_str = str(options.get('channels', {}).get('value', 'all'))
            self.baud_str = str(options.get('baud_rate', {}).get('value', '9600'))
            # TODO: read the rest of your board-level options here.
        except Exception as e:
            self.logger.log_warn("Failed to load options " + str(e))
            return self.active_paths_list

        self.retry = retry_enabled
        if self.retry:
            self.dont_retry_list = []

        ######################
        # 2) Purge nodes whose path has disappeared from the system. Kept
        #    separate from the reconciliation below so a board being unplugged
        #    and a path-enumeration hiccup stay distinguishable.
        for launch_id in list(self.active_devices_dict.keys()):
            entry = self.active_devices_dict[launch_id]
            path_gone = entry['path'] not in self.available_paths_list
            proc_dead = entry['sub_process'].poll() is not None
            if path_gone or proc_dead:
                self.killDevice(launch_id)

        ######################
        # 3) Enumerate boards
        [boards, enumerate_ok] = self.findBoards()
        if enumerate_ok is False:
            # Enumeration FAILED, which is not the same as finding no boards.
            # Reconciling against an empty list here would tear down every
            # running node and respawn it next pass, so change nothing.
            return self.active_paths_list

        ######################
        # 4) Build the desired set of (path, channel) pairs
        desired_dict = dict()
        for board in boards:
            for channel in self.resolveChannels(board):
                launch_id = board['path'] + ":ch" + str(channel)
                desired_dict[launch_id] = {'board': board, 'channel': channel}

        ######################
        # 5) Kill any running channel that is no longer wanted
        for launch_id in list(self.active_devices_dict.keys()):
            if launch_id not in desired_dict:
                self.killDevice(launch_id)

        ######################
        # 6) Launch any wanted channel that is not running yet
        for launch_id in sorted(desired_dict.keys()):
            if launch_id in self.active_devices_dict:
                continue
            entry = desired_dict[launch_id]
            if entry['board']['path'] in self.dont_retry_list:
                continue
            self.launchDeviceNode(entry['board'], entry['channel'])

        ######################
        # 7) Claim each board's path once any of its channels is live
        for board in boards:
            path_str = board['path']
            live = any(e['path'] == path_str for e in self.active_devices_dict.values())
            if live and path_str not in self.active_paths_list:
                self.active_paths_list.append(path_str)

        return self.active_paths_list

    ##########################################################################
    # Find controller boards. Returns [boards, enumerate_ok] -- the second
    # element distinguishes "no boards attached" from "could not look", which
    # step 3 above depends on.
    ##########################################################################
    def findBoards(self):
        boards = []
        try:
            ports = list(list_ports.comports())
        except Exception as e:
            self.logger.log_warn("Port enumeration failed: " + str(e))
            return [], False

        for port in ports:
            # TODO: match YOUR controller. A USB match on vid/pid is stable
            # across reboots; a serial handshake is the fallback for UART
            # boards. Keep it cheap -- this runs every cycle for every port.
            if self.VENDOR_ID is not None and getattr(port, 'vid', None) != self.VENDOR_ID:
                continue
            boards.append({
                'path': port.device,
                'serial_number': getattr(port, 'serial_number', '') or '',
                'channel_count': self.PRODUCT_CHANNEL_COUNTS.get(
                    getattr(port, 'pid', None), self.DEFAULT_CHANNEL_COUNT),
            })
        return boards, True

    ##########################################################################
    # Turn the "channels" option into this board's channel list, validated
    # against how many channels the board actually has. A stale or nonsense
    # value must never silently narrow discovery to a subset -- anything
    # unusable falls back to every channel, loudly.
    ##########################################################################
    def resolveChannels(self, board):
        count = int(board.get('channel_count', self.DEFAULT_CHANNEL_COUNT))
        all_channels = list(range(count))

        channels = self.parseChannels(self.channels_str)
        if channels is None:
            channels = all_channels
        else:
            valid = [c for c in channels if 0 <= c < count]
            if len(valid) != len(channels):
                self.logger.log_warn("Ignoring out-of-range channels for " +
                                     board['path'] + " (board has " + str(count) + ")")
            channels = valid if len(valid) > 0 else all_channels

        if self.last_channels_dict.get(board['path']) != channels:
            self.last_channels_dict[board['path']] = channels
            self.logger.log_info("Channels for " + board['path'] + ": " + str(channels))
        return channels

    ##########################################################################
    # "all" -> None (meaning every channel); "0,1,4" -> [0, 1, 4].
    ##########################################################################
    def parseChannels(self, channels_str):
        text = str(channels_str).strip().lower()
        if text in ('', 'all'):
            return None
        channels = []
        for token in text.split(','):
            token = token.strip()
            if token == '':
                continue
            try:
                channels.append(int(token))
            except ValueError:
                self.logger.log_warn("Ignoring unparseable channel: " + token)
        return channels if len(channels) > 0 else None

    ##########################################################################
    # Write the per-node drv_dict to the param server, then launch the node.
    # THIS IS THE CONFIG HANDSHAKE: the node reads '~drv_dict' back on startup.
    ##########################################################################
    def launchDeviceNode(self, board, channel):
        path_str = board['path']
        launch_id = path_str + ":ch" + str(channel)
        if launch_id in self.active_devices_dict:
            return True

        file_name = self.drv_dict['NODE_DICT']['file_name']
        device_name = (self.node_launch_name + "_" +
                       path_str.split('/')[-1] + "_ch" + str(channel))
        node_name = nepi_system.get_device_alias(device_name)
        self.logger.log_info("Launching node " + node_name + " on channel " + str(channel))

        # Deep-copied per launch so one channel's DEVICE_DICT can never leak
        # into another's -- the drv_dict is shared across every channel here.
        #
        # Board-level facts ONLY. The servo's own calibration belongs to the
        # node, persisted per device; writing it here would overwrite an
        # operator's calibration on every relaunch.
        node_drv_dict = copy.deepcopy(self.drv_dict)
        node_drv_dict['DEVICE_DICT'] = {
            'device_name': device_name,
            'device_path': path_str,
            'channel': channel,
            'baud_str': self.baud_str,
            'serial_number': board.get('serial_number', ''),
        }
        dict_param_name = nepi_sdk.create_namespace(self.base_namespace,
                                                    node_name + "/drv_dict")
        nepi_sdk.set_param(dict_param_name, node_drv_dict)

        [success, msg, sub_process] = nepi_drvs.launchDriverNode(
            file_name, node_name, device_path=path_str)
        if success:
            self.active_devices_dict[launch_id] = {
                'node_name': node_name,
                'sub_process': sub_process,
                'path': path_str,
                'channel': channel,
            }
            self.logger.log_info("Launched node " + node_name)
        else:
            self.logger.log_warn("Failed to launch node " + node_name + ": " + str(msg))
            if self.retry is False:
                self.dont_retry_list.append(path_str)
        return success

    ##########################################################################
    # Kill ONE channel. The board's path leaves active_paths_list only once
    # none of its channels remain.
    ##########################################################################
    def killDevice(self, launch_id):
        if launch_id not in self.active_devices_dict:
            return
        entry = self.active_devices_dict[launch_id]
        path_str = entry['path']
        self.logger.log_info("Killing node " + entry['node_name'])
        nepi_drvs.killDriverNode(entry['node_name'], entry['sub_process'])
        del self.active_devices_dict[launch_id]

        live = any(e['path'] == path_str for e in self.active_devices_dict.values())
        if not live:
            if path_str in self.active_paths_list:
                self.active_paths_list.remove(path_str)
            if path_str in self.last_channels_dict:
                del self.last_channels_dict[path_str]

    ##########################################################################
    # Called by drivers_mgr when the driver is disabled/removed. Remove from
    # and return the CALLER'S list.
    ##########################################################################
    def killAllDevices(self, active_paths_list):
        for launch_id in list(self.active_devices_dict.keys()):
            entry = self.active_devices_dict[launch_id]
            path_str = entry['path']
            if self.retry is False:
                self.dont_retry_list.append(path_str)
            nepi_drvs.killDriverNode(entry['node_name'], entry['sub_process'])
            if path_str in active_paths_list:
                active_paths_list.remove(path_str)
            if path_str in self.active_paths_list:
                self.active_paths_list.remove(path_str)
        self.active_devices_dict = dict()
        self.last_channels_dict = dict()
        nepi_sdk.sleep(1)
        return active_paths_list


if __name__ == '__main__':
    # CALL-model discovery is normally instantiated by drivers_mgr, not run
    # standalone; this stub is only here for import checks.
    disc = SvxTemplateDiscovery()
