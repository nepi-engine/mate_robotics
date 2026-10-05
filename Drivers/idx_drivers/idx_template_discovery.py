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
# IDX (Imaging / Camera) DRIVER TEMPLATE -- DISCOVERY (LAUNCH model)
# ---------------------------------------------------------------------------
# WHAT THIS FILE IS
#   The discovery half of an IDX camera driver. Copy the whole idx_template/
#   folder, rename every "template"/"Template" token, and fill the TODOs.
#
# DISCOVERY MODEL: "LAUNCH"  (differs from CALL used by lsx/ptx/npx/rbx!)
#   Because camera enumeration can be slow/blocking, IDX discovery runs as its
#   OWN node instead of being called inside drivers_mgr. drivers_mgr:
#     1. writes this driver's drv_dict to <discovery_node>/drv_dict, then
#     2. launchDriverNode()s THIS file as a standalone node.
#   So here __init__ DOES call init_node(), reads its config from '~drv_dict',
#   schedules its own detectAndManageDevices timer, and spin()s. drivers_mgr
#   afterward only checks whether this discovery node is still alive.
#
# The device-node launch handshake is the SAME as the CALL model: set the
# per-node '<node>/drv_dict' param (with an injected DEVICE_DICT), then
# nepi_drvs.launchDriverNode(file_name, node_name).
#
# LIVE OPTIONS: because this discovery is a long-lived node, it must RE-READ
# '~drv_dict' on a timer (updateDriverDictCb below) rather than only once in
# __init__. drivers_mgr rewrites that param when an operator edits the driver's
# OPTIONS in the RUI, and a discovery that reads it once never sees the edit.
##############################################################################

import os
import glob
import copy
import subprocess

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_drvs
from nepi_sdk import nepi_system

from nepi_api.messages_if import MsgIF

PKG_NAME = 'IDX_TEMPLATE'
FILE_TYPE = 'DISCOVERY'


class IdxTemplateDiscovery:

    NODE_LOAD_TIME_SEC = 10      # relaunch backoff per device path
    CHECK_INTERVAL_S = 3.0       # how often to re-scan for cameras

    launch_time_dict = dict()
    retry = True
    dont_retry_list = []
    check_for_devices = True

    deviceList = []              # [{'device_path','node_name','node_namespace','node_subprocess'}]
    drv_dict = dict()

    # Device basenames to ignore (e.g. cameras handled by other drivers).
    EXCLUDE_DEVICES = []

    DEFAULT_NODE_NAME = PKG_NAME.lower() + "_discovery"

    ##########################################################################
    def __init__(self):
        # LAUNCH-model discovery IS a node: init + read config + timer + spin.
        nepi_sdk.init_node(name=self.DEFAULT_NODE_NAME)
        self.class_name = type(self).__name__
        self.base_namespace = nepi_sdk.get_base_namespace()
        self.node_name = nepi_sdk.get_node_name()
        self.node_namespace = nepi_sdk.get_node_namespace()
        self.msg_if = MsgIF(log_name=self.class_name)
        self.msg_if.pub_info("Starting Discovery Node Initialization")

        if self.updateDiscoveryOptions() is False:
            nepi_sdk.signal_shutdown(self.node_name + ": no valid drv_dict")
            return

        nepi_sdk.start_timer_process(1.0, self.detectAndManageDevices, oneshot=True)
        nepi_sdk.start_timer_process(1.0, self.updateDriverDictCb, oneshot=True)
        nepi_sdk.on_shutdown(self.cleanup_actions)
        self.msg_if.pub_info("Discovery initialization complete")
        nepi_sdk.spin()

    ##########################################################################
    # Re-read the driver dict every second so an OPTIONS edit made in the RUI
    # takes effect without restarting this discovery node.
    ##########################################################################
    def updateDriverDictCb(self, timer):
        self.updateDiscoveryOptions()
        nepi_sdk.start_timer_process(1.0, self.updateDriverDictCb, oneshot=True)

    ##########################################################################
    # Read this driver's drv_dict (written to '~drv_dict' by drivers_mgr).
    ##########################################################################
    def updateDiscoveryOptions(self):
        last_drv_dict = copy.deepcopy(self.drv_dict)
        self.drv_dict = nepi_sdk.get_param('~drv_dict', dict())
        if 'DISCOVERY_DICT' not in self.drv_dict:
            return False
        if last_drv_dict != self.drv_dict:
            self.msg_if.pub_info("Updated Driver Dict: " + str(self.drv_dict))
        try:
            opts = self.drv_dict['DISCOVERY_DICT']['OPTIONS']
            if 'retry_enabled' in self.drv_dict:
                self.retry = self.drv_dict['retry_enabled']
            if self.retry:
                self.dont_retry_list = []
            # TODO: read any camera-specific OPTIONS here (data_products,
            # launch_delay_sec, ...). Read them HERE, not in __init__, so the
            # re-read timer above actually picks up an operator's edit.
        except Exception as e:
            self.msg_if.pub_warn("Failed to load options " + str(e))
            return False
        return True

    ##########################################################################
    # Self-rescheduling scan loop. Enumerate cameras, launch new ones, purge
    # gone ones, then re-arm the timer.
    ##########################################################################
    def detectAndManageDevices(self, timer):
        if self.check_for_devices:
            active_paths = []

            # ---- Enumerate hardware. TODO: swap for your camera's enumeration
            #      (v4l2-ctl --list-devices, an SDK device_info_list, a network
            #      scan...). Here we just glob V4L2 video nodes.
            for path_str in sorted(glob.glob('/dev/video*')):
                if any(x in path_str for x in self.EXCLUDE_DEVICES):
                    continue
                if not self._isCaptureCapable(path_str):
                    continue
                active_paths.append(path_str)

                known = any(d['device_path'] == path_str for d in self.deviceList)
                if not known:
                    # Launch ONE new device per cycle (keeps startup orderly).
                    if self.startDeviceNode(path_str):
                        break

            # ---- Purge nodes whose hardware disappeared or whose process died.
            for device in list(self.deviceList):
                gone = device['device_path'] not in active_paths
                dead = not self.deviceNodeIsRunning(device)
                if gone or dead:
                    self.stopAndPurgeDeviceNode(device['node_namespace'])

        nepi_sdk.start_timer_process(self.CHECK_INTERVAL_S, self.detectAndManageDevices, oneshot=True)

    ##########################################################################
    def _isCaptureCapable(self, path_str):
        # TODO: confirm this really is a usable capture device for your driver.
        # IMPORTANT: check 'Video Capture' ONLY inside the 'Device Caps' section.
        # Modern kernels create TWO /dev/video* nodes per USB camera (capture +
        # metadata); the metadata node's top-level 'Capabilities' section still
        # lists 'Video Capture' (the parent device's caps), but its own
        # 'Device Caps' says 'Metadata Capture'. Matching the whole output
        # launches a doomed node on every metadata sibling in a ~10s retry loop.
        try:
            out = subprocess.run(['v4l2-ctl', '-d', path_str, '--all'],
                                 capture_output=True, text=True, timeout=2)
            in_device_caps = False
            for line in out.stdout.splitlines():
                if 'Device Caps' in line:
                    in_device_caps = True
                elif in_device_caps:
                    if 'Video Capture' in line:
                        return True
                    if ':' in line:   # next section header ends Device Caps
                        in_device_caps = False
            return False
        except Exception:
            return False

    ##########################################################################
    # Set the per-node drv_dict param (with DEVICE_DICT), then launch the node.
    ##########################################################################
    def startDeviceNode(self, path_str):
        # Backoff: don't relaunch the same path faster than NODE_LOAD_TIME_SEC.
        if path_str in self.launch_time_dict:
            if (nepi_utils.get_time() - self.launch_time_dict[path_str]) < self.NODE_LOAD_TIME_SEC:
                return False

        device_name = nepi_utils.get_clean_name(PKG_NAME.lower() + "_" + path_str.split('/')[-1])
        node_name = nepi_system.get_device_alias(device_name)
        node_namespace = os.path.join(self.base_namespace, node_name)

        if node_name in self.dont_retry_list:
            return False

        # DEVICE_DICT: what discovery learned that the node needs.
        self.drv_dict['DEVICE_DICT'] = {'device_name': device_name, 'device_path': path_str}
        dict_param_name = os.path.join(self.base_namespace, node_name + "/drv_dict")
        nepi_sdk.set_param(dict_param_name, self.drv_dict)

        file_name = self.drv_dict['NODE_DICT']['file_name']
        self.msg_if.pub_info("Launching node " + node_name + " for " + path_str)
        [success, msg, sub_process] = nepi_drvs.launchDriverNode(file_name, node_name)
        self.launch_time_dict[path_str] = nepi_utils.get_time()
        if success:
            self.deviceList.append({'device_path': path_str,
                                    'node_name': node_name,
                                    'node_namespace': node_namespace,
                                    'node_subprocess': sub_process})
        elif self.retry is False:
            self.dont_retry_list.append(node_name)
        return success

    ##########################################################################
    def deviceNodeIsRunning(self, device):
        return device['node_subprocess'].poll() is None

    def stopAndPurgeDeviceNode(self, node_namespace='All'):
        for device in list(self.deviceList):
            if node_namespace == 'All' or device['node_namespace'] == node_namespace:
                nepi_drvs.killDriverNode(device['node_name'], device['node_subprocess'])
                self.deviceList.remove(device)
                if self.retry is False:
                    self.dont_retry_list.append(device['node_name'])

    def cleanup_actions(self):
        self.stopAndPurgeDeviceNode('All')


if __name__ == '__main__':
    node = IdxTemplateDiscovery()
