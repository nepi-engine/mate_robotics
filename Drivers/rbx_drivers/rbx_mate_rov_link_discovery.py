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
# Redistributions in source code must retain this top-level comment block.
# Plagiarizing this software to sidestep the license obligations is illegal.
#
# Contact Information:
# ====================
# - mailto:nepi@numurus.com
#

# Discovery for the MATE ROV link RBX driver. Same CALL model, same
# purge-then-probe-then-launch structure, and the same relaunch backoff as
# rbx_gazebo_discovery.py in nepi_drivers.
#
# The ROV's control Pi runs nepi_link.py (Nepi_Connect/pi), which sends a
# status datagram to this device's status port every 100 ms whether or not
# anything is listening. Liveness is therefore a listen, not a connect: the
# probe binds the status port, waits for one valid status from the configured
# ROV address, and closes the socket before the node is launched.
#
# The launched node binds that same port for as long as it runs, so the port
# is never probed while a node using it is alive -- the bind would fail
# anyway, since neither socket sets SO_REUSEADDR. That is the one place this
# file departs from rbx_gazebo: checkOnDevice checks only that the node
# process is alive and that the options it was launched with are still the
# current ones. The ROV going quiet is the node's business (its link reports
# down and teleop readiness goes False), not a reason to kill it.
#
# The NEPI container uses host networking, so no port mapping is needed.

import json
import socket
import time

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_drvs
from nepi_sdk import nepi_system

PKG_NAME = 'RBX_MATE_ROV_LINK' # Use in display menus
FILE_TYPE = 'DISCOVERY'


#########################################
# MATE ROV Link Discover Method
#########################################

### Function to try and find a MATE ROV link and also monitor and clean up previously connected devices
class MateRovLinkDiscovery:

  NODE_LOAD_TIME_SEC = 10
  launch_time_dict = dict()
  retry = True
  dont_retry_list = []

  active_devices_dict = dict()
  node_launch_name = "mate_rov_link"

  # Device id for the single ROV this driver discovers per configured address.
  DEVICE_ID = 'robot'

  # The Pi sends status every 100 ms, so 1.5 s spans many chances to hear one
  # while still bounding how long a discovery cycle blocks when no ROV is up.
  PROBE_TIMEOUT_SEC = 1.5
  # Same bound and same accept rule as fake_nepi.py's StatusReceiver.
  MAX_DATAGRAM_BYTES = 1200
  PROTOCOL_VERSION = 1

  ################################################
  def __init__(self):
    ############
    # Create Message Logger
    self.log_name = PKG_NAME.lower() + "_discovery"
    self.logger = nepi_sdk.logger(namespace = self.log_name)
    time.sleep(1)
    self.logger.log_info("Starting Initialization")
    self.logger.log_info("Initialization Complete")


  ##########  Drv Standard Discovery Function
  ### Function to try and find a MATE ROV link and also monitor and clean up previously connected devices
  def discoveryFunction(self, available_paths_list, active_paths_list, base_namespace, drv_dict, retry_enabled = True):
    self.drv_dict = drv_dict
    self.available_paths_list = available_paths_list
    self.active_paths_list = active_paths_list
    self.base_namespace = base_namespace

    ########################
    # Get discovery options
    try:
      options = drv_dict['DISCOVERY_DICT']['OPTIONS']
      self.host = str(options['host']['value'])
      self.status_port = str(options['status_port']['value'])
      self.demand_port = int(options['demand_port']['value'])
    except Exception as e:
      self.logger.log_warn("Failed to load options " + str(e))
      return None

    # Retry behavior
    self.retry = retry_enabled
    if self.retry == True:
      self.dont_retry_list = []
    ########################

    ### Purge Unresponsive Connections
    path_purge_list = []
    for path_str in self.active_devices_dict.keys():
      success = self.checkOnDevice(path_str)
      if success == False:
        path_purge_list.append(path_str)
    # Clean up the active_devices_dict
    for path_str in path_purge_list:
      del self.active_devices_dict[path_str]
      if path_str in self.active_paths_list:
        self.active_paths_list.remove(path_str)

    ### Listen for the configured ROV's status, unless a running node holds the port
    port_held = False
    for device_entry in self.active_devices_dict.values():
      if device_entry["status_port"] == self.status_port:
        port_held = True
    path_str = "MATE_ROV_LINK_" + self.host + "_" + self.status_port
    if port_held == False and path_str not in self.active_paths_list and path_str not in self.dont_retry_list:
      if self.checkForMateRovLinkDevice(self.host, self.status_port):
        self.logger.log_info("MATE ROV link status detected from " + self.host + " on port " +
                             self.status_port + ". Launching mate rov link rbx node")
        success = self.launchMateRovLinkDeviceNode(path_str)
        if success:
          self.active_paths_list.append(path_str)

    # Wrap Up
    return self.active_paths_list


  ################################################
  ##########  Device Monitor Processes

  def checkOnDevice(self, path_str):
    # Returns True if the device's rbx node process is still alive and was
    # launched with the current options. Never probes: the node holds the status
    # port. An options edit purges the node here, which frees the port for the
    # next probe with the new values.
    active = True
    if path_str not in self.active_devices_dict.keys():
      return False

    device_entry = self.active_devices_dict[path_str]
    rbx_subproc = device_entry["rbx_subproc"]

    purge_node = False
    # Check that the rbx node process is still running
    if rbx_subproc is None or rbx_subproc.poll() is not None:
      self.logger.log_warn("MATE ROV link rbx node process for " + path_str +
                           " is no longer running... purging from managed list")
      purge_node = True
    elif (device_entry["host"] != self.host or device_entry["status_port"] != self.status_port or
          device_entry["demand_port"] != self.demand_port):
      self.logger.log_warn("MATE ROV link options changed for " + path_str +
                           "... purging from managed list")
      purge_node = True

    if purge_node:
      self.killDeviceProcesses(device_entry)
      if path_str in self.active_paths_list:
        self.active_paths_list.remove(path_str)
      if path_str in self.dont_retry_list:
        self.dont_retry_list.remove(path_str)
      active = False
    return active


  def killDeviceProcesses(self, device_entry):
    # Kill the mate rov link rbx node subprocess for a device entry
    rbx_node_name = device_entry.get("rbx_node_name")
    rbx_subproc = device_entry.get("rbx_subproc")
    if rbx_subproc is not None:
      self.logger.log_info("Killing mate rov link rbx node: " + str(rbx_node_name))
      nepi_drvs.killDriverNode(rbx_node_name, rbx_subproc)


  ##########  MATE ROV LINK PROCESSES

  def checkForMateRovLinkDevice(self, host_str, port_str):
    # Listen, do not connect: bind the status port, wait up to PROBE_TIMEOUT_SEC
    # for one valid status from host_str, and close the socket either way. A
    # status is valid on the same terms fake_nepi.py's StatusReceiver accepts
    # one: from the configured address, at most MAX_DATAGRAM_BYTES, UTF-8 JSON,
    # an object with v 1 and type "status". A bind failure (the port is in use)
    # reads as not found.
    found_device = False
    sock = None
    try:
      sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
      sock.bind(('', int(port_str)))
      end_time = time.monotonic() + self.PROBE_TIMEOUT_SEC
      while found_device == False:
        remaining_sec = end_time - time.monotonic()
        if remaining_sec <= 0:
          break
        sock.settimeout(remaining_sec)
        try:
          [datagram, sender] = sock.recvfrom(65536)
        except socket.timeout:
          break
        if sender[0] != host_str or len(datagram) > self.MAX_DATAGRAM_BYTES:
          continue
        try:
          status = json.loads(datagram.decode('utf-8'))
        except Exception:
          continue
        if isinstance(status, dict) and status.get('v') == self.PROTOCOL_VERSION and status.get('type') == 'status':
          found_device = True
    except Exception:
      found_device = False
    finally:
      if sock is not None:
        try:
          sock.close()
        except Exception:
          pass
    return found_device


  def launchMateRovLinkDeviceNode(self, path_str):
    # path_str format: "MATE_ROV_LINK_<host>_<status_port>"
    success = False
    launch_id = path_str

    # Check if should try to launch (backoff to prevent rapid relaunch loops)
    launch_check = True
    if launch_id in self.launch_time_dict.keys():
      launch_time = self.launch_time_dict[launch_id]
      cur_time = nepi_sdk.get_time()
      launch_check = (cur_time - launch_time) > self.NODE_LOAD_TIME_SEC
    if launch_check == False:
      return False

    ### Start the mate rov link RBX node for this ROV
    rov_device_name = self.node_launch_name + "_" + self.DEVICE_ID
    rbx_node_name = nepi_system.get_device_alias(rov_device_name)

    # Setup required param server drv_dict for the mate rov link node. This
    # param is the entire contract between discovery and the node.
    file_name = self.drv_dict['NODE_DICT']['file_name']
    self.drv_dict['DEVICE_DICT'] = {
      'device_name': rov_device_name,
      'device_path': path_str,
      'host': self.host,
      'status_port': int(self.status_port),
      'demand_port': self.demand_port
    }
    dict_param_name = nepi_sdk.create_namespace(self.base_namespace, rbx_node_name + "/drv_dict")
    nepi_sdk.set_param(dict_param_name, self.drv_dict)

    self.logger.log_info("Starting mate rov link rbx node: " + rbx_node_name)
    # Guarded so a launch-helper exception (e.g. the node file not yet deployed)
    # reads as a failed launch instead of taking the whole driver offline --
    # drivers_mgr disables a driver whose discoveryFunction raises.
    try:
      [success, msg, rbx_subproc] = nepi_drvs.launchDriverNode(file_name, rbx_node_name)
    except Exception as e:
      [success, msg, rbx_subproc] = [False, str(e), None]

    # Process launch results
    self.launch_time_dict[launch_id] = nepi_sdk.get_time()
    if success:
      self.logger.log_info("Launched node: " + rbx_node_name)
      device_entry = dict()
      device_entry["rbx_node_name"] = rbx_node_name
      device_entry["rbx_subproc"] = rbx_subproc
      device_entry["host"] = self.host
      device_entry["status_port"] = self.status_port
      device_entry["demand_port"] = self.demand_port
      self.active_devices_dict[path_str] = device_entry
    else:
      self.logger.log_warn("Failed to launch node: " + rbx_node_name + " with msg: " + str(msg))
      if self.retry == False:
        self.logger.log_warn("Will not retry launch for node: " + rbx_node_name)
        self.dont_retry_list.append(launch_id)
    return success


  def killAllDevices(self, active_paths_list):
    path_purge_list = []
    for key in self.active_devices_dict.keys():
      path_purge_list.append(key)
    for path_str in path_purge_list:
      device_entry = self.active_devices_dict[path_str]
      if self.retry == False:
        self.dont_retry_list.append(path_str)
      self.killDeviceProcesses(device_entry)
      if path_str in active_paths_list:
        active_paths_list.remove(path_str)
    for path_str in path_purge_list:
      del self.active_devices_dict[path_str]
    nepi_sdk.sleep(1)
    return active_paths_list


#########################################
# Main
#########################################
if __name__ == '__main__':
  MateRovLinkDiscovery()
