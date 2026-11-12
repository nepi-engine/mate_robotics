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

# Discovery for the NEPI Connect RBX driver. Same CALL model, same
# purge-then-probe-then-launch structure, and the same relaunch backoff as
# rbx_gazebo_discovery.py in nepi_drivers.
#
# The robot runs NEPI Connect (Nepi_Connect/pi), which sends a vehicle message
# to this device's status port every 0.5 s, and a status every 100 ms, whether
# or not anything is listening (Nepi_Connect/LINK_PROTOCOL.md, version 2).
# Liveness is therefore a listen, not a connect: the probe binds the status
# port, waits for one valid vehicle message from the configured robot address,
# and closes the socket before the node is launched. The vehicle message says
# everything about the robot, so this driver holds nothing specific to one
# robot: the node is built from the announcement, which discovery hands it in
# DEVICE_DICT['vehicle'], and the device and node names follow the announced
# vehicle name.
#
# The launched node binds that same port for as long as it runs, so the port
# is never probed while a node using it is alive -- the bind would fail
# anyway, since neither socket sets SO_REUSEADDR. That is the one place this
# file departs from rbx_gazebo: checkOnDevice checks only that the node
# process is alive and that the options it was launched with are still the
# current ones. The robot going quiet is the node's business (its link
# reports down and teleop readiness goes False), not a reason to kill it.
#
# The NEPI container uses host networking, so no port mapping is needed.
#
# Servos: the vehicle message names the robot's servos (LINK_PROTOCOL
# "Servos"). With the rbx node, discovery launches one SVX node per servo
# (rbx_nepi_connect_servo_node.py) and manages them as one set: a lost servo
# node purges the set, and the next cycle relaunches it all. A vehicle file
# changed on the robot later needs the driver disabled and enabled; the rbx
# node warns when the announcement differs from the one it was launched with.

import copy
import json
import math
import re
import socket
import time

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_drvs
from nepi_sdk import nepi_system

PKG_NAME = 'RBX_NEPI_CONNECT' # Use in display menus
FILE_TYPE = 'DISCOVERY'


#########################################
# Vehicle Message
#########################################

# The robot's limits (Nepi_Connect/pi/nepi_connect/vehicle.py and link.py)
MAX_DATAGRAM_BYTES = 1200
PROTOCOL_VERSION = 2
SEQ_MAX = 4294967295
AXIS_NAMES = ['surge', 'sway', 'heave', 'yaw']
NAV_GROUPS = [['depth', ['depth_m', 'depth_m_s']],
              ['orientation', ['roll_deg', 'pitch_deg', 'yaw_deg', 'gyro_x_deg_s', 'gyro_y_deg_s', 'gyro_z_deg_s']]]
VEHICLE_NAME = re.compile(r'^[a-z][a-z0-9_]{0,23}$')
SERVO_NAME = re.compile(r'^[a-z][a-z0-9_]{0,15}$')
CAMERA_NAME = re.compile(r'^[a-z][a-z0-9_]{0,15}$')
FIELD_NAME = re.compile(r'^[a-z][a-z0-9_]{0,23}$')
IMU_MOUNT_NAME = re.compile(r'^X_[A-Z]{2,7}_Y_[A-Z]{2,7}$')
DEFAULT_IMU_MOUNT = 'X_FORWARD_Y_LEFT' # The node file's IMU_MOUNT_FLAT
CAMERA_TRANSPORTS = ['http', 'fastmjpg']
MAX_KIND_CHARS = 32
MAX_SERVOS = 4
MAX_RAW = 64
MAX_CAMERAS = 4

def isInteger(value):
  return isinstance(value, int) and not isinstance(value, bool)

def isNumber(value):
  if isinstance(value, bool):
    return False
  if isinstance(value, int):
    return True
  return isinstance(value, float) and math.isfinite(value)

def isNameList(value, pattern, limit):
  return (isinstance(value, list) and len(value) <= limit and len(set(value)) == len(value) and
          all(isinstance(item, str) and pattern.match(item) for item in value))

def parse_vehicle_message(datagram, source_ip, host):
  """Check one datagram against LINK_PROTOCOL "Vehicle" and return the announcement.

  The node file's NepiConnectLink reads the same announcement, so this is
  the one place a vehicle message is validated on NEPI.

  Args:
      datagram (bytes): The UDP payload.
      source_ip (str): The sender's address.
      host (str): The robot's address, the only accepted sender.

  Returns:
      list: [vehicle (dict or None), msg (str)]. vehicle holds name, kind,
          demand_port, axes (name to cap, in AXIS_NAMES order), servos, nav,
          raw, cameras and imu_mount, or is None when the datagram is not a
          valid vehicle message, with the reason in msg.
  """
  if source_ip != host:
    return None, 'Datagram from ' + str(source_ip) + ' ignored: expecting the robot at ' + str(host)
  if len(datagram) > MAX_DATAGRAM_BYTES:
    return None, 'Datagram of ' + str(len(datagram)) + ' bytes ignored: over ' + str(MAX_DATAGRAM_BYTES)
  try:
    message = json.loads(datagram.decode('utf-8'))
  except Exception as e:
    return None, 'Datagram ignored: not UTF-8 JSON (' + str(e) + ')'
  if not isinstance(message, dict):
    return None, 'Datagram ignored: not a JSON object'
  if message.get('v') == 1:
    return None, ('The robot speaks link protocol version 1; update its NEPI Connect and give it a vehicle file '
                  '(Nepi_Connect/VEHICLE_FILE.md)')
  if not isInteger(message.get('v')) or message['v'] != PROTOCOL_VERSION:
    return None, 'Datagram ignored: v must be ' + str(PROTOCOL_VERSION)
  if message.get('type') != 'vehicle':
    return None, 'Datagram ignored: not a vehicle message'
  seq = message.get('seq')
  if not isInteger(seq) or seq < 0 or seq > SEQ_MAX:
    return None, 'Vehicle message ignored: seq must be an integer from 0 to ' + str(SEQ_MAX)
  if not isNumber(message.get('t')):
    return None, 'Vehicle message ignored: t must be a finite number'
  name = message.get('name')
  if not isinstance(name, str) or not VEHICLE_NAME.match(name):
    return None, 'Vehicle message ignored: name must be 1 to 24 lowercase letters, digits or _, starting with a letter'
  kind = message.get('kind')
  if not isinstance(kind, str) or len(kind) > MAX_KIND_CHARS:
    return None, 'Vehicle message ignored: kind must be text of up to ' + str(MAX_KIND_CHARS) + ' characters'
  demand_port = message.get('demand_port')
  if not isInteger(demand_port) or demand_port < 1 or demand_port > 65535:
    return None, 'Vehicle message ignored: demand_port must be a port number from 1 to 65535'
  # A robot need not have every part: a missing axes, servos, nav, raw,
  # cameras or imu_mount field means it has none (imu_mount: the flat
  # default). The node's NepiConnectLink.VEHICLE_DEFAULTS is the same.
  defaults = dict(axes = dict(), servos = [], nav = [], raw = [], cameras = [], imu_mount = DEFAULT_IMU_MOUNT)
  message = dict(message)
  for [key, default] in defaults.items():
    if message.get(key) is None:
      message[key] = default
  axes = message.get('axes')
  if (not isinstance(axes, dict) or any(axis not in AXIS_NAMES for axis in axes) or
      any(not isNumber(cap) or cap < 0 or cap > 1 for cap in axes.values())):
    return None, 'Vehicle message ignored: axes must map up to 4 of ' + ', '.join(AXIS_NAMES) + ' to caps from 0 to 1'
  servos = message.get('servos')
  if not isNameList(servos, SERVO_NAME, MAX_SERVOS):
    return None, 'Vehicle message ignored: servos must list up to ' + str(MAX_SERVOS) + ' different servo names'
  nav = message.get('nav')
  standard = [name for [group, names] in NAV_GROUPS for name in names]
  if (not isinstance(nav, list) or len(set(nav)) != len(nav) or any(item not in standard for item in nav) or
      any(0 < len(set(names) & set(nav)) < len(names) for [group, names] in NAV_GROUPS)):
    return None, 'Vehicle message ignored: nav must list whole groups of standard navigation names'
  raw = message.get('raw')
  if not isNameList(raw, FIELD_NAME, MAX_RAW):
    return None, 'Vehicle message ignored: raw must list different field names'
  cameras = message.get('cameras')
  if not isinstance(cameras, list) or len(cameras) > MAX_CAMERAS:
    return None, 'Vehicle message ignored: cameras must list at most ' + str(MAX_CAMERAS) + ' cameras'
  camera_list = []
  for camera in cameras:
    if (not isinstance(camera, dict) or not isinstance(camera.get('name'), str) or
        not CAMERA_NAME.match(camera['name']) or camera.get('transport') not in CAMERA_TRANSPORTS or
        not isInteger(camera.get('port')) or camera['port'] < 1 or camera['port'] > 65535):
      return None, 'Vehicle message ignored: each camera needs a name, a transport (http or fastmjpg) and a port'
    camera_list.append(dict(name = camera['name'], transport = camera['transport'], port = camera['port']))
  if (len(set(camera['name'] for camera in camera_list)) != len(camera_list) or
      len(set(camera['port'] for camera in camera_list)) != len(camera_list)):
    return None, 'Vehicle message ignored: camera names and ports must differ'
  imu_mount = message.get('imu_mount')
  if not isinstance(imu_mount, str) or not IMU_MOUNT_NAME.match(imu_mount):
    return None, 'Vehicle message ignored: imu_mount must be X_<direction>_Y_<direction>'
  vehicle = dict(name = name, kind = kind, demand_port = demand_port,
                 axes = dict((axis, float(axes[axis])) for axis in AXIS_NAMES if axis in axes),
                 servos = list(servos), nav = list(nav), raw = list(raw), cameras = camera_list, imu_mount = imu_mount)
  return vehicle, 'Accepted'


#########################################
# NEPI Connect Discover Method
#########################################

### Function to try and find a NEPI Connect robot and also monitor and clean up previously connected devices
class NepiConnectDiscovery:

  NODE_LOAD_TIME_SEC = 10
  launch_time_dict = dict()
  retry = True
  dont_retry_list = []

  active_devices_dict = dict()
  node_launch_name = "nepi_connect"

  # The robot sends a vehicle message every 0.5 s, so 1.5 s spans several
  # chances to hear one while still bounding how long a discovery cycle
  # blocks when no robot is up.
  PROBE_TIMEOUT_SEC = 1.5
  last_probe_note = ''

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
  ### Function to try and find a NEPI Connect robot and also monitor and clean up previously connected devices
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

    ### Listen for the configured robot's vehicle message, unless a running node holds the port
    port_held = False
    for device_entry in self.active_devices_dict.values():
      if device_entry["status_port"] == self.status_port:
        port_held = True
    path_str = "NEPI_CONNECT_" + self.host + "_" + self.status_port
    if port_held == False and path_str not in self.active_paths_list and path_str not in self.dont_retry_list:
      [vehicle, note] = self.probeVehicle(self.host, self.status_port)
      if vehicle is not None:
        self.last_probe_note = ''
        self.logger.log_info("NEPI Connect vehicle " + vehicle['name'] + " (" + vehicle['kind'] + ") announced from " +
                             self.host + " on port " + self.status_port + ". Launching nepi connect rbx node, axes: " +
                             str(list(vehicle['axes'].keys())) + ", servos: " + str(vehicle['servos']))
        success = self.launchNepiConnectDeviceNode(path_str, vehicle)
        if success:
          self.active_paths_list.append(path_str)
      elif note != self.last_probe_note:
        # Each new reason once, not every cycle
        self.last_probe_note = note
        if note != '':
          self.logger.log_warn(note)

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
      self.logger.log_warn("NEPI Connect rbx node process for " + path_str +
                           " is no longer running... purging from managed list")
      purge_node = True
    elif device_entry["host"] != self.host or device_entry["status_port"] != self.status_port:
      self.logger.log_warn("NEPI Connect options changed for " + path_str +
                           "... purging from managed list")
      purge_node = True
    # and each servo node. A lost one relaunches the whole set, which also
    # hands the robot back to the pilot when the rbx node shuts down.
    for servo_entry in device_entry["servo_nodes"]:
      if purge_node == False and servo_entry["subproc"].poll() is not None:
        self.logger.log_warn("NEPI Connect servo node " + servo_entry["node_name"] +
                             " is no longer running... purging " + path_str + " from managed list")
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
    # Kill the nepi connect servo and rbx node subprocesses for a device entry
    for servo_entry in device_entry.get("servo_nodes", []):
      self.logger.log_info("Killing nepi connect servo node: " + servo_entry["node_name"])
      nepi_drvs.killDriverNode(servo_entry["node_name"], servo_entry["subproc"])
    rbx_node_name = device_entry.get("rbx_node_name")
    rbx_subproc = device_entry.get("rbx_subproc")
    if rbx_subproc is not None:
      self.logger.log_info("Killing nepi connect rbx node: " + str(rbx_node_name))
      nepi_drvs.killDriverNode(rbx_node_name, rbx_subproc)


  ##########  NEPI CONNECT PROCESSES

  def checkForNepiConnectDevice(self, host_str, port_str):
    return self.probeVehicle(host_str, port_str)[0] is not None

  def probeVehicle(self, host_str, port_str):
    # Listen, do not connect: bind the status port, wait up to PROBE_TIMEOUT_SEC
    # for one valid vehicle message from host_str (parse_vehicle_message), and
    # close the socket either way. Status messages are skipped. A bind failure
    # (the port is in use) reads as not found. Returns [vehicle or None, note]:
    # note says why the last datagram from host_str was not a vehicle
    # message, or is '' when nothing came from it.
    vehicle = None
    note = ''
    sock = None
    try:
      sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
      sock.bind(('', int(port_str)))
      end_time = time.monotonic() + self.PROBE_TIMEOUT_SEC
      while vehicle is None:
        remaining_sec = end_time - time.monotonic()
        if remaining_sec <= 0:
          break
        sock.settimeout(remaining_sec)
        try:
          [datagram, sender] = sock.recvfrom(65536)
        except socket.timeout:
          break
        if sender[0] != host_str:
          continue
        [vehicle, msg] = parse_vehicle_message(datagram, sender[0], host_str)
        if vehicle is None and msg != 'Datagram ignored: not a vehicle message':
          note = msg
    except Exception:
      vehicle = None
    finally:
      if sock is not None:
        try:
          sock.close()
        except Exception:
          pass
    return vehicle, note


  def launchNepiConnectDeviceNode(self, path_str, vehicle):
    # path_str format: "NEPI_CONNECT_<host>_<status_port>"
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

    ### Start the nepi connect RBX node for this robot, named after the vehicle
    device_name = self.node_launch_name + "_" + vehicle['name']
    rbx_node_name = nepi_system.get_device_alias(device_name)

    # Setup required param server drv_dict for the nepi connect node. This
    # param is the entire contract between discovery and the node: the whole
    # announcement goes in DEVICE_DICT['vehicle'].
    file_name = self.drv_dict['NODE_DICT']['file_name']
    self.drv_dict['DEVICE_DICT'] = {
      'device_name': device_name,
      'device_path': path_str,
      'host': self.host,
      'status_port': int(self.status_port),
      'vehicle': copy.deepcopy(vehicle)
    }
    dict_param_name = nepi_sdk.create_namespace(self.base_namespace, rbx_node_name + "/drv_dict")
    nepi_sdk.set_param(dict_param_name, self.drv_dict)

    self.logger.log_info("Starting nepi connect rbx node: " + rbx_node_name)
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
      device_entry["vehicle_name"] = vehicle['name']
      rbx_namespace = nepi_sdk.create_namespace(self.base_namespace, rbx_node_name)
      device_entry["servo_nodes"] = self.launchServoNodes(path_str, device_name, sorted(vehicle['servos']),
                                                          rbx_namespace)
      self.active_devices_dict[path_str] = device_entry
    else:
      self.logger.log_warn("Failed to launch node: " + rbx_node_name + " with msg: " + str(msg))
      if self.retry == False:
        self.logger.log_warn("Will not retry launch for node: " + rbx_node_name)
        self.dont_retry_list.append(launch_id)
    return success


  def launchServoNodes(self, path_str, rbx_device_name, servo_names, rbx_namespace):
    # One SVX node per servo the vehicle message listed, each talking to the
    # robot through the rbx node's set_servo and link_status topics. Returns
    # the launched ones, as servo entries; one that fails to launch is logged
    # and left out.
    servo_entries = []
    # The servo node is the rbx node's sibling file, derived from NODE_DICT so
    # a setup_new_driver.sh rename keeps the pair together
    file_name = self.drv_dict['NODE_DICT']['file_name'].replace('_node.py', '_servo_node.py')
    for servo_name in servo_names:
      device_name = rbx_device_name + "_servo_" + servo_name
      node_name = nepi_system.get_device_alias(device_name)
      # Deep-copied so a launch never mutates the drv_dict drivers_mgr owns
      node_drv_dict = copy.deepcopy(self.drv_dict)
      node_drv_dict['DEVICE_DICT'] = {
        'device_name': device_name,
        'device_path': path_str + "_" + servo_name,
        'servo_name': servo_name,
        'rbx_namespace': rbx_namespace
      }
      dict_param_name = nepi_sdk.create_namespace(self.base_namespace, node_name + "/drv_dict")
      nepi_sdk.set_param(dict_param_name, node_drv_dict)
      self.logger.log_info("Starting nepi connect servo node: " + node_name)
      try:
        [success, msg, subproc] = nepi_drvs.launchDriverNode(file_name, node_name)
      except Exception as e:
        [success, msg, subproc] = [False, str(e), None]
      if success:
        servo_entries.append(dict(node_name = node_name, subproc = subproc, servo_name = servo_name))
      else:
        self.logger.log_warn("Failed to launch servo node: " + node_name + " with msg: " + str(msg))
    return servo_entries


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
  NepiConnectDiscovery()
