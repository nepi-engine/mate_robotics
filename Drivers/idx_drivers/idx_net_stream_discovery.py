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

# Discovery for the IDX_NET_STREAM driver. Same LAUNCH node shape as
# idx_v4l2_discovery.py; the probe-before-launch and relaunch-after-purge
# behavior follows rbx_gazebo_discovery.py.
#
# There is no hardware to enumerate: the operator types stream URLs into the
# stream_urls option on the Drivers page. drivers_mgr re-publishes this node's
# ~drv_dict on every update cycle and updateDriverDictCb re-reads it every
# second, so an edit takes effect without a restart.
#
# A URL gets a node only after a probe of it returns a frame. Each probe runs in
# its own thread, so one slow or unreachable URL never holds up the others or
# the management loop. Unreachable URLs are re-probed quietly, because the
# stream source is often powered on after NEPI. A node that exits (its stream was
# lost for longer than the node's timeout) is purged and relaunched once a probe
# answers again; as in the gazebo discovery, retry_enabled = False only stops
# retrying launches that failed outright.

import copy
import hashlib
import re
import threading
from urllib.parse import urlsplit

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_drvs
from nepi_sdk import nepi_system

from nepi_api.messages_if import MsgIF

PKG_NAME = 'IDX_NET_STREAM' # Use in display menus
FILE_TYPE = 'DISCOVERY'

class NetStreamCamDiscovery:

  NODE_LOAD_TIME_SEC = 10
  CHECK_INTERVAL_S = 1.0

  PROBE_INTERVAL_SEC = 5.0 # Between probes of one unreachable URL
  PROBE_TIMEOUT_SEC = 20.0 # Ceiling on one probe; above the driver's own open + read timeouts
  LOG_THROTTLE_SEC = 60.0

  NAME_ROOT = 'netstream'
  NAME_MAX_LEN = 40 # Readable part of a device name, before the hash suffix

  launch_time_dict = dict()
  retry = True
  dont_retry_list = []

  check_for_devices = True

  drv_dict = dict()
  driver_class = None
  stream_urls = []
  devices_dict = dict() # url -> running node entry
  probe_dict = dict() # url -> in-flight probe entry
  last_probe_time_dict = dict()


  ################################################
  DEFAULT_NODE_NAME = PKG_NAME.lower() + "_discovery"

  def __init__(self):
    ####  NODE Initialization ####
    nepi_sdk.init_node(name= self.DEFAULT_NODE_NAME)
    self.class_name = type(self).__name__
    self.base_namespace = nepi_sdk.get_base_namespace()
    self.node_name = nepi_sdk.get_node_name()
    self.node_namespace = nepi_sdk.get_node_namespace()

    ##############################
    # Create Msg Class
    self.msg_if = MsgIF()
    self.msg_if.pub_info("Starting Node Initialization Processes")


    ########################
    # Update discovery options
    success = self.updateDiscoveryOptions()
    if success == False:
        self.msg_if.pub_warn("Shutting down because failed to get Driver Dict")
        nepi_sdk.signal_shutdown(self.node_name + ": Shutting down because failed to get Driver Dict")
        return

    ########################
    # Import the driver class for its URL helpers and stream probe, the same
    # way the node imports it
    try:
      driver_path = self.drv_dict['path']
      driver_file = self.drv_dict['DRIVER_DICT']['file_name']
      driver_module = driver_file.split('.')[0]
      driver_class_name = self.drv_dict['DRIVER_DICT']['class_name']
      [success, msg, self.driver_class] = nepi_drvs.importDriverClass(driver_file,driver_path,driver_module,driver_class_name)
    except Exception as e:
      success = False
      msg = str(e)
    if success == False or self.driver_class is None:
        self.msg_if.pub_warn("Shutting down because failed to import driver class: " + str(msg))
        nepi_sdk.signal_shutdown(self.node_name + ": Shutting down because failed to import driver class")
        return
    self.updateDiscoveryOptions()

    ########################
    # Start node processes
    nepi_sdk.start_timer_process((1), self.detectAndManageDevices, oneshot = True)
    nepi_sdk.start_timer_process((1), self.updateDriverDictCb, oneshot = True)
    nepi_sdk.on_shutdown(self.cleanup_actions)

    ########################
    # Now start the node
    self.msg_if.pub_info("Initialization Complete")
    nepi_sdk.spin()




  #**********************
  # Discovery functions
  #**********************

  def updateDriverDictCb(self,timer):
    updated = self.updateDiscoveryOptions()
    nepi_sdk.start_timer_process((1), self.updateDriverDictCb, oneshot = True)

  def updateDiscoveryOptions(self):
    ########################
    # Get discovery options
    success = False
    drv_dict = nepi_sdk.get_param('~drv_dict',dict())
    if len(list(drv_dict.keys())) == 0:
      self.msg_if.pub_warn("Failed to load Driver dict ", throttle_s = self.LOG_THROTTLE_SEC)
      return success
    if 'DISCOVERY_DICT' not in drv_dict.keys():
      self.msg_if.pub_warn("Discovery dict missing in Driver dict", throttle_s = self.LOG_THROTTLE_SEC)
      return success
    # Not logged whole, unlike the other discoveries: the stream_urls option can
    # carry credentials
    self.drv_dict = drv_dict
    success = True


    if 'retry_enabled' in self.drv_dict.keys():
      self.retry = self.drv_dict['retry_enabled']
    else:
      self.retry = True
    if self.retry == True:
      self.dont_retry_list = []

    if self.driver_class is not None:
      urls_value = ''
      try:
        urls_value = self.drv_dict['DISCOVERY_DICT']['OPTIONS']['stream_urls']['value']
      except Exception:
        pass
      stream_urls = self.parseStreamUrls(urls_value)
      if stream_urls != self.stream_urls:
        safe_urls = [self.driver_class.get_safe_url(url) for url in stream_urls]
        self.msg_if.pub_info("Stream URL list: " + str(safe_urls))
        self.stream_urls = stream_urls
    return success


  def parseStreamUrls(self, urls_value):
    # Returns the supported URLs from the stream_urls option, de-duplicated, in
    # the order the operator typed them
    if isinstance(urls_value, (list, tuple)):
      urls_value = ','.join([str(entry) for entry in urls_value])
    urls_value = str(urls_value).strip()
    if urls_value.lower() in ['', 'none']:
      return []
    stream_urls = []
    device_names = []
    local_ports = []
    for entry in urls_value.split(','):
      url = entry.strip()
      if url == '' or url in stream_urls:
        continue
      [supported, msg] = self.driver_class.is_supported_url(url)
      if supported == False:
        self.msg_if.pub_warn("Skipping stream URL: " + msg,
                             throttle_s = self.LOG_THROTTLE_SEC, throttle_n = False)
        continue
      device_name = self.getDeviceName(url)
      if device_name in device_names:
        self.msg_if.pub_warn("Skipping stream URL " + self.driver_class.get_safe_url(url) +
                             ": same host, port and path as an earlier entry",
                             throttle_s = self.LOG_THROTTLE_SEC, throttle_n = False)
        continue
      # A stream pushed to NEPI (fastmjpg://) holds a local UDP port, which a
      # second such stream could not open
      local_port = self.driver_class.get_local_port(url)
      if local_port is not None and local_port in local_ports:
        self.msg_if.pub_warn("Skipping stream URL " + self.driver_class.get_safe_url(url) +
                             ": UDP port " + str(local_port) + " is already used by an earlier entry",
                             throttle_s = self.LOG_THROTTLE_SEC, throttle_n = False)
        continue
      stream_urls.append(url)
      device_names.append(device_name)
      if local_port is not None:
        local_ports.append(local_port)
    return stream_urls


  def getDeviceName(self, url):
    # Deterministic name from host, port and path, whatever the scheme, so the
    # same stream keeps its name (and other apps' selections) across restarts.
    # Credentials never enter it. A missing port takes the scheme's default, so
    # rtsp://host/cam and rtsp://host:554/cam are one device. The readable part
    # is a valid ROS name; the hash suffix keeps two URLs that sanitize to the
    # same text (cam-1 vs cam_1, or a different query) apart.
    parts = urlsplit(url)
    scheme = parts.scheme.lower()
    host = parts.hostname.lower()
    port = parts.port
    if port is None:
      port = self.driver_class.DEFAULT_PORTS.get(scheme, 0)
    path = parts.path
    if path == '':
      path = '/'
    stream_id = host + ':' + str(port) + path
    if parts.query != '':
      stream_id += '?' + parts.query
    readable = re.sub(r'[^a-z0-9]+', '_', (host + '_' + str(port) + '_' + path).lower())
    readable = readable.strip('_')[:self.NAME_MAX_LEN].strip('_')
    suffix = hashlib.sha1(stream_id.encode('utf-8')).hexdigest()[:4]
    return self.NAME_ROOT + '_' + readable + '_' + suffix


  def detectAndManageDevices(self, timer): # Extra arg since this is a Timer callback
    if self.check_for_devices == False:
      self.msg_if.pub_warn("Stopping device discovery process")
      return
    stream_urls = list(self.stream_urls)

    ######################
    # Stop nodes for URLs removed from the list
    for url in list(self.devices_dict.keys()):
      if url not in stream_urls:
        self.msg_if.pub_info("Stream " + self.devices_dict[url]['safe_url'] + " removed from stream_urls. Stopping node " +
                             self.devices_dict[url]['node_name'])
        device_name = self.devices_dict[url]['device_name']
        self.stopAndPurgeDeviceNode(url)
        if device_name in self.dont_retry_list:
          self.dont_retry_list.remove(device_name)
    for url in list(self.probe_dict.keys()):
      if url not in stream_urls:
        del self.probe_dict[url] # The probe thread finishes on its own; its result is dropped

    ######################
    # Purge nodes that have exited, normally because their stream was lost.
    # They relaunch below once a probe answers again.
    for url in list(self.devices_dict.keys()):
      if not self.deviceNodeIsRunning(url):
        self.msg_if.pub_warn("Node " + self.devices_dict[url]['node_name'] + " for stream " + self.devices_dict[url]['safe_url'] +
                             " is not running. Will relaunch when the stream answers a probe")
        self.stopAndPurgeDeviceNode(url)

    ######################
    # Probe URLs without a node, and launch a node for each one that answers
    for url in stream_urls:
      if url in self.devices_dict.keys():
        continue
      if self.getDeviceName(url) in self.dont_retry_list:
        continue
      if self.checkProbe(url) == True:
        self.startDeviceNode(url)

    #########################
    # Setup next check process
    nepi_sdk.start_timer_process(self.CHECK_INTERVAL_S, self.detectAndManageDevices, oneshot = True)


  def checkProbe(self, url):
    # Returns True once a probe of url has succeeded. Starts a probe thread when
    # none is running and the URL is due for one, and collects finished results.
    safe_url = self.driver_class.get_safe_url(url)
    cur_time = nepi_utils.get_time()
    if url in self.probe_dict.keys():
      probe = self.probe_dict[url]
      if probe['result'] is None:
        if cur_time - probe['start_time'] > self.PROBE_TIMEOUT_SEC:
          # Keep the entry until the thread returns, so stalled probes never pile up
          self.msg_if.pub_warn("Probe of stream " + safe_url + " is taking longer than " +
                               str(int(self.PROBE_TIMEOUT_SEC)) + " s",
                               throttle_s = self.LOG_THROTTLE_SEC, throttle_n = False)
        return False
      del self.probe_dict[url]
      [success, msg] = probe['result']
      if success == True and cur_time - probe['start_time'] <= self.PROBE_TIMEOUT_SEC:
        self.msg_if.pub_info("Stream " + safe_url + " answered probe: " + msg)
        return True
      self.msg_if.pub_info("Stream " + safe_url + " not reachable yet (" + msg + "). Retrying every " +
                           str(int(self.PROBE_INTERVAL_SEC)) + " s",
                           throttle_s = self.LOG_THROTTLE_SEC, throttle_n = False)
      return False

    last_probe_time = self.last_probe_time_dict.get(url, None)
    if last_probe_time is None or cur_time - last_probe_time >= self.PROBE_INTERVAL_SEC:
      self.last_probe_time_dict[url] = cur_time
      probe = {'start_time': cur_time, 'result': None}
      self.probe_dict[url] = probe
      probe_thread = threading.Thread(target=self.runProbeThread, args=[url, probe])
      probe_thread.daemon = True
      probe_thread.start()
    return False


  def runProbeThread(self, url, probe):
    try:
      result = self.driver_class.probe_stream(url)
    except Exception as e:
      # Exception text is not passed on: it could quote the URL with credentials
      result = [False, "Probe of " + self.driver_class.get_safe_url(url) + " raised " + type(e).__name__]
    probe['result'] = result


  def startDeviceNode(self, url):
    success = False
    device_name = self.getDeviceName(url)
    safe_url = self.driver_class.get_safe_url(url)
    launch_id = device_name

    # Check if should try to launch
    launch_check = True
    if launch_id in self.launch_time_dict.keys():
      launch_time = self.launch_time_dict[launch_id]
      cur_time = nepi_utils.get_time()
      launch_check = (cur_time - launch_time) > self.NODE_LOAD_TIME_SEC
    if launch_check == False:
      return False  ###

    ### Start Node Launch Process
    node_name = nepi_system.get_device_alias(device_name)
    node_namespace = nepi_sdk.create_namespace(self.base_namespace,node_name)
    self.msg_if.pub_info("Initiating new network stream node " + node_namespace + " for stream " + safe_url)

    #Setup required param server drv_dict for the node. device_path is the
    #credential-free URL shown in the UI; stream_url is what the node connects to.
    drv_dict = copy.deepcopy(self.drv_dict)
    drv_dict['DEVICE_DICT']={'device_name': device_name,
                             'device_path': safe_url,
                             'stream_url': url
                             }
    dict_param_name = nepi_sdk.create_namespace(self.base_namespace,node_name + "/drv_dict")
    nepi_sdk.set_param(dict_param_name,drv_dict)

    file_name = drv_dict['NODE_DICT']['file_name']
    #Try and launch node. Guarded so a launch-helper exception reads as a failed
    #launch instead of stopping the management loop.
    try:
      [success, msg, sub_process] = nepi_drvs.launchDriverNode(file_name, node_name)
    except Exception as e:
      [success, msg, sub_process] = [False, str(e), None]

    # Process launch results
    self.launch_time_dict[launch_id] = nepi_utils.get_time()
    if success:
      self.devices_dict[url] = {'device_name': device_name, 'safe_url': safe_url,
                                'node_name': node_name, 'node_namespace': node_namespace,
                                'node_subprocess': sub_process}
      self.msg_if.pub_info(" Launched node: " + node_name)
    else:
      self.msg_if.pub_warn(" Failed to launch node: " + node_name + " with msg: " + str(msg))
      if self.retry == False:
        self.msg_if.pub_warn(" Will not try relaunch for node: " + node_name)
        self.dont_retry_list.append(launch_id)
      else:
        self.msg_if.pub_warn(" Will attempt relaunch for node: " + node_name + " in " + str(self.NODE_LOAD_TIME_SEC) + " secs")
    return success

  def stopAndPurgeDeviceNode(self, url = 'All'):
    if url == 'All':
      self.check_for_devices = False
    for dev_url in list(self.devices_dict.keys()):
      if dev_url == url or url == 'All':
        device = self.devices_dict.pop(dev_url)
        node_name = device['node_name']
        self.msg_if.pub_info("Killing device node: " + node_name)
        success = nepi_drvs.killDriverNode(node_name,device['node_subprocess'])
        if success == False:
          self.msg_if.pub_warn("Unable to kill device node " + node_name)
        else:
          self.msg_if.pub_info("Node killed. Removed device from active list " + node_name)

  def deviceNodeIsRunning(self, url):
    if url not in self.devices_dict.keys():
      return False
    return self.devices_dict[url]['node_subprocess'].poll() is None


  def cleanup_actions(self):
    self.msg_if.pub_info("Shutting down: Executing script cleanup actions")
    self.stopAndPurgeDeviceNode('All')

if __name__ == '__main__':
  node = NetStreamCamDiscovery()
