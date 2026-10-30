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

# SVX (servo) node for one servo on the Coral Crusaders MATE ROV, such as a
# claw servo, reached through the MATE ROV link RBX node
# (rbx_mate_rov_link_node.py). rbx_mate_rov_link_discovery.py launches one per
# servo the ROV Pi lists in its status (LINK_PROTOCOL "Servos").
#
# NEPI never drives this servo's pulses. The team's sketch moves it from a
# controller input, and the Pi's nepi_link.yaml names that input; this node
# only sets a value from -1 to 1 for it, which the rbx node sends with every
# demand and nepi_link writes into the input. So what a position means is the
# team sketch's business: if the input sets the servo's angle (a stick), a
# goto is a real position; if holding the input moves the servo (a D-pad or a
# button), turn on continuous mode on the servo page, where the SVX IF turns
# direction and speed into an offset from home, so holding is spinning.
#
# NEPI CONTROL ONLY: a value goes out only while NEPI has the ROV, which only
# the pilot's RB gives it. In pilot mode a goto is refused with a warning, and
# the rbx node drops every servo value when NEPI control ends, so after a
# handover the servo starts at neutral. The pilot can still override: on the
# Pi, the pilot's own input wins over NEPI's whenever it is off neutral.
#
# One SVX device = one servo = one node: SVXActuatorIF always lives at
# <node>/svx, as on the Maestro and the Teensy servo nodes.
#
# Degrees: 0 deg is the input's neutral, max_deg its high end (value 1) and
# min_deg its low end (value -1), each half scaled on its own, the same split
# nepi_link's servo_value uses. min_deg and max_deg are node settings, so set
# them to the angles the team's servo really reaches. Home defaults to 0 deg,
# the neutral, which is also where continuous mode stops.
#
# NO FEEDBACK: getPosition is the value NEPI is sending, as the rbx node's
# link_status reports it, not a measurement: 0 deg while NEPI does not have
# the ROV, whatever the pilot does with the servo.

import copy
import json
import threading

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_controls

from std_msgs.msg import String

from nepi_api.device_if_svx import SVXActuatorIF
from nepi_api.messages_if import MsgIF

PKG_NAME = 'RBX_MATE_ROV_LINK' # Use in display menus
FILE_TYPE = 'NODE'


#########################################
# Degree Map
#########################################

def deg_to_value(deg, min_deg, max_deg):
  """Map a servo degree to the -1 to 1 value the link carries.

  Args:
      deg (float): The commanded degree.
      min_deg (float): The degree of value -1, below 0.
      max_deg (float): The degree of value 1, above 0.

  Returns:
      float: -1 to 1; 0 at 0 deg.
  """
  if deg >= 0:
    value = deg / max_deg
  else:
    value = deg / -min_deg
  return max(-1.0, min(1.0, value)) + 0.0

def value_to_deg(value, min_deg, max_deg):
  """Map a link value back to degrees, the inverse of deg_to_value().

  Args:
      value (float): -1 to 1.
      min_deg (float): The degree of value -1, below 0.
      max_deg (float): The degree of value 1, above 0.

  Returns:
      float: min_deg to max_deg.
  """
  value = max(-1.0, min(1.0, float(value)))
  if value >= 0:
    return value * max_deg
  return value * -min_deg


#########################################
# Node Class
#########################################

class MateRovLinkServoNode:

  DEFAULT_MIN_DEG = -90.0
  DEFAULT_MAX_DEG = 90.0
  # 0 deg is the neutral, so the range must straddle it. 5 deg each way keeps
  # the span at the SVX IF's 10 deg minimum.
  MIN_DEG_BOUNDS = [-180.0, -5.0]
  MAX_DEG_BOUNDS = [5.0, 180.0]
  DEFAULT_SPEED_RATIO = 0.5

  # The rbx node publishes link_status every 100 ms; an older one, or none,
  # counts as NEPI not having the ROV.
  LINK_STATUS_TIMEOUT_SEC = 1.0
  REFUSED_WARN_SEC = 5.0

  CAP_SETTINGS = {
    'min_deg' : {"type":"Float", "bounds":MIN_DEG_BOUNDS},
    'max_deg' : {"type":"Float", "bounds":MAX_DEG_BOUNDS}
  }

  settingFunctions = {
    'min_deg' : {'get':'getMinDeg', 'set': 'setMinDeg'},
    'max_deg' : {'get':'getMaxDeg', 'set': 'setMaxDeg'}
  }

  FACTORY_LIMITS_DICT = dict()
  FACTORY_LIMITS_DICT['min_hardstop_deg'] = DEFAULT_MIN_DEG
  FACTORY_LIMITS_DICT['max_hardstop_deg'] = DEFAULT_MAX_DEG
  FACTORY_LIMITS_DICT['min_softstop_deg'] = DEFAULT_MIN_DEG
  FACTORY_LIMITS_DICT['max_softstop_deg'] = DEFAULT_MAX_DEG

  svx_if = None
  drv_dict = dict()

  #######################
  ### Node Initialization
  DEFAULT_NODE_NAME = PKG_NAME.lower() + "_servo_node"

  def __init__(self):
    ####  NODE Initialization ####
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
    # Gather Driver Settings from param server drv_dict
    self.drv_dict = nepi_sdk.get_param('~drv_dict', dict())
    try:
      device_dict = self.drv_dict['DEVICE_DICT']
      self.device_name = device_dict['device_name']
      self.device_path = device_dict['device_path']
      self.servo_name = device_dict['servo_name']
      self.rbx_namespace = device_dict['rbx_namespace']
    except Exception as e:
      self.msg_if.pub_warn("Failed to load Device Dict " + str(e))
      nepi_sdk.signal_shutdown(self.node_name + ": Shutting down because no valid Device Dict")
      return

    # Per-instance copies, so the settings write to this servo's own limits
    # and never to the class-level dict
    self.FACTORY_LIMITS_DICT = copy.deepcopy(MateRovLinkServoNode.FACTORY_LIMITS_DICT)
    self.limits_dict = copy.deepcopy(self.FACTORY_LIMITS_DICT)
    self.speed_ratio = self.DEFAULT_SPEED_RATIO
    self.home_pos_deg = 0.0

    ##############################
    # The rbx node's view of the link: link_status in, set_servo out.
    # link_lock guards the newest link_status, read by the SVX IF's callbacks.
    self.link_lock = threading.Lock()
    self.link_status_time = None # nepi_utils.get_time() of the newest link_status
    self.nepi_control = False
    self.servo_value = 0.0 # The value NEPI sends for this servo, as link_status reports it
    self.pilot_holds = False
    self.last_refused_warn = None
    self.set_servo_pub = nepi_sdk.create_publisher(nepi_sdk.create_namespace(self.rbx_namespace, 'set_servo'),
                                                   String, queue_size = 10)
    nepi_sdk.create_subscriber(nepi_sdk.create_namespace(self.rbx_namespace, 'link_status'), String,
                               self.linkStatusCb, queue_size = 1)

    ##############################
    # Settings. SettingsIF re-applies persisted values through
    # setSettingFunction, which can run before the SVX IF exists.
    self.settings_dict = self.initSettingsDict()
    self.settings_dict = self.refreshSettingsDict()

    ##############################
    # Launch the NEPI SVX interface. servo_connected defaults True: the Pi
    # listed this servo, so it is there, and the servo page lists only
    # connected servos.
    self.msg_if.pub_info("Launching NEPI SVX interface for servo " + self.servo_name + "...")
    self.device_info_dict = dict(device_name = self.device_name,
                                 path = self.device_path,
                                 serial_number = "",
                                 hw_version = "",
                                 sw_version = "")
    self.FACTORY_CONTROLS = {
      'reverse_enabled' : False,
      'speed_ratio' : self.DEFAULT_SPEED_RATIO,
      'servo_connected' : True
    }
    self.svx_if = SVXActuatorIF(device_info = self.device_info_dict,
                                getSettingsFunction = self.getSettingsFunction,
                                setSettingFunction = self.setSettingFunction,
                                factoryControls = self.FACTORY_CONTROLS,
                                factoryLimits = self.FACTORY_LIMITS_DICT,
                                stopMovingCb = self.stopMoving,
                                gotoPositionCb = self.gotoPosition,
                                getPositionCb = self.getPosition,
                                setSoftLimitsCb = self.setSoftLimits,
                                getSoftLimitsCb = self.getSoftLimits,
                                setSpeedRatioCb = self.setSpeedRatio,
                                getSpeedRatioCb = self.getSpeedRatio,
                                goHomeCb = self.goHome,
                                setHomePositionCb = self.setHomePosition,
                                setHomePositionHereCb = self.setHomePositionHere,
                                deviceResetCb = self.resetDevice,
                                msg_if = self.msg_if
                               )
    self.msg_if.pub_info("... SVX interface running")

    # Persisted min_deg/max_deg may have been applied while svx_if was still
    # None, so push the range now that the IF exists
    self.svx_if.setHardstopLimits(self.limits_dict['min_hardstop_deg'], self.limits_dict['max_hardstop_deg'])

    ## Initiation Complete
    self.msg_if.pub_info("Initialization Complete")
    nepi_sdk.spin()


  #**********************
  # Setting functions

  def initSettingsDict(self):
    init_settings_dict = dict()
    for setting_name in self.CAP_SETTINGS.keys():
      setting_dict = copy.deepcopy(self.CAP_SETTINGS[setting_name])
      default = self.readSettingValue(setting_name)
      if default is None:
        continue
      setting_dict['default'] = default
      init_settings_dict[setting_name] = setting_dict
    settings_dict = nepi_controls.create_controls_dict(init_settings_dict)
    self.msg_if.pub_info("Initialized Settings: " + str(nepi_controls.get_values_dict(settings_dict)))
    return settings_dict

  def refreshSettingsDict(self):
    # The settings live only here, so this reads the current values back
    settings_dict = copy.deepcopy(self.settings_dict)
    for setting_name in settings_dict.keys():
      value = self.readSettingValue(setting_name)
      if value is None:
        continue
      settings_dict = nepi_controls.set_value(settings_dict, setting_name, value)
    return settings_dict

  def readSettingValue(self, setting_name):
    if setting_name not in self.settingFunctions.keys():
      return None
    get_function = getattr(self, self.settingFunctions[setting_name]['get'], None)
    if get_function is None:
      return None
    try:
      return float(get_function())
    except Exception as e:
      self.msg_if.pub_warn("Failed to read setting " + setting_name + " : " + str(e))
      return None

  def getSettingsFunction(self):
    # Deep copy: SettingsIF keeps what this returns and mutates it in place
    return copy.deepcopy(self.settings_dict)

  def setSettingFunction(self, setting_name, setting_value):
    setting_str = setting_name + ":" + str(setting_value)
    success = False
    if setting_name not in self.settingFunctions.keys():
      msg = (self.node_name + " Setting name " + setting_str + " is not supported")
      return False, msg, copy.deepcopy(self.settings_dict)
    set_function = getattr(self, self.settingFunctions[setting_name]['set'])
    [success, msg] = set_function(setting_value)
    if success:
      msg = (self.node_name + " UPDATED SETTINGS " + setting_str)
    self.settings_dict = self.refreshSettingsDict()
    return success, msg, copy.deepcopy(self.settings_dict)

  def getMinDeg(self):
    return self.limits_dict['min_hardstop_deg']

  def setMinDeg(self, val):
    return self.setDegRange(new_min = val)

  def getMaxDeg(self):
    return self.limits_dict['max_hardstop_deg']

  def setMaxDeg(self, val):
    return self.setDegRange(new_max = val)

  def setDegRange(self, new_min = None, new_max = None):
    # The degrees the input's two ends stand for. Nothing is sent: the value
    # NEPI is sending stays, and only the degree it is reported as changes.
    try:
      min_deg = self.limits_dict['min_hardstop_deg'] if new_min is None else float(new_min)
      max_deg = self.limits_dict['max_hardstop_deg'] if new_max is None else float(new_max)
    except Exception as e:
      return False, "Degree limit is not a number: " + str(e)
    if min_deg < self.MIN_DEG_BOUNDS[0] or min_deg > self.MIN_DEG_BOUNDS[1]:
      return False, "min_deg must be from " + str(self.MIN_DEG_BOUNDS[0]) + " to " + str(self.MIN_DEG_BOUNDS[1])
    if max_deg < self.MAX_DEG_BOUNDS[0] or max_deg > self.MAX_DEG_BOUNDS[1]:
      return False, "max_deg must be from " + str(self.MAX_DEG_BOUNDS[0]) + " to " + str(self.MAX_DEG_BOUNDS[1])
    # Guarded because SettingsIF can apply a persisted value before the IF exists
    if self.svx_if is not None:
      if self.svx_if.setHardstopLimits(min_deg, max_deg) == False:
        return False, "SVX interface rejected degree range " + str(min_deg) + " to " + str(max_deg)
    for limits in [self.FACTORY_LIMITS_DICT, self.limits_dict]:
      limits['min_hardstop_deg'] = min_deg
      limits['max_hardstop_deg'] = max_deg
    self.FACTORY_LIMITS_DICT['min_softstop_deg'] = min_deg
    self.FACTORY_LIMITS_DICT['max_softstop_deg'] = max_deg
    return True, "Degree range set to " + str(min_deg) + " to " + str(max_deg)


  #######################
  ### Link Functions

  def linkStatusCb(self, msg):
    # The rbx node's get_health(), as JSON, once per demand tick
    try:
      health = json.loads(msg.data)
      servo = health.get('servos', dict()).get(self.servo_name)
      with self.link_lock:
        self.link_status_time = nepi_utils.get_time()
        self.nepi_control = health.get('nepi_control') == True
        if servo is None:
          [self.servo_value, self.pilot_holds] = [0.0, False]
        else:
          [self.servo_value, self.pilot_holds] = [float(servo['value']), servo['pilot'] == True]
    except Exception as e:
      self.msg_if.pub_warn("Bad link_status: " + str(e), throttle_s = 5.0)

  def nepiHasRov(self):
    with self.link_lock:
      if self.link_status_time is None:
        return False
      if nepi_utils.get_time() - self.link_status_time > self.LINK_STATUS_TIMEOUT_SEC:
        return False
      return self.nepi_control

  def sendValue(self, value):
    # Pilot mode refuses the move here, before it reaches the rbx node, which
    # would refuse it too
    if not self.nepiHasRov():
      now = nepi_utils.get_time()
      if self.last_refused_warn is None or now - self.last_refused_warn > self.REFUSED_WARN_SEC:
        self.last_refused_warn = now
        self.msg_if.pub_warn("Servo " + self.servo_name + " not moved: NEPI does not have the ROV. "
                             "The pilot hands it over with RB.")
      return False
    self.set_servo_pub.publish(String(data = json.dumps(dict(name = self.servo_name, value = value))))
    return True


  #######################
  ### SVX IF Callbacks
  #
  # The SVX IF clamps a commanded position to the range and applies the
  # reverse setting before calling gotoPosition, and in continuous mode turns
  # direction and speed into a position too, so every move arrives here.

  def gotoPosition(self, position_deg):
    value = deg_to_value(float(position_deg), self.limits_dict['min_hardstop_deg'],
                         self.limits_dict['max_hardstop_deg'])
    self.sendValue(value)

  def getPosition(self):
    with self.link_lock:
      value = self.servo_value
    return value_to_deg(value, self.limits_dict['min_hardstop_deg'], self.limits_dict['max_hardstop_deg'])

  def stopMoving(self):
    # Positional stop holds where it is: the value NEPI sends already holds
    # until it changes, so there is nothing to send
    pass

  def setSoftLimits(self, min_deg, max_deg):
    self.limits_dict['min_softstop_deg'] = min_deg
    self.limits_dict['max_softstop_deg'] = max_deg

  def getSoftLimits(self):
    return self.limits_dict['min_softstop_deg'], self.limits_dict['max_softstop_deg']

  def setSpeedRatio(self, ratio):
    # Only continuous mode uses it, through the SVX IF; in positional mode the
    # team's sketch decides how fast the servo moves
    self.speed_ratio = max(0.0, min(1.0, float(ratio)))

  def getSpeedRatio(self):
    return self.speed_ratio

  def goHome(self):
    self.gotoPosition(self.home_pos_deg)

  def setHomePosition(self, position_deg):
    self.home_pos_deg = position_deg

  def setHomePositionHere(self):
    self.home_pos_deg = self.getPosition()

  def resetDevice(self):
    # Restore this node's factory settings and send the servo home
    self.speed_ratio = self.DEFAULT_SPEED_RATIO
    self.setDegRange(self.DEFAULT_MIN_DEG, self.DEFAULT_MAX_DEG)
    self.settings_dict = self.refreshSettingsDict()
    self.goHome()


#########################################
# Main
#########################################
if __name__ == '__main__':
  MateRovLinkServoNode()
