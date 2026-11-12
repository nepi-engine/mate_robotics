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

# RBX driver node for any robot that runs NEPI Connect, following
# rbx_gazebo_node.py in nepi_drivers. It presents the robot as an RBX robot by
# speaking the NEPI Connect link protocol, version 2
# (Nepi_Connect/LINK_PROTOCOL.md, UDP and JSON), to the nepi_connect package
# on the robot. Nothing here is specific to one robot: discovery launches this
# node with the robot's vehicle message, its announcement of what it is, in
# DEVICE_DICT['vehicle'], and the node builds itself from that. The MATE ROV
# (Nepi_Connect/vehicles/mate_coral_crusaders.yaml) was the first.
#
# The robot sends a status to this device's status port every 100 ms, and a
# vehicle message every 0.5 s. This node sends a demand (a value for each
# announced axis, the servo values and an `active` flag) to the robot's
# announced demand port every 100 ms while the link is up.
#
# NEPI control (Nepi_Connect/BENCH_SETUP_PLAN.md D3, decided 2026-10-22): the
# pilot drives, and the pilot's toggle input (RB on the MATE ROV) hands the
# robot to NEPI; NEPI never takes it. The robot reports NEPI control as the
# status `control` flag. While it is on, the robot follows this node's teleop
# commands, which come from NEPI's stabilization process or from the Robots
# page Teleop. The toggle again, any axis input the pilot pushes past the
# takeover point, or 2 s with no demand hands the robot back to the pilot,
# all on the robot.
#
# One deliberate departure from rbx_gazebo's layout: rbx_gazebo keeps its
# bridge connection in the node class, because that bridge is thin glue. This
# link carries the safety logic -- link-up tracking, the demand rule, the
# timeouts and seq -- so all of it lives in NepiConnectLink below, structured
# after FastMJPGReceiver (Drivers/idx_drivers/idx_net_stream_driver.py) and
# NepiControl (Nepi_Connect/pi/nepi_connect/control.py). NepiConnectLink
# never calls nepi_sdk, nepi_api or ROS, so
# Nepi_Connect/tools/test_rbx_nepi_connect.py tests it against the real
# nepi_connect package without ROS. Everything else follows rbx_gazebo.
#
# The demand rule:
#   - The link is up while a valid status from the robot address arrived
#     within LINK_TIMEOUT_SEC. While it is down, no demands are sent, so the
#     robot counts NEPI as not ready and ignores its toggle input.
#   - While it is up, a demand goes out every DEMAND_PERIOD_SEC with active
#     true, which tells the robot NEPI is ready. Only release_to_pilot() sends
#     active false, once, which ends NEPI control on the robot (nepi_off).
#   - The demand carries the newest teleop command while NEPI control is on
#     and the command is newer than TELEOP_CMD_TIMEOUT_SEC; otherwise every
#     axis is 0 (neutral). While NEPI control is off the stored command is
#     dropped, so a handover always starts from neutral. Only the announced
#     axes go out.
#   - Teleop readiness is link up and NEPI control on, so RBXRobotIF accepts
#     teleop commands exactly while NEPI has the robot.
#   - Stop clears the stored command (neutral); NEPI keeps the robot.
#
# Servos (decided 2026-10-28): the robot's vehicle file can name up to four
# inputs that move a servo in the robot's own code, such as a claw's button
# (LINK_PROTOCOL "Servos"); the robot's code is not changed. The vehicle
# message lists them, discovery launches one SVX node per servo
# (rbx_nepi_connect_servo_node.py), and each servo node sends its value here
# on this node's set_servo topic. A value is accepted only while NEPI has the
# robot, goes out with every demand, holds until changed, and is dropped when
# NEPI control ends, so the pilot's toggle input is the only way NEPI moves a
# servo. RBX Stop leaves the servos alone; each has its own stop on the servo
# page.
#
# The robot state shows who has the robot, PILOT or NEPI. Setting PILOT hands
# it back (release_to_pilot); setting NEPI is refused, since only the pilot's
# toggle input hands it over. The teleop_movement_enabled setting is what
# makes the Robots page offer Teleop (NepiDeviceRBX-Controls.js), and FALSE
# turns teleop off.
#
# A NEPI Connect robot has no mode machinery, no battery, no position fix and
# no goto controller on the NEPI side, so modes, actions, gotos, home and
# manual motor control are empty or None and RBXRobotIF reports the matching
# capabilities False. The axis capabilities follow the announced axes (roll
# and pitch are never controllable). The navpose carries the navigation
# groups the robot announces: depth and depth rate, and orientation and
# angular rates turned into the robot's frame by the imu_mount setting
# (ImuMount), which starts at the announced imu_mount.

import copy
import json
import math
import re
import socket
import threading
import time

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_nav
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_controls

from std_msgs.msg import Empty, String

from nepi_api.device_if_rbx import RBXRobotIF
from nepi_api.device_if_npx import NPXDeviceIF
from nepi_api.messages_if import MsgIF

PKG_NAME = 'RBX_NEPI_CONNECT' # Use in display menus
FILE_TYPE = 'NODE'


#########################################
# Link Class
#########################################

class NepiConnectLink(object):
  """The NEPI side of the NEPI Connect link: status in, demands out, the demand rule.

  A background thread reads the status port. The node's timer calls
  send_demand() every DEMAND_PERIOD_SEC, and RBXRobotIF's callbacks call
  set_teleop_velocity(), go_stop(), release_to_pilot(), is_link_up() and
  is_nepi_control(). All of them touch the
  same state, so all of it sits behind one lock; methods named ...Locked
  expect the caller to hold it. clock is a monotonic seconds function,
  injectable so tests can control time; wall_clock fills the demand `t` field
  and seeds seq; log prints messages. Bad datagrams are counted and dropped,
  never raised, so a stray datagram on the port cannot stop the link.

  vehicle is the announcement discovery launched the node with
  (rbx_nepi_connect_discovery.parse_vehicle_message). It decides which axes
  go out, which servos exist, which navigation values a status must carry,
  and the demand port.
  """

  DEMAND_PERIOD_SEC = 0.1 # 10 Hz, the robot's control loop rate (the MATE ROV Pi's serial write rate)
  LINK_TIMEOUT_SEC = 0.5 # The link is down after this long without a valid status
  # An older teleop command sends neutral. The Robots page resends a held key
  # every 200 ms and sends neutral on release, so this only acts when the
  # sender stops; it also keeps a late resend from blipping the thrusters.
  TELEOP_CMD_TIMEOUT_SEC = 0.5
  MAX_DATAGRAM_BYTES = 1200 # LINK_PROTOCOL "Rules for Every Message", rule 1
  SEQ_MAX = 4294967295
  SEQ_RESTART_SEC = 1.0 # After this long, any seq is accepted (robot restarted)
  # The first demand seq is the wall clock times this. Demands go out at 10 Hz,
  # so a restarted node starts ahead of everything its previous run sent, and
  # the robot, which drops a seq not after the last one for SEQ_RESTART_SEC,
  # never drops its first demands. It wraps inside SEQ_MAX every 497 days; the
  # robot then drops demands for at most SEQ_RESTART_SEC.
  SEQ_PER_SEC = 100
  ERROR_PRINT_SEC = 10.0 # Each kind of message prints at most once per this period
  SOCKET_TIMEOUT_SEC = 0.2 # How often the receive thread checks for stop
  PROTOCOL_VERSION = 2
  AXIS_NAMES = ['surge', 'sway', 'heave', 'yaw'] # In the order every list of them follows
  CONTROL_REASONS = ['start', 'pilot_button', 'pilot_stick', 'timeout', 'nepi_off']
  # The navigation groups, each announced whole or not at all
  NAV_GROUPS = [['depth', ['depth_m', 'depth_m_s']],
                ['orientation', ['roll_deg', 'pitch_deg', 'yaw_deg', 'gyro_x_deg_s', 'gyro_y_deg_s',
                                 'gyro_z_deg_s']]]
  # The robot's servo limits (nepi_connect vehicle.py MAX_SERVOS and SERVO_NAME)
  MAX_SERVOS = 4
  SERVO_NAME = re.compile(r'^[a-z][a-z0-9_]{0,15}$')
  SERVO_DECIMALS = 4 # Servo values go out rounded to this, which keeps the demand short
  # The fields of the vehicle message that describe the robot
  VEHICLE_KEYS = ['name', 'kind', 'demand_port', 'axes', 'servos', 'nav', 'raw', 'cameras', 'imu_mount']
  # What each optional part means when the vehicle message leaves it out
  # (rbx_nepi_connect_discovery.parse_vehicle_message uses the same)
  VEHICLE_DEFAULTS = dict(axes = dict(), servos = [], nav = [], raw = [], cameras = [], imu_mount = 'X_FORWARD_Y_LEFT')

  def __init__(self, host, vehicle, status_port = 5700, bind_ip = '0.0.0.0',
               clock = time.monotonic, wall_clock = time.time, log = print):
    self.host = str(host)
    self.vehicle = self.vehicleBody(vehicle)
    self.axis_names = [name for name in self.AXIS_NAMES if name in self.vehicle['axes']]
    self.servo_names = list(self.vehicle['servos'])
    self.nav_names = list(self.vehicle['nav'])
    self.status_port = int(status_port)
    self.demand_port = int(self.vehicle['demand_port'])
    self.bind_ip = str(bind_ip)
    self.lock = threading.Lock()
    self.clock = clock
    self.wall_clock = wall_clock
    self.log = log
    self.status_sock = None
    self.demand_sock = None
    self.recv_thread = None
    self.stop_event = None
    self.last_print = dict()
    self.suppressed = dict()
    self.resetStateLocked()

  def resetStateLocked(self):
    self.status = None # Newest accepted status, as parseStatus() returns it
    self.status_time = None # clock() when it was accepted
    self.status_seq = -1
    self.status_bad = 0
    self.vehicle_seq = -1 # Vehicle messages have a seq of their own
    self.vehicle_time = None
    self.vehicle_changed = None # The newest announcement that differs from the launch one
    self.link_up = False # As of the last send tick, to log changes
    self.nepi_control = False # As of the last send tick, to log changes
    self.cmd = None # [surge, sway, heave, yaw] of the stored teleop command, protocol axes
    self.cmd_time = None # clock() of the newest teleop command
    self.servo_cmd = dict() # Servo name to the value NEPI sends, -1 to 1
    self.release_pending = False # The next demand is active false, handing the robot back to the pilot
    self.seq = int(self.wall_clock() * self.SEQ_PER_SEC) & self.SEQ_MAX


  def start(self):
    """Open the status port and the demand socket, and start receiving vehicle and status messages.

    Resets all link state, including seq, which is taken from the wall clock
    (see SEQ_PER_SEC).

    Returns:
        list: [success (bool), msg (str)].
    """
    if self.recv_thread is not None:
      return True, 'Already receiving on UDP port ' + str(self.status_port)
    status_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
      status_sock.bind((self.bind_ip, self.status_port))
    except OSError as e:
      status_sock.close()
      return False, 'Cannot receive on ' + self.bind_ip + ' UDP port ' + str(self.status_port) + ': ' + str(e)
    status_sock.settimeout(self.SOCKET_TIMEOUT_SEC)
    demand_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    with self.lock:
      self.resetStateLocked()
      self.demand_sock = demand_sock
    self.status_sock = status_sock
    self.stop_event = threading.Event()
    self.recv_thread = threading.Thread(target = self.runRecvThread, args = [status_sock, self.stop_event])
    self.recv_thread.daemon = True
    self.recv_thread.start()
    return True, ('Receiving status from ' + self.vehicle['name'] + ' at ' + self.host + ' on UDP port ' +
                  str(self.status_port) + ', sending demands to UDP port ' + str(self.demand_port) + ' for axes ' +
                  ', '.join(self.axis_names))

  def stop(self):
    """Stop receiving and close both sockets. Sends nothing.

    Returns:
        list: [success (bool), msg (str)].
    """
    if self.recv_thread is None:
      return True, 'Not receiving'
    self.stop_event.set()
    self.recv_thread.join(self.SOCKET_TIMEOUT_SEC * 2 + 1.0)
    self.recv_thread = None
    # A thread still inside recvfrom() gets an error from the closed socket,
    # sees the stop event and exits.
    self.status_sock.close()
    self.status_sock = None
    with self.lock:
      demand_sock = self.demand_sock
      self.demand_sock = None
    demand_sock.close()
    return True, 'Stopped receiving on UDP port ' + str(self.status_port)

  def is_link_up(self):
    """Report whether the link is up.

    Returns:
        bool: True while a valid status from the robot address arrived within
            LINK_TIMEOUT_SEC. False on any error.
    """
    try:
      with self.lock:
        return self.linkUpLocked(self.clock())
    except Exception as e:
      self.reportError('is_link_up', e)
      return False

  def is_nepi_control(self):
    """Report whether NEPI has the robot. This is the robot's teleop readiness.

    Returns:
        bool: True while the link is up and the newest status shows NEPI
            control on (the robot's `control`). False on any error.
    """
    try:
      with self.lock:
        return self.nepiControlLocked(self.clock())
    except Exception as e:
      self.reportError('is_nepi_control', e)
      return False

  def set_teleop_velocity(self, linear_x, linear_y, linear_z, angular_z):
    """Store a teleop command, mapped from ROS axes to protocol axes.

    This is the only place the axes are mapped: surge = linear_x, sway =
    -linear_y (ROS y is positive left, protocol sway positive right), heave =
    -linear_z (ROS z is positive up, protocol heave positive down), and yaw =
    -angular_z (ROS turns positive counter-clockwise, the protocol clockwise
    seen from above). Each value is clamped to -1 to 1. The robot applies its
    own caps and per-vehicle signs, so none are applied here. All four are
    stored; send_demand() sends only the announced ones.

    Args:
        linear_x (float): Forward ratio, -1 to 1.
        linear_y (float): Left ratio, -1 to 1.
        linear_z (float): Up ratio, -1 to 1.
        angular_z (float): Counter-clockwise turn ratio, -1 to 1.

    Returns:
        list: [success (bool), msg (str)]. success is False, and nothing is
            stored, when a value is not a finite number.
    """
    try:
      for value in [linear_x, linear_y, linear_z, angular_z]:
        if not self.isNumber(value):
          msg = 'Teleop command dropped: values must be finite numbers'
          self.printLimited('teleop', msg)
          return False, msg
      cmd = [self.clampAxis(linear_x), self.clampAxis(-linear_y), self.clampAxis(-linear_z),
             self.clampAxis(-angular_z)]
      with self.lock:
        self.cmd = cmd
        self.cmd_time = self.clock()
      return True, 'Stored'
    except Exception as e:
      self.reportError('set_teleop_velocity', e)
      return False, 'Teleop error: ' + str(e)

  def set_servo(self, name, value):
    """Store the value for one of the robot's servos, sent with every demand.

    Unlike a teleop command, a servo value never goes stale: it holds until it
    is changed. It is accepted only while NEPI has the robot, and every value is
    dropped when NEPI control ends, so a handover starts each servo at
    neutral, as it does the movement axes.

    Args:
        name (str): A servo name from the vehicle message.
        value (float): -1 to 1, clamped. 0 is the input's neutral.

    Returns:
        list: [success (bool), msg (str)]. Nothing is stored on failure.
    """
    try:
      if not self.isNumber(value):
        return False, 'Servo value dropped: it must be a finite number'
      with self.lock:
        if not self.nepiControlLocked(self.clock()):
          return False, ('Servo ' + str(name) + ' not moved: NEPI does not have the robot '
                         '(the pilot hands it over with the toggle input)')
        if name not in self.servo_names:
          return False, 'Servo ' + str(name) + ' not moved: the robot has no servo by that name'
        self.servo_cmd[name] = self.clampAxis(value)
      return True, 'Stored'
    except Exception as e:
      self.reportError('set_servo', e)
      return False, 'Servo error: ' + str(e)

  def go_stop(self):
    """Clear the stored teleop command, so the next demand is neutral.

    NEPI keeps the robot, and every axis goes to neutral; what neutral does
    is up to the robot's own code (the MATE ROV's Teensy holds depth and
    heading). Servo values are left alone.

    Returns:
        list: [success (bool), msg (str)].
    """
    try:
      with self.lock:
        self.cmd = None
      return True, 'Stopped'
    except Exception as e:
      self.reportError('go_stop', e)
      return False, 'Stop error: ' + str(e)

  def release_to_pilot(self):
    """Hand the robot back to the pilot.

    The next demand is active false, which ends NEPI control on the robot
    (nepi_off); the demands after it are active true again, so NEPI stays
    ready and the pilot's toggle input can hand the robot over again.

    Returns:
        list: [success (bool), msg (str)]. success is False while the link is
            down, since no demand can go out then.
    """
    try:
      with self.lock:
        if not self.linkUpLocked(self.clock()):
          return False, 'Link down: nothing to release'
        self.cmd = None
        self.servo_cmd = dict()
        self.release_pending = True
      return True, 'Releasing the robot to the pilot'
    except Exception as e:
      self.reportError('release_to_pilot', e)
      return False, 'Release error: ' + str(e)

  def send_demand(self):
    """Send the next demand, per the demand rule. The node calls this every DEMAND_PERIOD_SEC.

    Returns:
        list: [sent (bool), msg (str)]. Nothing is sent while the link is
            down or the link is stopped.
    """
    try:
      with self.lock:
        sock = self.demand_sock
        if sock is None:
          return False, 'Not running'
        [demand, notes] = self.nextDemandLocked(self.clock())
      for note in notes:
        self.printMessage(note)
      if demand is None:
        return False, 'Link down: no demand sent'
      datagram = json.dumps(demand, separators = (',', ':')).encode('utf-8')
      sock.sendto(datagram, (self.host, self.demand_port))
      return True, 'Sent'
    except Exception as e:
      self.reportError('send_demand', e)
      return False, 'Send error: ' + str(e)

  def handle_status(self, datagram, source_ip):
    """Validate one datagram from the status port: a status, or a vehicle message.

    A status becomes the newest status. A vehicle message is compared with
    the one the node was launched with; one that differs is kept for
    get_health(), so the node can say the driver must be disabled and
    enabled. Either way the node keeps running on the launch announcement.

    Args:
        datagram (bytes): The UDP payload.
        source_ip (str): The sender's address.

    Returns:
        list: [accepted (bool), msg (str)]. A dropped datagram adds 1 to
            status_bad and changes nothing else.
    """
    try:
      with self.lock:
        return self.handleStatusLocked(datagram, source_ip, self.clock())
    except Exception as e:
      self.reportError('handle_status', e)
      return False, 'Status error: ' + str(e)

  def get_health(self):
    """Report the link state, for the hold process, the servo nodes and status displays.

    Returns:
        dict: running (bool) -- the receive thread is started;
            link_up (bool) -- as in is_link_up();
            nepi_control (bool) -- as in is_nepi_control();
            control (bool), control_reason (str) -- from the newest status,
                False and '' before the first. control is NEPI control as
                the robot last reported it, and control_reason why it last
                changed: pilot_button (the toggle input), pilot_stick (the
                takeover point), timeout, nepi_off (release_to_pilot) or start;
            pilot (dict) -- each announced axis by name, True while the pilot
                holds one of its inputs; all False before the first status;
            demand_age_ms, nav_age_ms (int) -- the robot's ages plus the
                newest status's age on this computer, so they keep growing
                while the link is down; -1 when the robot reports none;
            status_bad (int) -- datagrams on the status port dropped;
            servos (dict) -- each of the robot's servos by name, as a dict:
                value (float), what NEPI sends, 0.0 when nothing is stored;
                pilot (bool), the pilot holds its input. Empty before the
                first status;
            raw (dict or None) -- the robot's raw telemetry values from the
                newest status, None when it sends none;
            vehicle (str), kind (str) -- the announced vehicle;
            cameras (list) -- each announced camera as a dict of name,
                transport and url, the address to add to NEPI's network
                stream camera driver;
            vehicle_changed (bool) -- a later vehicle message differs from
                the launch one, so the driver needs a disable and enable.
    """
    try:
      with self.lock:
        now = self.clock()
        status = self.status
        health = dict(running = self.recv_thread is not None, link_up = self.linkUpLocked(now),
                      nepi_control = self.nepiControlLocked(now), control = False, control_reason = '',
                      pilot = dict((name, False) for name in self.axis_names), demand_age_ms = -1,
                      nav_age_ms = -1, status_bad = self.status_bad, servos = dict(), raw = None,
                      vehicle = self.vehicle['name'], kind = self.vehicle['kind'], cameras = self.cameraUrls(),
                      vehicle_changed = self.vehicle_changed is not None)
        if status is not None:
          health['control'] = status['control']
          health['control_reason'] = status['control_reason']
          health['pilot'] = dict(status['pilot'])
          health['demand_age_ms'] = self.ageNowLocked(status['demand_age_ms'], now)
          health['nav_age_ms'] = self.ageNowLocked(status['nav_age_ms'], now)
          for [name, held] in status['servos'].items():
            health['servos'][name] = dict(value = self.servo_cmd.get(name, 0.0), pilot = held)
          if status['raw'] is not None:
            health['raw'] = dict(status['raw'])
        return health
    except Exception as e:
      self.reportError('get_health', e)
      return dict(running = False, link_up = False, nepi_control = False, servos = dict())

  def get_nav(self):
    """Return the newest navigation values and their age.

    Returns:
        list: [nav (dict or None), nav_age_ms (int)]. nav is a copy of the
            newest status's nav, by standard name, None before the first or
            on any error. nav_age_ms is as in get_health(), -1 when nav is None.
    """
    try:
      with self.lock:
        if self.status is None or self.status['nav'] is None:
          return None, -1
        return dict(self.status['nav']), self.ageNowLocked(self.status['nav_age_ms'], self.clock())
    except Exception as e:
      self.reportError('get_nav', e)
      return None, -1

  def get_vehicle_change(self):
    """Return the newest vehicle message that differs from the launch one.

    Returns:
        dict: That announcement, or None while every vehicle message matched
            the launch one.
    """
    try:
      with self.lock:
        if self.vehicle_changed is None:
          return None
        return copy.deepcopy(self.vehicle_changed)
    except Exception as e:
      self.reportError('get_vehicle_change', e)
      return None

  def cameraUrls(self):
    # Each announced camera with the address NEPI's network stream camera
    # driver reads it at (LINK_PROTOCOL "Video to NEPI")
    cameras = []
    for camera in self.vehicle['cameras']:
      if camera['transport'] == 'http':
        url = 'http://' + self.host + ':' + str(camera['port']) + '/stream'
      else:
        url = 'fastmjpg://0.0.0.0:' + str(camera['port'])
      cameras.append(dict(name = camera['name'], transport = camera['transport'], url = url))
    return cameras

  def vehicleBody(self, message):
    # The fields of a vehicle message that describe the robot, in one form, so
    # the launch announcement and a later one compare equal when they match
    body = dict()
    for key in self.VEHICLE_KEYS:
      value = message.get(key)
      # A part the robot leaves out is one it does not have, the same as an empty one
      if value is None and key in self.VEHICLE_DEFAULTS:
        value = copy.deepcopy(self.VEHICLE_DEFAULTS[key])
      if key == 'axes' and isinstance(value, dict):
        value = dict((name, float(cap) if self.isNumber(cap) else cap) for (name, cap) in value.items())
      elif key == 'cameras' and isinstance(value, list):
        value = [dict((field, camera.get(field)) for field in ['name', 'transport', 'port'])
                 if isinstance(camera, dict) else camera for camera in value]
      body[key] = copy.deepcopy(value)
    return body


  def linkUpLocked(self, now):
    return self.status_time is not None and now - self.status_time <= self.LINK_TIMEOUT_SEC

  def nepiControlLocked(self, now):
    return self.linkUpLocked(now) and self.status['control']

  def ageNowLocked(self, age_ms, now):
    if age_ms < 0:
      return -1
    return age_ms + max(0, int(round((now - self.status_time) * 1000)))

  def nextDemandLocked(self, now):
    # The demand rule, in the order it is decided. Returns [demand or None, notes to log].
    notes = []
    # Link down: no demand at all, so the robot counts NEPI as not ready
    link_up = self.linkUpLocked(now)
    if link_up != self.link_up:
      self.link_up = link_up
      if link_up:
        notes.append('Link up: status arriving from ' + self.vehicle['name'])
      else:
        notes.append('Link down: no valid status for ' + str(self.LINK_TIMEOUT_SEC) + ' s; demands stopped')
    nepi_control = self.nepiControlLocked(now)
    if nepi_control != self.nepi_control:
      self.nepi_control = nepi_control
      if nepi_control:
        notes.append('NEPI control on: the pilot handed over the robot')
      elif link_up:
        notes.append('NEPI control off (' + self.status['control_reason'] + '): the pilot has the robot')
    if not link_up:
      self.cmd = None
      self.servo_cmd = dict()
      self.release_pending = False
      return None, notes
    # Active true says NEPI is ready; a pending release sends active false once
    active = not self.release_pending
    self.release_pending = False
    # Values only while NEPI has the robot, from a fresh command; otherwise neutral.
    # Without NEPI control the command and the servo values are dropped, so a
    # handover starts neutral.
    if not nepi_control:
      self.cmd = None
      self.servo_cmd = dict()
    values = [0.0, 0.0, 0.0, 0.0]
    if active and self.cmd is not None and now - self.cmd_time <= self.TELEOP_CMD_TIMEOUT_SEC:
      values = self.cmd
    # Only the announced axes go out
    axes = dict((name, value) for [name, value] in zip(self.AXIS_NAMES, values) if name in self.axis_names)
    servos = dict()
    if active:
      servos = dict((name, round(value, self.SERVO_DECIMALS)) for [name, value] in self.servo_cmd.items())
    demand = {'v': self.PROTOCOL_VERSION, 'type': 'demand', 'seq': self.seq, 't': round(self.wall_clock(), 3),
              'active': active, 'axes': axes, 'servos': servos}
    self.seq = (self.seq + 1) & self.SEQ_MAX
    return demand, notes

  def handleStatusLocked(self, datagram, source_ip, now):
    [message, msg] = self.parseMessage(datagram, source_ip)
    if message is None:
      self.status_bad += 1
      return False, msg
    if message['type'] == 'vehicle':
      return self.handleVehicleLocked(message, now)
    [status, msg] = self.parseStatus(message)
    if status is None:
      self.status_bad += 1
      return False, msg
    if (self.status_seq >= 0 and status['seq'] <= self.status_seq and
        now - self.status_time <= self.SEQ_RESTART_SEC):
      self.status_bad += 1
      return False, 'Status dropped: seq ' + str(status['seq']) + ' is not after ' + str(self.status_seq)
    self.status = status
    self.status_seq = status['seq']
    self.status_time = now
    return True, msg

  def handleVehicleLocked(self, message, now):
    # A vehicle message after launch: only a change matters, and it never
    # changes what the node runs on (rbx_nepi_connect_discovery validated the
    # launch one; a changed one may be anything, which is why it needs a
    # relaunch, not a live update)
    if (self.vehicle_seq >= 0 and message['seq'] <= self.vehicle_seq and
        now - self.vehicle_time <= self.SEQ_RESTART_SEC):
      self.status_bad += 1
      return False, 'Vehicle message dropped: seq ' + str(message['seq']) + ' is not after ' + str(self.vehicle_seq)
    self.vehicle_seq = message['seq']
    self.vehicle_time = now
    body = self.vehicleBody(message)
    if body == self.vehicle:
      self.vehicle_changed = None
      return True, 'Vehicle unchanged'
    self.vehicle_changed = body
    return True, 'Vehicle changed'

  def parseMessage(self, datagram, source_ip):
    # LINK_PROTOCOL "Rules for Every Message" 1, 2, 3 and 5, common to both
    # message types; rule 4 (seq order) needs the previous message of the same
    # type, so the caller applies it. Same checks as nepi_connect's
    # parse_demand and fake_nepi.py's StatusReceiver.
    if source_ip != self.host:
      return None, 'Status from ' + str(source_ip) + ' dropped: expecting status from ' + self.host
    if len(datagram) > self.MAX_DATAGRAM_BYTES:
      return None, 'Status of ' + str(len(datagram)) + ' bytes dropped: over ' + str(self.MAX_DATAGRAM_BYTES)
    try:
      message = json.loads(datagram.decode('utf-8'))
    except Exception as e:
      return None, 'Status dropped: not UTF-8 JSON (' + str(e) + ')'
    if not isinstance(message, dict):
      return None, 'Status dropped: not a JSON object'
    if not self.isInteger(message.get('v')) or message['v'] != self.PROTOCOL_VERSION:
      return None, 'Status dropped: v must be ' + str(self.PROTOCOL_VERSION)
    if message.get('type') not in ['status', 'vehicle']:
      return None, 'Status dropped: type must be "status" or "vehicle"'
    seq = message.get('seq')
    if not self.isInteger(seq) or seq < 0 or seq > self.SEQ_MAX:
      return None, 'Status dropped: seq must be an integer from 0 to ' + str(self.SEQ_MAX)
    return message, 'Accepted'

  def parseStatus(self, message):
    # The status fields NEPI reads (LINK_PROTOCOL "Status"), against the
    # launch announcement
    if not isinstance(message.get('control'), bool):
      return None, 'Status dropped: control must be true or false'
    if message.get('control_reason') not in self.CONTROL_REASONS:
      return None, 'Status dropped: control_reason must be one of ' + ', '.join(self.CONTROL_REASONS)
    # A robot need not have every part, so a missing pilot, nav, nav_age_ms,
    # raw or servos field means it has none: no pilot on any axis, no
    # navigation, no servos. A field that is present must still be well formed.
    pilot = message.get('pilot')
    if pilot is None:
      pilot = dict((name, False) for name in self.axis_names)
    if (not isinstance(pilot, dict) or any(not isinstance(pilot.get(name), bool) for name in self.axis_names)):
      return None, 'Status dropped: pilot must map every announced axis to true or false'
    nav_age_ms = message.get('nav_age_ms')
    if nav_age_ms is None:
      nav_age_ms = -1
    for [name, value] in [['demand_age_ms', message.get('demand_age_ms')], ['nav_age_ms', nav_age_ms]]:
      if not self.isInteger(value) or value < -1:
        return None, 'Status dropped: ' + name + ' must be an integer from -1'
    nav = message.get('nav')
    if nav is not None:
      if not isinstance(nav, dict):
        return None, 'Status dropped: nav must be an object or null'
      for name in self.nav_names:
        if not self.isNumber(nav.get(name)):
          return None, 'Status dropped: nav ' + name + ' must be a finite number'
      nav = dict((name, nav[name]) for name in self.nav_names)
    raw = message.get('raw')
    if raw is not None:
      if not isinstance(raw, dict) or any(not self.isNumber(value) for value in raw.values()):
        return None, 'Status dropped: raw must map names to finite numbers, or be null'
      raw = dict(raw)
    servos = message.get('servos')
    if servos is None:
      servos = dict()
    if not isinstance(servos, dict) or len(servos) > self.MAX_SERVOS:
      return None, 'Status dropped: servos must be an object with at most ' + str(self.MAX_SERVOS) + ' servos'
    for [name, held] in servos.items():
      if not self.SERVO_NAME.match(name) or not isinstance(held, bool):
        return None, 'Status dropped: servos must map servo names to true or false'
    status = dict(seq = message['seq'], control = message['control'], control_reason = message['control_reason'],
                  pilot = dict((name, pilot[name]) for name in self.axis_names),
                  demand_age_ms = message['demand_age_ms'], nav = nav, nav_age_ms = nav_age_ms,
                  raw = raw, servos = dict(servos))
    return status, 'Accepted'

  def isInteger(self, value):
    return isinstance(value, int) and not isinstance(value, bool)

  def isNumber(self, value):
    if isinstance(value, bool):
      return False
    if isinstance(value, int):
      return True
    return isinstance(value, float) and math.isfinite(value)

  def clampAxis(self, value):
    # + 0.0 turns a negated zero into 0.0
    return max(-1.0, min(1.0, float(value))) + 0.0


  def runRecvThread(self, sock, stop_event):
    while not stop_event.is_set():
      try:
        [datagram, sender] = sock.recvfrom(65536)
      except socket.timeout:
        continue
      except OSError as e:
        if stop_event.is_set():
          break
        self.printLimited('status', 'Status receive error: ' + str(e))
        stop_event.wait(self.SOCKET_TIMEOUT_SEC)
        continue
      [accepted, msg] = self.handle_status(datagram, sender[0])
      if not accepted:
        self.printLimited('status', msg)


  def reportError(self, where, error):
    self.printLimited('error', where + ' error: ' + str(error))

  def printLimited(self, key, msg):
    # Never call this while holding the lock
    try:
      with self.lock:
        now = self.clock()
        last = self.last_print.get(key)
        if last is not None and now - last < self.ERROR_PRINT_SEC:
          self.suppressed[key] = self.suppressed.get(key, 0) + 1
          return
        self.last_print[key] = now
        skipped = self.suppressed.pop(key, 0)
      if skipped > 0:
        msg += ' (' + str(skipped) + ' more not shown)'
      self.log(msg)
    except Exception:
      pass

  def printMessage(self, msg):
    try:
      self.log(msg)
    except Exception:
      pass


#########################################
# IMU Mount Class
#########################################

# Directions in the robot's own axes, ROS body convention: x forward, y left, z up
IMU_DIRECTIONS = [('FORWARD', (1, 0, 0)), ('BACK', (-1, 0, 0)), ('LEFT', (0, 1, 0)),
                  ('RIGHT', (0, -1, 0)), ('UP', (0, 0, 1)), ('DOWN', (0, 0, -1))]
# One option per square mount: where the board's printed X and Y arrows point.
# The two must be perpendicular, which leaves 24.
IMU_MOUNT_OPTIONS = ['X_' + x_name + '_Y_' + y_name
                     for (x_name, x_dir) in IMU_DIRECTIONS for (y_name, y_dir) in IMU_DIRECTIONS
                     if sum(a * b for (a, b) in zip(x_dir, y_dir)) == 0]
IMU_MOUNT_FLAT = 'X_FORWARD_Y_LEFT' # Board flat, X forward: values pass through unchanged


class ImuMount(object):
  """The IMU board's mounting in the robot, set by the node's imu_mount setting.

  The robot reports the board's own orientation and gyro rates: Z-Y-X yaw,
  pitch and roll like NEPI's (the MATE ROV's Teensy uses Adafruit's
  quaternion conversion, new_ctl.ino:279-291), and the three gyro axes. On a
  board that does not lie flat with X forward, those are the board's angles,
  not the robot's. With the board on its side (the MATE team's ROV,
  2026-10-26) the conversion even reports pitch near +-90 deg, where yaw and
  roll stop being separate angles. This class rebuilds the board's rotation
  from the reported angles, which stays exact there, and turns it and the
  rates into the robot's frame.

  A mount is named by where the board's X and Y arrows point, for example
  X_UP_Y_LEFT; Z follows from the right-hand rule. It never calls nepi_sdk,
  nepi_api or ROS, so Nepi_Connect/tools/test_rbx_nepi_connect.py tests it
  without ROS.
  """

  def __init__(self, name = IMU_MOUNT_FLAT):
    """Builds the mount from an IMU_MOUNT_OPTIONS name.

    Raises:
        ValueError: name is not in IMU_MOUNT_OPTIONS.
    """
    if name not in IMU_MOUNT_OPTIONS:
      raise ValueError('Unknown IMU mount ' + str(name))
    directions = dict(IMU_DIRECTIONS)
    [x_name, y_name] = name[len('X_'):].split('_Y_')
    x_dir = directions[x_name]
    y_dir = directions[y_name]
    z_dir = (x_dir[1] * y_dir[2] - x_dir[2] * y_dir[1],
             x_dir[2] * y_dir[0] - x_dir[0] * y_dir[2],
             x_dir[0] * y_dir[1] - x_dir[1] * y_dir[0])
    self.name = name
    # Columns are the board's X, Y and Z arrows in robot axes, so this times a
    # board vector gives the same vector in robot axes
    self.matrix = [[x_dir[i], y_dir[i], z_dir[i]] for i in range(3)]

  def orientation_to_rov(self, roll_deg, pitch_deg, yaw_deg):
    """Turns the board's reported orientation into the robot's.

    Args:
        roll_deg, pitch_deg, yaw_deg (float): The board's Z-Y-X angles, as
            the robot reports them.

    Returns:
        list: [roll_deg, pitch_deg, yaw_deg] of the robot, Z-Y-X, yaw from
            -180 to 180. The input, unchanged, for the flat mount.
    """
    if self.name == IMU_MOUNT_FLAT:
      return [roll_deg, pitch_deg, yaw_deg]
    board = self.rotationFromEuler(roll_deg, pitch_deg, yaw_deg)
    # The robot's rotation is the board's times the mount's inverse (its transpose)
    rov = [[sum(board[i][k] * self.matrix[j][k] for k in range(3)) for j in range(3)] for i in range(3)]
    pitch = math.degrees(math.asin(max(-1.0, min(1.0, -rov[2][0]))))
    yaw = math.degrees(math.atan2(rov[1][0], rov[0][0]))
    roll = math.degrees(math.atan2(rov[2][1], rov[2][2]))
    return [roll, pitch, yaw]

  def rates_to_rov(self, gyro_x, gyro_y, gyro_z):
    """Turns the board's three gyro rates into the robot's.

    Args:
        gyro_x, gyro_y, gyro_z (float): Rates about the board's X, Y and Z
            arrows, deg/s.

    Returns:
        list: [roll_deg_per_sec, pitch_deg_per_sec, yaw_deg_per_sec] about the
            robot's forward, left and up axes.
    """
    gyro = (gyro_x, gyro_y, gyro_z)
    return [sum(self.matrix[i][k] * gyro[k] for k in range(3)) for i in range(3)]

  def rotationFromEuler(self, roll_deg, pitch_deg, yaw_deg):
    # Rz(yaw) Ry(pitch) Rx(roll), the rotation the reported angles came from
    [cr, sr] = [math.cos(math.radians(roll_deg)), math.sin(math.radians(roll_deg))]
    [cp, sp] = [math.cos(math.radians(pitch_deg)), math.sin(math.radians(pitch_deg))]
    [cy, sy] = [math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))]
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]]


#########################################
# Announcement Helpers
#########################################

def axis_capabilities(vehicle):
  """Build RBXRobotIF's axis capabilities from a vehicle announcement.

  Args:
      vehicle (dict): The announcement (DEVICE_DICT['vehicle']).

  Returns:
      dict: has_x/y/z/yaw_control True for each announced surge, sway, heave
          and yaw; has_roll_control and has_pitch_control always False, since
          the link carries no roll or pitch.
  """
  axes = vehicle['axes']
  return dict(has_x_control = 'surge' in axes, has_y_control = 'sway' in axes, has_z_control = 'heave' in axes,
              has_roll_control = False, has_pitch_control = False, has_yaw_control = 'yaw' in axes)

def navpose_groups(vehicle):
  """Say which navpose groups a vehicle announcement provides.

  Args:
      vehicle (dict): The announcement (DEVICE_DICT['vehicle']).

  Returns:
      list: [has_depth (bool), has_orientation (bool)], each True only when
          the announcement lists every standard name of that group.
  """
  nav = set(vehicle['nav'])
  return [set(NepiConnectNode.DEPTH_NAV) <= nav, set(NepiConnectNode.ORIENTATION_NAV) <= nav]

def fill_navpose(navpose_dict, nav, nav_time, has_depth, has_orientation, imu_mount):
  """Write one status's navigation values into a navpose dict.

  Only the announced groups are written, so one not announced keeps the
  blank dict's has_* False.

  Args:
      navpose_dict (dict): The navpose, changed in place.
      nav (dict): The status's nav, by standard name.
      nav_time (float): When the robot read it, on this computer's clock.
      has_depth (bool), has_orientation (bool): The announced groups.
      imu_mount (ImuMount): Turns orientation and rates into the robot's frame.
  """
  if has_depth:
    navpose_dict['has_depth'] = True
    navpose_dict['time_depth'] = nav_time
    navpose_dict['frame_depth'] = 'DEPTH' # Below the surface, not the blank dict's MSL
    navpose_dict['depth_m'] = float(nav['depth_m'])
    navpose_dict['depth_m_per_sec'] = float(nav['depth_m_s'])
  if has_orientation:
    # Orientation and rates in the robot's frame, through the imu_mount
    # setting. The rates are about the board's X, Y and Z arrows (on the MATE
    # ROV, the Teensy's yaw_deg_s, pitch_deg_s and roll_deg_s columns, which
    # its vehicle file maps to gyro X, Y and Z).
    [roll, pitch, yaw] = imu_mount.orientation_to_rov(float(nav['roll_deg']), float(nav['pitch_deg']),
                                                      float(nav['yaw_deg']))
    [roll_rate, pitch_rate, yaw_rate] = imu_mount.rates_to_rov(float(nav['gyro_x_deg_s']), float(nav['gyro_y_deg_s']),
                                                               float(nav['gyro_z_deg_s']))
    navpose_dict['has_orientation'] = True
    navpose_dict['time_orientation'] = nav_time
    navpose_dict['roll_deg'] = roll
    navpose_dict['pitch_deg'] = pitch
    navpose_dict['yaw_deg'] = yaw
    navpose_dict['roll_deg_per_sec'] = roll_rate
    navpose_dict['pitch_deg_per_sec'] = pitch_rate
    navpose_dict['yaw_deg_per_sec'] = yaw_rate


#########################################
# Node Class
#########################################

class NepiConnectNode:

  # The state is who has the robot. The robot has no mode machinery and no
  # one-shot actions; RBXRobotIF handles empty lists correctly (bounds checks reject any
  # index, status shows "Not Set"), the same as rbx_gazebo's empty modes and go
  # actions. The get*Ind functions must still be real callables, since
  # RBXRobotIF calls them unconditionally.
  RBX_STATES = ["PILOT", "NEPI"]
  RBX_MODES = []
  RBX_SETUP_ACTIONS = []
  RBX_GO_ACTIONS = []

  NAVPOSE_UPDATE_RATE = 10
  NAV_MAX_AGE_MS = 500 # The navpose skips telemetry older than this
  # The navpose groups a robot can announce (LINK_PROTOCOL "Navigation")
  DEPTH_NAV = ['depth_m', 'depth_m_s']
  ORIENTATION_NAV = ['roll_deg', 'pitch_deg', 'yaw_deg', 'gyro_x_deg_s', 'gyro_y_deg_s', 'gyro_z_deg_s']

  # teleop_movement_enabled is the setting the Robots page looks for before it
  # offers Teleop (NepiDeviceRBX-Controls.js), as in rbx_ardupilot. imu_mount
  # says where the IMU board's X and Y arrows point in the robot (ImuMount);
  # one setting rather than one per arrow, so no order of changes passes
  # through a mount whose arrows are parallel. Its factory default is the
  # announced imu_mount (FACTORY_SETTINGS_OVERRIDES, set per node).
  CAP_SETTINGS = dict(
    teleop_movement_enabled = {"type":"Discrete","name":"teleop_movement_enabled","options":["TRUE","FALSE"]},
    imu_mount = {"type":"Discrete","name":"imu_mount","options":IMU_MOUNT_OPTIONS}
  )

  FACTORY_SETTINGS = dict(
    teleop_movement_enabled = {"type":"Discrete","name":"teleop_movement_enabled","value":"TRUE"},
    imu_mount = {"type":"Discrete","name":"imu_mount","value":IMU_MOUNT_FLAT}
  )

  FACTORY_SETTINGS_OVERRIDES = dict()

  #######################
  ### Node Initialization
  DEFAULT_NODE_NAME = PKG_NAME.lower() + "_node"
  drv_dict = dict()

  rbx_if = None
  link = None

  # NPX device that publishes the robot's navpose (see selectOwnNavPoseCb)
  npx_if = None
  node_if = None
  settings_if = None
  save_data_if = None
  connect_navpose_if = None
  OWN_NAVPOSE_MAX_TRIES = 60 # 1 s apart
  own_navpose_topic = None
  own_navpose_tries = 0

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
      self.device_name = self.drv_dict['DEVICE_DICT']['device_name']
      self.device_path = self.drv_dict['DEVICE_DICT']['device_path']
      self.robot_host = self.drv_dict['DEVICE_DICT']['host']
      self.status_port = self.drv_dict['DEVICE_DICT']['status_port']
      # The robot's announcement, which discovery validated and launched this node with
      self.vehicle = self.drv_dict['DEVICE_DICT']['vehicle']
      self.vehicle_name = self.vehicle['name']
      # The servos discovery launched SVX nodes for, from the announcement
      self.servo_names = sorted(self.vehicle['servos'])
      self.caps_dict = axis_capabilities(self.vehicle)
      [self.has_depth_nav, self.has_orientation_nav] = navpose_groups(self.vehicle)
    except Exception as e:
      self.msg_if.pub_warn("Failed to load Device Dict " + str(e))
      nepi_sdk.signal_shutdown(self.node_name + ": Shutting down because no valid Device Dict")
      return

    ##############################
    # Settings, stop and navpose state. The announced imu_mount is this
    # node's factory default; a value the operator saved still wins.
    self.FACTORY_SETTINGS_OVERRIDES = dict(NepiConnectNode.FACTORY_SETTINGS_OVERRIDES)
    if self.vehicle.get('imu_mount') in IMU_MOUNT_OPTIONS:
      self.FACTORY_SETTINGS_OVERRIDES['imu_mount'] = self.vehicle['imu_mount']
    else:
      self.msg_if.pub_warn("Announced imu_mount " + str(self.vehicle.get('imu_mount')) + " is not one of " +
                           str(IMU_MOUNT_OPTIONS) + "; starting at " + IMU_MOUNT_FLAT)
    self.settings_dict = self.initSettingsDict()
    self.imu_mount = ImuMount()
    self.applyImuMount()
    self.stop_triggered = False
    self.state_ind_last = None # As of the last demand tick, to republish RBX info on a change
    self.vehicle_change_seen = None # As of the last demand tick, to warn once each time the announcement changes
    self.navpose_dict = copy.deepcopy(nepi_nav.BLANK_NAVPOSE_DICT)
    self.nav_seen = False # The navpose has had one fresh telemetry update

    ##############################
    # Link state for the hold process: get_health() as JSON on this node's own
    # link_status topic, once per demand tick. RBXRobotIF's status has no place
    # for driver-specific state, and no other RBX driver publishes any.
    self.link_status_pub = nepi_sdk.create_publisher(nepi_sdk.create_namespace(self.node_namespace, 'link_status'),
                                                     String, queue_size = 1)

    ##############################
    # Start the link to the robot. A bind failure means another process holds
    # the status port; shutting down lets discovery purge and relaunch later.
    self.link = NepiConnectLink(self.robot_host, self.vehicle, self.status_port, log = self.msg_if.pub_info)
    [success, msg] = self.link.start()
    if not success:
      self.msg_if.pub_warn(msg)
      nepi_sdk.signal_shutdown(self.node_name + ": Shutting down because the link did not start")
      return
    self.msg_if.pub_info(msg)

    ##############################
    # Launch the NEPI RBX interface. Every callback a NEPI Connect robot does
    # not support is passed None, so the matching capability flags fall out of
    # construction rather than being declared anywhere; the axis capabilities
    # follow the announced axes. data_source_description is left at its default.
    self.msg_if.pub_info("Launching NEPI RBX interface...")
    self.device_info_dict = dict(device_name = self.device_name,
                                 path = self.device_path,
                                 serial_number = "",
                                 hw_version = "",
                                 sw_version = "")
    self.msg_if.pub_info(str(self.device_info_dict))

    self.rbx_if = RBXRobotIF(device_info = self.device_info_dict,
                             getSettingsFunction = self.getSettingsFunction,
                             setSettingFunction = self.setSettingFunction,
                             axisCapabilities = self.caps_dict,
                             getBatteryPercentFunction = None,
                             states = self.RBX_STATES,
                             getStateIndFunction = self.getStateInd,
                             setStateIndFunction = self.setStateInd,
                             modes = self.RBX_MODES,
                             getModeIndFunction = self.getModeInd,
                             setModeIndFunction = self.setModeInd,
                             checkStopFunction = self.checkStopFunction,
                             setup_actions = self.RBX_SETUP_ACTIONS,
                             setSetupActionIndFunction = self.setSetupActionInd,
                             go_actions = self.RBX_GO_ACTIONS,
                             setGoActionIndFunction = self.setGoActionInd,
                             manualControlsReadyFunction = None,
                             teleopControlsReadyFunction = self.teleopControlsReady,
                             setTeleopVelocityFunction = self.setTeleopVelocity,
                             autonomousControlsReadyFunction = None,
                             getHomeFunction = None,
                             setHomeFunction = None,
                             goHomeFunction = None,
                             goStopFunction = self.goStop,
                             gotoPoseFunction = None,
                             gotoPositionFunction = None,
                             gotoLocationFunction = None,
                             msg_if = self.msg_if
                            )

    self.msg_if.pub_info("... RBX interface running")
    ready = self.rbx_if.wait_for_ready()

    self.node_if = self.rbx_if.node_if
    self.settings_if = self.rbx_if.settings_if
    self.save_data_if = self.rbx_if.save_data_if
    self.connect_navpose_if = self.rbx_if.connect_navpose_if
    ##################################
    # Setup NPX IF Class ####################
    # The robot's navpose is published by an NPX device at
    # <node>/<device_name>/npx/navpose, from getNavPoseCb, and selected as the
    # RBX navpose source, as in rbx_ardupilot.
    self.msg_if.pub_debug("Starting NPX Device IF Initialization")
    self.npx_if = NPXDeviceIF(device_info = self.device_info_dict,
        getNavPoseCb = self.getNavPoseCb,
        max_navpose_update_rate = self.NAVPOSE_UPDATE_RATE,
        msg_if = self.msg_if,
        node_if = self.node_if,
        settings_if = self.settings_if,
        save_data_if = self.save_data_if,
        connect_navpose_if = self.connect_navpose_if
        )
    # Make the robot's own navpose the RBX navpose source.
    self.own_navpose_topic = self.npx_if.navpose_if.get_namespace()
    nepi_sdk.start_timer_process(1.0, self.selectOwnNavPoseCb, oneshot = True)

    # Servo values from this robot's servo nodes, as JSON {"name": ..., "value": ...}
    nepi_sdk.create_subscriber(nepi_sdk.create_namespace(self.node_namespace, 'set_servo'), String,
                               self.setServoCb, queue_size = 10)

    ## Start the demand tick
    nepi_sdk.start_timer_process(self.link.DEMAND_PERIOD_SEC, self.sendDemandCb)

    ## Initiation Complete
    self.msg_if.pub_info("Initialization Complete")
    nepi_sdk.on_shutdown(self.cleanup_actions)
    nepi_sdk.spin()


  #**********************
  # Setting functions

  def initSettingsDict(self):
    init_settings_dict = dict()
    for setting_name in self.CAP_SETTINGS.keys():
      cap_setting = self.CAP_SETTINGS[setting_name]
      setting_type = cap_setting['type']
      setting_dict = dict()
      setting_dict['type'] = setting_type
      if 'options' in cap_setting.keys():
        setting_dict['options'] = [str(option) for option in cap_setting['options']]
      default = None
      if setting_name in self.FACTORY_SETTINGS.keys():
        default = self.FACTORY_SETTINGS[setting_name]['value']
      if setting_name in self.FACTORY_SETTINGS_OVERRIDES.keys():
        default = self.FACTORY_SETTINGS_OVERRIDES[setting_name]
      if default is None:
        continue
      setting_dict['default'] = str(default)
      init_settings_dict[setting_name] = setting_dict
    settings_dict = nepi_controls.create_controls_dict(init_settings_dict)
    self.msg_if.pub_info("Initialized Settings: " + str(nepi_controls.get_values_dict(settings_dict)))
    return settings_dict

  def refreshSettingsDict(self):
    # Nothing on the robot to read back; the settings live only here
    return copy.deepcopy(self.settings_dict)

  def getSettingSelection(self, setting_name):
    # nepi_controls.get_value() hands a Selection back as a one-element list
    # (['TRUE']), so this returns the selected option itself.
    value = nepi_controls.get_value(self.settings_dict, setting_name)
    if isinstance(value, list):
      value = value[0] if len(value) > 0 else None
    return value

  def getSettingsFunction(self):
    # Deep copy: SettingsIF keeps what this returns and mutates it in place
    return copy.deepcopy(self.settings_dict)

  def setSettingFunction(self, setting_name, setting_value):
    success = False
    setting_str = setting_name + ":" + str(setting_value)
    if setting_name not in self.settings_dict.keys():
      msg = (self.node_name + " Setting name " + setting_str + " is not supported")
      return success, msg, copy.deepcopy(self.settings_dict)
    if nepi_controls.get_clean_value(self.settings_dict, setting_name, setting_value) is None:
      msg = (self.node_name + " Setting data " + setting_str + " is not valid")
      return success, msg, copy.deepcopy(self.settings_dict)
    self.settings_dict = nepi_controls.set_value(self.settings_dict, setting_name, setting_value)
    if setting_name == 'imu_mount':
      self.applyImuMount()
    success = True
    msg = (self.node_name + " UPDATED SETTINGS " + setting_str)
    return success, msg, copy.deepcopy(self.settings_dict)

  def applyImuMount(self):
    # The navpose reads self.imu_mount from its own timer, so it is replaced
    # whole, never changed in place. nepi_controls keeps a Selection to one of
    # its options, so the ValueError path is only a guard.
    name = self.getSettingSelection('imu_mount')
    try:
      self.imu_mount = ImuMount(name)
      self.msg_if.pub_info("IMU mount: " + str(name))
    except ValueError as e:
      self.msg_if.pub_warn("Kept IMU mount " + self.imu_mount.name + ": " + str(e))

  ##########################
  # RBX Interface Functions

  def getStateInd(self):
    # PILOT (0) or NEPI (1): who has the robot, as it last reported it
    if self.link is not None and self.link.is_nepi_control():
      return 1
    return 0

  def setStateInd(self, state_ind):
    # Only the pilot's toggle input hands the robot to NEPI; NEPI can only hand it back
    if self.RBX_STATES[state_ind] == "PILOT":
      if self.link.is_nepi_control():
        [success, msg] = self.link.release_to_pilot()
        self.msg_if.pub_info(msg)
        return success
      return True
    self.msg_if.pub_warn("Only the pilot's toggle input on the controller hands the robot to NEPI")
    return False

  def getModeInd(self):
    return 0

  def setModeInd(self, mode_ind):
    # Unreachable with empty RBX_MODES
    return False

  def checkStopFunction(self):
    triggered = self.stop_triggered
    self.stop_triggered = False # Reset Stop Trigger
    return triggered

  def teleopControlsReady(self):
    # Teleop drives the robot only while NEPI has it, and only while the
    # teleop_movement_enabled setting allows it
    if self.getSettingSelection('teleop_movement_enabled') != 'TRUE':
      return False
    # A robot with no motor axes, only servos or sensors, has nothing to teleop
    if len(self.link.axis_names) == 0:
      return False
    return self.link.is_nepi_control()

  def setTeleopVelocity(self, linear_x, linear_y, linear_z, angular_z):
    # Ratios from -1 to 1 in ROS axes. The link maps them to protocol axes and
    # holds the newest one; sendDemandCb sends it on the next tick.
    self.link.set_teleop_velocity(linear_x, linear_y, linear_z, angular_z)

  def goStop(self):
    self.stop_triggered = True
    self.link.go_stop()
    return True

  def getNavPoseCb(self):
    # The announced navigation groups from the newest telemetry. The update is
    # skipped while there is none or it is older than NAV_MAX_AGE_MS, which
    # includes the whole time the link is down, since the telemetry's age keeps
    # growing then. Until the first fresh telemetry it returns None, not the
    # blank dict: NPXDeviceIF fixes has_depth and has_orientation from the first
    # dict it gets, so a blank one would leave both False for the node's life
    # whenever the driver starts before the robot is sending. A group the
    # robot does not announce is never filled, so it stays False from the
    # first dict on.
    [nav, nav_age_ms] = self.link.get_nav()
    if nav is None or nav_age_ms < 0 or nav_age_ms > self.NAV_MAX_AGE_MS:
      if self.nav_seen == False:
        return None
      return self.navpose_dict
    nav_time = nepi_utils.get_time() - nav_age_ms / 1000.0
    fill_navpose(self.navpose_dict, nav, nav_time, self.has_depth_nav, self.has_orientation_nav, self.imu_mount)
    self.nav_seen = self.navpose_dict['has_depth'] or self.navpose_dict['has_orientation']
    if self.nav_seen == False:
      return None
    return self.navpose_dict

  def selectOwnNavPoseCb(self, timer):
    # Select the robot's own NPX navpose as the RBX navpose source once the
    # connect has discovered it, as in rbx_ardupilot. Only a startup default is
    # replaced ('None', '', or the node namespace that older saved configs
    # hold), never a source the operator picked, and it gives up after a
    # bounded number of tries.
    connect_if = self.connect_navpose_if
    if connect_if is None or self.own_navpose_topic is None or nepi_sdk.is_shutdown():
      return
    selected = connect_if.get_selected_topic()
    if selected == self.own_navpose_topic:
      return
    if selected not in ['None', '', None, self.node_namespace]:
      self.msg_if.pub_info("Keeping operator navpose source: " + str(selected))
      return
    if self.own_navpose_topic in connect_if.get_available_topics():
      connect_if.set_selected_topic(self.own_navpose_topic)
      self.msg_if.pub_info("Selected own navpose source: " + self.own_navpose_topic)
      return
    self.own_navpose_tries += 1
    if self.own_navpose_tries < self.OWN_NAVPOSE_MAX_TRIES:
      nepi_sdk.start_timer_process(1.0, self.selectOwnNavPoseCb, oneshot = True)
    else:
      self.msg_if.pub_warn("Own navpose source never appeared, leaving navpose unselected: " + self.own_navpose_topic)

  #######################
  ### Setup-Action Functions

  def setSetupActionInd(self, action_ind):
    # Unreachable with empty RBX_SETUP_ACTIONS (RBXRobotIF bounds-checks first)
    return False

  #######################
  ### Go-Action Functions

  def setGoActionInd(self, action_ind):
    # Unreachable with empty RBX_GO_ACTIONS (RBXRobotIF bounds-checks first)
    return False

  #######################
  ### Link Processes

  def sendDemandCb(self, timer):
    # One demand per tick, decided by the link's demand rule, then the link
    # state for the hold process and the servo nodes, and the RBX info when
    # the state changed.
    self.link.send_demand()
    health = self.link.get_health()
    self.link_status_pub.publish(String(data = json.dumps(health)))
    # The pilot's toggle input changes the state on the robot, so RBXRobotIF republishes its
    # status, which carries the state the Robots page shows under Set State,
    # whenever the state changes, not only on its own timer.
    state_ind = self.getStateInd()
    if state_ind != self.state_ind_last:
      self.state_ind_last = state_ind
      self.rbx_if.publish_status()
    # The node, its axes and its servo nodes were built from the launch
    # announcement, so say once when the robot announces something else
    changed = self.link.get_vehicle_change()
    if changed != self.vehicle_change_seen:
      self.vehicle_change_seen = changed
      if changed is not None:
        self.msg_if.pub_warn("The robot's vehicle message changed (now " + json.dumps(changed) + "), but this "
                             "driver runs for the one it was launched with. Disable and enable the driver to match it.")

  def setServoCb(self, msg):
    # One servo node's value. A refusal (pilot mode, unknown name) is logged,
    # at most once per 5 s for each message.
    try:
      request = json.loads(msg.data)
      [success, text] = self.link.set_servo(request['name'], request['value'])
    except Exception as e:
      [success, text] = [False, "Bad set_servo message " + str(msg.data) + ": " + str(e)]
    if not success:
      self.msg_if.pub_warn(text, throttle_s = 5.0)

  #######################
  # Node Cleanup Function

  def cleanup_actions(self):
    """Hands the robot back to the pilot on node shutdown.

    Sends one last active false demand if the link is up, so the robot ends
    NEPI control at once (nepi_off) instead of after its 2 s timeout, then
    stops the link.
    """
    self.msg_if.pub_info("Shutting down: Executing script cleanup actions")
    if self.link is not None:
      self.link.release_to_pilot()
      self.link.send_demand()
      self.link.stop()


#########################################
# Main
#########################################
if __name__ == '__main__':
  NepiConnectNode()
