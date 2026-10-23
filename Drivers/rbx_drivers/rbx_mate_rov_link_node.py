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

# RBX driver node for the Coral Crusaders MATE ROV, following rbx_gazebo_node.py
# in nepi_drivers. It presents the ROV as an RBX robot by speaking the NEPI
# link protocol (Nepi_Connect/LINK_PROTOCOL.md, UDP and JSON) to nepi_link.py on
# the ROV's control Pi.
#
# The Pi sends a status to this device's status port every 100 ms. This node
# sends a demand (surge, sway, heave, yaw and an `active` flag) to the Pi's
# demand port every 100 ms while the link is up.
#
# NEPI control (Nepi_Connect/BENCH_SETUP_PLAN.md D3, decided 2026-10-22): the
# pilot flies, and the pilot's RB hands the ROV to NEPI; NEPI never takes it.
# The Pi reports NEPI control as the status `assist` flag. While it is on, the
# ROV follows this node's teleop commands, which come from NEPI's
# stabilization process or from the Robots page Teleop. RB again, any
# movement input the pilot pushes past the takeover point, or 2 s with no
# demand hands the ROV back to the pilot, all on the Pi.
#
# One deliberate departure from rbx_gazebo's layout: rbx_gazebo keeps its
# bridge connection in the node class, because that bridge is thin glue. This
# link carries the safety logic -- link-up tracking, the demand rule, the
# timeouts and seq -- so all of it lives in MateRovLink below, structured
# after FastMJPGReceiver (Drivers/idx_drivers/idx_net_stream_driver.py) and
# NepiLink (Nepi_Connect/pi/nepi_link.py). MateRovLink never calls nepi_sdk,
# nepi_api or ROS, so Nepi_Connect/tools/test_rbx_mate_rov_link.py tests it
# against the real nepi_link.py without ROS. Everything else follows
# rbx_gazebo.
#
# The demand rule:
#   - The link is up while a valid status from the ROV address arrived within
#     LINK_TIMEOUT_SEC. While it is down, no demands are sent, so the Pi counts
#     NEPI as not ready and ignores RB.
#   - While it is up, a demand goes out every DEMAND_PERIOD_SEC with active
#     true, which tells the Pi NEPI is ready. Only release_to_pilot() sends
#     active false, once, which ends NEPI control on the Pi (nepi_off).
#   - The demand carries the newest teleop command while NEPI control is on
#     and the command is newer than TELEOP_CMD_TIMEOUT_SEC; otherwise every
#     axis is 0 (neutral). While NEPI control is off the stored command is
#     dropped, so a handover always starts from neutral.
#   - Teleop readiness is link up and NEPI control on, so RBXRobotIF accepts
#     teleop commands exactly while NEPI has the ROV.
#   - Stop clears the stored command (neutral); NEPI keeps the ROV.
#
# The robot state shows who has the ROV, PILOT or NEPI. Setting PILOT hands it
# back (release_to_pilot); setting NEPI is refused, since only the pilot's RB
# hands it over. The teleop_movement_enabled setting is what makes the Robots
# page offer Teleop (NepiDeviceRBX-Controls.js), and FALSE turns teleop off.
#
# The ROV has no mode machinery, no battery, no position fix and no goto
# controller on the NEPI side, so modes, actions, gotos, home and manual motor
# control are empty or None and RBXRobotIF reports the matching capabilities
# False. The navpose carries depth and orientation from the Teensy telemetry in
# each status. Which physical direction each of their signs means has not been
# measured, and the bench Teensy has no sensors, so on the bench every value
# reads 0.

import copy
import json
import math
import socket
import threading
import time

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_nav
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_controls

from std_msgs.msg import Empty, String

from nepi_interfaces.msg import AxisControls

from nepi_api.device_if_rbx import RBXRobotIF
from nepi_api.messages_if import MsgIF

PKG_NAME = 'RBX_MATE_ROV_LINK' # Use in display menus
FILE_TYPE = 'NODE'


#########################################
# Link Class
#########################################

class MateRovLink(object):
  """The NEPI side of the NEPI link: status in, demands out, the demand rule.

  A background thread reads the status port. The node's timer calls
  send_demand() every DEMAND_PERIOD_SEC, and RBXRobotIF's callbacks call
  set_teleop_velocity(), go_stop(), release_to_pilot(), is_link_up() and
  is_nepi_control(). All of them touch the
  same state, so all of it sits behind one lock; methods named ...Locked
  expect the caller to hold it. clock is a monotonic seconds function,
  injectable so tests can control time; wall_clock fills the demand `t` field
  and seeds seq; log prints messages. Bad datagrams are counted and dropped,
  never raised, so a stray datagram on the port cannot stop the link.
  """

  DEMAND_PERIOD_SEC = 0.1 # 10 Hz, the Pi's serial write rate
  LINK_TIMEOUT_SEC = 0.5 # The link is down after this long without a valid status
  # An older teleop command sends neutral. The Robots page resends a held key
  # every 200 ms and sends neutral on release, so this only acts when the
  # sender stops; it also keeps a late resend from blipping the thrusters.
  TELEOP_CMD_TIMEOUT_SEC = 0.5
  MAX_DATAGRAM_BYTES = 1200 # LINK_PROTOCOL "Rules for Every Message", rule 1
  SEQ_MAX = 4294967295
  SEQ_RESTART_SEC = 1.0 # After this long, any status seq is accepted (Pi restarted)
  # The first demand seq is the wall clock times this. Demands go out at 10 Hz,
  # so a restarted node starts ahead of everything its previous run sent, and
  # the Pi, which drops a seq not after the last one for SEQ_RESTART_SEC, never
  # drops its first demands. It wraps inside SEQ_MAX every 497 days; the Pi
  # then drops demands for at most SEQ_RESTART_SEC.
  SEQ_PER_SEC = 100
  ERROR_PRINT_SEC = 10.0 # Each kind of message prints at most once per this period
  SOCKET_TIMEOUT_SEC = 0.2 # How often the receive thread checks for stop
  STATUS_FLAGS = ['assist', 'pilot_surge', 'pilot_sway', 'pilot_yaw', 'pilot_depth', 'slow']
  TN_VALUES = ['depth_m', 'yaw_deg', 'pitch_deg', 'roll_deg'] # The tn values NEPI reads

  def __init__(self, host, status_port = 5700, demand_port = 5701, bind_ip = '0.0.0.0',
               clock = time.monotonic, wall_clock = time.time, log = print):
    self.host = str(host)
    self.status_port = int(status_port)
    self.demand_port = int(demand_port)
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
    self.link_up = False # As of the last send tick, to log changes
    self.nepi_control = False # As of the last send tick, to log changes
    self.cmd = None # [surge, sway, heave, yaw] of the stored teleop command, protocol axes
    self.cmd_time = None # clock() of the newest teleop command
    self.release_pending = False # The next demand is active false, handing the ROV back to the pilot
    self.seq = int(self.wall_clock() * self.SEQ_PER_SEC) & self.SEQ_MAX


  def start(self):
    """Open the status port and the demand socket, and start receiving status.

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
    return True, ('Receiving status from ' + self.host + ' on UDP port ' + str(self.status_port) +
                  ', sending demands to UDP port ' + str(self.demand_port))

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
        bool: True while a valid status from the ROV address arrived within
            LINK_TIMEOUT_SEC. False on any error.
    """
    try:
      with self.lock:
        return self.linkUpLocked(self.clock())
    except Exception as e:
      self.reportError('is_link_up', e)
      return False

  def is_nepi_control(self):
    """Report whether NEPI has the ROV. This is the robot's teleop readiness.

    Returns:
        bool: True while the link is up and the newest status shows NEPI
            control on (the Pi's `assist`). False on any error.
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
    seen from above). Each value is clamped to -1 to 1. The Pi applies its own
    caps and per-vehicle signs, so none are applied here.

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

  def go_stop(self):
    """Clear the stored teleop command, so the next demand is neutral.

    NEPI keeps the ROV; the Teensy's own depth and heading holds keep it steady
    while every axis is neutral.

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
    """Hand the ROV back to the pilot.

    The next demand is active false, which ends NEPI control on the Pi
    (nepi_off); the demands after it are active true again, so NEPI stays
    ready and the pilot's RB can hand the ROV over again.

    Returns:
        list: [success (bool), msg (str)]. success is False while the link is
            down, since no demand can go out then.
    """
    try:
      with self.lock:
        if not self.linkUpLocked(self.clock()):
          return False, 'Link down: nothing to release'
        self.cmd = None
        self.release_pending = True
      return True, 'Releasing the ROV to the pilot'
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
    """Validate one status datagram and keep it as the newest status.

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
    """Report the link state, for the hold process and status displays.

    Returns:
        dict: running (bool) -- the receive thread is started;
            link_up (bool) -- as in is_link_up();
            nepi_control (bool) -- as in is_nepi_control();
            assist (bool), assist_reason (str), pilot_surge, pilot_sway,
                pilot_yaw, pilot_depth, slow (bool) -- from the newest status,
                False and '' before the first. assist is NEPI control as
                the Pi last reported it, and assist_reason why it last changed:
                pilot_button (RB), pilot_stick (the takeover point), timeout,
                nepi_off (release_to_pilot) or start;
            demand_age_ms, tn_age_ms (int) -- the Pi's ages plus the newest
                status's age on this computer, so they keep growing while the
                link is down; -1 when the Pi reports none;
            status_bad (int) -- status datagrams dropped.
    """
    try:
      with self.lock:
        now = self.clock()
        status = self.status
        health = dict(running = self.recv_thread is not None, link_up = self.linkUpLocked(now),
                      nepi_control = self.nepiControlLocked(now), assist = False, assist_reason = '',
                      pilot_surge = False, pilot_sway = False, pilot_yaw = False, pilot_depth = False,
                      slow = False, demand_age_ms = -1, tn_age_ms = -1, status_bad = self.status_bad)
        if status is not None:
          for name in self.STATUS_FLAGS:
            health[name] = status[name]
          health['assist_reason'] = status['assist_reason']
          health['demand_age_ms'] = self.ageNowLocked(status['demand_age_ms'], now)
          health['tn_age_ms'] = self.ageNowLocked(status['tn_age_ms'], now)
        return health
    except Exception as e:
      self.reportError('get_health', e)
      return dict(running = False, link_up = False, nepi_control = False)

  def get_tn(self):
    """Return the newest Teensy telemetry and its age.

    Returns:
        list: [tn (dict or None), tn_age_ms (int)]. tn is a copy of the newest
            status's tn, None before the first or on any error. tn_age_ms is
            as in get_health(), -1 when tn is None.
    """
    try:
      with self.lock:
        if self.status is None or self.status['tn'] is None:
          return None, -1
        return dict(self.status['tn']), self.ageNowLocked(self.status['tn_age_ms'], self.clock())
    except Exception as e:
      self.reportError('get_tn', e)
      return None, -1


  def linkUpLocked(self, now):
    return self.status_time is not None and now - self.status_time <= self.LINK_TIMEOUT_SEC

  def nepiControlLocked(self, now):
    return self.linkUpLocked(now) and self.status['assist']

  def ageNowLocked(self, age_ms, now):
    if age_ms < 0:
      return -1
    return age_ms + max(0, int(round((now - self.status_time) * 1000)))

  def nextDemandLocked(self, now):
    # The demand rule, in the order it is decided. Returns [demand or None, notes to log].
    notes = []
    # Link down: no demand at all, so the Pi counts NEPI as not ready
    link_up = self.linkUpLocked(now)
    if link_up != self.link_up:
      self.link_up = link_up
      if link_up:
        notes.append('Link up: status arriving from the ROV')
      else:
        notes.append('Link down: no valid status for ' + str(self.LINK_TIMEOUT_SEC) + ' s; demands stopped')
    nepi_control = self.nepiControlLocked(now)
    if nepi_control != self.nepi_control:
      self.nepi_control = nepi_control
      if nepi_control:
        notes.append('NEPI control on: the pilot handed over the ROV')
      elif link_up:
        notes.append('NEPI control off (' + self.status['assist_reason'] + '): the pilot has the ROV')
    if not link_up:
      self.cmd = None
      self.release_pending = False
      return None, notes
    # Active true says NEPI is ready; a pending release sends active false once
    active = not self.release_pending
    self.release_pending = False
    # Values only while NEPI has the ROV, from a fresh command; otherwise neutral.
    # Without NEPI control the command is dropped, so a handover starts neutral.
    if not nepi_control:
      self.cmd = None
    [surge, sway, heave, yaw] = [0.0, 0.0, 0.0, 0.0]
    if active and self.cmd is not None and now - self.cmd_time <= self.TELEOP_CMD_TIMEOUT_SEC:
      [surge, sway, heave, yaw] = self.cmd
    demand = {'v': 1, 'type': 'demand', 'seq': self.seq, 't': round(self.wall_clock(), 3), 'active': active,
              'surge': surge, 'sway': sway, 'heave': heave, 'yaw': yaw, 'pitch': 0.0, 'roll': 0.0}
    self.seq = (self.seq + 1) & self.SEQ_MAX
    return demand, notes

  def handleStatusLocked(self, datagram, source_ip, now):
    [status, msg] = self.parseStatus(datagram, source_ip)
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

  def parseStatus(self, datagram, source_ip):
    # LINK_PROTOCOL "Rules for Every Message" 1, 2, 3 and 5, for the fields
    # NEPI reads; rule 4 (seq order) needs the previous status, so
    # handleStatusLocked applies it. Same checks as nepi_link.py's
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
    if not self.isInteger(message.get('v')) or message['v'] != 1:
      return None, 'Status dropped: v must be 1'
    if message.get('type') != 'status':
      return None, 'Status dropped: type must be "status"'
    seq = message.get('seq')
    if not self.isInteger(seq) or seq < 0 or seq > self.SEQ_MAX:
      return None, 'Status dropped: seq must be an integer from 0 to ' + str(self.SEQ_MAX)
    for name in self.STATUS_FLAGS:
      if not isinstance(message.get(name), bool):
        return None, 'Status dropped: ' + name + ' must be true or false'
    if not isinstance(message.get('assist_reason'), str):
      return None, 'Status dropped: assist_reason must be a string'
    for name in ['demand_age_ms', 'tn_age_ms']:
      if not self.isInteger(message.get(name)) or message[name] < -1:
        return None, 'Status dropped: ' + name + ' must be an integer from -1'
    tn = message.get('tn')
    if tn is not None:
      if not isinstance(tn, dict):
        return None, 'Status dropped: tn must be an object or null'
      for name in self.TN_VALUES:
        if not self.isNumber(tn.get(name)):
          return None, 'Status dropped: tn ' + name + ' must be a finite number'
    elif 'tn' not in message:
      return None, 'Status dropped: tn is missing'
    status = dict(seq = seq, assist_reason = message['assist_reason'], demand_age_ms = message['demand_age_ms'],
                  tn = tn, tn_age_ms = message['tn_age_ms'])
    for name in self.STATUS_FLAGS:
      status[name] = message[name]
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
# Node Class
#########################################

class MateRovLinkNode:

  # The state is who has the ROV. The ROV has no mode machinery and no one-shot
  # actions; RBXRobotIF handles empty lists correctly (bounds checks reject any
  # index, status shows "Not Set"), the same as rbx_gazebo's empty modes and go
  # actions. The get*Ind functions must still be real callables, since
  # RBXRobotIF calls them unconditionally.
  RBX_STATES = ["PILOT", "NEPI"]
  RBX_MODES = []
  RBX_SETUP_ACTIONS = []
  RBX_GO_ACTIONS = []

  NAVPOSE_UPDATE_RATE = 10
  TN_MAX_AGE_MS = 500 # The navpose skips Teensy telemetry older than this

  # teleop_movement_enabled is the setting the Robots page looks for before it
  # offers Teleop (NepiDeviceRBX-Controls.js), as in rbx_ardupilot.
  CAP_SETTINGS = dict(
    teleop_movement_enabled = {"type":"Discrete","name":"teleop_movement_enabled","options":["TRUE","FALSE"]}
  )

  FACTORY_SETTINGS = dict(
    teleop_movement_enabled = {"type":"Discrete","name":"teleop_movement_enabled","value":"TRUE"}
  )

  FACTORY_SETTINGS_OVERRIDES = dict()

  #######################
  ### Node Initialization
  DEFAULT_NODE_NAME = PKG_NAME.lower() + "_node"
  drv_dict = dict()

  rbx_if = None
  link = None

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
      self.rov_host = self.drv_dict['DEVICE_DICT']['host']
      self.status_port = self.drv_dict['DEVICE_DICT']['status_port']
      self.demand_port = self.drv_dict['DEVICE_DICT']['demand_port']
    except Exception as e:
      self.msg_if.pub_warn("Failed to load Device Dict " + str(e))
      nepi_sdk.signal_shutdown(self.node_name + ": Shutting down because no valid Device Dict")
      return

    ##############################
    # Settings, stop and navpose state
    self.settings_dict = self.initSettingsDict()
    self.stop_triggered = False
    self.state_ind_last = None # As of the last demand tick, to republish RBX info on a change
    self.navpose_dict = copy.deepcopy(nepi_nav.BLANK_NAVPOSE_DICT)

    # Surge, sway, heave and yaw are commandable. With heave at 0 the Teensy's
    # own depth hold keeps the depth, and version 1 of the link carries no
    # pitch or roll.
    self.axis_controls = AxisControls()
    self.axis_controls.x = True
    self.axis_controls.y = True
    self.axis_controls.z = True
    self.axis_controls.roll = False
    self.axis_controls.pitch = False
    self.axis_controls.yaw = True

    ##############################
    # Link state for the hold process: get_health() as JSON on this node's own
    # link_status topic, once per demand tick. RBXRobotIF's status has no place
    # for driver-specific state, and no other RBX driver publishes any.
    self.link_status_pub = nepi_sdk.create_publisher(nepi_sdk.create_namespace(self.node_namespace, 'link_status'),
                                                     String, queue_size = 1)

    ##############################
    # Start the link to the ROV Pi. A bind failure means another process holds
    # the status port; shutting down lets discovery purge and relaunch later.
    self.link = MateRovLink(self.rov_host, self.status_port, self.demand_port, log = self.msg_if.pub_info)
    [success, msg] = self.link.start()
    if not success:
      self.msg_if.pub_warn(msg)
      nepi_sdk.signal_shutdown(self.node_name + ": Shutting down because the link did not start")
      return
    self.msg_if.pub_info(msg)

    ##############################
    # Launch the NEPI RBX interface. Every callback the ROV does not support is
    # passed None, so the matching capability flags fall out of construction
    # rather than being declared anywhere. data_source_description is left at
    # its default.
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
                             axisControls = self.axis_controls,
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
                             getMotorControlRatios = None,
                             setMotorControlRatio = None,
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
                             getNavPoseCb = self.getNavPoseCb,
                             navpose_update_rate = self.NAVPOSE_UPDATE_RATE,
                             msg_if = self.msg_if
                            )

    self.msg_if.pub_info("... RBX interface running")

    # RBXRobotIF republishes its info, which carries the state the Robots page
    # shows under Set State, only when a state is set through NEPI. The pilot's
    # RB changes it on the Pi, so sendDemandCb asks for a republish through the
    # IF's own publish_info topic whenever the state changes.
    self.publish_info_pub = nepi_sdk.create_publisher(nepi_sdk.create_namespace(self.rbx_if.namespace, 'publish_info'),
                                                      Empty, queue_size = 1)

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
    # Nothing on the ROV to read back; the settings live only here
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
    success = True
    msg = (self.node_name + " UPDATED SETTINGS " + setting_str)
    return success, msg, copy.deepcopy(self.settings_dict)

  ##########################
  # RBX Interface Functions

  def getStateInd(self):
    # PILOT (0) or NEPI (1): who has the ROV, as the Pi last reported it
    if self.link is not None and self.link.is_nepi_control():
      return 1
    return 0

  def setStateInd(self, state_ind):
    # Only the pilot's RB hands the ROV to NEPI; NEPI can only hand it back
    if self.RBX_STATES[state_ind] == "PILOT":
      if self.link.is_nepi_control():
        [success, msg] = self.link.release_to_pilot()
        self.msg_if.pub_info(msg)
        return success
      return True
    self.msg_if.pub_warn("Only the pilot's RB on the controller hands the ROV to NEPI")
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
    # Teleop drives the ROV only while NEPI has it, and only while the
    # teleop_movement_enabled setting allows it
    if self.getSettingSelection('teleop_movement_enabled') != 'TRUE':
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
    # Depth and orientation from the newest Teensy telemetry. The update is
    # skipped while there is none or it is older than TN_MAX_AGE_MS, which
    # includes the whole time the link is down, since the telemetry's age keeps
    # growing then. Until the first fresh telemetry it returns None, not the
    # blank dict: NPXDeviceIF fixes has_depth and has_orientation from the first
    # dict it gets, so a blank one would leave both False for the node's life
    # whenever the driver starts before the ROV Pi is sending.
    [tn, tn_age_ms] = self.link.get_tn()
    if tn is None or tn_age_ms < 0 or tn_age_ms > self.TN_MAX_AGE_MS:
      if self.navpose_dict['has_depth'] == False:
        return None
      return self.navpose_dict
    tn_time = nepi_utils.get_time() - tn_age_ms / 1000.0
    self.navpose_dict['has_depth'] = True
    self.navpose_dict['time_depth'] = tn_time
    self.navpose_dict['depth_m'] = float(tn['depth_m'])
    self.navpose_dict['has_orientation'] = True
    self.navpose_dict['time_orientation'] = tn_time
    self.navpose_dict['roll_deg'] = float(tn['roll_deg'])
    self.navpose_dict['pitch_deg'] = float(tn['pitch_deg'])
    self.navpose_dict['yaw_deg'] = float(tn['yaw_deg'])
    return self.navpose_dict

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
    # state for the hold process, and the RBX info when the state changed.
    self.link.send_demand()
    self.link_status_pub.publish(String(data = json.dumps(self.link.get_health())))
    state_ind = self.getStateInd()
    if state_ind != self.state_ind_last:
      self.state_ind_last = state_ind
      self.publish_info_pub.publish(Empty())

  #######################
  # Node Cleanup Function

  def cleanup_actions(self):
    """Hands the ROV back to the pilot on node shutdown.

    Sends one last active false demand if the link is up, so the Pi ends NEPI
    control at once (nepi_off) instead of after its 2 s timeout, then stops
    the link.
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
  MateRovLinkNode()
