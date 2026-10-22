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
# the ROV's control Pi. Where LINK_PROTOCOL.md and nepi_link.py disagree, the
# code wins; Nepi_Connect/BENCH_SETUP_PLAN.md "Doc Changes Pending" lists where.
#
# The Pi sends a status to this device's status port every 100 ms. This node
# sends a demand (surge, sway, yaw and an `active` flag) to the Pi's demand
# port every 100 ms while the link is up. The pilot's stick wins any axis it
# moves, on the Pi, at all times; NEPI's demand only fills the axes the pilot
# leaves alone.
#
# One deliberate departure from rbx_gazebo's layout: rbx_gazebo keeps its
# bridge connection in the node class, because that bridge is thin glue. This
# link carries the safety logic -- link-up tracking, the on/off rule, the latch,
# the timeouts and seq -- so all of it lives in MateRovLink below, structured
# after FastMJPGReceiver (Drivers/idx_drivers/idx_net_stream_driver.py) and
# NepiLink (Nepi_Connect/pi/nepi_link.py). MateRovLink never calls nepi_sdk,
# nepi_api or ROS, so Nepi_Connect/tools/test_rbx_mate_rov_link.py tests it
# against the real nepi_link.py without ROS. Everything else follows
# rbx_gazebo.
#
# The on/off rule (BENCH_SETUP_PLAN.md D3 option c on the Pi side):
#   - The link is up while a valid status from the ROV address arrived within
#     LINK_TIMEOUT_SEC. While it is down, no demands are sent and teleop
#     readiness is False.
#   - While it is up, a demand goes out every DEMAND_PERIOD_SEC, always:
#     active true while a teleop command newer than TELEOP_CMD_TIMEOUT_SEC is
#     held and the link is not latched, otherwise active false with every axis
#     0. The Pi turns assist on only at a false-to-true edge, so the idle
#     active false stream is what makes the first command engage.
#   - Teleop readiness is link up, not assist on. RBXRobotIF drops commands
#     while the robot reports not ready, and assist only comes on after NEPI
#     sends active true, so gating readiness on assist would make engaging
#     impossible.
#   - Latch: while sending active true, the newest status showing assist false
#     latches the link to active false, if assist was true during this
#     engagement (RB, the Pi's 2 s timeout) or LATCH_ENGAGE_SEC has passed
#     since this engagement's first active true (a link outage, or a restarted
#     Pi script, which ignores a first-ever active true). The latch clears only
#     after no teleop command has arrived for LATCH_CLEAR_SEC, so the hold
#     process must stop and restart its commands to engage again. NEPI never
#     turns assist back on by itself.
#   - Stop clears the stored command and sets the latch.
#
# The ROV has no arm/disarm or mode machinery, no battery, no position fix and
# no goto controller on the NEPI side, so states, modes, actions, gotos, home
# and manual motor control are empty or None and RBXRobotIF reports the
# matching capabilities False. The navpose carries depth and orientation from
# the Teensy telemetry in each status. Which physical direction each of their
# signs means has not been measured, and the bench Teensy has no sensors, so
# on the bench every value reads 0.

import copy
import json
import math
import socket
import threading
import time

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_nav
from nepi_sdk import nepi_utils

from std_msgs.msg import String

from nepi_interfaces.msg import AxisControls

from nepi_api.device_if_rbx import RBXRobotIF
from nepi_api.messages_if import MsgIF

PKG_NAME = 'RBX_MATE_ROV_LINK' # Use in display menus
FILE_TYPE = 'NODE'


#########################################
# Link Class
#########################################

class MateRovLink(object):
  """The NEPI side of the NEPI link: status in, demands out, the on/off rule.

  A background thread reads the status port. The node's timer calls
  send_demand() every DEMAND_PERIOD_SEC, and RBXRobotIF's callbacks call
  set_teleop_velocity(), go_stop() and is_link_up(). All of them touch the
  same state, so all of it sits behind one lock; methods named ...Locked
  expect the caller to hold it. clock is a monotonic seconds function,
  injectable so tests can control time; wall_clock fills the demand `t` field
  and seeds seq; log prints messages. Bad datagrams are counted and dropped,
  never raised, so a stray datagram on the port cannot stop the link.
  """

  DEMAND_PERIOD_SEC = 0.1 # 10 Hz, the Pi's serial write rate
  LINK_TIMEOUT_SEC = 0.5 # The link is down after this long without a valid status
  TELEOP_CMD_TIMEOUT_SEC = 0.25 # An older teleop command sends active false
  LATCH_ENGAGE_SEC = 0.5 # Assist still off this long after an engagement's first active true latches
  LATCH_CLEAR_SEC = 1.0 # The latch clears after this long with no teleop command
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
    self.cmd = None # [surge, sway, yaw] of the stored teleop command, protocol axes
    self.cmd_time = None # clock() of the newest teleop command; go_stop() keeps it
    self.latched = False
    self.engage_time = None # clock() of this engagement's first active true; None while sending active false
    self.engage_assist = False # A status during this engagement showed assist true
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
    """Report whether the link is up. This is the robot's teleop readiness.

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

  def set_teleop_velocity(self, linear_x, linear_y, linear_z, angular_z):
    """Store a teleop command, mapped from ROS axes to protocol axes.

    This is the only place the axes are mapped: surge = linear_x, sway =
    -linear_y (ROS y is positive left, protocol sway positive right), and yaw =
    -angular_z (ROS turns positive counter-clockwise, the protocol clockwise
    seen from above). linear_z is ignored, because the Teensy holds depth
    itself. Each value is clamped to -1 to 1. The Pi applies its own caps and
    per-vehicle signs, so none are applied here.

    Args:
        linear_x (float): Forward ratio, -1 to 1.
        linear_y (float): Left ratio, -1 to 1.
        linear_z (float): Up ratio. Ignored.
        angular_z (float): Counter-clockwise turn ratio, -1 to 1.

    Returns:
        list: [success (bool), msg (str)]. success is False, and nothing is
            stored, when a used value is not a finite number.
    """
    try:
      for value in [linear_x, linear_y, angular_z]:
        if not self.isNumber(value):
          msg = 'Teleop command dropped: values must be finite numbers'
          self.printLimited('teleop', msg)
          return False, msg
      cmd = [self.clampAxis(linear_x), self.clampAxis(-linear_y), self.clampAxis(-angular_z)]
      with self.lock:
        self.cmd = cmd
        self.cmd_time = self.clock()
      return True, 'Stored'
    except Exception as e:
      self.reportError('set_teleop_velocity', e)
      return False, 'Teleop error: ' + str(e)

  def go_stop(self):
    """Clear the stored teleop command and set the latch.

    The next demand is active false, so the Pi turns assist off (nepi_off). The
    latch then holds active false until no teleop command has arrived for
    LATCH_CLEAR_SEC.

    Returns:
        list: [success (bool), msg (str)].
    """
    try:
      with self.lock:
        self.cmd = None
        latched = self.latched
        self.latched = True
      if not latched:
        self.printMessage('Stop: latched to active false until teleop commands pause for ' +
                          str(self.LATCH_CLEAR_SEC) + ' s')
      return True, 'Stopped'
    except Exception as e:
      self.reportError('go_stop', e)
      return False, 'Stop error: ' + str(e)

  def send_demand(self):
    """Send the next demand, per the on/off rule. The node calls this every DEMAND_PERIOD_SEC.

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
            latched (bool) -- demands are held at active false;
            assist (bool), assist_reason (str), pilot_surge, pilot_sway,
                pilot_yaw, pilot_depth, slow (bool) -- from the newest status,
                False and '' before the first;
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
                      latched = self.latched, assist = False, assist_reason = '',
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
      return dict(running = False, link_up = False, latched = True)

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

  def ageNowLocked(self, age_ms, now):
    if age_ms < 0:
      return -1
    return age_ms + max(0, int(round((now - self.status_time) * 1000)))

  def nextDemandLocked(self, now):
    # The on/off rule, in the order it is decided. Returns [demand or None, notes to log].
    notes = []
    # Latch clear: no teleop command for LATCH_CLEAR_SEC
    if self.latched and (self.cmd_time is None or now - self.cmd_time >= self.LATCH_CLEAR_SEC):
      self.latched = False
      notes.append('Latch cleared: no teleop command for ' + str(self.LATCH_CLEAR_SEC) + ' s')
    # Link down: no demand at all
    link_up = self.linkUpLocked(now)
    if link_up != self.link_up:
      self.link_up = link_up
      if link_up:
        notes.append('Link up: status arriving from the ROV')
      else:
        notes.append('Link down: no valid status for ' + str(self.LINK_TIMEOUT_SEC) + ' s; demands stopped')
    if not link_up:
      self.engage_time = None
      return None, notes
    # Active true needs a fresh command and no latch
    active = (not self.latched and self.cmd is not None and
              now - self.cmd_time <= self.TELEOP_CMD_TIMEOUT_SEC)
    if active:
      if self.engage_time is None:
        self.engage_time = now
        self.engage_assist = False
      elif (self.status['assist'] is False and
            (self.engage_assist or now - self.engage_time >= self.LATCH_ENGAGE_SEC)):
        self.latched = True
        active = False
        notes.append('Latched to active false: assist off on the ROV (' + self.status['assist_reason'] +
                     '); pause teleop commands for ' + str(self.LATCH_CLEAR_SEC) + ' s to engage again')
    if not active:
      self.engage_time = None
    [surge, sway, yaw] = [0.0, 0.0, 0.0]
    if active:
      [surge, sway, yaw] = self.cmd
    demand = {'v': 1, 'type': 'demand', 'seq': self.seq, 't': round(self.wall_clock(), 3), 'active': active,
              'surge': surge, 'sway': sway, 'yaw': yaw, 'pitch': 0.0, 'roll': 0.0}
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
    if self.engage_time is not None and status['assist']:
      self.engage_assist = True
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

  # The ROV has no arm/disarm or mode machinery and no one-shot actions.
  # RBXRobotIF handles empty lists correctly (bounds checks reject any index,
  # status shows "Not Set"), the same as rbx_gazebo's empty states, modes and
  # go actions. The get*Ind functions must still be real callables, since
  # RBXRobotIF calls them unconditionally.
  RBX_STATES = []
  RBX_MODES = []
  RBX_SETUP_ACTIONS = []
  RBX_GO_ACTIONS = []

  NAVPOSE_UPDATE_RATE = 10
  TN_MAX_AGE_MS = 500 # The navpose skips Teensy telemetry older than this

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
    # Stop and navpose state
    self.stop_triggered = False
    self.navpose_dict = copy.deepcopy(nepi_nav.BLANK_NAVPOSE_DICT)

    # Surge, sway and yaw are commandable. Depth stays with the Teensy's own
    # depth hold, and version 1 of the link carries no pitch or roll.
    self.axis_controls = AxisControls()
    self.axis_controls.x = True
    self.axis_controls.y = True
    self.axis_controls.z = False
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
                             getSettingsFunction = None,
                             setSettingFunction = None,
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

    ## Start the demand tick
    nepi_sdk.start_timer_process(self.link.DEMAND_PERIOD_SEC, self.sendDemandCb)

    ## Initiation Complete
    self.msg_if.pub_info("Initialization Complete")
    nepi_sdk.on_shutdown(self.cleanup_actions)
    nepi_sdk.spin()


  ##########################
  # RBX Interface Functions

  def getStateInd(self):
    # No robot states (empty RBX_STATES). RBXRobotIF still calls this
    # unconditionally and displays "Not Set" for the empty list.
    return 0

  def setStateInd(self, state_ind):
    # Unreachable with empty RBX_STATES (RBXRobotIF bounds-checks first)
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
    # Link up, not assist on: RBXRobotIF drops teleop commands while this is
    # False, and assist only comes on after a command has gone out as active
    # true, so gating on assist would make engaging impossible.
    return self.link.is_link_up()

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
    # growing then.
    [tn, tn_age_ms] = self.link.get_tn()
    if tn is None or tn_age_ms < 0 or tn_age_ms > self.TN_MAX_AGE_MS:
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
    # One demand per tick, decided by the link's on/off rule, then the link
    # state for the hold process.
    self.link.send_demand()
    self.link_status_pub.publish(String(data = json.dumps(self.link.get_health())))

  #######################
  # Node Cleanup Function

  def cleanup_actions(self):
    """Hands the ROV back to the pilot on node shutdown.

    Sends one last active false demand if the link is up, so the Pi turns
    assist off at once (nepi_off) instead of after its 2 s timeout, then stops
    the link.
    """
    self.msg_if.pub_info("Shutting down: Executing script cleanup actions")
    if self.link is not None:
      self.link.go_stop()
      self.link.send_demand()
      self.link.stop()


#########################################
# Main
#########################################
if __name__ == '__main__':
  MateRovLinkNode()
