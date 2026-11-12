##
## Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
##
## This file is part of nepi-engine
## (see https://github.com/nepi-engine).
##
## License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
##

# The robot's side of the NEPI link: the NEPI control state machine, the
# merge, and the two socket threads (Nepi_Connect/LINK_PROTOCOL.md).
#
# NEPI sends a demand (a value per announced axis, plus servos) to the
# vehicle file's demand port at 10 Hz. An adapter hands the pilot's inputs to
# merge_inputs() just before the robot's code uses them, and this file
# merges the demand into a COPY of them. It sends NEPI a status every 100 ms
# and a vehicle message every 0.5 s. Every demand is capped.
#
# NEPI control (LINK_PROTOCOL.md "Robot Rules", decided 2026-10-22): the pilot drives,
# and pressing the toggle input hands the robot to NEPI. In NEPI control, NEPI
# owns every axis input: the robot follows NEPI's demand, and a demand older
# than 250 ms means neutral. Inputs that belong to no axis pass through, so
# the pilot can still work tools, except the servo inputs: in NEPI control
# those carry the demand's servo values, and the pilot's own input wins
# whenever the pilot holds it, for that input only. NEPI control ends when
# the pilot presses the toggle again, when the pilot moves any axis input to
# or past its takeover point (the safety exit), when NEPI sends active false,
# or after 2 s with no accepted demand. NEPI never turns it on: `active` true
# only means NEPI is connected and ready, and the toggle is ignored while it
# is not.
#
# Nothing here may stop the robot's code. Every public method catches every
# exception and falls back to passing the pilot's inputs through untouched.
# Errors print at most once per 10 s. Standard library plus PyYAML, Python
# 3.8 or newer.

import socket
import threading
import time

from . import link
from . import mapping
from . import telemetry as telemetry_mod
from . import vehicle as vehicle_mod


STATUS_PERIOD_SEC = 0.1
VEHICLE_PERIOD_SEC = 0.5 # A vehicle message goes out once at start and then this often
DEMAND_STALE_SEC = 0.25 # An older demand counts as 0 on every axis
NEPI_READY_SEC = 0.5 # NEPI is ready while an active true demand was accepted within this
CONTROL_TIMEOUT_SEC = 2.0 # NEPI control ends after this long with no accepted demand
ERROR_PRINT_SEC = 10.0 # Each kind of message prints at most once per this period
SOCKET_TIMEOUT_SEC = 0.2 # How often the receive thread checks for stop
SOCKET_RETRY_SEC = 1.0 # Wait before retrying a socket that failed


def safeTuple(inputs):
    try:
        return tuple(inputs)
    except Exception:
        return inputs

def inputHeld(value, robot_input):
    # The one rule for every input: the pilot holds it at or past its dead zone
    return abs(value - robot_input['neutral']) >= robot_input['deadzone']

def pastTakeover(value, axis_input):
    offset = value - axis_input['neutral']
    if offset > 0:
        return axis_input['takeover_high'] is not None and offset >= axis_input['takeover_high']
    if offset < 0:
        return axis_input['takeover_low'] is not None and -offset >= axis_input['takeover_low']
    return False


class NepiControl(object):
    """The robot side of the NEPI link: toggle, merge, status and demands.

    The robot's control loop (through an adapter), the demand receiver thread
    and the status sender thread all touch the same state, so all of it sits
    behind one lock. Methods named ...Locked expect the caller to hold it;
    messages they want printed go in pending_msgs, printed once the lock is
    released. clock is a monotonic seconds function, injectable so tests can
    control time; wall_clock fills the `t` field; log prints messages, each
    starting with name.
    """

    def __init__(self, clock = time.monotonic, wall_clock = time.time, log = print, name = 'nepi_connect'):
        self.lock = threading.Lock()
        self.clock = clock
        self.wall_clock = wall_clock
        self.log = log
        self.name = name
        self.vehicle = None # None means the link is off
        self.vehicle_path = None
        self.stop_event = None
        self.threads = []
        self.control_event = threading.Event() # Wakes the status sender early
        self.last_print = dict()
        self.suppressed = dict()
        self.resetStateLocked()

    def resetStateLocked(self):
        self.control = False # NEPI control
        self.control_reason = 'start'
        self.toggle_btn_avl = True
        self.pending_msgs = [] # [key, msg] pairs for printLimited, once the lock is released
        self.pilot = dict() # Axis name to whether the pilot holds one of its inputs
        self.servo_pilot = dict() # Servo name to whether the pilot holds its input
        self.out = dict() # Axis input name to the value last handed on
        if self.vehicle is not None:
            for axis in self.vehicle['control']['axes']:
                self.pilot[axis['name']] = False
                for axis_input in axis['inputs']:
                    self.out[axis_input['name']] = axis_input['neutral']
            for servo in self.vehicle['control']['servos']:
                self.servo_pilot[servo['name']] = False
        self.demand = None
        self.demand_seq = -1
        self.demand_time = None
        self.demand_bad = 0
        self.telemetry_values = None # The newest parsed telemetry line, field name to value
        self.nav_time = None
        self.nav_count = 0
        self.nav_bad = 0
        self.status_seq = 0
        self.vehicle_seq = 0


    def configure(self, raw, vehicle_path = None):
        """Validate a raw vehicle file and reset all link state. Opens no sockets.

        Args:
            raw (dict): The vehicle file's contents, or None for link off.
            vehicle_path (str): Where raw came from, for messages.

        Returns:
            list: [on (bool), msg (str)]. on is False when the link is off.
        """
        try:
            [vehicle, msg] = vehicle_mod.validate_vehicle(raw)
            if vehicle is not None and vehicle['network']['nepi_ip'] == '':
                vehicle = None
            with self.lock:
                self.vehicle = vehicle
                self.vehicle_path = vehicle_path
                self.resetStateLocked()
            if vehicle_path is not None:
                msg += ' (' + vehicle_path + ')'
            return vehicle is not None, msg
        except Exception as e:
            self.reportError('configure', e)
            return False, 'Vehicle file error: ' + str(e)

    def start(self, raw = None, vehicle_path = None, module_dir = None):
        """Read the vehicle file and, if the link is on, start the socket threads.

        With no raw vehicle given, the vehicle file is found as
        VEHICLE_FILE.md "Where the Robot Looks" says. Prints one line saying
        whether the link is on.

        Args:
            raw (dict): Raw vehicle to use instead of a file (tests).
            vehicle_path (str): Name of that vehicle, for messages.
            module_dir (str): The folder that holds the hooks file, where the
                vehicle file may sit (default: beside this package).

        Returns:
            list: [on (bool), msg (str)].
        """
        try:
            return self.startLink(raw, vehicle_path, module_dir)
        except Exception as e:
            self.reportError('start', e)
            return False, 'nepi_connect did not start: ' + str(e)

    def startLink(self, raw, vehicle_path, module_dir):
        with self.lock:
            if self.stop_event is not None:
                return self.vehicle is not None, 'Already started'
        load_msg = ''
        if raw is None:
            [raw, vehicle_path, load_msg] = vehicle_mod.load_vehicle(module_dir)
        [on, msg] = self.configure(raw, vehicle_path)
        if raw is None:
            msg = load_msg
        self.printMessage(self.name + ': ' + msg)
        if not on:
            return False, msg
        with self.lock:
            demand_port = self.vehicle['network']['demand_port']
            status_address = (self.vehicle['network']['nepi_ip'], self.vehicle['network']['status_port'])
            stop_event = threading.Event()
            self.stop_event = stop_event
            self.control_event.clear()
            self.threads = [threading.Thread(target = self.runDemandThread, args = [stop_event, demand_port],
                                             name = 'nepi_connect_demand'),
                            threading.Thread(target = self.runStatusThread, args = [stop_event, status_address],
                                             name = 'nepi_connect_status')]
            threads = list(self.threads)
        for thread in threads:
            thread.daemon = True
            thread.start()
        return True, msg

    def stop(self):
        """Stop the socket threads and close their sockets. For tests.

        Returns:
            list: [success (bool), msg (str)].
        """
        try:
            with self.lock:
                stop_event = self.stop_event
                threads = self.threads
                self.stop_event = None
                self.threads = []
            if stop_event is None:
                return True, 'Not started'
            stop_event.set()
            self.control_event.set()
            for thread in threads:
                thread.join(SOCKET_TIMEOUT_SEC * 2 + 1.0)
            return True, 'Stopped'
        except Exception as e:
            self.reportError('stop', e)
            return False, 'Stop error: ' + str(e)

    def merge_inputs(self, pilot_inputs):
        """Merge NEPI's newest demand into a copy of the pilot's inputs.

        Args:
            pilot_inputs (list): The pilot's inputs, indexed by slot. Never modified.

        Returns:
            tuple: The inputs for the robot. tuple(pilot_inputs) unchanged
                while the link or NEPI control is off, and on any error. In
                NEPI control, the axis inputs carry NEPI's demand, or neutral
                while the demand is older than 250 ms.
        """
        try:
            with self.lock:
                merged = self.mergeLocked(pilot_inputs, self.clock())
                pending = self.pending_msgs
                self.pending_msgs = []
            for [key, msg] in pending:
                self.printLimited(key, msg)
            return merged
        except Exception as e:
            self.reportError('merge_inputs', e)
            return safeTuple(pilot_inputs)

    def mergeLocked(self, pilot_inputs, now):
        pilot = tuple(pilot_inputs)
        vehicle = self.vehicle
        if vehicle is None:
            return pilot
        control = vehicle['control']
        self.updatePilotLocked(pilot, now)
        self.checkTimeoutLocked(now)
        merged = pilot
        if self.control:
            # NEPI owns every axis input. The pilot's own are all short of the
            # takeover point here (past it, updatePilotLocked ended NEPI
            # control), so they are replaced, not mixed in. A stale demand
            # leaves every axis input at neutral.
            out = list(pilot)
            for axis in control['axes']:
                for axis_input in axis['inputs']:
                    out[axis_input['slot']] = axis_input['neutral']
            fresh = self.demandFreshLocked(now)
            # A servo input the pilot holds stays the pilot's. Otherwise it
            # carries NEPI's value, or neutral while the demand is stale or
            # has no value for it.
            for servo in control['servos']:
                if self.servo_pilot[servo['name']]:
                    continue
                value = None
                if fresh:
                    value = self.demand['servos'].get(servo['name'])
                if value is None:
                    out[servo['slot']] = servo['neutral']
                else:
                    out[servo['slot']] = mapping.servo_value(value, servo['min'], servo['neutral'], servo['max'])
            if fresh:
                for axis in control['axes']:
                    mapping.apply_axis(axis, self.demand['axes'][axis['name']], out)
            merged = tuple(out)
        for axis in control['axes']:
            for axis_input in axis['inputs']:
                self.out[axis_input['name']] = merged[axis_input['slot']]
        return merged

    def updatePilotLocked(self, pilot, now):
        control = self.vehicle['control']
        for axis in control['axes']:
            self.pilot[axis['name']] = any(inputHeld(pilot[axis_input['slot']], axis_input)
                                           for axis_input in axis['inputs'])
        for servo in control['servos']:
            self.servo_pilot[servo['name']] = inputHeld(pilot[servo['slot']], servo)
        # The safety exit: any axis input past its takeover point ends NEPI control
        takeover = self.takeoverInputLocked(pilot)
        if self.control and takeover is not None:
            self.setControlLocked(False, 'pilot_stick')
            self.pending_msgs.append(['takeover', self.name + ': NEPI control off: the pilot moved ' +
                                      takeover['label']])
        # The toggle input, with the "button available" rule: one press, one toggle
        if inputHeld(pilot[control['toggle']['slot']], control['toggle']):
            if self.toggle_btn_avl:
                self.toggleControlLocked(takeover, now)
            self.toggle_btn_avl = False
        else:
            self.toggle_btn_avl = True

    def takeoverInputLocked(self, pilot):
        # The first axis input at or past its takeover point, in axis order, or None
        for axis in self.vehicle['control']['axes']:
            for axis_input in axis['inputs']:
                if pastTakeover(pilot[axis_input['slot']], axis_input):
                    return axis_input
        return None

    def toggleControlLocked(self, takeover, now):
        label = self.vehicle['control']['toggle']['label']
        if self.control:
            self.setControlLocked(False, 'pilot_button')
        elif not self.nepiReadyLocked(now):
            self.pending_msgs.append(['button', self.name + ': ' + label + ' ignored: NEPI is not connected'])
        elif takeover is not None:
            self.pending_msgs.append(['button', self.name + ': ' + label + ' ignored: release ' + takeover['label'] +
                                      ' first'])
        else:
            self.setControlLocked(True, 'pilot_button')

    def setControlLocked(self, on, reason):
        if self.control == on:
            return
        self.control = on
        self.control_reason = reason
        self.control_event.set()

    def checkTimeoutLocked(self, now):
        if self.control and (self.demand_time is None or now - self.demand_time >= CONTROL_TIMEOUT_SEC):
            self.setControlLocked(False, 'timeout')

    def demandFreshLocked(self, now):
        return self.demand_time is not None and now - self.demand_time <= DEMAND_STALE_SEC

    def nepiReadyLocked(self, now):
        return (self.demand is not None and self.demand['active'] and
                now - self.demand_time <= NEPI_READY_SEC)

    def handle_demand(self, datagram, source_ip):
        """Validate one demand datagram and apply it. Active false ends NEPI control.

        Args:
            datagram (bytes): The UDP payload.
            source_ip (str): The sender's address.

        Returns:
            list: [accepted (bool), msg (str)]. A dropped datagram adds 1 to
                demand_bad and changes nothing else.
        """
        try:
            with self.lock:
                return self.handleDemandLocked(datagram, source_ip, self.clock())
        except Exception as e:
            self.reportError('handle_demand', e)
            return False, 'Demand error: ' + str(e)

    def handleDemandLocked(self, datagram, source_ip, now):
        if self.vehicle is None:
            return False, 'Link off'
        axis_names = [axis['name'] for axis in self.vehicle['control']['axes']]
        [demand, msg] = link.parse_demand(datagram, source_ip, self.vehicle['network']['nepi_ip'], axis_names)
        if demand is None:
            self.demand_bad += 1
            return False, msg
        if link.seq_out_of_order(demand['seq'], self.demand_seq, self.demand_time, now):
            self.demand_bad += 1
            return False, ('Demand dropped: seq ' + str(demand['seq']) + ' is not after ' +
                           str(self.demand_seq))
        self.demand = demand
        self.demand_seq = demand['seq']
        self.demand_time = now
        # Active true only marks NEPI ready; only the pilot's toggle turns NEPI control on
        if not demand['active']:
            self.setControlLocked(False, 'nepi_off')
        return True, msg

    def on_telemetry_line(self, line):
        """Record one telemetry line from the robot. Never changes the line.

        Args:
            line (str): The decoded line. Anything that does not match the
                vehicle file's line format adds 1 to nav_bad. Does nothing for
                a vehicle with no telemetry section.
        """
        try:
            with self.lock:
                if self.vehicle is None or self.vehicle['telemetry'] is None:
                    return
                telemetry = self.vehicle['telemetry']
            values = telemetry_mod.parse_line(line, telemetry)
            with self.lock:
                if values is None:
                    self.nav_bad += 1
                else:
                    self.telemetry_values = values
                    self.nav_time = self.clock()
                    self.nav_count += 1
        except Exception as e:
            self.reportError('on_telemetry_line', e)

    def status_tag(self):
        """Return the NEPI control state, for the robot to show the pilot.

        Returns:
            str: 'ON', 'WAIT' (NEPI control on, demand older than 250 ms),
                'OFF' (off, NEPI ready, so the toggle hands over) or
                'NOTREADY' (off, NEPI not connected, so the toggle does
                nothing). An empty string when the link is off or on any error.
        """
        try:
            with self.lock:
                if self.vehicle is None:
                    return ''
                now = self.clock()
                self.checkTimeoutLocked(now)
                if not self.control:
                    if self.nepiReadyLocked(now):
                        return 'OFF'
                    return 'NOTREADY'
                if self.demandFreshLocked(now):
                    return 'ON'
                return 'WAIT'
        except Exception as e:
            self.reportError('status_tag', e)
            return ''

    def build_status(self):
        """Build the status that would be sent now, without advancing seq.

        Returns:
            dict: Every LINK_PROTOCOL "Status" field, or None when the link is
                off or on any error.
        """
        try:
            with self.lock:
                if self.vehicle is None:
                    return None
                return self.buildStatusLocked(self.clock())
        except Exception as e:
            self.reportError('build_status', e)
            return None

    def buildStatusLocked(self, now):
        self.checkTimeoutLocked(now)
        control = self.vehicle['control']
        telemetry = self.vehicle['telemetry']
        nav = None
        raw = None
        has_raw = telemetry is not None and len(telemetry['raw']) > 0
        if telemetry is not None and self.telemetry_values is not None:
            nav = telemetry_mod.nav_values(self.telemetry_values, telemetry)
            raw = telemetry_mod.raw_values(self.telemetry_values, telemetry)
        # The input ranges are part of the contract, so a pilot value past
        # them (a controller or vehicle file mismatch) is reported at the limit.
        out = dict()
        for axis in control['axes']:
            for axis_input in axis['inputs']:
                out[axis_input['name']] = link.clampInt(self.out.get(axis_input['name'], axis_input['neutral']),
                                                        axis_input['min'], axis_input['max'])
        return link.build_status(seq = self.status_seq, t = round(self.wall_clock(), 3), control = self.control,
                                 control_reason = self.control_reason, nepi_ready = self.nepiReadyLocked(now),
                                 pilot = dict((axis['name'], self.pilot.get(axis['name'], False))
                                              for axis in control['axes']),
                                 servos = dict((servo['name'], self.servo_pilot.get(servo['name'], False))
                                               for servo in control['servos']),
                                 out = out, demand_seq = self.demand_seq, demand_age_ms = link.age_ms(now, self.demand_time),
                                 demand_bad = link.clampCount(self.demand_bad), nav = nav,
                                 nav_age_ms = link.age_ms(now, self.nav_time), nav_count = link.clampCount(self.nav_count),
                                 nav_bad = link.clampCount(self.nav_bad), raw = raw, has_raw = has_raw)

    def build_vehicle_message(self):
        """Build the vehicle message that would be sent now, without advancing seq.

        Returns:
            dict: The LINK_PROTOCOL "Vehicle" message, or None when the link is
                off or on any error.
        """
        try:
            with self.lock:
                if self.vehicle is None:
                    return None
                return vehicle_mod.build_vehicle_message(self.vehicle, self.vehicle_seq, round(self.wall_clock(), 3))
        except Exception as e:
            self.reportError('build_vehicle_message', e)
            return None

    def nextStatusDatagram(self):
        with self.lock:
            if self.vehicle is None:
                return None
            status = self.buildStatusLocked(self.clock())
            self.status_seq = link.next_seq(self.status_seq)
        return self.checkedDatagram(status)

    def nextVehicleDatagram(self):
        with self.lock:
            if self.vehicle is None:
                return None
            message = vehicle_mod.build_vehicle_message(self.vehicle, self.vehicle_seq, round(self.wall_clock(), 3))
            self.vehicle_seq = link.next_seq(self.vehicle_seq)
        return self.checkedDatagram(message)

    def checkedDatagram(self, message):
        datagram = link.encode_message(message)
        if len(datagram) > link.MAX_DATAGRAM_BYTES:
            raise ValueError(message['type'] + ' of ' + str(len(datagram)) + ' bytes is over ' +
                             str(link.MAX_DATAGRAM_BYTES))
        return datagram


    def runDemandThread(self, stop_event, port):
        sock = None
        while not stop_event.is_set():
            try:
                if sock is None:
                    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                    sock.bind(('', port))
                    sock.settimeout(SOCKET_TIMEOUT_SEC)
                try:
                    [datagram, sender] = sock.recvfrom(65536)
                except socket.timeout:
                    continue
                [accepted, msg] = self.handle_demand(datagram, sender[0])
                if not accepted:
                    self.printLimited('demand', self.name + ': ' + msg)
            except Exception as e:
                if sock is not None:
                    sock.close()
                    sock = None
                if stop_event.is_set():
                    break
                self.reportError('demand receiver on UDP ' + str(port), e)
                stop_event.wait(SOCKET_RETRY_SEC)
        if sock is not None:
            sock.close()

    def runStatusThread(self, stop_event, address):
        # A status every STATUS_PERIOD_SEC, or at once on a control change, and
        # a vehicle message at start and every VEHICLE_PERIOD_SEC
        sock = None
        next_send = time.monotonic()
        next_vehicle = next_send
        while not stop_event.is_set():
            wait_sec = next_send - time.monotonic()
            if wait_sec > 0:
                self.control_event.wait(wait_sec)
            if stop_event.is_set():
                break
            self.control_event.clear()
            now = time.monotonic()
            next_send = now + STATUS_PERIOD_SEC
            try:
                if sock is None:
                    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                if now >= next_vehicle:
                    next_vehicle = now + VEHICLE_PERIOD_SEC
                    datagram = self.nextVehicleDatagram()
                    if datagram is not None:
                        sock.sendto(datagram, address)
                datagram = self.nextStatusDatagram()
                if datagram is not None:
                    sock.sendto(datagram, address)
            except Exception as e:
                self.reportError('status sender to ' + address[0] + ':' + str(address[1]), e)
        if sock is not None:
            sock.close()


    def reportError(self, where, error):
        self.printLimited('error', self.name + ': ' + where + ' error: ' + str(error))

    def printLimited(self, key, msg):
        # Never call this while holding the lock
        try:
            with self.lock:
                now = self.clock()
                last = self.last_print.get(key)
                if last is not None and now - last < ERROR_PRINT_SEC:
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
