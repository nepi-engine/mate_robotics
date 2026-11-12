#!/usr/bin/env python3
##
## Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
##
## This file is part of nepi-engine
## (see https://github.com/nepi-engine).
##
## License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
##

# Stands in for NEPI on the bench, so a robot's nepi_connect can be tested
# without the NEPI device and rbx_nepi_connect.
#
# It speaks the NEPI side of Nepi_Connect/LINK_PROTOCOL.md, version 2: it
# receives the robot's vehicle and status messages on UDP 5700 and sends
# demands to the demand port the vehicle message announces. The robot accepts
# demands only from its vehicle file's network.nepi_ip, so when this runs on
# the bench laptop, that must be the laptop's address, 192.168.68.12
# (BENCH_SETUP_PLAN.md P9, "Bench install").
#
#   python3 fake_nepi.py --pi 192.168.68.22 watch
#       Prints the vehicle message once, and again whenever it changes. Then
#       one line per status: NEPI control, the reason it last changed, whether
#       the robot counts NEPI as ready, each axis input's merged value,
#       dropped demands, the navigation values and the raw values when present.
#   python3 fake_nepi.py --pi 192.168.68.22 hold --surge 0.05 --seconds 10
#       Waits for the vehicle message (as long as it takes), then sends active true with the given
#       surge, sway, heave and yaw at 10 Hz (only the axes the robot
#       announced), so the robot counts NEPI as ready, and prints each status.
#       The pilot's toggle input (RB on the MATE ROV) then hands the robot to
#       it (BENCH_SETUP_PLAN.md D3); the toggle again, an axis input past the
#       takeover point, or the robot's 2 s timeout hands it back, and the
#       toggle can hand it over again while this keeps running. On finish or
#       Ctrl+C it sends active false for 0.5 s, which ends NEPI control at
#       once (nepi_off). Ctrl+\ quits with no active false, to test the timeout.
#
# Standard library only, Python 3.8 or newer. Bench use only; it never goes
# on the robot or on NEPI.

import argparse
import json
import math
import socket
import sys
import threading
import time


DEMAND_PERIOD_SEC = 0.1 # 10 Hz, the robot's control loop rate
RELEASE_SEC = 0.5 # Time spent sending active false after a hold
VEHICLE_WAIT_SEC = 3.0 # hold says it is still waiting for the vehicle message this often
MAX_DATAGRAM_BYTES = 1200
PROTOCOL_VERSION = 2
AXIS_NAMES = ['surge', 'sway', 'heave', 'yaw']


def make_demand(seq, active, surge = 0.0, sway = 0.0, yaw = 0.0, heave = 0.0, axis_names = None, servos = None):
    """Build one demand message (LINK_PROTOCOL "Demand: NEPI to Robot").

    Args:
        seq (int): Sequence number, 0 to 4294967295.
        active (bool): NEPI is connected and ready to take the robot.
        surge (float): -1 to 1, positive forward.
        sway (float): -1 to 1, positive right.
        yaw (float): -1 to 1, positive clockwise seen from above.
        heave (float): -1 to 1, positive down.
        axis_names (list): The axes to send, as the vehicle message announced
            them (default: all four).
        servos (dict): Servo name to value, -1 to 1 (default: none).

    Returns:
        dict: The demand.
    """
    if axis_names is None:
        axis_names = AXIS_NAMES
    values = dict(surge = float(surge), sway = float(sway), heave = float(heave), yaw = float(yaw))
    return {'v': PROTOCOL_VERSION, 'type': 'demand', 'seq': int(seq), 't': round(time.time(), 3),
            'active': bool(active), 'axes': dict((name, values[name]) for name in AXIS_NAMES if name in axis_names),
            'servos': dict(servos or dict())}


class DemandSender(object):
    """Send demand datagrams to one robot's nepi_connect.

    seq starts at 0 and goes up by 1 per demand sent. axis_names limits each
    demand to the axes the robot announced (default: all four).
    """

    def __init__(self, pi_ip, port = 5701, axis_names = None):
        self.address = (str(pi_ip), int(port))
        self.axis_names = axis_names
        self.seq = 0
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send(self, active, surge = 0.0, sway = 0.0, yaw = 0.0, heave = 0.0):
        """Send one demand.

        Args:
            active (bool): As in make_demand().
            surge (float): As in make_demand().
            sway (float): As in make_demand().
            yaw (float): As in make_demand().
            heave (float): As in make_demand().

        Returns:
            list: [success (bool), msg (str)].
        """
        datagram = json.dumps(make_demand(self.seq, active, surge, sway, yaw, heave, self.axis_names),
                              separators = (',', ':')).encode('utf-8')
        self.seq = (self.seq + 1) & 0xFFFFFFFF
        try:
            self.sock.sendto(datagram, self.address)
        except OSError as e:
            return False, 'Cannot send to ' + self.address[0] + ':' + str(self.address[1]) + ': ' + str(e)
        return True, 'Sent'

    def close(self):
        """Close the socket."""
        self.sock.close()


class StatusReceiver(object):
    """Receive nepi_connect vehicle and status datagrams from one robot.

    A background thread reads the UDP port. Datagrams from any other address,
    or that are not a version 2 vehicle or status object, are counted and
    dropped. Callers collect new statuses with get_statuses() and the newest
    vehicle message with get_vehicle().
    """

    SOCKET_TIMEOUT_SEC = 0.2 # How often the receive thread checks for stop
    MAX_QUEUED = 100 # Statuses held for a slow caller; older ones are dropped

    def __init__(self, pi_ip, port = 5700, bind_ip = '0.0.0.0'):
        self.pi_ip = str(pi_ip)
        self.port = int(port)
        self.bind_ip = str(bind_ip)
        self.lock = threading.Lock()
        self.cond = threading.Condition(self.lock)
        self.queue = []
        self.vehicle = None # The newest vehicle message
        self.vehicle_count = 0
        self.status_count = 0
        self.bad_count = 0
        self.last_error = ''
        self.sock = None
        self.recv_thread = None
        self.stop_event = None

    def start(self):
        """Open the UDP port and start receiving in a background thread.

        Returns:
            list: [success (bool), msg (str)].
        """
        if self.recv_thread is not None:
            return True, 'Already receiving on UDP port ' + str(self.port)
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.bind((self.bind_ip, self.port))
        except OSError as e:
            sock.close()
            return False, 'Cannot receive on ' + self.bind_ip + ' UDP port ' + str(self.port) + ': ' + str(e)
        sock.settimeout(self.SOCKET_TIMEOUT_SEC)
        self.sock = sock
        self.stop_event = threading.Event()
        self.recv_thread = threading.Thread(target = self.runRecvThread, args = [sock, self.stop_event])
        self.recv_thread.daemon = True
        self.recv_thread.start()
        return True, 'Receiving vehicle and status messages from ' + self.pi_ip + ' on UDP port ' + str(self.port)

    def stop(self):
        """Stop receiving and close the UDP port.

        Returns:
            list: [success (bool), msg (str)].
        """
        if self.recv_thread is None:
            return True, 'Not receiving'
        self.stop_event.set()
        self.recv_thread.join(self.SOCKET_TIMEOUT_SEC * 2 + 1.0)
        self.recv_thread = None
        self.sock.close()
        self.sock = None
        return True, 'Stopped'

    def get_statuses(self, wait_sec = 0.1):
        """Return the statuses received since the last call.

        Args:
            wait_sec (float): Longest wait for one to arrive, in seconds.

        Returns:
            list: Status dicts, oldest first; empty if none arrived.
        """
        with self.cond:
            if len(self.queue) == 0 and wait_sec > 0:
                self.cond.wait(wait_sec)
            statuses = self.queue
            self.queue = []
            return statuses

    def get_vehicle(self, wait_sec = 0.0):
        """Return the newest vehicle message.

        Args:
            wait_sec (float): Longest wait for the first one, in seconds.

        Returns:
            dict: The vehicle message, or None when none has arrived.
        """
        end = time.monotonic() + wait_sec
        with self.cond:
            while self.vehicle is None:
                remaining = end - time.monotonic()
                if remaining <= 0:
                    break
                self.cond.wait(remaining)
            return self.vehicle

    def get_health(self):
        """Report counts for display.

        Returns:
            dict: status_count (int) -- statuses accepted; vehicle_count (int)
                -- vehicle messages accepted; bad_count (int) -- datagrams
                dropped; last_error (str) -- why the latest was dropped.
        """
        with self.lock:
            return dict(status_count = self.status_count, vehicle_count = self.vehicle_count,
                        bad_count = self.bad_count, last_error = self.last_error)


    def runRecvThread(self, sock, stop_event):
        while not stop_event.is_set():
            try:
                [datagram, sender] = sock.recvfrom(65536)
            except socket.timeout:
                continue
            except OSError as e:
                if stop_event.is_set():
                    break
                self.recordBad('Receive error: ' + str(e))
                stop_event.wait(self.SOCKET_TIMEOUT_SEC)
                continue
            self.handleDatagram(datagram, sender)

    def handleDatagram(self, datagram, sender):
        if sender[0] != self.pi_ip:
            self.recordBad('Datagram from ' + sender[0] + ' dropped: expecting status from ' + self.pi_ip)
            return
        if len(datagram) > MAX_DATAGRAM_BYTES:
            self.recordBad('Status of ' + str(len(datagram)) + ' bytes dropped: over ' + str(MAX_DATAGRAM_BYTES))
            return
        try:
            status = json.loads(datagram.decode('utf-8'))
        except Exception as e:
            self.recordBad('Status dropped: not UTF-8 JSON (' + str(e) + ')')
            return
        if (not isinstance(status, dict) or status.get('v') != PROTOCOL_VERSION or
                status.get('type') not in ['status', 'vehicle']):
            self.recordBad('Datagram dropped: not a version 2 vehicle or status message')
            return
        if status['type'] == 'vehicle':
            with self.cond:
                self.vehicle_count += 1
                self.vehicle = status
                self.cond.notify_all()
            return
        with self.cond:
            self.status_count += 1
            self.queue.append(status)
            if len(self.queue) > self.MAX_QUEUED:
                del self.queue[0]
            self.cond.notify_all()

    def recordBad(self, msg):
        with self.lock:
            self.bad_count += 1
            self.last_error = msg


#**********************
# Command line

def formatVehicle(vehicle):
    axes = vehicle.get('axes')
    axes_text = 'none'
    if isinstance(axes, dict) and len(axes) > 0:
        axes_text = ', '.join(str(name) + ' cap ' + str(cap) for (name, cap) in axes.items())
    cameras = vehicle.get('cameras')
    cameras_text = 'none'
    if isinstance(cameras, list) and len(cameras) > 0:
        cameras_text = ', '.join(str(camera.get('name')) + ' ' + str(camera.get('transport')) + ':' +
                                 str(camera.get('port')) for camera in cameras if isinstance(camera, dict))
    def names(key):
        value = vehicle.get(key)
        if isinstance(value, list) and len(value) > 0:
            return ', '.join(str(item) for item in value)
        return 'none'
    return ('Vehicle ' + str(vehicle.get('name')) + ' (' + str(vehicle.get('kind')) + '), demands to UDP ' +
            str(vehicle.get('demand_port')) + '\n  axes: ' + axes_text + '\n  servos: ' + names('servos') +
            '\n  nav: ' + names('nav') + '\n  raw: ' + names('raw') + '\n  cameras: ' + cameras_text +
            '\n  imu_mount: ' + str(vehicle.get('imu_mount')))

def formatStatus(status):
    line = ('control ' + ('ON ' if status.get('control') else 'off') +
            ' ' + str(status.get('control_reason', '?')).ljust(12) +
            ' ready ' + ('yes' if status.get('nepi_ready') else 'no '))
    out = status.get('out')
    if isinstance(out, dict):
        for (name, value) in out.items():
            line += ' ' + str(name) + ' ' + str(value).rjust(6)
    line += ' demand_bad ' + str(status.get('demand_bad'))
    nav = status.get('nav')
    if isinstance(nav, dict) and len(nav) > 0:
        line += ' | nav ' + ','.join(str(name) + '=' + str(value) for (name, value) in nav.items())
    raw = status.get('raw')
    if isinstance(raw, dict) and len(raw) > 0:
        line += ' | raw ' + ','.join(str(value) for value in raw.values())
    return line

def demandFraction(text):
    value = float(text)
    if not math.isfinite(value) or value < -1 or value > 1:
        raise argparse.ArgumentTypeError('must be a number from -1 to 1')
    return value

def positiveSeconds(text):
    value = float(text)
    if not math.isfinite(value) or value <= 0:
        raise argparse.ArgumentTypeError('must be more than 0')
    return value

def printVehicleChange(receiver, shown):
    # Prints the vehicle message when it is new or changed; returns the one shown
    vehicle = receiver.get_vehicle()
    if vehicle is None:
        return shown
    body = dict((key, value) for (key, value) in vehicle.items() if key not in ['seq', 't'])
    if body != shown:
        print(formatVehicle(vehicle))
    return body

def printStatuses(receiver, wait_sec, quiet_since):
    statuses = receiver.get_statuses(wait_sec)
    for status in statuses:
        print(formatStatus(status))
    now = time.monotonic()
    if len(statuses) > 0:
        return statuses, now
    if now - quiet_since >= 2.0:
        health = receiver.get_health()
        msg = 'No status from the robot for ' + str(int(now - quiet_since)) + ' s'
        if health['last_error'] != '':
            msg += ' (last dropped: ' + health['last_error'] + ')'
        print(msg)
        return statuses, now
    return statuses, quiet_since

def runWatch(receiver):
    quiet_since = time.monotonic()
    shown = None
    while True:
        shown = printVehicleChange(receiver, shown)
        [statuses, quiet_since] = printStatuses(receiver, 0.2, quiet_since)

def sendFor(sender, receiver, seconds, active, surge = 0.0, sway = 0.0, yaw = 0.0):
    end = time.monotonic() + seconds
    next_send = time.monotonic()
    while True:
        now = time.monotonic()
        if now >= end:
            return
        if now >= next_send:
            [success, msg] = sender.send(active, surge, sway, yaw)
            if not success:
                print(msg)
            next_send += DEMAND_PERIOD_SEC
        for status in receiver.get_statuses(max(0.0, min(next_send, end) - time.monotonic())):
            print(formatStatus(status))

def runHold(args, receiver, sender):
    print('Ready with surge ' + str(args.surge) + ', sway ' + str(args.sway) + ', heave ' + str(args.heave) +
          ', yaw ' + str(args.yaw) + ' for ' + str(args.seconds) +
          ' s. Press the toggle input on the controller (RB on the MATE ROV) to hand the robot to NEPI.')
    try:
        was_on = False
        end = time.monotonic() + args.seconds
        next_send = time.monotonic()
        while True:
            now = time.monotonic()
            if now >= end:
                break
            if now >= next_send:
                [success, msg] = sender.send(True, args.surge, args.sway, args.yaw, args.heave)
                if not success:
                    print(msg)
                next_send += DEMAND_PERIOD_SEC
            for status in receiver.get_statuses(max(0.0, min(next_send, end) - time.monotonic())):
                print(formatStatus(status))
                on = bool(status.get('control'))
                if on != was_on:
                    if on:
                        print('NEPI control on (' + str(status.get('control_reason')) + ')')
                    else:
                        print('NEPI control off (' + str(status.get('control_reason')) +
                              '); the toggle input hands it over again')
                    was_on = on
    except KeyboardInterrupt:
        print('Stopping')
    finally:
        print('Sending active false for ' + str(RELEASE_SEC) + ' s')
        try:
            sendFor(sender, receiver, RELEASE_SEC, False)
        except KeyboardInterrupt:
            pass

def main():
    parser = argparse.ArgumentParser(description = 'Stand in for NEPI: watch nepi_connect messages and send demands.')
    parser.add_argument('--pi', default = '192.168.68.22',
                        help = 'The robot: the only accepted sender, and where demands go (default 192.168.68.22)')
    parser.add_argument('--status-port', type = int, default = 5700,
                        help = 'UDP port to receive vehicle and status messages on (default 5700)')
    parser.add_argument('--demand-port', type = int, default = None,
                        help = "The robot's UDP port for demands (default: as its vehicle message announces)")
    commands = parser.add_subparsers(dest = 'command')
    commands.add_parser('watch', help = 'Print the vehicle message, then one line per status')
    hold = commands.add_parser('hold', help = 'Stay ready with a fixed demand; the toggle input hands over')
    hold.add_argument('--surge', type = demandFraction, default = 0.0, help = 'Surge, -1 to 1, positive forward (default 0)')
    hold.add_argument('--sway', type = demandFraction, default = 0.0, help = 'Sway, -1 to 1, positive right (default 0)')
    hold.add_argument('--heave', type = demandFraction, default = 0.0, help = 'Heave, -1 to 1, positive down (default 0)')
    hold.add_argument('--yaw', type = demandFraction, default = 0.0, help = 'Yaw, -1 to 1, positive clockwise (default 0)')
    hold.add_argument('--seconds', type = positiveSeconds, default = 10.0, help = 'How long to hold (default 10)')
    args = parser.parse_args()
    if args.command is None:
        parser.print_help()
        return 1

    receiver = StatusReceiver(args.pi, args.status_port)
    [success, msg] = receiver.start()
    print(msg)
    if not success:
        return 1
    sender = None
    try:
        if args.command == 'watch':
            print('Ctrl+C quits.')
            runWatch(receiver)
        else:
            # Waits as long as it takes, so this can start before the robot
            # does; with --demand-port it sends to all four axes at once
            vehicle = receiver.get_vehicle(VEHICLE_WAIT_SEC)
            while vehicle is None and args.demand_port is None:
                print('Waiting for a vehicle message from ' + args.pi + ' (Ctrl+C quits; --demand-port sends without one)')
                vehicle = receiver.get_vehicle(VEHICLE_WAIT_SEC)
            axis_names = None
            demand_port = args.demand_port
            if vehicle is not None:
                print(formatVehicle(vehicle))
                if isinstance(vehicle.get('axes'), dict):
                    axis_names = list(vehicle['axes'].keys())
                if demand_port is None:
                    demand_port = vehicle.get('demand_port')
            sender = DemandSender(args.pi, demand_port, axis_names)
            runHold(args, receiver, sender)
    except KeyboardInterrupt:
        pass
    finally:
        receiver.stop()
        if sender is not None:
            sender.close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
