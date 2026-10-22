#!/usr/bin/env python3
##
## Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
##
## This file is part of nepi-engine
## (see https://github.com/nepi-engine).
##
## License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
##

# Stands in for NEPI on the bench, so the control Pi's nepi_link.py can be
# tested before rbx_rov_link and the hold process exist.
#
# It speaks the NEPI side of Nepi_Connect/LINK_PROTOCOL.md: it receives the
# Pi's status on UDP 5700 and sends demands to the Pi's UDP 5701. The Pi
# accepts demands only from its NEPI_LAN_IP, so when this runs on the bench
# laptop, the Pi's nepi_link.yaml must say NEPI_LAN_IP: 192.168.68.12
# (BENCH_SETUP_PLAN.md P9, "Bench install").
#
#   python3 fake_nepi.py --pi 192.168.68.22 watch
#       One line per status: assist, the reason it last changed, the merged
#       LY, LX, LT and RT the Pi handed the Teensy, dropped demands, and the
#       Teensy's eight thruster values when present.
#   python3 fake_nepi.py --pi 192.168.68.22 hold --surge 0.05 --seconds 10
#       Sends active false for 0.5 s, so the Pi sees the rising edge that turns
#       assist on (BENCH_SETUP_PLAN.md D3 option c), then active true with the
#       given surge, sway and yaw at 10 Hz, printing each status. If the Pi
#       turns assist off (RB, or its 2 s timeout), it prints the reason and
#       sends active false from then on. On finish or Ctrl+C it sends active
#       false for 0.5 s.
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


DEMAND_PERIOD_SEC = 0.1 # 10 Hz, the Pi's serial write rate
EDGE_SEC = 0.5 # Time spent sending active false before and after a hold
MAX_DATAGRAM_BYTES = 1200


def make_demand(seq, active, surge = 0.0, sway = 0.0, yaw = 0.0):
    """Build one demand message (LINK_PROTOCOL "Demand: NEPI to Pi").

    Args:
        seq (int): Sequence number, 0 to 4294967295.
        active (bool): NEPI's hold wants these values used.
        surge (float): -1 to 1, positive forward.
        sway (float): -1 to 1, positive right.
        yaw (float): -1 to 1, positive clockwise seen from above.

    Returns:
        dict: The demand, with pitch and roll 0 as version 1 requires.
    """
    return {'v': 1, 'type': 'demand', 'seq': int(seq), 't': round(time.time(), 3), 'active': bool(active),
            'surge': float(surge), 'sway': float(sway), 'yaw': float(yaw), 'pitch': 0.0, 'roll': 0.0}


class DemandSender(object):
    """Send demand datagrams to one Pi's nepi_link.

    seq starts at 0 and goes up by 1 per demand sent.
    """

    def __init__(self, pi_ip, port = 5701):
        self.address = (str(pi_ip), int(port))
        self.seq = 0
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send(self, active, surge = 0.0, sway = 0.0, yaw = 0.0):
        """Send one demand.

        Args:
            active (bool): As in make_demand().
            surge (float): As in make_demand().
            sway (float): As in make_demand().
            yaw (float): As in make_demand().

        Returns:
            list: [success (bool), msg (str)].
        """
        datagram = json.dumps(make_demand(self.seq, active, surge, sway, yaw), separators = (',', ':')).encode('utf-8')
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
    """Receive nepi_link status datagrams from one Pi.

    A background thread reads the UDP port. Datagrams from any other address,
    or that are not a version 1 status object, are counted and dropped.
    Callers collect new statuses with get_statuses().
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
        return True, 'Receiving status from ' + self.pi_ip + ' on UDP port ' + str(self.port)

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

    def get_health(self):
        """Report counts for display.

        Returns:
            dict: status_count (int) -- statuses accepted; bad_count (int) --
                datagrams dropped; last_error (str) -- why the latest was dropped.
        """
        with self.lock:
            return dict(status_count = self.status_count, bad_count = self.bad_count, last_error = self.last_error)


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
        if not isinstance(status, dict) or status.get('v') != 1 or status.get('type') != 'status':
            self.recordBad('Datagram dropped: not a version 1 status')
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

def formatStatus(status):
    line = ('assist ' + ('ON ' if status.get('assist') else 'off') +
            ' ' + str(status.get('assist_reason', '?')).ljust(12) +
            ' out_ly ' + str(status.get('out_ly')).rjust(6) +
            ' out_lx ' + str(status.get('out_lx')).rjust(6) +
            ' out_lt ' + str(status.get('out_lt')).rjust(4) +
            ' out_rt ' + str(status.get('out_rt')).rjust(4) +
            ' demand_bad ' + str(status.get('demand_bad')))
    tn = status.get('tn')
    if isinstance(tn, dict):
        names = ['vfl_pwr', 'vfr_pwr', 'vbl_pwr', 'vbr_pwr', 'hfl_pwr', 'hfr_pwr', 'hbl_pwr', 'hbr_pwr']
        line += ' | tn ' + ','.join(str(tn.get(name)) for name in names)
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

def printStatuses(receiver, wait_sec, quiet_since):
    statuses = receiver.get_statuses(wait_sec)
    for status in statuses:
        print(formatStatus(status))
    now = time.monotonic()
    if len(statuses) > 0:
        return statuses, now
    if now - quiet_since >= 2.0:
        health = receiver.get_health()
        msg = 'No status from the Pi for ' + str(int(now - quiet_since)) + ' s'
        if health['last_error'] != '':
            msg += ' (last dropped: ' + health['last_error'] + ')'
        print(msg)
        return statuses, now
    return statuses, quiet_since

def runWatch(receiver):
    quiet_since = time.monotonic()
    while True:
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
    print('Sending active false for ' + str(EDGE_SEC) + ' s, then holding surge ' + str(args.surge) +
          ', sway ' + str(args.sway) + ', yaw ' + str(args.yaw) + ' for ' + str(args.seconds) + ' s')
    try:
        sendFor(sender, receiver, EDGE_SEC, False)
        active = True
        was_on = False
        end = time.monotonic() + args.seconds
        next_send = time.monotonic()
        while True:
            now = time.monotonic()
            if now >= end:
                break
            if now >= next_send:
                [success, msg] = sender.send(active, args.surge, args.sway, args.yaw)
                if not success:
                    print(msg)
                next_send += DEMAND_PERIOD_SEC
            for status in receiver.get_statuses(max(0.0, min(next_send, end) - time.monotonic())):
                print(formatStatus(status))
                if status.get('assist'):
                    was_on = True
                elif was_on and active:
                    print('Assist turned off on the Pi (' + str(status.get('assist_reason')) +
                          '); sending active false from now on')
                    active = False
    except KeyboardInterrupt:
        print('Stopping')
    finally:
        print('Sending active false for ' + str(EDGE_SEC) + ' s')
        try:
            sendFor(sender, receiver, EDGE_SEC, False)
        except KeyboardInterrupt:
            pass

def main():
    parser = argparse.ArgumentParser(description = 'Stand in for NEPI: watch nepi_link status and send demands.')
    parser.add_argument('--pi', default = '192.168.68.22',
                        help = 'The control Pi: the only accepted status sender, and where demands go (default 192.168.68.22)')
    parser.add_argument('--status-port', type = int, default = 5700,
                        help = 'UDP port to receive status on (default 5700)')
    parser.add_argument('--demand-port', type = int, default = 5701,
                        help = "The Pi's UDP port for demands (default 5701)")
    commands = parser.add_subparsers(dest = 'command')
    commands.add_parser('watch', help = 'Print one line per status')
    hold = commands.add_parser('hold', help = 'Turn assist on and send a fixed demand')
    hold.add_argument('--surge', type = demandFraction, default = 0.0, help = 'Surge, -1 to 1, positive forward (default 0)')
    hold.add_argument('--sway', type = demandFraction, default = 0.0, help = 'Sway, -1 to 1, positive right (default 0)')
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
            sender = DemandSender(args.pi, args.demand_port)
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
