"""Copy the active quick-start camera UDP stream to NEPI without reopening V4L2.

Run on the onboard Pi with CAP_NET_RAW (normally ``sudo -n``). The raw socket
sees packets already leaving for the topside viewer; only camera 0 packets are
copied. No packet is removed from or injected into the existing ROV stream.
"""

import argparse
import os
import signal
import socket
import struct


ETH_P_ALL = 0x0003
ETH_P_IP = 0x0800
PACKET_OUTGOING = 4
CAMERA_IP = socket.inet_aton("192.168.68.22")
TOPSIDE_IP = socket.inet_aton("192.168.68.12")


def camera_payload(packet, packet_type):
    if packet_type != PACKET_OUTGOING or len(packet) < 14 + 28:
        return None
    if packet[12:14] != b"\x08\x00":
        return None
    packet = packet[14:]
    if packet[0] >> 4 != 4 or packet[9] != socket.IPPROTO_UDP:
        return None
    header_length = (packet[0] & 15) * 4
    if header_length < 20 or len(packet) < header_length + 8:
        return None
    if packet[12:16] != CAMERA_IP or packet[16:20] != TOPSIDE_IP:
        return None
    if struct.unpack_from("!H", packet, 6)[0] & 0x3FFF:
        return None  # fragmented IP packet
    source_port, dest_port, udp_length = struct.unpack_from("!HHH", packet, header_length)
    if source_port != 5600 or dest_port != 5600 or udp_length < 8:
        return None
    end = header_length + udp_length
    if end > len(packet):
        return None
    return packet[header_length + 8:end]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pid-file", required=True)
    parser.add_argument("--rui-port", type=int, default=5604,
                        help="second NEPI UDP port for the RUI camera driver; 0 disables this copy")
    args = parser.parse_args()

    capture = socket.socket(socket.AF_PACKET, socket.SOCK_RAW, socket.htons(ETH_P_ALL))
    capture.bind(("eth0", 0))
    outgoing = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    outgoing.bind(("192.168.68.22", 0))
    running = True

    def stop(_signum, _frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    capture.settimeout(0.5)
    forwarded = 0
    try:
        with open(args.pid_file, "w", encoding="ascii") as pid_file:
            pid_file.write(f"{os.getpid()}\n")
        print("Reusing quick-start camera 0; forwarding UDP to NEPI", flush=True)
        while running:
            try:
                packet, address = capture.recvfrom(65535)
            except socket.timeout:
                continue
            payload = camera_payload(packet, address[2])
            if payload:
                outgoing.sendto(payload, ("192.168.68.103", 5600))
                if args.rui_port:
                    outgoing.sendto(payload, ("192.168.68.103", args.rui_port))
                forwarded += 1
    finally:
        print(f"Forwarded {forwarded} camera UDP packets", flush=True)
        capture.close()
        outgoing.close()
        try:
            os.unlink(args.pid_file)
        except FileNotFoundError:
            pass


if __name__ == "__main__":
    main()
