# Bench Setup Plan: Coral Crusaders ROV Replica, Then NEPI Stabilization

## Goal

Build a working copy of the Coral Crusaders' ROV control system on the bench, using the team's own code. A game controller on the topside laptop drives eight thrusters through the control Raspberry Pi and the Teensy, and the pilot sees camera video, exactly as on the team's robot. Then add NEPI: first watching only, then one control that starts stabilization mode on the NEPI Raspberry Pi 5. In stabilization mode NEPI holds position and orientation. Its commands reach the Teensy only through the control Pi; NEPI never talks to the Teensy directly. The thrusters and ESCs are here now and the Teensy 4.1 arrives on 2026-10-20, so everything that does not need the Teensy is done on 2026-10-19.

## How to Read This Plan

- **Citations.** `file:line` points to the team's repo, github.com/CoralCrusaders/rov-26, at commit `1af4984` (the newest public commit on 2026-10-19). `Docs/...` points to this repo's part references. `network_mgr.py` is in the nepi_engine repo (`nepi_managers/scripts/network_mgr.py`). README and LINK_PROTOCOL are the two other files in this folder.
- **ASSUMED** marks a value or command that does not come from those sources, usually a standard operating-system or vendor tool. Check it on the bench.
- **ASK VICTORIA** marks a value only the team can give. The questions are collected at the end.
- **NOT IN CODE** means the team's code does not say.
- Every command block starts with a comment naming the machine it runs on: `[laptop]`, `[control Pi]` or `[NEPI Pi]`. Never run a block on another machine.
- **Not this repo's Teensy firmware.** This repo also holds Teensy thruster firmware and a driver (protocol 1.1, Teensy on USB to the NEPI Pi). They belong to a different vehicle design. On this bench the Teensy runs the team's sketch, plugged into the control Pi.

## Facts Used Here

**Team answers (Victoria Zhang, 2026-10-16).**
- **A1.** The Teensy firmware is `mcu/modes/tuning/tuning.ino`. `power_thrusters()` is commented out in the repo because the public repo is an earlier version than what is on their Teensy.
- **A2.** RB is free for the assist toggle. For testing stabilization she prefers a hotkey on the computer itself, so the toggle generalizes to other systems.
- **A3.** NEPI can use 192.168.68.23.
- **A4.** There are four cameras:
  - one faces down, on the claw;
  - one faces forward;
  - one is on the main gripper and sees the pool floor when the claw is rotated to face the bottom;
  - one faces down toward the back claw.
- **A5.** Camera streaming latency is about 60 ms from the control Pi to the topside computer. Tether bandwidth is still unknown.

**Weekly report (2026-10-18).** The team will use Idea 1 or Idea 2. Both put the NEPI Pi on the same hub as the laptop and the control Pi, so the network and software are the same for both.
- **Idea 1.** NEPI is topside and wall powered, and gets a camera copy over UDP.
- **Idea 2.** NEPI is inside the enclosure with its own bottom camera and its own 5 V supply.

**Earlier bench work and reading.**
- **F4.** FastMJPG is built on each machine (apt packages, then `./build`), and links glfw even when headless. Rendering on a Pi 5 needs `MESA_GL_VERSION_OVERRIDE=3.3 MESA_GLSL_VERSION_OVERRIDE=330`.
- **F5.** The user's bench Pi 5 (`raspberrypi.local`, 10.0.0.116 on the office LAN) has a rov-26 checkout with FastMJPG built. An exploreHD camera was verified there on 2026-10-16 at 640x480 MJPEG, 30 fps. `/dev/videoN` numbers shift with plug order.
- **F6.** From reading the FastMJPG source, not bench tested:
  - Each send output is its own UDP socket, so NEPI being off does not break the pilot's video.
  - A destination with no route, or a local address the Pi does not own, ends the whole sender process, pilot video included.
- **F7.** The Nepi_Connect design (README, LINK_PROTOCOL). On a simulated link at 3% loss, 1 send round delivered 12 of 60 frames and 2 rounds delivered 58 of 60.
- **F8.**
  - `Nepi_Connect/scripts/nepi_connect_setup.sh` runs ustreamer for a test Pi. It must not run on a Pi where FastMJPG holds the camera.
  - `Nepi_Connect/tools/fastmjpg_viewer.py` is a pure-Python FastMJPG receiver.
- **F12.** Stabilization covers orientation (pitch, roll, yaw) as well as horizontal drift. This is a requirement from the user.
- **F13.** The Black Box hub is an unmanaged switch. There is no DHCP on 192.168.68.x.

## What the Team's Code Says (Read Before Teensy Day)

These findings shape the whole plan. They come from reading the code, not from a bench test.

1. **The public repo cannot reproduce "tuning.ino plus its Pi script".** The team runs `mcu/modes/tuning/tuning.ino` (Victoria Zhang, 2026-10-16), and their Teensy holds a newer version than the repo. In the repo:
   - `tuning.ino` reads a 313-byte frame: `$RPCTL`, then 20 PID gains as doubles, then 18 controller values as 64-bit integers, then `*` and a 2-digit checksum (`tuning.ino:127-166, 356-377`).
   - Both Pi scripts send a 154-byte frame with only the 18 controller values (`rpi/modes/main.py:101-102`, `rpi/modes/tuning.py:109-113`). The encoder accepts exactly 18 values (`rpi/modes/nmea_encode_c_ext/nmea_encode.c:8, 62`). The 38-value line in `tuning.py` is commented out (`rpi/modes/tuning.py:112`).
   - `tuning.ino` copies the bytes into `input_data` before it checks the frame (`tuning.ino:364` vs `:365-377`), then uses them, including as PID gains on every loop (`:658-682`). So with a repo Pi script, stick bytes land in the wrong fields.
   - **Never enable `power_thrusters()` in the repo's `tuning.ino` and run it with a repo Pi script.**
2. **The only self-consistent trio in the repo** is `topside/modes/main.py` + `rpi/modes/main.py` + `mcu/modes/new_ctl/new_ctl.ino` (Pair M). `new_ctl.ino` reads exactly the 154-byte frame (`new_ctl.ino:130-149, 325-362`).
3. **The two script pairs do not mix.** `topside/modes/main.py` sends only changed controller events (`topside/modes/main.py:37-48`). `topside/modes/tuning.py` sends 20 gains first (`topside/modes/tuning.py:63-79`). Each Pi script crashes on the other pair's message, at `rpi/modes/tuning.py:76` or `rpi/modes/main.py:71`.
4. **The Teensy will not start without its sensors.** Both sketches loop forever in `setup()` until a Bar02 depth sensor and a BNO085 IMU answer on I2C (`tuning.ino:244-273`, `new_ctl.ino:227-256`). The thruster pins are attached only after that (`tuning.ino:285-292`). The P6 bench copy of `new_ctl.ino` skips both sensors (D2).
5. **Nothing stops a thruster when the link drops.** The Teensy keeps its last frame (`tuning.ino:342-379`). The Pi keeps resending its last frame when the topside drops (`rpi/modes/main.py:61-64, 84-88`). Ctrl+C on either script does not stop a spinning thruster. The bench supply's output switch does.

## Bench Diagram

```
                    BLACK BOX ETHERNET HUB (unmanaged switch, no DHCP)
       +--------------------------------+--------------------------------+
       |                                |                                |
 TOPSIDE LAPTOP 192.168.68.12     CONTROL Pi 5 192.168.68.22       NEPI Pi 5 192.168.68.23
 Xbox-layout pad on USB           rpi/modes/main.py                (address added through
 topside/modes/main.py                                              NEPI's network manager)
  TCP server :10110 <------------- TCP client dials in
  gamepad events (changed) ------> 18-input state                   Later (P8, P9):
  prints Pi line   <-------------- $RPCTL,<Teensy line>, /0.2 s     :5600+N <- video copy, Idea 1
 quick_render.sh                  quick_stream.sh                     (Pi :5610+N, 1 round)
  UDP :5600+N <------------------- FastMJPG :5600+N, 2 rounds       :5700 <- status, 10 Hz
                                   cameras /dev/videoN (4, A4)      :5701 -> demand, 10 Hz
                                   (ports past 5600 ASSUMED)        own bottom camera, Idea 2
                                  USB /dev/ttyACM0
                                  $RPCTL frame every 0.1 s
                                  $TNCTL lines back
                                        |
                                  TEENSY 4.1 (bench copy of new_ctl.ino, D1)
                                   I2C Wire SDA 18 / SCL 19:
                                     Bar02 0x76, BNO085 0x4A
                                   pins VFL 8  VFR 4  VBL 7  VBR 3
                                        HFL 9  HFR 5  HBL 6  HBR 2
                                        |  servo pulses 1100-1900 us
                                  8 x Basic ESC --> 8 x T200 (in water)
                                        |
                                  BENCH SUPPLY 12 V, current-limited (1.5 A, then 5 A, 2026-10-21)
```

Sources for the diagram: addresses `topside/modes/main.py:9`, `quick_stream.sh:1`, Victoria Zhang 2026-10-16 (A3); TCP `topside/modes/main.py:10, 14-20, 37-52`; Pi `rpi/modes/main.py:13, 75-88`; video `quick_stream.sh:1`, `quick_render.sh:1`; Teensy pins `tuning.ino:47-54`; I2C `Docs/tbd/pjrc_teensy_4_1.md:46`, `MS5837/MS5837.cpp:4`, `Docs/README.md:49`; NEPI ports LINK_PROTOCOL "Addresses and Ports".

## Addresses and Ports

| What | Value | Source |
|---|---|---|
| Topside laptop | 192.168.68.12 | `topside/modes/main.py:9`, `config.yaml:1` |
| Control Pi | 192.168.68.22 | `quick_stream.sh:1`, `config.yaml:2` |
| NEPI Pi | 192.168.68.23 | Victoria Zhang, 2026-10-16 (A3) |
| Netmask | /24 (255.255.255.0) | ASSUMED (README uses 192.168.68.0/24) |
| Gateway, DNS | none | ASSUMED: no router on the bench hub |
| Control link | TCP, laptop listens on 192.168.68.12:10110; Pi dials it | `topside/modes/main.py:9-10, 16`; `rpi/modes/main.py:10-11, 32` |
| Pilot video, camera 0 | UDP, Pi :5600 to laptop :5600, 2 send rounds | `quick_stream.sh:1`, `quick_render.sh:1` |
| Pilot video, cameras 1 to 3 | UDP, Pi :5601-5603 to laptop :5601-5603 | ASSUMED from `config.yaml:5-7` (no script reads that file) |
| Video copy to NEPI, camera N | UDP, Pi :5610+N to NEPI :5600+N, 1 send round | LINK_PROTOCOL "Video to NEPI" |
| Status to NEPI | UDP JSON, to NEPI :5700, 10 Hz | LINK_PROTOCOL |
| Demand from NEPI | UDP JSON, to Pi :5701, 10 Hz | LINK_PROTOCOL |
| Teensy serial | `/dev/ttyACM0`, 115200 (the Teensy ignores the baud rate on USB). It appears only after the first flash; a fresh board shows as port `usb1/1-1` (P5) | `rpi/modes/main.py:13, 23`; `Docs/tbd/pjrc_teensy_4_1.md:62`; bench 2026-10-21 |
| Frame to the Teensy | every 0.1 s | `rpi/modes/main.py:84-88` |
| Line back to the topside | every 0.2 s | `rpi/modes/main.py:75-82` |

## Timeline

| When | Steps | Result at the end |
|---|---|---|
| Today, 2026-10-19 (no Teensy) | P0, P1, P2, P3, P4 (optional); the "today" parts of P5 and P6; send the questions to Victoria; optionally the address part of P8 | Network up. Controller events reach the Pi. Video renders on the laptop with measured latency. Both sketches compile on the control Pi. Supply, tank and wiring ready. |
| Teensy day, planned 2026-10-20, held 2026-10-21 | P5, P6, P7 | Planned: firmware running with sensors, one thruster then all eight driven from the controller, acceptance checklist passed. Held: every pin spun one T200 through one ESC, and the trio drove it from the controller on pins 3 and 9, with no sensors (Bench Log). |
| Later | P8, P9 | NEPI watching the video and status. Then the stabilization toggle through the control Pi. |

## Bench Log, 2026-10-21

The Teensy day results. "Passed" was seen on the bench. "Computed" comes from the team's code or tables, not from a measurement. Each line points to the step that holds the detail.

**Passed.**
- arduino-cli 1.5.1 and `teensy:avr` 1.62.0 installed on the control Pi, after adding `~/.local/bin` to PATH (P5).
- Adafruit BNO08x 1.2.7 (with Unified Sensor 1.1.15 and BusIO 1.17.4) and BlueRobotics MS5837 Library 1.1.1 installed. The repo's `MS5837/` folder does not compile (P5).
- A fresh Teensy 4.1 flashed from the control Pi with `-p usb1/1-1`, without the program button. `/dev/ttyACM0` appeared after the first flash (P5).
- The bench Pi runs Python 3.13. `nmea_encode` was rebuilt in place with the P1 command (P1).
- The Basic ESC armed with three rising tones, then one tone, then one more (P6).
- `esc_all.ino` spun one T200 through one ESC from each of pins 2 to 9 (P6).
- The bench copy of `new_ctl.ino` (power on, triggers 1023 and 40, no sensors, `delay(5)`) compiled, flashed and sent `$TNCTL` about 4 times a second (P6).
- The laptop script ran on the laptop's system Python 3.8.10, without a venv (P2).
- End to end: controller, laptop script, Pi script, Teensy, ESC and T200. RY moved the thruster on pin 3 (VBR); LY, LX and the triggers moved it on pin 9 (HFL) (P7).
- Slow mode: full LY gave 100 on values 5 to 8 (P7).
- Holding LY full and pushing RY past about 50% cut HFL from 1700 to 1651 µs, seen as a slight slowdown (P6, V11).
- Ctrl+C on the scripts leaves the Teensy on its last command, so the supply goes off first at shutdown (P7).

**Computed, not measured.**
- 1.5 A per thruster is reached at about 1648 µs forward and 1352 µs reverse; 5 A at about 1735 and 1265 µs (P6).
- Any single input over its whole range stays at or under about 4.8 A on one thruster (P6).
- LY, LX and a trigger together can drive one horizontal thruster to about 1890 µs, about 17 A (P6).
- `limit_current()` makes thrust non-monotonic in RY and trigger travel past about 60% (V11).

**Still open.**
- The physical push direction for LY up, LX right, LT and RY up (P7).
- The heading-hold sign, which needs the IMU (P7).
- The depth and IMU checks of P5 and P7, which need the Bar02 and the BNO085 (D2).
- All eight ESCs and thrusters at once: only one ESC and one T200 were on the bench (P6).
- Where the startup tones on a USB unplug come from (P6).

## Plan Steps

### P0. Bench Network

**Goal.** All three machines have fixed addresses on 192.168.68.0/24 and reach each other through the hub.

**When.** Today.

**Needs.** The hub, three Ethernet cables, the laptop, the control Pi and the NEPI Pi.

**Main info.**
- Laptop .12, control Pi .22, NEPI Pi .23, netmask /24 (table above). The team's scripts have these addresses typed in (`topside/modes/main.py:9`, `rpi/modes/main.py:10`, `quick_stream.sh:1`), so they cannot change.
- The hub hands out no addresses (F13), so each machine needs a static address.
- NEPI adds .23 through its own network manager, not by editing the OS network files. It listens on `<base namespace>/add_ip_addr`, a `std_msgs/String` topic (`network_mgr.py:322-329`). The address must be in CIDR form, `x.x.x.x/y` (`network_mgr.py:813-815`).

**Do.**

```bash
# [control Pi] ASSUMED: Raspberry Pi OS Bookworm (NetworkManager), eth0 to the hub, office LAN on Wi-Fi
nmcli -t -f NAME,DEVICE,TYPE con show
sudo nmcli con add type ethernet ifname eth0 con-name rov-bench \
  ipv4.method manual ipv4.addresses 192.168.68.22/24 ipv4.never-default yes \
  ipv6.method disabled connection.autoconnect-priority 10
sudo nmcli con up rov-bench
ip -4 addr show eth0          # expect: inet 192.168.68.22/24
```

```bash
# [laptop] ASSUMED: Ubuntu with NetworkManager; replace <eth> with the wired device from "ip link"
sudo nmcli con add type ethernet ifname <eth> con-name rov-bench \
  ipv4.method manual ipv4.addresses 192.168.68.12/24 ipv4.never-default yes \
  ipv6.method disabled connection.autoconnect-priority 10
sudo nmcli con up rov-bench
ip -4 addr show <eth>         # expect: inet 192.168.68.12/24
```

```bash
# [NEPI Pi] in a NEPI shell where rostopic works; topic and CIDR form from network_mgr.py:322-329, 813-815
ip -4 addr                                         # note the addresses NEPI already has
rostopic list | grep add_ip_addr                   # the full topic name
rostopic pub -1 <topic from the line above> std_msgs/String "data: '192.168.68.23/24'"
ip -4 addr | grep 192.168.68.23                    # expect one line
```

The RUI's IP alias page does the same thing (README Section 2).

```bash
# [laptop] 100 pings per target (README Phase 0)
ping -c 100 -i 0.2 192.168.68.22 | tail -2
ping -c 100 -i 0.2 192.168.68.23 | tail -2
# [control Pi]
ping -c 5 192.168.68.12 && ping -c 5 192.168.68.23
```

**Done when.** Every ping shows 0% packet loss, and the laptop shows 0% over 100 pings to each Pi (README Phase 0).

**Gotchas.**
- If the control Pi reaches the office LAN (10.0.0.116, F5) over eth0 rather than Wi-Fi, `rov-bench` takes it off that LAN. Then use Wi-Fi or a USB Ethernet adapter for the office (ASSUMED).
- Set no gateway on the bench network. A gateway here would steal the default route from the office LAN.
- Adding an address in NEPI *replaces* NEPI's list of managed alias addresses (`network_mgr.py:982`) and the saved `NEPI_ALIAS_IPS` (`:990`). Compare `ip -4 addr` before and after, and add back any factory alias that disappears.
- FastMJPG binds its sender to 192.168.68.22. If that address is missing, or the network has no route, the whole sender process exits (F6). Finish P0 before P3.

### P1. Control Pi

**Goal.** The team's Pi side is installed and starts the way it does on the robot.

**When.** Today.

**Needs.**
- **Pi and OS.** A Raspberry Pi 5. ASSUMED: the bench Pi at `raspberrypi.local` (10.0.0.116), which already has a rov-26 checkout and FastMJPG built, and runs an exploreHD camera at 640x480 MJPEG 30 fps (F5). It runs 64-bit Raspberry Pi OS with Python 3.13 (bench, 2026-10-21). The committed C extension is built for 3.11 (`nmea_encode.cpython-311-aarch64-linux-gnu.so`, `rpi/modes/nmea_encode_c_ext/`), so on this Pi it must be rebuilt in place (Do, below).
- **Power.** A 5 V, 5 A USB-C supply. With a 3 A supply the Pi limits all USB devices to 600 mA in total, too little for cameras plus the Teensy (`Docs/tbd/raspberry_pi_5.md:15-17, 28-30`).
- **Cameras.** One to four USB cameras.

**Main info.**
- **Script.** `rpi/modes/main.py` (Pair M). The team's deployed script may differ: ASK VICTORIA (V1).
- **Dependencies.** pyserial 3.5 (`requirements.txt:3`) and the local C extension `nmea_encode`, built by `setup.py` (`rpi/modes/nmea_encode_c_ext/setup.py:1-13`). It needs `Python.h` (`nmea_encode.c:4`).
- **Serial.** `/dev/ttyACM0` at 115200 (`rpi/modes/main.py:13, 23`). The port is opened at import, before anything else.
- **Network.** The script dials 192.168.68.12:10110 and exits if nobody listens (`rpi/modes/main.py:29-37`).
- **Boot.** Nothing in the repo starts the script at boot: NOT IN CODE.

**Do.**

```bash
# [control Pi]
grep VERSION_CODENAME /etc/os-release     # ASSUMED: bookworm; the bench Pi's Python 3.13 hints at a newer release (inference)
python3 --version                          # bench Pi: 3.13 (2026-10-21); only 3.11.x loads the committed .so
cd ~/rov-26 && git log --oneline -1        # ASSUMED path; expect 1af4984
sudo apt install -y python3-serial python3-dev python3-setuptools build-essential socat   # ASSUMED Debian package names
```

```bash
# [control Pi] test the committed C extension first
cd ~/rov-26 && python3 -c "import sys; sys.path.insert(0,'rpi/modes'); import nmea_encode_c_ext.nmea_encode as n; print(len(n.nmea_encode(tuple([0]*18))))"
# expect 154: 6 header + 144 data + "*XX" + one zero byte (nmea_encode.c:6-10, 82-87)
# if the import fails (it does on Python 3.13): build it in place, then test again
# this command rebuilt it on the bench Pi under Python 3.13, 2026-10-21
cd ~/rov-26/rpi/modes/nmea_encode_c_ext && python3 setup.py build_ext --inplace
```

```bash
# [control Pi] FastMJPG: reuse the F5 build, or build it per FastMJPG/README.md "Install"
ls -l ~/rov-26/FastMJPG/bin/FastMJPG
cd ~/rov-26 && ./FastMJPG/bin/FastMJPG devices      # lists cameras, sizes and rates (FastMJPG/README.md)
ls -l /dev/v4l/by-path/                              # ASSUMED: stable names by USB port
```

```bash
# [control Pi] launch: only after the laptop prints "listening for connection" (P2)
cd ~/rov-26 && python3 rpi/modes/main.py
```

Today, with no Teensy, the script stops at the serial open (`rpi/modes/main.py:23`). Expect a pyserial error that names `/dev/ttyACM0`, before "socket created" (`:30`). That is the expected result. P4 gets past it with a fake Teensy.

**Done when.**
- The import check prints `154`.
- `FastMJPG devices` lists each camera with 640x480 at 30 fps.
- Launching the script with no Teensy stops at the serial open, as described.

**Gotchas.**
- `/dev/videoN` numbers change with plug order (F5). Pick devices with `FastMJPG devices` every time, or use the `/dev/v4l/by-path/` names (ASSUMED: FastMJPG opens any device path). The exploreHD may not report a unique serial number, so `by-id` names can collide (`Docs/README.md:14`).
- If another USB serial device enumerates first, the Teensy may become `/dev/ttyACM1` while the script still opens `ttyACM0`. Check `ls -l /dev/serial/by-id/` (ASSUMED).
- An event code missing from `gamepad_map` raises `KeyError` at `rpi/modes/main.py:72`, outside any `try`, and the script exits. P2 lists the codes your controller sends.
- When the script dies, the Teensy keeps the last frame it received (finding 5).
- Quit miniterm (Ctrl+]) before starting the script. It holds `/dev/ttyACM0` (P5).
- There is no reconnect. To recover, restart the laptop script, then the Pi script.
- Do not run `Nepi_Connect/scripts/nepi_connect_setup.sh` on this Pi. Its ustreamer service would take the camera FastMJPG needs (F8).
- `rpi/modes/tuning.py` prints on every pass of a busy loop (`:111`), and it crashes on messages from `topside/modes/main.py` (`:76`). Use it only with `topside/modes/tuning.py`.

### P2. Topside Computer

**Goal.** The laptop reads the controller, runs the team's topside script, and can render FastMJPG video.

**When.** Today.

**Needs.**
- **OS.** A Linux laptop with Ethernet. The bench laptop runs Ubuntu 20.04 (2026-10-21). The team's topside OS is NOT IN CODE (ASK VICTORIA, V4), but `quick_render.sh` runs a FastMJPG binary whose only documented install is `apt-get` plus `./build` (`FastMJPG/README.md` "Install").
- **Python.** Python 3. The repo pins 3.12.8 (`.python-version:1`), but `topside/modes/main.py` runs on the bench laptop's system Python 3.8.10 (2026-10-21). If you run `topside/modes/tuning.py`, its pinned numpy and matplotlib (`requirements.txt:4-5`) set their own minimum version (ASSUMED: 3.10).
- **Controller.** An Xbox-layout controller (button names in `rpi/modes/main.py:39-58`). The model is NOT IN CODE (V4).

**Main info.**
- **Launch.** `python3 topside/modes/main.py`. It binds 192.168.68.12:10110 and accepts one connection (`topside/modes/main.py:9-10, 14-20`).
- **Imports.** `inputs` (pinned `inputs==0.5`, `requirements.txt:2`) and `tkinter` (`:7`, imported but unused). `topside/modes/tuning.py` also imports matplotlib and numpy (`:8-11`).
- **Controller check.** `topside/modes/gamepadtest.py` prints `ev_type code state` for every event (`gamepadtest.py:1-10`).
- **Input map** (`rpi/modes/main.py:39-58`). These are the inputs the firmware reacts to (`tuning.ino:381-396, 443-499`); every other input is sent but ignored:

  | Code | Input | Firmware action |
  |---|---|---|
  | `ABS_Y` | LY | surge |
  | `ABS_X` | LX | sway |
  | `ABS_RY` | RY | depth rate |
  | `ABS_Z`, `ABS_RZ` | LT, RT | yaw rate |
  | `BTN_TL` | LB | slow mode |
- **RB** is `BTN_TR`, slot 13 (`rpi/modes/main.py:53`). Neither sketch uses it.
- **No keyboard.** No topside script reads the keyboard; `get_gamepad()` reads only the controller (`topside/modes/main.py:38`).
- **Trigger range.** The two sketches disagree: `tuning.ino:28` divides by 1023 and `new_ctl.ino:28` by 255. The controller decides which is right. The bench controller reports 0 to 1023 (2026-10-21), so the P6 bench copy of `new_ctl.ino` uses 1023 and a trigger dead zone of 40, the `tuning.ino` values (`tuning.ino:28, 30`).

**Do.**

```bash
# [laptop] ASSUMED: Ubuntu package names
sudo apt install -y git python3-tk
git clone https://github.com/CoralCrusaders/rov-26.git ~/rov-26 && cd ~/rov-26 && git checkout 1af4984
python3 -m pip show inputs                     # bench laptop (2026-10-21): 0.5, in the user's site packages
python3 -m pip install --user inputs==0.5      # only if the line above finds nothing (ASSUMED: pip installed)
python3 topside/modes/gamepadtest.py
```

The bench laptop had `inputs` 0.5 and `tkinter` already, so the team's scripts run on the system Python. A venv is needed only where `pip --user` is not allowed. Then run `sudo apt install -y python3-venv`, `python3 -m venv ~/rov-venv && ~/rov-venv/bin/pip install inputs==0.5`, and use `~/rov-venv/bin/python` in place of `python3` in this step.

Move every stick and trigger to both ends and press every button. Write down:
- The full-pull value of LT and RT: 255 or 1023. The bench controller gives 1023 (2026-10-21).
- The sign of `ABS_Y` with the stick pushed up, and of `ABS_X` with the stick pushed right.
- Any code that is not in `rpi/modes/main.py:39-58`. For example, a `BTN_MODE` from the Xbox button would crash the Pi script. Do not press that input while the Pi script runs.

```bash
# [laptop] FastMJPG, from FastMJPG/README.md "Install"
sudo apt-get install -y build-essential libturbojpeg0 libturbojpeg0-dev libglfw3-dev v4l-utils \
  libavutil-dev libavcodec-dev libavformat-dev libswscale-dev
cd ~/rov-26/FastMJPG && chmod +x ./build && ./build && ./bin/FastMJPG help
```

```bash
# [laptop] launch: always BEFORE the Pi script; system Python, verified on the bench laptop 2026-10-21
cd ~/rov-26 && python3 topside/modes/main.py
# prints "socket created", "socket bound", "listening for connection" (topside/modes/main.py:15-19)
# then "connection address: ('192.168.68.22', ...)" once the Pi connects (:21)
```

**Done when.**
- `gamepadtest.py` shows every input, and the trigger range and stick signs are recorded. Bench trigger range: 0 to 1023 (2026-10-21).
- The topside script prints "listening for connection".
- `FastMJPG help` runs.

**Gotchas.**
- If `gamepadtest.py` finds no gamepad, check that the user can read `/dev/input/event*`. Adding the user to the `input` group is ASSUMED to fix it.
- If apt reports that `libturbojpeg0` does not exist on this Ubuntu release, install only `libturbojpeg0-dev` (ASSUMED).
- The bind fails if the laptop does not own 192.168.68.12 (`topside/modes/main.py:16`).
- The script sends nothing while the controller is still, because `get_gamepad()` waits for a change (`:38`). The Pi keeps the last state.
- If the Pi drops, the topside script exits on the next controller event (`:50-52`). Restart the topside first, then the Pi.
- `topside/modes/tuning.py` does not print the Pi's line (`:57` is commented out), and it pairs only with `rpi/modes/tuning.py`.

### P3. Video End to End

**Goal.** The pilot's video works as on the robot, and its latency is measured against the team's 60 ms.

**When.** Today.

**Needs.** P0 to P2 done, a camera on the control Pi, FastMJPG on both machines, and two phones (one runs a stopwatch with milliseconds, one takes photos).

**Main info.**
- **Sender:** `./FastMJPG/bin/FastMJPG capture /dev/video0 640 480 1 30 send 192.168.68.22 5600 192.168.68.12 5600 1400 1000000 2` (`quick_stream.sh:1`). That is 640x480 at 30 fps, 1400-byte packets, frames up to 1000000 bytes, each frame sent twice.
- **Receiver:** `./FastMJPG/bin/FastMJPG receive 192.168.68.12 5600 1400 1000000 640 480 1 30 render 1280 720` (`quick_render.sh:1`).
- **Process limits.** One FastMJPG process takes exactly one input, and has at most one render window (`FastMJPG/README.md` "Syntax", "Render" note 4). So each camera needs its own sender and its own receiver.
- **Four cameras.** There are four (A4). Which one is `/dev/video0`, and the ports of the others, are NOT IN CODE (ASK VICTORIA, V5). This plan assumes camera N uses Pi port 5600+N to laptop port 5600+N (`config.yaml:4-7`).
- **Team latency:** about 60 ms from the control Pi to the topside (Victoria Zhang, 2026-10-16).

**Do.**

```bash
# [laptop] first
cd ~/rov-26 && bash quick_render.sh
```

```bash
# [control Pi] second; if the camera is not /dev/video0, run the line from quick_stream.sh with the right device
cd ~/rov-26 && bash quick_stream.sh
```

```bash
# [control Pi] a second camera, ASSUMED port numbering (config.yaml:5)
cd ~/rov-26 && ./FastMJPG/bin/FastMJPG capture /dev/videoX 640 480 1 30 send 192.168.68.22 5601 192.168.68.12 5601 1400 1000000 2
# [laptop]
cd ~/rov-26 && ./FastMJPG/bin/FastMJPG receive 192.168.68.12 5601 1400 1000000 640 480 1 30 render 1280 720
```

**Latency check.**
1. Point the camera at the stopwatch phone, placed next to the laptop screen.
2. Photograph both together, 10 times.
3. For each photo, latency = the stopwatch reading minus the reading shown on screen.
4. Record the median and the worst, and compare them with 60 ms.

**Stream bandwidth.** On the control Pi, read `ip -s link show eth0` twice, 10 s apart. The TX byte difference x 8 / 10 gives bits per second (README Phase 1). This number is needed against the tether's capacity, which is still unknown (V2).

**Done when.**
- Live video renders on the laptop.
- The latency median and worst are recorded next to the team's 60 ms.
- The Pi's transmit rate per camera is recorded.

**Gotchas.**
- The scripts are not executable in the checkout, so run them with `bash`. They use relative paths, so run them from the repo folder.
- Every receive setting must equal the sender's (`FastMJPG/README.md` "Receive" note 2). An unsupported size or rate crashes the capture (`FastMJPG/README.md` "Capture" note 1).
- Nothing else may ever send to the laptop's port 5600. FastMJPG's receiver exits on a malformed packet (README Section 5, rule 11).
- A render window that stops responding while the stream pauses is normal. Close it with Ctrl+C (`FastMJPG/README.md` "Render" notes 2 and 3).
- Rendering on a Pi 5, not the laptop, needs `MESA_GL_VERSION_OVERRIDE=3.3 MESA_GLSL_VERSION_OVERRIDE=330`, or glfw cannot open a window (F4).

### P4. Fake Teensy (Optional)

**Goal.** Check the whole controller-to-serial path today, before the Teensy arrives, and settle the trigger range and stick signs.

**When.** Today. This is optional, but recommended.

**Needs.** `socat` on the control Pi (installed in P1) and a small decoder script, `fake_teensy.py`. **The decoder is not written yet.** This step describes it. When written, it belongs in `Nepi_Connect/tools/` and never goes on the robot (README Section 7).

**Main info.**
- **Frame** (`nmea_encode.c:6-10, 23-42, 46-54, 78-91`). 154 bytes in this order:
  - the ASCII `$RPCTL`;
  - 18 little-endian 64-bit integers, in `gamepad_map` order (`rpi/modes/main.py:39-58`);
  - `*` and two uppercase hex digits, the XOR of every byte after `$` up to the end of the data;
  - one zero byte.
- **Rate:** every 0.1 s (`rpi/modes/main.py:84-88`).
- **What the Pi reads back.** Text lines (`rpi/modes/main.py:103-106`). `new_ctl.ino` prints `$TNCTL,` then 20 values, then `,*FF` (`new_ctl.ino:584-629`). `tuning.ino` prints 25 values, the last one `millis()` (`tuning.ino:601-656`).

**Do.**

```bash
# [control Pi] terminal 1: a pseudo-terminal pair; /dev/ttyACM0 is the end the team's script opens (README Section 7)
sudo socat pty,raw,echo=0,link=/dev/ttyACM0 pty,raw,echo=0,link=/dev/ttyFAKE
# [control Pi] terminal 2: the decoder, NOT WRITTEN YET
sudo python3 fake_teensy.py /dev/ttyFAKE
# [laptop] start topside/modes/main.py as in P2, then:
# [control Pi] terminal 3: sudo, because the pty links are root-owned (ASSUMED)
cd ~/rov-26 && sudo python3 rpi/modes/main.py
```

The decoder must:
- sync on `$RPCTL`;
- read the next 148 bytes and check the `*` and the XOR;
- unpack 18 integers (`<18q`) and print each field that changed, by name;
- optionally print a fake 20-value `$TNCTL` line every 250 ms, so the laptop shows data.

**Done when.**
- Each stick, trigger and button changes exactly its named field.
- Every frame passes the checksum, and frames arrive about 10 times a second.
- The trigger full-pull value and the stick signs agree with P2.

**Gotchas.**
- Start socat before the Pi script, which exits if `/dev/ttyACM0` is missing.
- Stop socat before plugging in the real Teensy, so the link name is free.
- This fake decodes the 154-byte frame of the repo's Pi scripts. If Victoria's real Pi script sends gains too (V1), the decoder needs the 313-byte layout in `tuning.ino:127-166`.
- The fake becomes the test harness for `nepi_link.py` in P9.

### P5. Teensy Bring-Up, No ESCs Attached

**Goal.** The team's firmware runs, receives the Pi's frames, and sends telemetry back. Nothing is connected to the ESC pins yet.

**When.** Toolchain and compile checks today. Flashing on Teensy day.

**Needs.**
- **Board.** The Teensy 4.1 and a USB cable to the control Pi.
- **Sensors.** A Bar02 depth sensor and a BNO085 IMU, wired to the Teensy's I2C before the first boot (D2). Both are in the MATE kit (`Docs/README.md:15-16`), but neither was on the 2026-10-21 bench, so that bench ran the sensor-skip copy (D2, P6).
- **Firmware.** Chosen in D1. On the 2026-10-21 bench: the P6 bench copy of `new_ctl.ino`.

**Main info.**
- **Board and flashing.** The team's command is `arduino-cli compile -u -b teensy:avr:teensy41 -p /dev/ttyACM0 <sketch>.ino` (`mcu/modes/tuning/upload.sh:1`, `mcu/modes/new_ctl/upload.sh:1`). It fails on a fresh board.
  - A new Teensy 4.1 runs PJRC's factory blink sketch as USB RawHID. `lsusb` shows `16c0:0486 Teensyduino RawHID`, and `/dev/ttyACM0` does not exist until a sketch with USB serial is flashed (bench, 2026-10-21).
  - `arduino-cli board list` shows the board as port `usb1/1-1`, protocol `teensy`, Teensy 4.1. The port name depends on the USB socket, so always take it from that list.
  - What worked every time on the bench: `arduino-cli compile -u -b teensy:avr:teensy41 -p usb1/1-1 .`, run inside the sketch folder. The program button was not needed. After the first flash, `/dev/ttyACM0` appears.
  - Flashing from the control Pi worked (D7, resolved for the bench).
- **Libraries.**
  - `Servo` and `Wire` come with the Teensy core (`tuning.ino:7, 9`).
  - The Adafruit BNO08x library is installed separately (`tuning.ino:10`). The library manager installed 1.2.7 and pulled in Adafruit Unified Sensor 1.1.15 and Adafruit BusIO 1.17.4 (bench, 2026-10-21).
  - `MS5837` (`tuning.ino:11`): **do not use the repo's `MS5837/` folder. It does not compile.** Its `MS5837.h` is the newer Blue Robotics version, with `init(TwoWire &wirePort = Wire)` and members `D1_pres`, `D2_temp` (`MS5837/MS5837.h:55, 96`). Its `MS5837.cpp` is the older version, with `init()` and members `D1`, `D2` (`MS5837/MS5837.cpp:24, 75-78`). The compiler stops with "no declaration matches 'bool MS5837::init()'" and "'D1' was not declared in this scope". Install "BlueRobotics MS5837 Library" from the library manager instead (1.1.1 on the bench). The team probably builds against that install (V6).
- **Sensor wiring and addresses.**
  - Teensy `Wire` uses SDA 18 and SCL 19 (`Docs/tbd/pjrc_teensy_4_1.md:46`), at 400 kHz (`tuning.ino:279-280`).
  - The Bar02 is at 0x76 (`MS5837/MS5837.cpp:4`). It needs 3.3 V logic and has no pull-ups (`Docs/npx/bluerobotics_bar_depth_sensor.md:27, 46, 51-53`).
  - The BNO085 is at 0x4A (`Docs/README.md:49`). Its board has 10 kΩ pull-ups to VIN (`Docs/npx/adafruit_bno085_imu.md:31, 42`). Power both from the Teensy's 3.3 V pin so the pull-ups sit at 3.3 V (ASSUMED wiring).
- **Sensors missing.** `setup()` prints `Failed to initialize bar02` or `Failed to initialize BNO08x` once a second, forever (`tuning.ino:244-273`).
- **Telemetry.** `new_ctl.ino` prints 20 values every 50 loops (`new_ctl.ino:14, 584-629`). `tuning.ino` prints 25 values every loop (`tuning.ino:14, 601-656`). Values 1 to 8 are thruster outputs in µs away from 1500. They are computed even while `power_thrusters()` is commented out (`tuning.ino:687-690`).
- **Error lines.** The firmware prints `ERRSTX`, `ERRSIG`, `ERRTRM` or `ERRCHK` for bad frames (`tuning.ino:350-376`).

**Do (today).**

```bash
# [control Pi] arduino-cli install script and PJRC board index URL, per their vendors' docs; verified 2026-10-21
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | BINDIR=$HOME/.local/bin sh
# the installer warns that ~/.local/bin is not on PATH in this shell; fix it for now and for new shells
echo 'export PATH=$HOME/.local/bin:$PATH' >> ~/.bashrc
export PATH=$HOME/.local/bin:$PATH
arduino-cli config init
arduino-cli config add board_manager.additional_urls https://www.pjrc.com/teensy/package_teensy_index.json
arduino-cli core update-index && arduino-cli core install teensy:avr   # bench: arduino-cli 1.5.1, teensy:avr 1.62.0
arduino-cli lib install "Adafruit BNO08x"                 # bench: 1.2.7, plus Unified Sensor 1.1.15, BusIO 1.17.4
rm -rf ~/Arduino/libraries/MS5837                         # only if an earlier try copied the repo's MS5837 folder there
arduino-cli lib install "BlueRobotics MS5837 Library"     # bench: 1.1.1; never the repo's MS5837/ folder
```

The core install brought teensy-tools 1.62.0 and teensy-compile 15.2.1 with it (bench, 2026-10-21).

```bash
# [control Pi] udev rules for the Teensy (Docs/tbd/pjrc_teensy_4_1.md:187, 194, 255)
sudo curl -fsSL -o /etc/udev/rules.d/00-teensy.rules https://www.pjrc.com/teensy/00-teensy.rules
sudo udevadm control --reload-rules
systemctl status ModemManager --no-pager     # if running, it can interfere (Docs/tbd/pjrc_teensy_4_1.md:171, 233)
```

```bash
# [control Pi] compile only; no Teensy needed. Proves the toolchain and libraries.
# the P6 bench copy of new_ctl.ino compiled on the bench Pi with the libraries above, 2026-10-21
cd ~/rov-26/mcu/modes/new_ctl && arduino-cli compile -b teensy:avr:teensy41 new_ctl.ino
cd ~/rov-26/mcu/modes/tuning  && arduino-cli compile -b teensy:avr:teensy41 tuning.ino
```

**Do (Teensy day).**
1. With the Teensy unplugged, wire the Bar02 and BNO085 to 3.3 V, GND, SDA 18 and SCL 19. Leave pins 2 to 9 empty. With no sensors, use the P6 bench copy in step 3 instead (D2).
2. Quit miniterm (Ctrl+]) and stop any Pi script. Either one holds `/dev/ttyACM0`.
3. Plug in the Teensy and flash the D1 sketch **unmodified**, with `power_thrusters()` still commented out. On a bench with no sensors, flash the P6 bench copy from `~/bench_fw/new_ctl` instead; nothing is on pins 2 to 9 yet.

```bash
# [control Pi] upload.sh:1 with the port from "arduino-cli board list"; verified 2026-10-21 (usb1/1-1)
arduino-cli board list                  # fresh board: port usb1/1-1, protocol teensy, Teensy 4.1
cd ~/rov-26/mcu/modes/<sketch> && arduino-cli compile -u -b teensy:avr:teensy41 -p usb1/1-1 .
ls -l /dev/ttyACM0                      # appears after the first flash
python3 -m serial.tools.miniterm /dev/ttyACM0 115200     # expect $TNCTL lines; Ctrl+] quits
```

`<sketch>` is `new_ctl` or `tuning`, or Victoria's sketch. Replace `usb1/1-1` with the port `arduino-cli board list` shows. The unmodified sketch with no sensors prints `Failed to initialize bar02` once a second and never sends `$TNCTL` (`new_ctl.ino:227-231`).

4. Quit miniterm. Quitting it does not stop the sketch. Start the laptop script (P2), then `python3 rpi/modes/main.py` (P1).
5. The Pi prints the serial port list (`rpi/modes/main.py:26-27`), then every Teensy line (`:106`).
6. Move LY. Values 5 to 8 (the horizontal outputs) change. Turn the IMU by hand. Value 11 (`yaw_deg`) changes. With no sensors, skip the IMU check.

**Done when.**
- `$TNCTL` lines reach the Pi and appear on the laptop, with no `ERR...` lines. Passed on 2026-10-21 with the bench copy.
- Each stick moves the expected outputs (P7 table).
- `yaw_deg` follows the IMU and `depth_m` reads near 0 in air. Not checked on 2026-10-21: no sensors.

**Gotchas.**
- `ERRTRM` lines with the repo's `tuning.ino` and a repo Pi script confirm the frame-size mismatch (finding 1). Do not go on to P6 with that pair.
- Flashing needs the port free. Quit miniterm (Ctrl+]) and stop the Pi script first, before every upload. Quitting miniterm does not stop the sketch.
- Never open the Teensy's port at 134 baud. That reboots it into the bootloader (`Docs/tbd/pjrc_teensy_4_1.md:65`).
- `/dev/ttyACM0` exists only while a sketch with USB serial is running (`Docs/tbd/pjrc_teensy_4_1.md:138`). A brand-new Teensy 4.1 runs the factory blink sketch as RawHID (`lsusb`: `16c0:0486 Teensyduino RawHID`), so `upload.sh`'s `-p /dev/ttyACM0` fails on it. Use the port from `arduino-cli board list`. The program button was not needed (2026-10-21).
- The `usb1/1-1` port name depends on which USB socket the cable is in. Read it from `arduino-cli board list` after moving the cable.
- The repo's `MS5837/` folder does not compile (Main info). If it was copied into `~/Arduino/libraries`, run `rm -rf ~/Arduino/libraries/MS5837` before installing the library-manager version, as the bench did.
- `tuning.ino` prints a line on every loop, so the Pi terminal scrolls fast. That is normal.
- `Starting...` (`tuning.ino:277`) is printed before the port is open, so it is usually missed.

### P6. ESC and Thruster Bring-Up

**Goal.** One thruster, then all eight, follow the controller in water, with a known way to stop them.

**When.** Teensy day, after P5. Get the supply, tank and leads ready today.

**Needs.**
- **Supply.** A 12 V bench supply with a current limit and an output switch, with one person on that switch. The 2026-10-21 bench used a 12 V 1.5 A supply, then a 12 V 5 A supply. 12 V matches the firmware's current table for the T200 "@ 12V" (`new_ctl.ino:19-21`) and its 25 A fuse comment (`:16`). The team's real supply voltage and fuse are still open (V7).
- **Water.** A tank or bucket deep enough to cover each thruster.
- **Wiring.** ESC signal leads to the Teensy pins, with a common ground.
- **Firmware.** The bench copy of `new_ctl.ino` in `~/bench_fw/new_ctl` (Do, step 1), with `power_thrusters();` uncommented (`new_ctl.ino:640`). `~/rov-26` stays unmodified. Never the repo's `tuning.ino` with a repo Pi script (finding 1).
- **Pin check.** The bench-only sketch `~/bench_fw/esc_all/esc_all.ino` (Do, step 9).

**Main info.**
- **Pins:** VFL 8, VFR 4, VBL 7, VBR 3, HFL 9, HFR 5, HBL 6, HBR 2 (`tuning.ino:47-54`, `new_ctl.ino:50-57`). Every direction flag is +1 (`tuning.ino:56-63`).
- **Pulse:** `1500 + power`, with power from -400 to 400, giving 1100 to 1900 µs (`tuning.ino:26, 33, 569-578`). The ESC reads 1100 as full reverse, 1500 as stop and 1900 as full forward, with a ±25 µs dead band (`Docs/Motors/bluerobotics_basic_esc.md:7-11, 24-25`).
- **Arming.** Power the ESC with the motor connected and it plays three rising tones. It then needs a stop pulse "for a few seconds"; it plays one tone when it detects a signal and a second tone when it detects stop. Blue Robotics' example holds 7 s (`Docs/Motors/bluerobotics_basic_esc.md:113-123`).
- **Tones on the bench (2026-10-21).**
  - The ESC plays its three rising startup tones at supply power-up whether or not the Teensy is plugged into the Pi.
  - Armed is three rising tones, then one tone, then one more. Three tones followed by silence means no signal reaches the ESC.
  - It also replays the three tones when the Teensy's USB cable is unplugged from the Pi. The cause is unknown, and it is harmless.
- **Firmware and arming.** The firmware has no arming hold. Before the first `power_thrusters()` write, the pins carry the Servo library's default pulse (NOT IN CODE; expected 1500 µs).
  - With `tuning.ino`, every gain is zero until a valid frame arrives (`tuning.ino:658-682`; `input_data` starts at zero). So all outputs are 1500 µs until the Pi script starts.
  - `new_ctl.ino` has fixed gains (`new_ctl.ino:206-214`), so its depth and heading holds act from the first loop.
- **Current per thruster at 12 V:** 0.5 A at 1600 µs, 3.3 A at 1700, 8.9 A at 1800 and 16.9 A at 1900 (`Docs/Motors/bluerobotics_t200_thruster.md:101-111`).
- **What the sticks ask for.** Full LY asks for 200 µs (gain 0.5, `tuning.ino:35, 483`), which is 1700 µs. The firmware caps its *estimated* total at 20 A: 10 A horizontal and 10 A vertical (`tuning.ino:16-18, 538-567`). That cap scales outputs in a way that makes thrust non-monotonic in stick travel (V11).
- **What a bench supply allows.** COMPUTED from the team's T200 current table (`AMP_LIST`, `new_ctl.ino:21`), for one thruster:
  - 1.5 A at about 1648 µs forward and 1352 µs reverse.
  - 5 A at about 1735 µs forward and 1265 µs reverse.
- **Single inputs.** COMPUTED by simulating `new_ctl.ino:411-550` with the sensors reading zero and one thruster connected. Any single input over its whole range stays at or under about 4.8 A on one thruster:
  - LY full: 1700 µs (3.3 A).
  - LX full: 1660 or 1340 µs (1.8 A).
  - One trigger alone: peaks near 1284 µs (3.9 A) at a 60 to 70 percent pull, then falls to 1370 µs (1.1 A) at a full pull.
  - RY alone: peaks at 1730 µs (4.8 A) near 60 percent, then falls to 1618 µs (0.8 A) at full.
- **Combined inputs.** COMPUTED the same way. LY, LX and a trigger together can drive one horizontal thruster to about 1890 µs, about 17 A. Even each input at half travel can reach about 14 A. Slow mode does not prevent this, because it does not scale the yaw term (`new_ctl.ino:479-498`).
- **Bench rule.** One input at a time on any supply under 17 A. On a 1.5 A supply, also use slow mode (LB) and avoid the triggers.
- **Dry running:** a T200 must not run more than 10 s out of water (`Docs/Motors/bluerobotics_t200_thruster.md:183`).
- **Signal loss.** The ESC stops about 0.3 s after its pulses stop, and needs re-arming (`Docs/Motors/bluerobotics_basic_esc.md:180-189`). The Teensy keeps pulsing as long as it runs, so this only helps when the Teensy itself stops.

**Do.**
1. **Bench copy.** Make the bench copy of `new_ctl.ino` on the control Pi. It is Pair M's sketch (D1 option c) with `power_thrusters()` on and the trigger constants set to the bench controller's 0 to 1023 (P2). The third and fourth `sed` lines are only for a bench with no Bar02 and no BNO085 (D2). Every line below was verified on 2026-10-21.

```bash
# [control Pi] bench copy of new_ctl.ino; ~/rov-26 stays unmodified (verified 2026-10-21)
mkdir -p ~/bench_fw/new_ctl && cp ~/rov-26/mcu/modes/new_ctl/new_ctl.ino ~/bench_fw/new_ctl/
cd ~/bench_fw/new_ctl
sed -i 's|^  //power_thrusters();|  power_thrusters();|' new_ctl.ino
sed -i 's|^#define TRIGER_MAGNITUDE (255)|#define TRIGER_MAGNITUDE (1023)|; s|^#define TRIGGER_DEADZONE_CONST 10|#define TRIGGER_DEADZONE_CONST 40|' new_ctl.ino
sed -i -E 's#^(  )(bar02_setup|BNO08x_setup|read_imu|read_bar02)\(\);#\1// BENCH: \2();#' new_ctl.ino
sed -i 's|^  // BENCH: read_imu();|  delay(5); // BENCH: stands in for read_imu(), which waits about 5 ms for IMU reports|' new_ctl.ino
grep -n "BENCH\|^  power_thrusters();\|TRIGER_MAGNITUDE (\|TRIGGER_DEADZONE_CONST " new_ctl.ino
# expect: line 28 (1023), line 30 (40), lines 265 and 266 BENCH, line 635 delay(5), line 636 BENCH, line 640 power_thrusters();
```

   **Why the `delay(5)` is required.** `PIDFS::compute` divides by `passed_time` in whole milliseconds (`new_ctl.ino:104-122`). With the sensor reads removed, `loop()` takes microseconds, so `passed_time` is 0 on most passes and (0 - 0) / 0 is NaN. Kd = 0 does not help, because 0 times NaN is NaN. The NaN reaches `thruster_power` and converts to 0, so the thrusters ignore the sticks. In the real firmware, `read_imu()` paces the loop at about 5 ms (both reports enabled at 5000 µs, `new_ctl.ino:247-256`). With `delay(5)`, `$TNCTL` arrives about 4 times a second (bench, 2026-10-21).

2. **One thruster.** Connect one ESC signal lead to HFL, pin 9, and its signal ground to a Teensy GND (`Docs/Motors/bluerobotics_basic_esc.md:44`). LY drives this channel directly.
3. **Supply.** Set the supply to 12 V, output off. The 2026-10-21 bench used a 12 V 1.5 A supply, then a 12 V 5 A supply. Main info lists what each allows. On a 1.5 A supply, use slow mode (LB) and no triggers.
4. **Thruster.** Put the thruster in water, propeller clear, hands clear.
5. **Flash.** Quit miniterm (Ctrl+]) and stop the Pi script, then flash the bench copy:

```bash
# [control Pi] port from "arduino-cli board list" (usb1/1-1 on the bench, 2026-10-21)
cd ~/bench_fw/new_ctl && arduino-cli compile -u -b teensy:avr:teensy41 -p usb1/1-1 .
```

6. **Arm.** Turn the supply output on. Listen for three rising tones, then one tone, then one more. Three tones followed by silence means no signal reaches the ESC.
7. **Drive.** Start the laptop script first; it prints "socket created", "socket bound", "listening for connection". Then, with miniterm closed, start the Pi script. One input at a time (Main info, "Bench rule"). Push LY slowly: the thruster turns, and value 5 (`hfl_pwr`) follows. Release it and the thruster stops.

```bash
# [laptop] system Python (P2); verified 2026-10-21
cd ~/rov-26 && python3 topside/modes/main.py
```

```bash
# [control Pi] after quitting miniterm; verified 2026-10-21
cd ~/rov-26 && python3 rpi/modes/main.py
```

8. **Link-loss check.** Hold LY at about 30%, then stop the laptop script with Ctrl+C. Expected: the thruster **keeps turning** (finding 5). Stop it with the supply switch. Record this. Then repeat, and unplug the Teensy's USB instead: the ESC should stop within about 0.3 s. On 2026-10-21 the ESC replayed its three startup tones when the USB cable was unplugged (Main info).
9. **Pin check, all eight pins.** Flash `esc_all.ino` (below) and move the one ESC's signal lead from pin to pin. It sends 1500 µs on pins 2 to 9. Typing a digit from 2 to 9 in miniterm spins that pin at 1600 µs for 3 s. On 2026-10-21 this verified all eight pins with one ESC and one T200.

```bash
# [control Pi] write the 18 lines below in short pastes, then check the length before compiling
mkdir -p ~/bench_fw/esc_all && cd ~/bench_fw/esc_all
wc -l esc_all.ino                                               # expect 18
arduino-cli compile -u -b teensy:avr:teensy41 -p usb1/1-1 .     # port from arduino-cli board list
python3 -m serial.tools.miniterm /dev/ttyACM0 115200            # type 2 to 9; Ctrl+] quits
```

   `~/bench_fw/esc_all/esc_all.ino`, 18 lines, verified on the bench 2026-10-21:

```cpp
// Bench test, not part of rov-26. Stop pulse on pins 2-9; press 2-9 to spin that pin forward for 3 s.
#include <Servo.h>
Servo esc[10];
void setup() {
  for (int p = 2; p <= 9; p++) { esc[p].attach(p); esc[p].writeMicroseconds(1500); }
  Serial.begin(115200);
}
void loop() {
  int c = Serial.read();
  if (c < '2' || c > '9') return;
  int p = c - '0';
  Serial.printf("pin %d forward\n", p);
  esc[p].writeMicroseconds(1600);
  delay(3000);
  esc[p].writeMicroseconds(1500);
  Serial.println("stop");
  while (Serial.available()) Serial.read();
}
```

10. **Direction.** Record each thruster's flow direction with a short, gentle stick push (the T200 doc suggests 1550 µs, `Docs/Motors/bluerobotics_t200_thruster.md:227`). Still open after 2026-10-21 (P7).
11. **All eight together.** Raise the current limit in steps toward the firmware's 20 A (ASSUMED supply capability), still one input at a time below 17 A. Not run on 2026-10-21: the bench had one ESC and one T200.

**Done when.**
- Every ESC arms: three rising tones, then one tone, then one more. Passed on 2026-10-21 with one ESC.
- `esc_all.ino` spins the thruster from each of pins 2 to 9. Passed on 2026-10-21.
- Every thruster follows its stick and stops when the stick is centered.
- The link-loss result is recorded, and the supply switch is confirmed as the stop.

**Gotchas.**
- **Ctrl+C does not stop a thruster. The supply switch does.** So does unplugging the Teensy.
- Regenerative braking on direction changes can trip a bench supply's over-current protection (`Docs/Motors/bluerobotics_basic_esc.md:210-215`).
- A signal line held high for more than 15 ms at ESC power-up puts the ESC in its bootloader (`Docs/Motors/bluerobotics_basic_esc.md:127-131`). Teensy pins are inputs until the sketch runs (`Docs/tbd/pjrc_teensy_4_1.md:138`), so power the Teensy first. On 2026-10-21 this case never occurred: the ESC played its normal startup tones at supply power-up whether or not the Teensy was plugged in.
- **The robot never moves on the bench, so RY and trigger requests saturate.**
  - RY asks for a depth rate (`tuning.ino:443-455`), and the sensor never sees one. With `new_ctl.ino`'s fixed gains, RY drives the vertical thrusters into the current cap. Because of how the cap scales, RY alone peaks at about 1730 µs near 60 percent and falls to about 1618 µs at full (COMPUTED, Main info; V11).
  - Triggers ask for a yaw rate the IMU never sees. One trigger alone peaks near 1284 µs at a 60 to 70 percent pull (COMPUTED).
  - Use one input at a time, gently.
- **Combining inputs can reach about 17 A on one thruster** (COMPUTED, Main info). Slow mode does not prevent it. Stay below a supply's limit with one input at a time.
- **The holds act with the sticks centered.**
  - The depth hold differentiates pressure over one loop (`tuning.ino:419-426`), so sensor noise or lifting the sensor moves the vertical thrusters.
  - The heading hold reacts to turning the IMU. Its error sign looks inverted, and yaw rate is read from gyro x (`tuning.ino:404, 505-508`; README Section 8, note 3).
  - Keep the sensor board still and taped down.
  - In the sensor-skip bench copy, every sensor value stays 0, so neither hold has an error to act on (from the code: `new_ctl.ino:429-435, 484-491`).
- If `read_imu()` stops getting reports, `loop()` waits forever (`tuning.ino:398-413`), while the last pulses keep going out.
- **Without the `delay(5)`, the sticks do nothing** in the sensor-skip copy: the PID divides by zero milliseconds and the NaN becomes 0 (Do, step 1).
- **A single-pin test sketch drives only the pin in its `attach()` call.** Moving the wire to another pin without re-uploading looks like a dead pin. That happened on 2026-10-21. Use `esc_all.ino`, which drives pins 2 to 9.
- **Long pastes get cut off.** Long heredocs pasted into the Pi's terminal were truncated twice on 2026-10-21. Keep pasted blocks short and check the result with `wc -l` (`esc_all.ino` is 18 lines).
- The ESC is not waterproof, and its heat spreader must be in air (`Docs/Motors/bluerobotics_basic_esc.md:48-54`). Only the thrusters go in the water.

### P7. Acceptance: The Bench Behaves Like the Team's ROV

**Goal.** Every stick, trigger and button does what the team's firmware says, with thrusters in water.

**When.** Teensy day, after P6.

**Needs.** P6 done, all eight thrusters in water, telemetry visible on the Pi or the laptop.

**Main info.**
- The rows below are read from `tuning.ino`. `new_ctl.ino` has the same mixing at the lines given in brackets.
- With `tuning.ino`, the depth and yaw response also depends on the gains in each frame (`tuning.ino:658-682`). `topside/modes/tuning.py:28-47` holds the team's last values, but they never reach the Teensy in the repo version (finding 1).
- "pwr" is µs away from 1500.
- On the 2026-10-21 bench the trio was Pair M with the P6 bench copy of `new_ctl.ino`: triggers 0 to 1023 with a dead zone of 40, and no sensors, so the two hold rows cannot be tested.
- **`$TNCTL` order.** The first eight values are offsets from 1500 µs for VFL, VFR, VBL, VBR, HFL, HFR, HBL, HBR, which are pins 8, 4, 7, 3, 9, 5, 6, 2 (`new_ctl.ino:50-57, 584-601`).

| Input | Expected | Source |
|---|---|---|
| LY push | All four horizontal outputs move together, up to 200 | `tuning.ino:482-487` (`new_ctl.ino:465-470`) |
| LX push | HFL and HBR move one way, HFR and HBL the other, up to 160 | `tuning.ino:489-494` (`new_ctl.ino:472-477`) |
| LB press | LY and LX outputs halve; press again to restore | `tuning.ino:381-396` (`new_ctl.ino:364-379`) |
| LT alone | Yaw rate request (LT - RT) / 1023 x 180 deg/s (255 in `new_ctl.ino`, 1023 in the bench copy); HFL and HBL one way, HFR and HBR the other | `tuning.ino:28, 498-499, 510-515` (`new_ctl.ino:28, 481-482, 493-498`) |
| RT alone | The opposite of LT | same |
| Triggers released, IMU turned by hand | Heading hold drives the horizontal pairs. If the sign is wrong, it pushes further away. Needs the IMU | `tuning.ino:501-508` (`new_ctl.ino:484-491`) |
| RY push | All four vertical outputs move together | `tuning.ino:443-445, 454-460` (`new_ctl.ino:426-428, 437-443`) |
| RY released, sensor raised or lowered | Depth hold drives the vertical outputs. Needs the Bar02 | `tuning.ino:446-452` (`new_ctl.ino:429-435`) |
| Sticks inside ±3000; triggers under 40 (10 in `new_ctl.ino`, 40 in the bench copy) | No response | `tuning.ino:29-30, 580-599` (`new_ctl.ino:29-30, 563-582`) |
| RX, A, B, X, Y, RB, D-pad, Start, Back, stick clicks | No thruster response | `rpi/modes/main.py:39-58`; no sketch reads them |
| Several axes at full | Outputs scale down to the 20 A estimate. Not monotonic: past about 60 percent, RY or a trigger gives less thrust (V11) | `tuning.ino:538-567` (`new_ctl.ino:521-550`) |
| Thruster lines on the laptop | The laptop prints `$RPCTL,<Teensy line>,*FF` (topside/main.py only) | `rpi/modes/main.py:75-82`, `topside/modes/main.py:24-34` |

**Do.** Run the trio as in P6, step 7: the laptop script first, then the Pi script with miniterm closed. Go down the table one row at a time, with gentle inputs, one input at a time (P6 "Bench rule"). Write down which physical direction "LY up", "LX right", "LT" and "RY up" produce. Those signs are NOT IN CODE and P9 needs them.

**Shutdown order.** Supply off first, then the Pi script, then the laptop script. Ctrl+C leaves the Teensy on its last command (bench, 2026-10-21; finding 5).

**Bench result, 2026-10-21.** Verified end to end: the controller, the laptop script, the Pi script, the Teensy running the P6 bench copy, one ESC and one T200.
- On pin 3 (VBR), the right stick moves the thruster. On pin 9 (HFL), LY, LX and the triggers move it.
- In slow mode, full LY gave 100 on values 5 to 8, and a slightly sideways stick gave 90,109,109,90.
- A real line from the bench:

```
$TNCTL,0,0,0,0,100,100,100,100,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,*FF
```

- Still open: the physical push direction for LY up, LX right, LT and RY up. The heading-hold sign cannot be tested without the IMU.

**Done when.** Every row behaves as listed, the four signs are recorded, and the heading-hold direction is recorded as "corrects" or "runs away". On 2026-10-21 the LY, LX, LB, trigger and RY rows were seen working (bench result above); the four signs, the two hold rows and the heading-hold direction are still open.

**Gotchas.**
- On the bench, rate requests saturate (see P6).
- If the heading hold runs away, release the IMU and stop with the supply switch. Report it to Victoria (V8). Their deployed sketch may already be fixed.
- Ctrl+C on either script does not stop a spinning thruster. Turn the supply off before stopping the scripts.

### P8. NEPI Joins and Watches

**Goal.** NEPI sits on the bench network and sees the same video and data the hold will use. It sends nothing.

**When.** Later. The address can be added today (P0).

**Needs.** The NEPI Pi on the hub. For Idea 1, a video copy from the control Pi. For Idea 2, a USB camera on the NEPI Pi and a separate 5 V supply for it.

**Main info.**
- **Address.** 192.168.68.23/24 through the network manager topic (P0; `network_mgr.py:322-329, 813-815`). It replaces the managed alias list (`network_mgr.py:982, 990`).
- **Idea 1, NEPI topside.** Add a second `send` output to the camera's FastMJPG command, from Pi port 5610+N to NEPI port 5600+N, with 1 send round (LINK_PROTOCOL "Video to NEPI").
  - **Send rounds.** In a simulated link test at 3% loss, 1 round delivered 12 of 60 frames and 2 rounds delivered 58 of 60 (F7). On the bench hub, 1 round is enough. If NEPI's frame rate drops on the real tether, use 2 rounds, at twice that copy's bandwidth (D5).
  - **Pilot video.** The pilot's video does not depend on NEPI being on. But a destination with no route, or a local address the Pi does not own, ends the whole sender process, pilot video included (F6).
- **NEPI side.** The `idx_net_stream` driver with `fastmjpg://192.168.68.22:5600`: the host is the sender to accept, and the port is NEPI's own (`Drivers/idx_drivers/idx_net_stream_params.yaml:26-38`). That mode is uncommitted work in this repo and has been tested only on a host computer. For a quick look without NEPI, `Nepi_Connect/tools/fastmjpg_viewer.py` receives the same stream (F8). It is untracked in this repo.
- **Idea 2, NEPI in the enclosure.** NEPI uses its own bottom-facing camera on its own USB port, so no video copy crosses the tether. The ROV's 12 V-to-5 V buck cannot supply the extra current (weekly report, 2026-10-18), so the NEPI Pi needs its own 5 V, 5 A supply. On a 3 A supply the Pi 5 limits USB devices to 600 mA in total (`Docs/tbd/raspberry_pi_5.md:15-17, 28-30`).
- **Network.** The network and software are the same for both ideas. Only power and camera wiring differ.

**Do.**

```bash
# [control Pi] Idea 1, camera 0, with the NEPI copy (LINK_PROTOCOL "Video to NEPI")
cd ~/rov-26 && ./FastMJPG/bin/FastMJPG capture /dev/video0 640 480 1 30 \
  send 192.168.68.22 5600 192.168.68.12 5600 1400 1000000 2 \
  send 192.168.68.22 5610 192.168.68.23 5600 1400 1000000 1
```

```bash
# [NEPI Pi] quick look without the driver; needs Python 3, OpenCV, numpy (fastmjpg_viewer.py header) and a display
python3 ~/mate_robotics/Nepi_Connect/tools/fastmjpg_viewer.py --port 5600     # ASSUMED path and display
```

Then add `fastmjpg://192.168.68.22:5600` to the `idx_net_stream` stream list in NEPI.

**Done when.**
- NEPI shows the camera at 25 fps or more (README Phase 1).
- The pilot's frame rate is unchanged.
- The pilot's video keeps running with the NEPI Pi switched off.
- The bandwidth of both copies is recorded.

**Gotchas.**
- Never aim the copy at an address outside 192.168.68.0/24 (F6).
- Do not run `nepi_connect_setup.sh` on the control Pi (F8).
- NEPI sends nothing in this step: no `rbx_rov_link` and no demands.

### P9. Stabilization Toggle Through the Control Pi

**Goal.** One control starts stabilization mode on the NEPI Pi. NEPI then holds position (surge, sway) and orientation (yaw, pitch, roll). Its commands reach the Teensy only through the control Pi, and the pilot can take back control at once.

**When.** Later, after P8.

**Needs.**
- P7 passed, and the trio Victoria confirms (V1).
- The team's agreement to the Pi change (README Section 9, Q7) and to a firmware change for pitch and roll (D6).
- The fake Teensy from P4.

**Main info.** The wire contract is LINK_PROTOCOL; this step does not repeat it. In outline:
- **On the control Pi: `nepi_link.py`.**
  - It is imported by the Pi script that writes the Teensy. That is `rpi/modes/main.py` in Pair M, or the team's deployed script (V1). There are about four hook lines: import, `start()`, the merge just before `nmea_encode`, and the Teensy line (README Section 3).
  - It merges NEPI's demand into a *copy* of the pilot's inputs. It applies the 250 ms timeout, the caps, the per-axis pilot override and the dead-zone mapping. It sends status to 192.168.68.23:5700 at 10 Hz, and accepts demands on port 5701 only from 192.168.68.23.
  - The trigger constants must match the deployed sketch: 40 and 1023 for `tuning.ino` (`tuning.ino:28, 30`). The smallest yaw step is then 40 / 1023 x 180, about 7.0 deg/s.
- **The toggle** (D3; recommended option).
  - It is a "Stabilize" control on NEPI's hold process, used from the topside computer's browser. That gives Victoria's "hotkey on the computer itself" for every vehicle NEPI connects to, with no change to the team's topside script.
  - NEPI's request arrives as a rising edge of the demand's `active` flag. The Pi engages only while the link is up.
  - **Pilot takeback.** Any stick or trigger outside its dead zone wins its axis on the next frame (0.1 s). RB (`BTN_TR`, slot 13, unused by both sketches, `rpi/modes/main.py:53`) is the pilot's off-latch: it drops every axis to the pilot and stays off until NEPI sends a fresh request. Assist also turns off after 2 s without a fresh demand (LINK_PROTOCOL "Pi Rules").
- **On NEPI: `rbx_rov_link`.** An RBX driver. Discovery listens on UDP 5700. It builds a navpose from telemetry depth, heading, pitch and roll, and turns teleop velocity into demands (README Section 4, item 3).
- **On NEPI: the hold process**, a ProcessIF process (README Section 4, item 5). On engage it takes a reference image. It estimates shift and rotation in each frame and sends surge, sway and yaw demands at up to 10 Hz.
- **Pitch and roll** (F12; required, not optional).
  - No controller input reaches them today. The pitch and roll controllers are declared but unused (`tuning.ino:225-240`, `new_ctl.ino:208-223`).
  - The plan: the team adds Teensy leveling that turns on with stabilization (README Section 3 sketch). NEPI reports pitch and roll, and later sends pitch and roll targets in the reserved demand fields, through an input the team agrees on (D6, V9).
- **Camera** (D4). Idea 2: the NEPI Pi's own bottom camera. Idea 1: the down camera that is fixed to the hull (V3), never the gripper camera, which moves with an actuator.

**Do (build order).**
1. Finish, commit and test `idx_net_stream`'s fastmjpg mode on the NEPI Pi with the P8 video.
2. Write `nepi_link.py`. Test it against the fake Teensy (P4) with no thrust: toggle, timeout, override and mapping, per README Phase 3.
3. Write `rbx_rov_link`. Check read-only telemetry (README Phase 2).
4. Write the hold process. Test signs, timeout and override with propellers out of the water (README Phase 3), then in water with low caps (README Phase 4).

**Done when.**
- The README Phase 3 checks pass on the bench.
- The pilot's display shows NEPI's state. This works with `topside/modes/main.py` only; `topside/modes/tuning.py` does not print.
- Every pilot override and the RB off-latch return control within one frame.

**Gotchas.**
- LINK_PROTOCOL parses only 20-value `$TNCTL` lines. `tuning.ino` sends 25 values, and `rpi/modes/tuning.py` strips the header (`rpi/modes/tuning.py:118-119`). Fix the parser before using `tuning.ino`.
- Nothing downstream of the merge times out (finding 5). A Pi crash leaves the last merged frame on the Teensy.
- Anything on the hub can pretend to be NEPI (README Section 5, rule 8). Keep the bench hub closed.

## DECISIONS NEEDED

1. **D1. Script trio and firmware for Teensy day** (P5, P6, P7).
   - (a) The sketch, Pi script and topside script the team actually runs, from Victoria. This is the only true replica, but its contents and safety behavior are unknown until we read it.
   - (b) The repo's `tuning.ino` with `power_thrusters()` uncommented. Reject: no repo Pi script sends its frame, and misaligned bytes reach the gains and outputs (finding 1).
   - (c) Fallback: Pair M with a local `new_ctl.ino` with `power_thrusters()` uncommented. It is self-consistent, but it is not the team's current firmware: triggers 0 to 255, fixed gains, slower telemetry.
   - **Recommendation:** (a) if it arrives by 2026-10-20, otherwise (c).
   - **Resolved for the bench (2026-10-21): (c).** The P6 bench copy of `new_ctl.ino` with `power_thrusters()` on, and the trigger constants set to 1023 and 40 for the bench controller. V1 is still open for the robot itself.
2. **D2. Sensors on the bench Teensy** (P5, P6).
   - Wire a Bar02 and a BNO085: the team's configuration, needed for P7.
   - Or a bench-only sketch that skips both sensors. That needs edits in `setup()`, `read_imu()` and `read_bar02()`.
   - **Recommendation:** wire the sensors.
   - **Resolved for the bench (2026-10-21): the sensor skip**, because no sensors were on the bench. The skip is two `sed` lines in the P6 bench copy, plus a `delay(5)` in place of `read_imu()` so the PID timing does not divide by zero (P6, step 1). P7's two hold rows wait for the sensors.
3. **D3. Stabilization toggle** (P9).
   - (a) RB on the controller, handled on the Pi. Small Pi change only, but tied to one controller and one free button.
   - (b) A keyboard hotkey in the team's topside script, sent as a new event code. This changes both team scripts: an unknown code crashes the unmodified Pi script at `rpi/modes/main.py:72`. It needs keyboard focus, and generalizes only to this topside script.
   - (c) A toggle in NEPI that the Pi accepts, with RB as the pilot's off-latch. No topside change, the same control for every vehicle NEPI connects to, and per-axis pilot override at all times.
   - **Recommendation:** (c).
   - **Being built as (c)** for the bench (P9). Victoria has not confirmed it yet (V10).
4. **D4. Stabilization camera** (P8, P9).
   - Idea 1: the hull-fixed down camera (V3), with the claw masked out of the image.
   - Idea 2: the NEPI Pi's own bottom camera.
   - **Recommendation:** Idea 2's camera is best for the hold. If the team picks Idea 1, use the hull-fixed down camera.
5. **D5. Send rounds for NEPI's video copy** (P8). 1 round on the bench. 2 rounds if the tether loses packets, at twice the bandwidth (F7).
6. **D6. Orientation path** (P9).
   - (a) Teensy leveling for pitch and roll, switched on with stabilization.
   - (b) NEPI pitch and roll targets through an agreed spare input.
   - Both are team firmware changes. **Recommendation:** (a) first, with (b) once (a) works.
7. **D7. Where to flash the Teensy** (P5). From the control Pi, as `upload.sh` implies, or from the laptop. **Recommendation:** the control Pi, which matches the sealed robot. **Resolved for the bench (2026-10-21): the control Pi**, with the port taken from `arduino-cli board list` rather than `upload.sh`'s `/dev/ttyACM0` (P5).

## QUESTIONS FOR VICTORIA

1. **V1.** Please send the sketch that is on the Teensy now, and the Pi and topside scripts you run with it. Does your Pi script send the 20 gains as well as the 18 inputs? Blocks P5, P6, P7, P9.
2. **V2.** What is the tether's link type and speed? We have your 60 ms latency, but not the bandwidth. Blocks P8 (send rounds, Idea 1) and P9.
3. **V3.** Which down camera is fixed to the hull: the one on the claw, or the one facing the back claw? Does the back claw move through its view? Blocks P8 and P9.
4. **V4.** Which OS and Python version does the topside computer run? Which controller model? Do its triggers report 0 to 255 or 0 to 1023? Blocks P2.
   - Our bench note, not an answer from the team: the bench controller's triggers report 0 to 1023 (P2, 2026-10-21). Your controller may differ.
5. **V5.** Which `/dev/videoN` and which UDP port is each of the four cameras, and how do you start the four streams? Blocks P3.
6. **V6.** How do you flash the Teensy: from the Pi with `upload.sh`? How did you install the MS5837 library? Blocks P5.
   - Added after the bench (2026-10-21): the repo's `MS5837/` folder does not compile, because its `MS5837.h` is the newer Blue Robotics version and its `MS5837.cpp` the older one (P5). Do you build against the library-manager "BlueRobotics MS5837 Library" instead? Also, `upload.sh`'s `-p /dev/ttyACM0` fails on a fresh Teensy; do you flash with the port from `arduino-cli board list`?
7. **V7.** What supply voltage and fuse do the thrusters run on? Blocks P6.
8. **V8.** Does your deployed sketch have an arming hold, or a timeout that stops the thrusters when frames stop? Is the heading-hold sign correct in the water? Blocks P6 and P7.
9. **V9.** Will you add pitch and roll leveling on the Teensy, and an input for pitch and roll targets? Blocks P9.
10. **V10.** For testing, is a "Stabilize" control on NEPI's page on the topside computer, with RB as the pilot's off switch, what you meant by a hotkey on the computer? Blocks P9.
11. **V11.** Is the current limiter meant to reduce thrust as a stick goes further? `limit_current()` (`new_ctl.ino:521-550`) scales pulse width linearly against a summed-amps estimate for all four thrusters in a group. So thrust is not monotonic in stick travel. COMPUTED: RY past about 60 percent gives less vertical thrust than 60 percent does, and the triggers behave the same way. The limiter also models all eight thrusters on the bench, where only one is connected. Seen on 2026-10-21: holding LY full and pushing RY past about 50 percent cut HFL from 1700 to 1651 µs, a slight slowdown. Does your deployed sketch do the same? Affects P6, P7 and P9.

## Doc Changes Pending

**LINK_PROTOCOL.md**
- `NEPI_LAN_IP: ''` becomes `192.168.68.23` (A3). The same goes for every `<NEPI_LAN_IP>` in "Video to NEPI".
- The **Toggle** rule and `NEPI_ASSIST_BUTTON` follow D3. The recommendation is a NEPI request (rising edge of `active`) with RB as the pilot's off-latch.
- `NEPI_TRIGGER_DEADZONE` and `NEPI_TRIGGER_MAX` must match the deployed sketch: 40 and 1023 for `tuning.ino`. Recompute the mapping table and the 7.1 deg/s figure from those values.
- The Teensy telemetry parser must accept `tuning.ino`'s 25-value lines, including `millis()`, and header-stripped lines from `rpi/modes/tuning.py`. Add a second field table.
- Demand: "the Pi's 0.1 s serial write (`rpi/modes/main.py:84-88`)" becomes "the deployed Pi script's serial write", which is 0.1 s in both repo scripts.
- Video: record the send-rounds result (F7) and the option of 2 rounds (D5).
- Terms: "the topside NEPI computer" should cover Idea 2, where NEPI is inside the enclosure on the same hub.

**README.md** (sections other than the open questions, which this commit updates)
- Section 1: the deployed firmware is `tuning.ino`, newer than the repo (A1). The repo's `tuning.ino` does not match either repo Pi script (finding 1).
- Section 2: the diagram, the systemd excerpt and "Network" use 192.168.68.23. Add the Idea 1 and Idea 2 layouts.
- Section 3: the hook goes into the deployed Pi script, not necessarily `rpi/modes/main.py`. "The assist button" follows D3. "Heading and the 7 deg/s step" uses the deployed sketch's trigger constants.
- Section 3 "Optional: pitch and roll leveling" and Section 6 Phase 5: pitch and roll are part of stabilization (F12), not optional.
- Section 4, item 3: axis controls include pitch and roll.
- Section 5, rule 5 and Section 6, Phase 3: update for the D3 toggle.
- Section 7: point to BENCH_SETUP_PLAN.md.
- Section 8, note 1: the firmware is answered (A1). The Pi script is still open (V1).
