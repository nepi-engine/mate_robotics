# System Guide: How NEPI Connects to Your ROV and Who Controls It

2026-11-02, from the NEPI team. Start at [README.md](README.md) if you have not read it yet.

## Summary

- NEPI joins your topside Ethernet switch as a fourth computer. It never talks to the Teensy. Your ROV Pi script stays the only program that writes the serial port, and your Teensy sketch keeps all mixing, depth hold, heading hold and current limiting.
- One extra Python file on the ROV Pi, `nepi_link.py`, sends NEPI a status message ten times a second and receives NEPI's demands. While the pilot has handed the ROV over with RB, it writes those demands into a copy of the controller inputs, just before your script encodes them for the Teensy.
- This file covers the connections, the handover rules and their timing, the stick mapping and the two messages. The code is the authority for every rule and number here: [nepi_link.py](../Nepi_Connect/pi/nepi_link.py) on the ROV Pi, and [rbx_mate_rov_link_node.py](../Drivers/rbx_drivers/rbx_mate_rov_link_node.py) on NEPI.

Citations of your code look like `rov-26 rpi/modes/main.py:101` and point to github.com/CoralCrusaders/rov-26 at commit `1af4984`, the newest version we have seen. Your current code may differ, so read each one as "rov-26 does this; check whether your current code still does".

## Terms

- **NEPI**: the edge-computing software that runs on your NEPI computer (a Raspberry Pi 5 with a Hailo-8). It is built on ROS (Robot Operating System); a **node** is one running ROS program, and a **topic** is a named channel that nodes publish messages on.
- **Driver**: the part of NEPI that connects one device to NEPI. **Discovery** is the driver step that finds a device, here by listening for one valid status message, and then starts the driver's node.
- **RBX**: NEPI's driver type for robots. Our RBX driver, `rbx_mate_rov_link`, presents your ROV to NEPI as a robot. **Teleop** commands are remote-driving commands sent to an RBX robot.
- **IDX**: NEPI's driver type for cameras.
- **NavPose**: NEPI's bundle of navigation and pose data: position, depth, heading, pitch, roll and their rates.
- **RUI**: NEPI's browser-based user interface, served by the NEPI computer.
- **nepi_link**: `nepi_link.py`, the bridge module on the ROV Pi. Our older docs also call it "the bridge".
- **NEPI control**: the mode in which the ROV follows NEPI's demands. Older docs call it **assist**, and the status message still carries it as the `assist` field. **Pilot mode** is everything else.
- **Status, demand**: the two link messages. The status goes from the ROV Pi to NEPI; the demand goes from NEPI to the ROV Pi.
- **UDP, datagram**: UDP sends one short message, a datagram, with no connection and no resending. A datagram can be lost or arrive out of order. **JSON** is the plain-text format the messages use, such as `{"surge": 0.1}`.
- **Sequence number (`seq`)**: a counter the sender raises by 1 on every message, so the receiver can drop old or repeated ones.
- **Dead zone**: the range around a stick's or trigger's center that the sketch treats as zero.
- **Surge, sway, heave, yaw**: forward or back, sideways, down or up, and turning left or right. **Heading** is the yaw angle.
- **Takeover point**: how far the pilot must move a stick or trigger to end NEPI control at once (25% of its travel by default).

## Machines, Addresses and Ports

| Machine | Address | What runs there | Listens on |
|---|---|---|---|
| Topside laptop | 192.168.68.12 | `topside/modes/main.py` with the controller, and your video viewers | TCP 10110, one connection (`rov-26 topside/modes/main.py:9-10, 16-20`); video on UDP 5600 to 5604 (`rov-26 config.yaml:4-8`) |
| ROV Pi | 192.168.68.22 | `rpi/modes/main.py` with `nepi_link.py` beside it, and FastMJPG | UDP 5701 for demands (`NEPI_DEMAND_UDP_PORT`) |
| NEPI | 192.168.68.103 | NEPI, the `rbx_mate_rov_link` driver, optionally the `idx_net_stream` camera driver | UDP 5700 for status (the driver's `status_port`); the RUI in a browser; video ports per [VIDEO.md](VIDEO.md) |
| Teensy 4.1 | USB `/dev/ttyACM0` on the ROV Pi | Your sketch | Serial, 115200 (`rov-26 rpi/modes/main.py:13, 23`) |

The ROV Pi and topside addresses are the ones in `rov-26 config.yaml:1-2` and in your scripts. The NEPI address is the one your own scripts use (`Apps/nepi_app_robot_stab/README.md`). Both link ports are free in rov-26: nothing there uses 5700 or 5701.

The addresses must match in two places. On the ROV Pi, `NEPI_LAN_IP` names NEPI, and nepi_link accepts demands only from that address. On NEPI, the driver's `host` option names the ROV Pi, and the driver accepts status only from that address and sends demands only to it. [PI_INTEGRATION.md](PI_INTEGRATION.md) covers the first, [NEPI_SETUP.md](NEPI_SETUP.md) the second.

## Data Paths

```
                        TOPSIDE ETHERNET SWITCH
      +----------------------------+------------------------------------+
      |                            |                                    |
 TOPSIDE 192.168.68.12             |                          NEPI 192.168.68.103
 controller on USB                 |                          rbx_mate_rov_link driver
 topside/modes/main.py           TETHER                        :5700 <- status, every 0.1 s
  TCP server :10110 <---------+    |                           demand -> ROV Pi :5701, every 0.1 s
  changed events ---------+   |    |                             (only while status arrives)
  prints the Pi's line <--|---|-+  |                           RUI: Robots page, NavPose
                          |   |  | |                           video in: see VIDEO.md
                          v   |  | |
 ROV PI 192.168.68.22 ---------------------------------------------------------------
  rpi/modes/main.py: TCP client, holds all 18 inputs (gamepad_inputs)
   main loop: monitor_socket_input() -> nepi_link.merge_inputs() -> nmea_encode()
   serial thread: newest $RPCTL frame to the Teensy every 0.1 s
   topside thread: $RPCTL,<newest Teensy line>,NEPI:<tag>,*FF every 0.2 s
  nepi_link.py: status thread -> NEPI :5700 every 0.1 s
                demand thread <- NEPI on :5701
  FastMJPG: camera video, see VIDEO.md
        |  USB /dev/ttyACM0
        |  $RPCTL frame down, $TNCTL telemetry lines back
        v
 TEENSY 4.1 (your sketch): dead zones, slow mode, mixing, depth hold,
        |                  heading hold, current limit
        v
 8 x ESC --> 8 x T200
```

Sources: `rov-26 topside/modes/main.py:14-52`, `rov-26 rpi/modes/main.py:20, 60-108`, and the threads in `NepiLink.startLink`, `runStatusThread` and `runDemandThread` in [nepi_link.py](../Nepi_Connect/pi/nepi_link.py).

Each path in words:

- **Controller to Teensy.** Your topside script sends only the inputs that changed. Your Pi script keeps the full 18-input state, and its main loop encodes it on every pass. With nepi_link added, the main loop hands `nepi_link.merge_inputs(gamepad_inputs)` to the encoder instead of `tuple(gamepad_inputs)`. In pilot mode that returns the same tuple, unchanged. Your serial thread writes the newest frame every 0.1 s, as before.
- **Status, ROV Pi to NEPI.** nepi_link's own thread sends one status every 0.1 s, whether or not a new Teensy line has arrived, plus one at once whenever NEPI control turns on or off. It carries the handover state, what the Teensy was last sent, and the newest Teensy telemetry line.
- **Demands, NEPI to ROV Pi.** The driver sends one demand every 0.1 s while status is arriving, and nothing while it is not.
- **Topside display.** nepi_link adds a short tag such as `,NEPI:ON` to the line your Pi script already sends the topside, so the pilot sees the handover state.
- **Video.** Two ways to get camera video into NEPI are in [VIDEO.md](VIDEO.md).

## Control Modes and Handover Rules

These are in the order a pilot meets them. Every number is a constant in [nepi_link.py](../Nepi_Connect/pi/nepi_link.py) or in `MateRovLink` in [rbx_mate_rov_link_node.py](../Drivers/rbx_drivers/rbx_mate_rov_link_node.py), named in the right-hand column.

### 1. Start: pilot mode

nepi_link always starts in pilot mode (`assist_reason` `start`). With no config file, or an empty `NEPI_LAN_IP`, nepi_link opens no sockets and starts no threads, and your script behaves exactly as it does without it.

In pilot mode every input passes through untouched, and NEPI's demands are read but never used. The topside tag reads `NEPI:NOTREADY` until NEPI is ready, then `NEPI:OFF`.

### 2. NEPI becomes ready

| Rule | Value | Where |
|---|---|---|
| The driver counts the link up while a valid status from the ROV Pi arrived within this time | 0.5 s | `MateRovLink.LINK_TIMEOUT_SEC` |
| While the link is up, the driver sends a demand with `active` true this often | 0.1 s | `MateRovLink.DEMAND_PERIOD_SEC` |
| nepi_link counts NEPI ready while an `active` true demand was accepted within this time | 0.5 s | `NEPI_READY_SEC` |

So NEPI is ready whenever the driver is running and hearing your status. No stabilization process has to run for that. With no teleop command, every demand is neutral.

### 3. The pilot presses RB

RB hands the ROV to NEPI only if both of these hold at the moment of the press:

- NEPI is ready (tag `NEPI:OFF`). Otherwise nepi_link ignores the press and prints `RB ignored: NEPI is not connected`.
- No movement input (LY, LX, RY, LT or RT) is at or past the takeover point. Otherwise nepi_link ignores the press and prints `RB ignored: release LY first`, naming the input.

Only a fresh press counts: RB must be released before the next press does anything, the same "button available" rule your sketch uses for LB (`rov-26 mcu/modes/new_ctl/new_ctl.ino:364-379`). The code is `updatePilotLocked` and `toggleAssistLocked`. NEPI can never start NEPI control itself. RB still reaches the Teensy unchanged; rov-26 does not read it.

### 4. NEPI control

The ROV follows NEPI's newest demand ([What NEPI owns](#what-nepi-owns-during-nepi-control) below). The tag reads `NEPI:ON` while the newest demand is fresh and `NEPI:WAIT` while it is stale.

| Rule | Value | Where |
|---|---|---|
| A demand older than this counts as neutral on every movement axis | 0.25 s | `DEMAND_STALE_SEC` |
| On NEPI, a teleop command older than this sends neutral | 0.5 s | `MateRovLink.TELEOP_CMD_TIMEOUT_SEC` |
| Each axis is capped on the ROV Pi | `NEPI_MAX_*` keys | `mergeLocked`, [PI_INTEGRATION.md](PI_INTEGRATION.md#config-keys) |

With no command, the driver keeps sending neutral demands with `active` true. NEPI control then continues with every movement input at 0, and the Teensy's own depth and heading holds keep the ROV where it is, the same as when the pilot lets go of the sticks. It continues until one of the exits below happens. A stopped stabilization process therefore does not hand the ROV back by itself.

While NEPI control is on, the pilot's sticks and triggers short of the takeover point do nothing, because nepi_link replaces them. This holds in `NEPI:WAIT` too.

### 5. Ending NEPI control

Any one of these ends NEPI control. After it, the pilot's inputs pass through on the same pass of your main loop.

| Exit | Who | When it acts | `assist_reason` | Where |
|---|---|---|---|---|
| RB pressed again | Pilot | At once | `pilot_button` | `toggleAssistLocked` |
| LY, LX or RY at or past 25% of full stick (8192 of 32767), or LT or RT at or past 25% of `NEPI_TRIGGER_MAX` (256 of 1023) | Pilot | At once; that input passes through on the same pass | `pilot_stick` | `takeoverInputLocked`, `NEPI_TAKEOVER_FRACTION` |
| A demand with `active` false | NEPI: Set State PILOT on the RUI Robots page, or the driver shutting down | At once, on arrival | `nepi_off` | `handleDemandLocked`; `release_to_pilot` and `cleanup_actions` in the node |
| No accepted demand for 2 s | Nobody: the network, the driver or NEPI stopped | 2 s after the last accepted demand | `timeout` | `ASSIST_TIMEOUT_SEC`, `checkTimeoutLocked` |

The takeover numbers are `ceil(NEPI_TAKEOVER_FRACTION x full travel)`, which `validate_config` works out once at start and prints. The trigger number assumes `NEPI_TRIGGER_MAX` is 1023, as in [nepi_link_team.yaml](nepi_link_team.yaml).

If the link fails during NEPI control, the rules act in this order, counting from the last demand that arrived:

| After | What happens | Tag |
|---|---|---|
| 0.25 s | Every movement input NEPI owns goes to 0 | `NEPI:WAIT` |
| 0.5 s | NEPI no longer counts as ready, so a later RB press does nothing | `NEPI:WAIT` |
| 2 s | NEPI control ends; the pilot's inputs pass through | `NEPI:NOTREADY` |

Between 0.25 s and 2 s the pilot can take the ROV back at once with RB or by pushing a stick or trigger past the takeover point.

### What the pilot sees

| Tag | NEPI control | Meaning | RB does |
|---|---|---|---|
| `NEPI:NOTREADY` | Off | NEPI is not connected or not sending | Nothing |
| `NEPI:OFF` | Off | NEPI is ready | Hands the ROV to NEPI |
| `NEPI:ON` | On | The newest demand is fresh | Takes the ROV back |
| `NEPI:WAIT` | On | The newest demand is older than 0.25 s; the movement inputs are at 0 | Takes the ROV back |

`topside_tag` returns the tag with a leading comma, and an empty string while the link is off, so the line to the topside does not change then. nepi_link also prints one line on the ROV Pi for each handover event, at most once per 10 s per kind (`ERROR_PRINT_SEC`).

### What NEPI sees

The driver shows the ROV's state on the RUI Robots page as `PILOT` or `NEPI` (`RBX_STATES`). The state is NEPI while the link is up and the newest status has `assist` true. The driver accepts teleop commands only in that state, and only while its `teleop_movement_enabled` setting is `TRUE` (`teleopControlsReady`). Setting the state to NEPI from the RUI is refused, since only RB hands the ROV over. Setting it to PILOT hands the ROV back. The RBX Stop button clears the stored command, so demands go neutral while NEPI keeps the ROV (`go_stop`). [NEPI_SETUP.md](NEPI_SETUP.md) covers these controls on the RUI.

## What NEPI Owns During NEPI Control

| Input | Pilot mode | NEPI control |
|---|---|---|
| LY (surge), LX (sway), RY (heave) | Pilot | NEPI's demand, or 0 while the demand is stale or that axis is 0. The pilot's own value is ignored until it reaches the takeover point. |
| LT, RT (yaw) | Pilot | NEPI drives one trigger at a time; the other is 0. |
| LB (slow mode), RB (NEPI control) | Pilot | Pilot, passed through. Slow mode still halves surge and sway power on the Teensy, NEPI's included (`rov-26 new_ctl.ino:466-476`). |
| RX, D-pad, A, B, X, Y, stick buttons, Start, Back | Pilot | Pilot, passed through, so the pilot keeps the claw and every tool |
| An input named as a NEPI servo (optional, not in the team config) | Pilot | NEPI's servo value, unless the pilot holds that input off neutral ([NEPI_SETUP.md](NEPI_SETUP.md#optional-servos)) |

With heave at 0, RY is 0, so the Teensy captures the depth at that moment and holds it (`rov-26 new_ctl.ino:426-435`). With yaw at 0, both triggers are 0, so the Teensy captures the heading and holds it (`rov-26 new_ctl.ino:484-491`). Those holds are your firmware's. [SAFETY.md](SAFETY.md) lists what in rov-26 affects them.

## Demand-to-Stick Mapping

### The chain

1. **NEPI command.** A teleop command reaches the driver as ratios from -1 to 1 in ROS axes: x forward, y left, z up, and yaw counter-clockwise seen from above.
2. **Protocol axes.** `MateRovLink.set_teleop_velocity` turns them into the demand's axes, and this is the only place that happens: surge = x, sway = -y (positive right), heave = -z (positive down), yaw = -yaw rate (positive clockwise seen from above). It clamps each to -1 to 1 and applies no caps or signs.
3. **Cap.** On the ROV Pi, `mergeLocked` limits each axis to its `NEPI_MAX_*` key (defaults: surge, sway and heave 0.2, yaw 0.1).
4. **Sign.** It then multiplies by the axis's `NEPI_*_SIGN` key, 1 or -1, which makes "positive" match your controller and wiring.
5. **Stick or trigger value.** `stick_value` or `trigger_value` maps the result onto the part of the input's range that the Teensy acts on, because the Teensy zeroes anything inside its dead zone and passes the rest through unscaled (`rov-26 new_ctl.ino:563-582`):

```
stick(d)   = 0                                                         if |d| < 0.01
           = sign(d) x round(3000 + (|d| - 0.01) / 0.99 x (32767 - 3000))  otherwise
trigger(m) = 0                                                         if m < 0.01
           = round(TD + (m - 0.01) / 0.99 x (TM - TD))                     otherwise
```

3000 and 32767 are `STICK_DEADZONE` and `STICK_MAX`, copied from `JOYSTICK_DEADZONE_CONST` and `JOYSTICK_MAGNITUDE` (`rov-26 new_ctl.ino:27, 29`). TD and TM are the config keys `NEPI_TRIGGER_DEADZONE` and `NEPI_TRIGGER_MAX`, which must equal your sketch's `TRIGGER_DEADZONE_CONST` and `TRIGER_MAGNITUDE`. A positive yaw goes to LT, a negative one to RT.

### One worked example per axis

Each example uses the values in [nepi_link_team.yaml](nepi_link_team.yaml): default caps, `NEPI_SURGE_SIGN` -1 and the other signs 1, and triggers 40 and 1023. The Teensy column is rov-26 `new_ctl.ino` with `TRIGER_MAGNITUDE` 1023, and assumes slow mode is off. Its power is µs away from 1500 µs, as in `$TNCTL` columns 1 to 8 (`THRUSTER_POWER`, `rov-26 new_ctl.ino:33`).

| Axis | Teleop command (ROS) | Demand | After cap and sign | Input sent | What rov-26 does with it |
|---|---|---|---|---|---|
| Surge | x = 0.1 (forward) | surge 0.1 | 0.1 x -1 = -0.1 | LY = -5706 | -5706 / 32767 x 0.5 (`FOR_GAIN`) x 400 = -34 on each horizontal thruster, -17 in slow mode, the same sign as the pilot's forward stick (`new_ctl.ino:465-470`) |
| Sway | y = -0.1 (right) | sway 0.1 | 0.1 | LX = 5706 | 5706 / 32767 x 0.4 (`LAT_GAIN`) x 400 = 27 per horizontal thruster, signs crossed for sideways thrust, 13 in slow mode (`new_ctl.ino:472-477`) |
| Heave | z = -0.1 (down) | heave 0.1 | 0.1 | RY = 5706 | A depth-rate request of 5706 / 32767 = 0.17 m/s, which the depth-rate loop turns into vertical power (`new_ctl.ino:426-438`); in air, about 69 per vertical thruster |
| Yaw | yaw rate = -0.1 (clockwise) | yaw 0.1 | 0.1 | LT = 129, RT = 0 | Intended: 129 / 1023 x 180 = 22.7 deg/s, a yaw-rate request (`new_ctl.ino:481-482`). rov-26 actually asks for 23220 deg/s here: [SAFETY.md](SAFETY.md#rov-26-findings), finding 1. |

A cap of 0.2 gives at most 8713 on a stick, and a yaw cap of 0.1 gives at most 129 on a trigger. The surge numbers were seen on our bench (an LY of 5706 gave 34 in `$TNCTL` columns 5 to 8, and 17 in slow mode), and so was the yaw target. The sway and heave rows are computed from rov-26.

Which physical direction each sign gives is not known until you measure it on your ROV: [TEST_PLAN.md](TEST_PLAN.md#measuring-the-signs-and-the-imu-mount). The ESC deadband decides the smallest demand that turns a thruster at all: [SAFETY.md](SAFETY.md#the-esc-deadband-and-the-smallest-useful-demand).

## Messages

Both messages are one JSON object per UDP datagram, UTF-8, at most 1200 bytes (`MAX_DATAGRAM_BYTES`). Both receivers drop a datagram, count it and keep their last good values when it is too big, is not JSON, has the wrong `v` or `type`, or has a field of the wrong type, out of range, NaN or infinite. Unknown extra fields are ignored. A receiver drops a datagram whose `seq` is not above the last accepted one, unless the last accepted one is more than 1 s old (`SEQ_RESTART_SEC`), because the sender has probably restarted. nepi_link accepts demands only from `NEPI_LAN_IP`; the driver accepts status only from its `host`. Our older [LINK_PROTOCOL.md](../Nepi_Connect/LINK_PROTOCOL.md) "Rules for Every Message" states the same rules.

### Status: ROV Pi to NEPI

Built by `NepiLink.buildStatusLocked` and checked by `MateRovLink.parseStatus`.

| Field | Type | Meaning |
|---|---|---|
| `v`, `type` | 1, `"status"` | Protocol version and message type |
| `seq` | integer, 0 to 4294967295 | Starts at 0 when nepi_link starts, up by 1 per status |
| `t` | number | ROV Pi wall clock, seconds since 1970, for logs only. The clocks are not synchronized; no rule uses `t`. |
| `assist` | bool | NEPI control is on |
| `assist_reason` | string | Why it last changed: `start`, `pilot_button`, `pilot_stick`, `nepi_off` or `timeout` |
| `nepi_ready` | bool | An `active` true demand was accepted within 0.5 s |
| `pilot_surge`, `pilot_sway`, `pilot_depth` | bool | The pilot's LY, LX or RY is at least 3000 from center |
| `pilot_yaw` | bool | The pilot's LT or RT is at least `NEPI_TRIGGER_DEADZONE` |
| `slow` | bool | nepi_link's copy of slow mode, from LB with the Teensy's rule. The Teensy does not report it, so the copy can be wrong after a Teensy reset. |
| `out_lx`, `out_ly`, `out_ry` | integer, -32768 to 32767 | The values last handed to your encoder, after the merge |
| `out_lt`, `out_rt` | integer, 0 to `NEPI_TRIGGER_MAX` | The same for the triggers |
| `demand_seq` | integer | `seq` of the last accepted demand, -1 before the first |
| `demand_age_ms` | integer | Its age on the ROV Pi clock, -1 before the first |
| `demand_bad` | integer | Demands dropped since start |
| `tn` | object or null | The newest valid Teensy line by name (next table), null before the first |
| `tn_age_ms` | integer | Its age, from when nepi_link parsed it, -1 before the first |
| `tn_count`, `tn_bad` | integer | Teensy lines parsed, and lines ignored, since start |
| `servos` | object | Each configured NEPI servo by name: true while the pilot holds its input. `{}` with none configured, as in the team config. |

### Teensy telemetry (`tn`)

`parse_teensy_line` accepts only a line that is `$TNCTL`, then exactly 20 comma-separated values, then a `*XX` field with two hex digits. The first 8 values must be integers and the other 12 decimals with at most 6 digits after the point. Anything else, including Arduino's `nan`, `inf` or `ovf`, adds 1 to `tn_bad`, and the line never reaches NEPI. The names are `TN_NAMES`, copied from rov-26's print order (`new_ctl.ino:584-629`):

| # | Name | rov-26 value |
|---|---|---|
| 1-4 | `vfl_pwr`, `vfr_pwr`, `vbl_pwr`, `vbr_pwr` | Vertical thruster power, µs from 1500 |
| 5-8 | `hfl_pwr`, `hfr_pwr`, `hbl_pwr`, `hbr_pwr` | Horizontal thruster power, µs from 1500 |
| 9, 10 | `depth_m`, `depth_m_s` | Depth, m, and its rate, m/s |
| 11-13 | `yaw_deg`, `pitch_deg`, `roll_deg` | The IMU board's angles, from the sketch's quaternion conversion |
| 14-16 | `yaw_deg_s`, `pitch_deg_s`, `roll_deg_s` | Gyro X, Y and Z, deg/s (`new_ctl.ino:387-389`) |
| 17, 18 | `depth_rate_target_m_s`, `depth_pwr_out` | Depth controller target and output |
| 19, 20 | `yaw_rate_target_deg_s`, `yaw_pwr_out` | Yaw controller target and output |

### Demand: NEPI to ROV Pi

Built by `MateRovLink.nextDemandLocked` and checked by `parse_demand` and `NepiLink.handleDemandLocked`.

| Field | Type | Meaning |
|---|---|---|
| `v`, `type` | 1, `"demand"` | Protocol version and message type |
| `seq` | integer | Starts at the NEPI wall clock x 100 (`SEQ_PER_SEC`), so a restarted driver is never dropped as old; up by 1 per demand |
| `t` | number | NEPI wall clock, for logs only |
| `active` | bool | True: NEPI is ready. False, sent once on a handback: end NEPI control (`nepi_off`). It never turns NEPI control on. |
| `surge`, `sway`, `heave`, `yaw` | number, -1 to 1 | Protocol axes: forward, right, down, clockwise seen from above. 0 unless NEPI control is on and a teleop command newer than 0.5 s is stored. A demand without `heave` is accepted as heave 0. |
| `pitch`, `roll` | number, -1 to 1 | Always 0. rov-26 has no input for them, so nepi_link ignores them. |
| `servos` | object | Servo name to value, -1 to 1, rounded to 4 decimals. Empty unless servos are configured and NEPI control is on. |

The driver sends nothing while it hears no status, and starts again within one tick of the next valid status.

## What NEPI Needs From Teensy Telemetry

The driver turns the newest `tn` into NEPI's NavPose for the ROV (`MateRovLinkNode.getNavPoseCb`). It reads 8 of the 20 values, `MateRovLink.TN_VALUES`: depth, depth rate, the three angles and the three gyro rates.

- **Format.** The line format above, exactly. If your sketch prints more or fewer values, for example the PHT readings, every line is dropped. `TN_NAMES` in `nepi_link.py` and `TN_VALUES` in the driver node then have to change together. We can make that change for you if you tell us only the new column order.
- **Depth.** `depth_m` in meters, positive down. rov-26 sets the Bar02 model and fresh water (`new_ctl.ino:232-233`), which matches your answer.
- **Orientation.** NEPI expects the ROV's own angles. With your IMU on its side, the printed angles are the board's, so the driver's `imu_mount` setting turns them, and the three gyro rates, into the ROV's frame ([NEPI_SETUP.md](NEPI_SETUP.md#choosing-imu_mount)). The rate columns are read as gyro X, Y and Z whatever their names say.
- **Age.** The NavPose skips telemetry older than 0.5 s (`TN_MAX_AGE_MS`). After that the driver keeps publishing the last NavPose with its old timestamp, so code that reads depth should check the timestamp, or `tn_age_ms` on `link_status`, before trusting it.
- **Rate.** A position or heading hold needs fresh readings often. By our estimate, a stabilization loop needs at least 20 new readings a second, with a 95th-percentile age of 100 ms or less ([DRIVER_HANDOFF.md](../Nepi_Connect/DRIVER_HANDOFF.md) "Targets"; these are targets we proposed, not measured needs). Two things stand in the way today:
  - rov-26 prints one line every 50 passes through `loop()` (`SERIAL_TRANSMISSION_WRAP`, `new_ctl.ino:14, 641-644`), and the line's speed depends on your depth-sensor library ([SAFETY.md](SAFETY.md#rov-26-findings), finding 8).
  - nepi_link puts only the newest line in each status, so at most 10 new lines a second reach NEPI (`STATUS_PERIOD_SEC`). Raising that is ours to change in `nepi_link.py`.
- **Resolution.** rov-26 prints decimals with Arduino's default 2 digits, so depth arrives in 1 cm steps. The firmware suggestions in [DRIVER_HANDOFF.md](../Nepi_Connect/DRIVER_HANDOFF.md) "Problems to fix", rows 1, 2 and 6, are for you to take or leave: a fixed print timer, 4 decimals, and the Teensy's `millis()` in the line.
