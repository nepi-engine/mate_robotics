# NEPI Link Protocol, Version 1

## Summary

This is the wire contract between the Coral Crusaders' ROV Pi and a NEPI computer on the same network. It has three parts: a status message from the Pi to NEPI, a demand message from NEPI to the Pi, and a second copy of the camera video. [README.md](README.md) explains why the link is built this way and how to test it. This file covers only what each side must implement.

Code references point to the team's repository (github.com/CoralCrusaders/rov-26) at commit `1af4984`, unless they name a NEPI repository.

## Terms

- **ROV Pi, Teensy**: the Raspberry Pi inside the robot, and the microcontroller it talks to over USB serial. The Teensy reads the sensors and drives the thrusters.
- **NEPI, driver**: NEPI is the edge-computing software on the topside NEPI computer. A NEPI driver is the part that talks to one device, here the ROV.
- **FastMJPG, JPEG**: FastMJPG is the team's video tool. It sends video as a series of JPEG pictures over its own UDP format.
- **Hz**: times per second. **UTF-8**: the standard way to store text as bytes.
- **Slow mode**: the team's LB-button toggle that halves stick power on the Teensy.
- **UDP**: a way to send one short message, called a *datagram*, across a network. It is fast and needs no connection. A datagram can be lost or arrive out of order, and nothing resends it.
- **JSON**: a plain-text format for named values, such as `{"surge": 0.1}`.
- **Sequence number (`seq`)**: a counter the sender increases by 1 on every message. The receiver uses it to spot old or repeated messages.
- **Timeout**: a rule that treats data as missing once it is older than a set age. A *watchdog* is the code that enforces it. Here, a timeout hands the robot back to the pilot alone.
- **Dead zone**: the range around the center of a stick or trigger that the firmware treats as zero, so a resting stick does nothing.
- **Surge, sway**: moving forward or back (surge), and sideways (sway).
- **Yaw, pitch, roll**: turning left or right (yaw; its angle is the *heading*), tipping nose up or down (pitch), and tilting side to side (roll).
- **Assist**: NEPI's position and heading hold. The pilot turns it on and off with one button.
- **Bridge**: the `nepi_link.py` module on the Pi, which implements the Pi side of this document.

## Addresses and Ports

| What | Protocol | From | To | Rate |
|---|---|---|---|---|
| Status | UDP, JSON | ROV Pi, any port | `NEPI_LAN_IP`, port 5700 | 10 Hz, always |
| Demand | UDP, JSON | NEPI, any port | ROV Pi `192.168.68.22`, port 5701 | 10 Hz while assist is on |
| Video, camera N | UDP, FastMJPG | ROV Pi, port 5610+N | `NEPI_LAN_IP`, port 5600+N | camera frame rate |

The team's ports are UDP 5600 to 5604 for video and TCP 10110 for control (`config.yaml:4-10`). Ports 5700, 5701 and 5610 to 5614 appear nowhere in the team's code. Video to NEPI reuses the numbers 5600+N because it goes to a different address.

## Pi Configuration

The bridge reads these new keys from the team's `config.yaml`. No script reads that file today; each one hardcodes its addresses (`rpi/modes/main.py:10-11`). So the bridge opens the file itself. If `NEPI_LAN_IP` is missing or empty, the bridge does nothing, and the robot behaves exactly as it does today.

```yaml
NEPI_LAN_IP: ''             # a free address the team chooses; empty = link off
NEPI_STATUS_UDP_PORT: 5700
NEPI_DEMAND_UDP_PORT: 5701
NEPI_ASSIST_BUTTON: BTN_TR  # RB, pending team confirmation
NEPI_MAX_SURGE: 0.2         # Pi-side cap on each demand, 0 to 1
NEPI_MAX_SWAY: 0.2
NEPI_MAX_YAW: 0.1
NEPI_SURGE_SIGN: 1          # +1 or -1, measured on the bench (README, Phase 3)
NEPI_SWAY_SIGN: 1
NEPI_YAW_SIGN: 1
NEPI_TRIGGER_DEADZONE: 10   # must equal TRIGGER_DEADZONE_CONST in the deployed sketch
NEPI_TRIGGER_MAX: 255       # must equal TRIGER_MAGNITUDE in the deployed sketch
```

The stick dead zone is 3000 and full stick is 32767 in both sketches (`mcu/modes/new_ctl/new_ctl.ino:27, 29`). The trigger defaults above come from `new_ctl.ino:28, 30`. The tuning sketch uses 40 and 1023 instead (`mcu/modes/tuning/tuning.ino:28, 30`).

## Rules for Every Message

1. **Format.** One JSON object per UDP datagram, as UTF-8 text, at most 1200 bytes, so it always fits in one network frame.
2. **Common fields.** Every message carries these:

| Field | Type, range | Meaning |
|---|---|---|
| `v` | integer | Protocol version. This document defines `1`. |
| `type` | string | `"status"` or `"demand"`. |
| `seq` | integer, 0 to 4294967295 | Starts at 0 when the sender starts and goes up by 1 per message. |
| `t` | number, seconds | Sender's clock, in seconds since 1970-01-01 UTC. It is for logs and latency checks only. The two clocks are not synchronized, so no rule below uses `t`. |

3. **Malformed datagram.** The receiver drops the whole datagram, counts it, and keeps its last good values when the datagram is not valid JSON, `v` is not 1, `type` is wrong, a field below is missing or has the wrong type, or a number is NaN (not a number), infinite or outside its stated range. A bad datagram must never crash or stop the receiver. Unknown extra fields are ignored, so version 1 can gain fields without breaking either side. Renaming, removing, or changing the meaning of a field requires `v: 2`.
4. **Out of order.** The receiver drops a datagram whose `seq` is less than or equal to the last accepted `seq`. The exception: if the last accepted datagram arrived more than 1 s ago, any `seq` is accepted, because the sender has probably restarted. At 10 Hz, `seq` would take over 13 years to wrap, so wrapping needs no other rule.
5. **Source check.** The Pi accepts demands only from `NEPI_LAN_IP`. NEPI accepts status only from `192.168.68.22`.
6. **Ages.** Each receiver measures age on its own clock, from the moment it accepted the datagram.

## Status: Pi to NEPI

The bridge sends a status every 100 ms on its own timer, whether or not a new Teensy line has arrived. It may send one extra status immediately when assist changes. NEPI therefore sees a toggle within about 100 ms.

| Field | Type, range | Meaning |
|---|---|---|
| `assist` | bool | Assist is on. The toggle state lives on the Pi. |
| `pilot_surge` | bool | The pilot's LY is outside the stick dead zone. |
| `pilot_sway` | bool | The pilot's LX is outside the stick dead zone. |
| `pilot_yaw` | bool | The pilot's LT or RT is outside the trigger dead zone. |
| `pilot_depth` | bool | The pilot's RY is outside the stick dead zone. |
| `slow` | bool | The bridge's copy of slow mode, using the Teensy's LB rule (`new_ctl.ino:364-379`). The Teensy does not report slow mode, so this copy can be wrong after a Teensy reset. |
| `out_lx`, `out_ly` | integer, -32768 to 32767 | LX and LY the bridge last handed to the encoder, after merging. |
| `out_lt`, `out_rt` | integer, 0 to `NEPI_TRIGGER_MAX` | LT and RT the bridge last handed to the encoder, after merging. |
| `demand_seq` | integer, -1 or more | `seq` of the last accepted demand; -1 before the first. |
| `demand_age_ms` | integer, -1 or more | Age of the last accepted demand, in ms; -1 before the first. |
| `demand_bad` | integer | Demand datagrams dropped since start (malformed, out of order, or wrong sender). |
| `tn` | object or null | The newest parsed Teensy line (next table); null before the first. |
| `tn_age_ms` | integer, -1 or more | Age of `tn`, in ms; -1 before the first. |
| `tn_count` | integer | Teensy lines parsed since start. Its change over time gives the telemetry rate. |
| `tn_bad` | integer | Teensy lines ignored because they did not start with `$TNCTL,` or did not hold 20 values. |

### Teensy telemetry (`tn`)

The Teensy prints one line every 50 passes through `loop()` (`new_ctl.ino:14, 641-644`), as `$TNCTL,` then 20 comma-separated values, then `,*FF` (`new_ctl.ino:584-629`). The `*FF` is a fixed placeholder, not a checksum. Decimals arrive with 2 digits after the point, which is the Arduino default for printing a `double`. The bridge parses only lines with exactly 20 values. The tuning sketch prints 25 (`tuning.ino:601-656`), so its lines count as `tn_bad`.

| # | Name in `tn` | Firmware value | Unit and range |
|---|---|---|---|
| 1-4 | `vfl_pwr`, `vfr_pwr`, `vbl_pwr`, `vbr_pwr` | `thruster_power`, vertical thrusters | µs away from 1500 µs, -400 to 400 (`new_ctl.ino:26, 33`) |
| 5-8 | `hfl_pwr`, `hfr_pwr`, `hbl_pwr`, `hbr_pwr` | `thruster_power`, horizontal thrusters | same as above |
| 9 | `depth_m` | `sensor_data.depth_m` | m; 0 at 101300 Pa in fresh water (`MS5837/MS5837.cpp:178`, `new_ctl.ino:233`) |
| 10 | `depth_m_s` | `sensor_data.depth_m_s` | m/s |
| 11 | `yaw_deg` | `sensor_data.yaw_deg` | degrees, -180 to 180; slowly drifts (see README) |
| 12 | `pitch_deg` | `sensor_data.pitch_deg` | degrees, -90 to 90 |
| 13 | `roll_deg` | `sensor_data.roll_deg` | degrees, -180 to 180 |
| 14 | `yaw_deg_s` | gyroscope x axis (`new_ctl.ino:387`) | deg/s |
| 15 | `pitch_deg_s` | gyroscope y axis (`new_ctl.ino:388`) | deg/s |
| 16 | `roll_deg_s` | gyroscope z axis (`new_ctl.ino:389`) | deg/s |
| 17 | `depth_rate_target_m_s` | `depth_pwr_controller.target` | requested depth rate, m/s |
| 18 | `depth_pwr_out` | `depth_pwr_controller.last` | vertical power, no unit; 1.0 = 400 µs before limits |
| 19 | `yaw_rate_target_deg_s` | `yaw_pwr_controller.target` | requested yaw rate, deg/s |
| 20 | `yaw_pwr_out` | `yaw_pwr_controller.last` | yaw power, no unit; 1.0 = 400 µs before limits |

The names copy the firmware's labels. Which physical direction each sign means is not known yet; Phase 3 in the README measures it.

## Demand: NEPI to Pi

NEPI sends a demand every 100 ms while the link is up, with `active` false while its hold is off, so the Pi can see the rising edge of `active` that turns assist on. That matches the Pi's 0.1 s serial write (`rpi/modes/main.py:84-88`), so sending faster would change nothing.

| Field | Type, range | Meaning |
|---|---|---|
| `active` | bool | NEPI's hold is running and wants these values used. `false` means every axis counts as 0. |
| `surge` | number, -1 to 1 | Fraction of full left-stick forward travel. Positive = forward. |
| `sway` | number, -1 to 1 | Fraction of full left-stick sideways travel. Positive = right. |
| `yaw` | number, -1 to 1 | Fraction of a full trigger pull. Positive = clockwise seen from above (nose turns right). |
| `pitch` | number, -1 to 1 | Reserved. The Pi ignores it until the Teensy firmware supports pitch. NEPI sends 0. |
| `roll` | number, -1 to 1 | Reserved, the same as `pitch`. |

These values are not speeds. On the Teensy, surge and sway are open-loop power, which means the stick sets thruster power directly and nothing measures the result (`new_ctl.ino:465-477`). A surge demand therefore means "this much stick". Yaw is a rate request: a full trigger asks the Teensy for 180 deg/s (`new_ctl.ino:482`).

Example datagram:

```json
{"v": 1, "type": "demand", "seq": 812, "t": 1792200000.25, "active": true,
 "surge": 0.06, "sway": -0.03, "yaw": 0.0, "pitch": 0.0, "roll": 0.0}
```

## Pi Rules: Toggle, Timeout, Clamp, Override, Mapping

**Toggle.** A rising edge of `NEPI_ASSIST_BUTTON` (the moment it goes from released to pressed) flips `assist`. The bridge uses the same "button available" pattern as the Teensy's slow mode (`new_ctl.ino:364-372`). Assist is off when the bridge starts. It also turns off by itself if no fresh demand has been accepted for 2 s, counting from the moment assist was turned on. The pilot then presses the button again. The button is still passed to the Teensy unchanged, and no sketch reads it.

**Pilot display.** The bridge adds `NEPI:OFF`, `NEPI:ON`, or `NEPI:WAIT` (assist on, but no fresh demand) to the line the Pi already sends the topside, just before `*FF`.

**Timeout.** A demand older than 250 ms counts as 0 on every axis.

**Clamp.** Before mapping, the bridge limits `|surge|` to `NEPI_MAX_SURGE`, `|sway|` to `NEPI_MAX_SWAY`, and `|yaw|` to `NEPI_MAX_YAW`.

**Override, per axis.** The bridge merges into a copy of the pilot's inputs, never into `gamepad_inputs` itself. While assist is on, the pilot's own input wins on its axis whenever it is outside the dead zone:

| Axis | Pilot wins when | Otherwise the bridge sends |
|---|---|---|
| Surge | `abs(LY) >= 3000` | `LY = stick(surge * NEPI_SURGE_SIGN)` |
| Sway | `abs(LX) >= 3000` | `LX = stick(sway * NEPI_SWAY_SIGN)` |
| Yaw | `LT >= NEPI_TRIGGER_DEADZONE` or `RT >= NEPI_TRIGGER_DEADZONE` | With `y = yaw * NEPI_YAW_SIGN`: if `y > 0`, `LT = trigger(y)` and `RT = 0`; if `y < 0`, `LT = 0` and `RT = trigger(-y)`; if `y == 0`, both 0 |

Every other input (RX, RY and all buttons) passes through from the pilot unchanged. NEPI never supplies RY, so depth stays with the Teensy's depth hold (`new_ctl.ino:426-435`).

**Dead-zone mapping.** The Teensy sets anything inside the dead zone to 0 and passes everything else through without rescaling (`new_ctl.ino:563-582`). So the bridge maps a demand onto the part of the range the Teensy acts on. `TD` is `NEPI_TRIGGER_DEADZONE` and `TM` is `NEPI_TRIGGER_MAX`:

```
stick(d)   = 0                                                        if abs(d) < 0.01
           = sign(d) * round(3000 + (abs(d) - 0.01) / 0.99 * (32767 - 3000))   otherwise
trigger(m) = 0                                                        if m < 0.01
           = round(TD + (m - 0.01) / 0.99 * (TM - TD))                otherwise
```

| Demand | Bridge sends | What the Teensy does |
|---|---|---|
| surge 0.01 | LY = 3000 | 9% stick |
| surge 0.2 | LY = 8713 | 27% stick |
| surge 1.0 | LY = 32767 | full stick |
| yaw 0.01 | one trigger = 10 | 7.1 deg/s |
| yaw 0.1 | one trigger = 32 | 22.6 deg/s |
| yaw 1.0 | one trigger = 255 | 180 deg/s |

**Heading hand-off.** While the yaw the bridge sends is 0, the Teensy holds the heading it captured when the triggers last returned to 0 (`new_ctl.ino:484-491`). While NEPI's yaw is nonzero, NEPI's rate request drives the turn. When NEPI's yaw returns to 0, the Teensy captures the heading at that moment and holds it. The smallest nonzero rate is about 7 deg/s.

## NEPI Rules

- NEPI treats the link as down when no valid status has arrived for 500 ms. While it is down, NEPI reports the robot as not ready and sends no demands.
- While the link is up, NEPI sends a demand every 100 ms, always. It sends `active` true while it holds a command from its hold newer than 250 ms and is not latched. Otherwise it sends `active` false with every axis 0.
- **Latch.** An engagement is an unbroken run of demands with `active` true. While NEPI sends `active` true, a newest status with `assist: false` latches NEPI to `active` false, if assist was on during this engagement (the pilot's button, or the bridge's 2 s timeout) or 0.5 s has passed since the engagement's first `active` true (a link outage, or a restarted bridge, which ignores a first `active` true). A stop command on NEPI also sets the latch. It clears only after NEPI's hold has sent no command for 1 s, so engaging again takes a fresh request. NEPI never turns assist back on by itself.
- ROS, which NEPI is built on, counts sideways as positive to the left and turning as positive counter-clockwise. This protocol uses right and clockwise. The NEPI driver flips both signs, in one place.
- NEPI never sends a nonzero `pitch` or `roll` in version 1.

## Video to NEPI

FastMJPG accepts one input and any number of outputs (`FastMJPG/src/FastMJPG.c:636-650`). Each camera's FastMJPG command gains a second `send` output, and the pilot's existing output stays as it is (`quick_stream.sh:1`). For camera 0:

```
send 192.168.68.22 5610 <NEPI_LAN_IP> 5600 1400 1000000 1
```

That is: from local port 5610, to NEPI port 5600, at most 1400 bytes per packet, at most 1000000 bytes per frame, and 1 send round.

Each UDP packet is a 20-byte header followed by one slice of a JPEG image (`FastMJPG/include/VideoUDPShared.h:7-16`). Every header field is big-endian, meaning the most significant byte comes first (`FastMJPG/src/VideoUDPSender.c:42-49`):

| Bytes | Field | Type |
|---|---|---|
| 0-7 | Capture time, µs since 1970-01-01 on the Pi's clock (`FastMJPG/src/VideoCapture.c:16-24, 177`) | unsigned 64-bit |
| 8-11 | Packet index, starting at 0 | unsigned 32-bit |
| 12-15 | Number of packets in this frame | unsigned 32-bit |
| 16-19 | Number of body bytes in this packet | unsigned 32-bit |
| 20 onward | Body: a slice of the JPEG | bytes |

The body of packet `i` belongs at byte `i * 1380` of the JPEG (1400 minus the 20-byte header). The sender sends every packet of a frame in order, then repeats the whole frame once for each extra send round (`VideoUDPSender.c:45-46`).

NEPI's receiver must:

- Drop, never exit on, a packet that is shorter than 20 bytes, is not exactly 20 + body length long, has an index at or above its count, or has `count * 1380` above 1000000.
- Group packets by capture time, and ignore a packet index it already holds.
- Deliver a frame once it holds every index, and abandon an incomplete frame once a newer frame completes.
- Drop any frame whose capture time is not newer than the last frame it delivered. This removes repeat rounds.
- Keep only the newest complete frame.
