# NEPI Connect Link Protocol, Version 2

## Summary

This is the wire contract between a robot and a NEPI computer on the same network. It has four parts: a vehicle message, in which the robot describes itself; a status message from the robot to NEPI; a demand message from NEPI to the robot; and, optionally, a copy of the robot's camera video.

The robot side is the `nepi_connect` package in `pi/`, set up by the robot's vehicle file ([VEHICLE_FILE.md](VEHICLE_FILE.md)). The NEPI side is the `rbx_nepi_connect` driver in `Drivers/rbx_drivers/`. Neither side holds anything specific to one robot: the robot announces what it is, and NEPI builds its driver from that announcement. [README.md](README.md) explains why the link is built this way and how to test it. This file covers only what each side must implement.

Version 2 replaced version 1 in a single cutover on 2026-11-11. Neither side accepts version 1, so the robot and the NEPI driver must update together. "Changes from Version 1" at the end lists what moved.

## Terms

- **Robot, NEPI, driver**: the robot is the vehicle the pilot drives; it runs `nepi_connect` on its own computer. NEPI is the edge-computing software on a separate NEPI computer. A NEPI driver is the part of NEPI that talks to one device, here the robot.
- **Vehicle file**: the YAML file on the robot that describes it ([VEHICLE_FILE.md](VEHICLE_FILE.md)).
- **Input**: one value the robot's control code reads from the pilot's controller, such as a stick axis or a button. An *axis input* carries one of the four movement axes. The *toggle input* hands the robot to NEPI and back.
- **Pilot, NEPI control**: the pilot drives the robot from its own controller. In *NEPI control*, NEPI drives the robot's axes instead. Only the pilot turns NEPI control on.
- **Adapter**: the part of `nepi_connect` that reaches the robot's inputs. The hooks adapter is called from the robot's own control loop.
- **Hz**: times per second. **UTF-8**: the standard way to store text as bytes.
- **UDP**: a way to send one short message, called a *datagram*, across a network. It is fast and needs no connection. A datagram can be lost or arrive out of order, and nothing resends it.
- **JSON**: a plain-text format for named values, such as `{"surge": 0.1}`.
- **Sequence number (`seq`)**: a counter the sender increases by 1 on every message of one type. The receiver uses it to spot old or repeated messages.
- **Timeout**: a rule that treats data as missing once it is older than a set age. Here, a timeout hands the robot back to the pilot alone.
- **Dead zone**: the range around an input's neutral that the robot's own code treats as neutral, so a resting stick does nothing.
- **Surge, sway, heave, yaw**: moving forward or back (surge), sideways (sway), down or up (heave), and turning left or right (yaw).
- **FRD**: forward, right, down, the axis convention of this protocol (see "Axes").

## Addresses and Ports

| What | Protocol | From | To | Rate |
|---|---|---|---|---|
| Vehicle | UDP, JSON | Robot, any port | NEPI, the vehicle file's `status_port` (default 5700) | Once at start, then every 0.5 s |
| Status | UDP, JSON | Robot, any port | NEPI, the same port | 10 Hz, always |
| Demand | UDP, JSON | NEPI, any port | Robot, the announced `demand_port` (default 5701) | 10 Hz while the link is up |
| Video | HTTP or FastMJPG | Robot | NEPI | camera frame rate (see "Video to NEPI") |

The vehicle file names NEPI's address (`network.nepi_ip`). NEPI's driver names the robot's address in its `host` option, and its `status_port` option must equal the vehicle file's `network.status_port`. The demand port reaches NEPI in the vehicle message, so there is nothing to match for it.

## Rules for Every Message

1. **Format.** One JSON object per UDP datagram, as UTF-8 text, at most 1200 bytes, so it always fits in one network frame. The robot checks its own worst case when it loads the vehicle file, and a file that could exceed 1200 bytes is invalid.
2. **Common fields.** Every message carries these:

| Field | Type, range | Meaning |
|---|---|---|
| `v` | integer | Protocol version. This document defines `2`. |
| `type` | string | `"vehicle"`, `"status"` or `"demand"`. |
| `seq` | integer, 0 to 4294967295 | Goes up by 1 per message of this type from this sender, and wraps from 4294967295 to 0. The robot starts each type at 0. The NEPI driver starts at its wall clock in seconds times 100, wrapped into range, so a restarted driver starts ahead of everything its previous run sent. |
| `t` | number, seconds | Sender's clock, in seconds since 1970-01-01 UTC. It is for logs and latency checks only. The two clocks are not synchronized, so no rule below uses `t`. |

3. **Malformed datagram.** The receiver drops the whole datagram, counts it, and keeps its last good values when the datagram is not valid JSON, `v` is not 2, `type` is wrong, a field below is missing or has the wrong type, or a number is NaN (not a number), infinite or outside its stated range. A bad datagram must never crash or stop the receiver. Unknown extra fields are ignored, so version 2 can gain fields without breaking either side. Renaming, removing, or changing the meaning of a field requires `v: 3`.
4. **Out of order.** For each message type, the receiver drops a datagram whose `seq` is less than or equal to the last accepted `seq` of that type. The exception: if the last accepted datagram of that type arrived more than 1 s ago, any `seq` is accepted, because the sender has probably restarted.
5. **Source check.** The robot accepts demands only from `network.nepi_ip`. NEPI accepts vehicle and status messages only from its `host` option.
6. **Ages.** Each receiver measures age on its own clock, from the moment it accepted the datagram.

## Vehicle: Robot to NEPI

The robot sends a vehicle message once when the link starts and then every 0.5 s, to the same port as the status. It never changes while the robot runs: it is built from the vehicle file at start. NEPI's discovery launches the driver only after it hears one valid vehicle message, and builds the driver from it.

| Field | Type, range | Meaning |
|---|---|---|
| `name` | string, 1 to 24 characters: lowercase letters, digits or `_`, starting with a letter | The vehicle's name. NEPI names the driver's devices after it. |
| `kind` | string, up to 32 characters | Free text such as `ROV`. |
| `demand_port` | integer, 1 to 65535 | The robot's port for demands. |
| `axes` | object, axis name to number 0 to 1 | Each axis NEPI may drive (`surge`, `sway`, `heave`, `yaw`, at least one), with the robot's cap on it. |
| `servos` | list of names, at most 4 | Each servo NEPI may move ("Servos" below). Names follow the servo name rule. |
| `nav` | list of standard names | The navigation values each status carries ("Navigation" below). Empty when the robot sends none. |
| `raw` | list of names | The extra telemetry values each status carries in `raw`. Empty when none. |
| `cameras` | list of objects | Each camera, as `{"name", "transport", "port"}`; `transport` is `"http"` or `"fastmjpg"`. At most 4. |
| `imu_mount` | string | Where the robot's IMU board points; the starting value of the driver's `imu_mount` setting (see the driver's `ImuMount`). |

Example datagram:

```json
{"v": 2, "type": "vehicle", "seq": 0, "t": 1792200000.25, "name": "mate_coral_crusaders", "kind": "ROV",
 "demand_port": 5701, "axes": {"surge": 0.2, "sway": 0.2, "heave": 0.2, "yaw": 0.1}, "servos": [],
 "nav": ["depth_m", "depth_m_s", "roll_deg", "pitch_deg", "yaw_deg", "gyro_x_deg_s", "gyro_y_deg_s", "gyro_z_deg_s"],
 "raw": ["vfl_pwr", "depth_pwr_out"], "cameras": [], "imu_mount": "X_FORWARD_Y_LEFT"}
```

If a later vehicle message differs from the one the driver was launched with, the robot's file changed. The driver keeps running on the first one and warns once that it must be disabled and enabled to pick up the change.

## Status: Robot to NEPI

The robot sends a status every 100 ms on its own timer, whether or not new telemetry has arrived. It sends one extra status at once when NEPI control turns on or off, so NEPI sees a hand-over within about 100 ms.

| Field | Type, range | Meaning |
|---|---|---|
| `control` | bool | NEPI control is on. The toggle state lives on the robot. |
| `control_reason` | string | Why `control` last changed: `start`, `pilot_button` (the toggle input), `pilot_stick` (an axis input past its takeover point), `timeout` (2 s with no accepted demand) or `nepi_off` (a demand with `active` false). |
| `nepi_ready` | bool | A demand with `active` true was accepted within 0.5 s, so the toggle input hands the robot over. |
| `pilot` | object, axis name to bool | For each announced axis, the pilot holds one of its inputs: \|value − neutral\| ≥ dead zone. |
| `servos` | object, servo name to bool | For each announced servo, the pilot holds its input. `{}` when there are none. |
| `out` | object, input name to integer | Each axis input's value the robot last handed on, after merging, limited to that input's range. |
| `demand_seq` | integer, -1 to 4294967295 | `seq` of the last accepted demand; -1 before the first. |
| `demand_age_ms` | integer, -1 to 4294967295 | Age of the last accepted demand, in ms; -1 before the first. |
| `demand_bad` | integer, 0 to 4294967295 | Demand datagrams dropped since start (malformed, out of order, or wrong sender). |
| `nav` | object or null | The newest navigation values, by standard name ("Navigation" below); null before the first telemetry line, and always null for a robot that announces none. |
| `nav_age_ms` | integer, -1 to 4294967295 | Age of the newest telemetry line, in ms; -1 before the first. |
| `nav_count` | integer, 0 to 4294967295 | Telemetry lines parsed since start. Its change over time gives the telemetry rate. |
| `nav_bad` | integer, 0 to 4294967295 | Telemetry lines ignored because they did not match the vehicle file's line format. |
| `raw` | object or null | Present only when the vehicle file lists `raw` fields: each by name, from the newest telemetry line; null before the first. |

Ages and counters stop at 4294967295 rather than grow without limit, which keeps the status inside its size limit.

### Navigation

The standard names come in two groups, and a robot sends a group whole or not at all:

| Group | Standard names | Units |
|---|---|---|
| Depth | `depth_m`, `depth_m_s` | m below the surface, positive down; m/s |
| Orientation | `roll_deg`, `pitch_deg`, `yaw_deg`, `gyro_x_deg_s`, `gyro_y_deg_s`, `gyro_z_deg_s` | Z-Y-X angles of the IMU board, degrees; rates about the board's X, Y and Z arrows, deg/s |

Orientation and rates are in the IMU board's own frame. NEPI turns them into the robot's frame with its `imu_mount` setting. NEPI drops a status whose `nav` is an object that lacks any announced standard name, or holds one that is not a finite number.

## Demand: NEPI to Robot

NEPI sends a demand every 100 ms while the link is up. That matches a 10 Hz robot control loop; a robot whose loop is slower simply uses the newest demand.

| Field | Type, range | Meaning |
|---|---|---|
| `active` | bool | `true`: NEPI is connected and ready. It does not turn NEPI control on; only the pilot does. `false` ends NEPI control at once (`nepi_off`). |
| `axes` | object, axis name to number -1 to 1 | The value NEPI wants on each announced axis, as a fraction of the axis's full travel, in FRD. NEPI sends only the announced axes. The robot ignores an axis name it did not announce, and an announced axis that is missing counts as 0. |
| `servos` | object, servo name to number -1 to 1 | Optional. The value NEPI wants for each named servo ("Servos" below); 0 is the input's neutral. A name the robot has no servo for is ignored. No field means no servo values. |

These values are not speeds. A demand means "this much of the input": what that does is up to the robot's own code, which the vehicle file's comments should say.

Example datagram:

```json
{"v": 2, "type": "demand", "seq": 3126340889, "t": 1792200000.25, "active": true,
 "axes": {"surge": 0.06, "sway": -0.03, "heave": 0.0, "yaw": 0.0}, "servos": {}}
```

## Axes

The protocol uses FRD for every axis value in a demand:

| Axis | Positive means |
|---|---|
| `surge` | forward |
| `sway` | right |
| `heave` | down |
| `yaw` | clockwise seen from above (nose turns right) |

ROS, which NEPI is built on, counts sideways as positive to the left, up as positive, and turning as positive counter-clockwise. The NEPI driver maps its teleop command to FRD in one place (`NepiConnectLink.set_teleop_velocity`): surge = x, sway = −y, heave = −z, yaw = −yaw rate, each limited to −1 to 1. The robot applies its own caps and the vehicle file's per-axis signs, which make a positive FRD value move the robot the FRD way.

## Robot Rules: NEPI Control, Timeouts, Caps, Mapping

The robot merges NEPI's demand into a copy of the pilot's inputs, never into the pilot's inputs themselves, just before its control loop hands them on.

**Off at start.** NEPI control is off when the link starts.

**NEPI ready.** NEPI is ready while a demand with `active` true was accepted within 0.5 s.

**Toggle.** Pressing the toggle input, \|value − neutral\| ≥ its dead zone, counts once per press: the input must be released before the next press counts (the "button available" rule). A press turns NEPI control off when it is on (`pilot_button`). When it is off, a press turns it on (`pilot_button`), except that it is ignored while NEPI is not ready, and while any axis input is at or past its takeover point. The toggle input itself passes through to the robot unchanged.

**What NEPI owns.** In NEPI control, NEPI owns every axis input of every announced axis: each carries NEPI's demand, and the pilot's own value is replaced, not mixed in. Inputs that belong to no axis and no servo pass through from the pilot unchanged.

**Takeover (the safety exit).** Any axis input at or past its takeover point ends NEPI control at once (`pilot_stick`). The takeover point is `ceil(takeover_fraction × travel)` away from neutral, where travel is `max − neutral` above neutral and `neutral − min` below it. A side of an input with no travel never takes over. Every takeover point must lie past its input's dead zone, or the vehicle file is invalid.

**Other exits.** NEPI control also ends when the pilot presses the toggle input again, when a demand with `active` false is accepted (`nepi_off`), and after 2 s with no accepted demand (`timeout`). Nothing but the pilot's toggle input turns it back on.

**Stale demand.** A demand older than 250 ms counts as 0 on every axis, so in NEPI control every axis input sits at its neutral until a fresh demand arrives.

**Cap and sign.** Before mapping, the robot limits each axis value to ±its `cap`, then multiplies it by its `sign`.

**Mapping.** A value `d` becomes an input value that lands just past the dead zone for the smallest demand and at the end of the range for a full one. With `n` the input's neutral, `dz` its dead zone and `travel` the side's travel:

```
stick(d)    = n                                                           if |d| < 0.01
            = n ± min(travel, round(dz + (|d| − 0.01) / 0.99 × (travel − dz)))   otherwise, + for d > 0
trigger(m)  = n                                                           if m < 0.01
            = n + min(travel, round(dz + (m − 0.01) / 0.99 × (travel − dz)))    otherwise
```

`round` here rounds half up. A `trigger_pair` axis with value `y` writes `trigger(y)` to its first input if `y > 0`, `trigger(−y)` to its second if `y < 0`, and leaves both at neutral if `y = 0`.

**Pilot display.** The hooks adapter's `status_tag()` returns `ON` (NEPI control on, demand fresh), `WAIT` (NEPI control on, demand older than 250 ms), `OFF` (off, NEPI ready, so the toggle hands over) or `NOTREADY` (off, NEPI not connected, so the toggle does nothing), and an empty string while the link is off. The robot's code can show it to the pilot.

**Errors.** Each kind of message prints at most once per 10 s.

## NEPI Rules

- NEPI treats the link as down when no valid status has arrived for 500 ms. While it is down, NEPI sends no demands, so the robot counts NEPI as not ready and ignores the toggle input.
- While the link is up, NEPI sends a demand every 100 ms with `active` true. Only handing the robot back (`release_to_pilot`) sends `active` false, once; the demands after it are `active` true again, so the pilot can hand the robot over again.
- The demand carries the newest teleop command while NEPI control is on and the command is no older than 500 ms. Otherwise every axis is 0. While NEPI control is off the stored command is dropped, so a hand-over always starts from neutral.
- Teleop is ready while the link is up and NEPI control is on, so NEPI accepts teleop commands exactly while it has the robot.
- Stop clears the stored command, so the demands go neutral; NEPI keeps the robot.
- The robot state is `PILOT` or `NEPI`, who has the robot. Setting `PILOT` hands it back (`release_to_pilot`). Setting `NEPI` is refused, since only the pilot's toggle input hands it over.
- The driver offers only the announced axes: it reports roll and pitch control as unavailable always, and sends only announced axes.

## Servos

NEPI can move up to four servos that the robot's code already moves from an input, such as a claw on a button. NEPI never drives a servo's pulses: the robot writes the input, the same way it writes the axis inputs.

**Configuration.** The vehicle file names each servo and its input ([VEHICLE_FILE.md](VEHICLE_FILE.md) "Servos"). A servo input cannot be an axis input, the toggle input or a reserved input, and two servos cannot share a name or an input.

**Mapping.** A value v from -1 to 1 becomes neutral + v × (max − neutral) for v ≥ 0, and neutral + v × (neutral − min) below 0, rounded half away from neutral and limited to min to max. So a three-way input (-1, 0, 1) is pressed for \|v\| ≥ 0.5 and a button (0, 0, 1) for v ≥ 0.5; a negative v leaves a button released.

**Robot rules.**
- **Pilot mode:** the pilot's input passes through, and NEPI's values are ignored.
- **NEPI control:** the input carries NEPI's value from the newest demand. It is neutral while the demand is older than 250 ms or has no value for that servo.
- **The pilot wins per input:** while the pilot holds a servo's input, \|value − neutral\| ≥ its dead zone, the pilot's value passes through for that input alone. Unlike an axis input, it never ends NEPI control.

**NEPI rules.**
- The driver accepts a servo value only while the newest status shows NEPI control. It drops every value when NEPI control ends, so a hand-over starts each servo at neutral.
- A value holds until it is changed; unlike a teleop command it never goes stale. RBX Stop leaves the servos alone.
- Values go out rounded to 4 decimals, and only with `active` true.

**What a value means is up to the robot's code.**
- If the input sets the servo's angle (a stick), a value is a position.
- If holding the input moves the servo (a held button), a value is a direction to keep moving. NEPI 3.4's servo page has no continuous mode, so such an input takes gotos only: a goto away from neutral holds the input in that direction until another goto, or the end of NEPI control, brings it back.
- Code that moves one step per press, rather than while held, cannot be driven this way, because the robot holds the input rather than pressing it again.

**On NEPI.**
- Discovery reads the servo names from the vehicle message and launches one SVX node per servo (`Drivers/rbx_drivers/rbx_nepi_connect_servo_node.py`), so each servo has its own page on NEPI.
- Each servo node sends its value to the RBX node's `set_servo` topic as a `std_msgs/String` with JSON `{"name": ..., "value": ...}`, and reads the RBX node's `link_status`, whose `servos` object holds `value` and `pilot` per servo.
- In the servo node, 0° is the input's neutral, `max_deg` its high end and `min_deg` its low end.

## Video to NEPI

A camera reaches NEPI in one of two ways, and the vehicle message lists each one. NEPI does not set up its camera driver from the list: the driver's `link_status` shows each camera's address, which the user adds to NEPI's network stream camera driver (`idx_net_stream`) by hand.

| Transport | Who sends it | Address in `link_status` |
|---|---|---|
| `http` | The robot's NEPI Connect service streams a USB camera with ustreamer, as MJPEG over HTTP | `http://<robot>:<port>/stream` |
| `fastmjpg` | The robot's own FastMJPG command sends to NEPI's `port` | `fastmjpg://0.0.0.0:<port>` (any sender) |

### FastMJPG packets

FastMJPG accepts one input and any number of outputs (`FastMJPG/src/FastMJPG.c:636-650`). Each camera's FastMJPG command gains a second `send` output, and the pilot's existing output stays as it is. For camera 0:

```
send <robot address> 5610 <nepi_ip> 5600 1400 1000000 1
```

That is: from local port 5610, to NEPI port 5600, at most 1400 bytes per packet, at most 1000000 bytes per frame, and 1 send round.

Each UDP packet is a 20-byte header followed by one slice of a JPEG image (`FastMJPG/include/VideoUDPShared.h:7-16`). Every header field is big-endian, meaning the most significant byte comes first (`FastMJPG/src/VideoUDPSender.c:42-49`):

| Bytes | Field | Type |
|---|---|---|
| 0-7 | Capture time, µs since 1970-01-01 on the robot's clock (`FastMJPG/src/VideoCapture.c:16-24, 177`) | unsigned 64-bit |
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

## Size

The robot computes the worst-case vehicle and status message for its own vehicle file when it loads it, with every value at its longest, and turns the link off if either could exceed 1200 bytes. With `vehicles/mate_coral_crusaders.yaml`, the worst cases are 506 bytes for the vehicle message and 1035 bytes for the status, and four servos with 16-character names would raise them to 581 and 1134 (`pi/test_nepi_link.py`).

## Changes from Version 1

- The robot's configuration moved from flat `NEPI_*` keys to the vehicle file, which describes any robot rather than one.
- New: the vehicle message, which NEPI's discovery waits for and builds the driver from.
- Status: `assist` and `assist_reason` became `control` and `control_reason`; `pilot_surge`, `pilot_sway`, `pilot_yaw` and `pilot_depth` became the `pilot` object, keyed by axis (`pilot_depth` is `heave`); `out_lx` and the other `out_*` fields became the `out` object, keyed by input name; `tn`, `tn_age_ms`, `tn_count` and `tn_bad` became `nav` (standard names only), `nav_age_ms`, `nav_count` and `nav_bad`, plus `raw` for extra telemetry values. `slow` is gone.
- Demand: `surge`, `sway`, `heave`, `yaw`, `pitch` and `roll` became the `axes` object, holding only announced axes.
- `seq` is counted per message type.
- The driver's `demand_port` option is gone; the port comes from the vehicle message.
