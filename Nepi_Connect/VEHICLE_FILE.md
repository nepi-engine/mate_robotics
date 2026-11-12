# NEPI Connect Vehicle File

## Summary

A vehicle file describes one robot to NEPI Connect. It lives on the robot and says how to reach NEPI, which of the robot's inputs NEPI may drive and how, what telemetry the robot prints, and which cameras it streams. The robot reads it at start and announces the result to NEPI in a vehicle message ([LINK_PROTOCOL.md](LINK_PROTOCOL.md)), and NEPI builds its driver from that announcement. Connecting a new robot therefore takes one vehicle file and no new NEPI code.

[vehicles/template.yaml](vehicles/template.yaml) is a fully commented starting point. [vehicles/mate_coral_crusaders.yaml](vehicles/mate_coral_crusaders.yaml) is a complete, working file, and "Worked Example" below walks through it.

## Terms

- **Robot, NEPI**: the robot is the vehicle the pilot drives. NEPI is the edge-computing software on a separate NEPI computer.
- **YAML**: the plain-text format of the file: `key: value` pairs, with indentation for nesting. `#` starts a comment.
- **PyYAML**: the Python library that reads YAML. The robot needs it: `sudo apt install python3-yaml`.
- **Input**: one value the robot's control code reads from the pilot's controller, such as a stick axis or a button, held at a fixed position (its *slot*) in the list or tuple the code hands on.
- **Neutral, dead zone**: an input's resting value, and how far from neutral the robot's own code starts to act on it.
- **Axis**: one of surge (forward), sway (right), heave (down) and yaw (clockwise seen from above), the four movements NEPI can drive.
- **Toggle input**: the input the pilot presses to hand the robot to NEPI and back.
- **NEPI control**: the state in which NEPI drives the robot's axes. Only the pilot turns it on.
- **Adapter**: the part of NEPI Connect that reaches the robot's inputs.

## Where the Robot Looks

The robot uses the first of these that exists:

1. The path in the `NEPI_CONNECT_VEHICLE` environment variable.
2. `nepi_connect_vehicle.yaml` beside `nepi_link.py`, the hooks file the robot's code imports.
3. `~/nepi_connect/config/nepi_connect_vehicle.yaml`, where `scripts/nepi_connect_setup.sh` stages the template.

When none exists, the link stays off and says so. When none exists but an old version 1 `nepi_link.yaml` does (beside `nepi_link.py`, or named by `NEPI_LINK_CONFIG`), the link stays off and says the file must be converted to a vehicle file; the "Converting a Version 1 File" section below shows how.

When PyYAML does not import, the link stays off and prints once: `PyYAML missing: sudo apt install python3-yaml`.

## Validation

The robot checks the whole file when it loads it. Any invalid value turns the link off and prints one message that names the key, such as `control.axes.surge.cap must be a number from 0 to 1: link off`. With the link off, the robot opens no sockets, starts no threads, and passes the pilot's inputs through untouched, so it runs exactly as it does without NEPI.

Keys the robot does not know are invalid too, so a misspelled key cannot be silently ignored.

At load, the robot also builds the longest vehicle message and the longest status its file can produce, with every counter, age and value at its longest. If either is over 1200 bytes, the file is invalid, and the message gives both sizes. Long names, many `raw` fields and four servos are what fill a status.

## Keys

### vehicle

| Key | Meaning | Default | Valid values |
|---|---|---|---|
| `vehicle.name` | The vehicle's name. NEPI names the driver's devices after it, such as `nepi_connect_<name>`. | required | 1 to 24 characters: lowercase letters, digits or `_`, starting with a letter |
| `vehicle.kind` | Free text NEPI shows, such as `ROV`, `USV` or `rover`. | `''` | A string of up to 32 characters |

### network

| Key | Meaning | Default | Valid values |
|---|---|---|---|
| `network.nepi_ip` | NEPI's address. Demands from any other address are dropped. Empty turns the link off. | `''` (link off) | An IPv4 address, or empty |
| `network.status_port` | The port on NEPI that vehicle and status messages go to. It must equal the `status_port` option of NEPI's NEPI Connect driver. | `5700` | 1 to 65535 |
| `network.demand_port` | The port on the robot that demands arrive on. NEPI reads it from the vehicle message. | `5701` | 1 to 65535 |

### control

| Key | Meaning | Default | Valid values |
|---|---|---|---|
| `control.adapter` | How NEPI Connect reaches the robot's inputs. `hooks`: the robot's own control loop calls the hook functions. | `hooks` | `hooks` |
| `control.toggle_input` | The input the pilot presses to hand the robot to NEPI, and again to take it back. | required | An input in `control.inputs` that is not an axis input, a servo input or a reserved input, and that can reach its dead zone |
| `control.takeover_fraction` | The pilot takes the robot back at once by moving any axis input to or past this share of its travel ("Takeover"). | `0.25` | A number above 0 and at most 1 that puts every axis input's takeover point past its dead zone |
| `control.reserved_inputs` | Inputs NEPI must never use, such as a mode button the robot's code reads. They pass through from the pilot. | `[]` | A list of inputs in `control.inputs` |
| `control.inputs` | Every input the robot's code hands on, by name ("Inputs"). | required | 1 to 64 entries |
| `control.axes` | The axes NEPI may drive ("Axes"). | required | 1 to 4 of `surge`, `sway`, `heave`, `yaw` |
| `control.servos` | Servos the robot's code moves from an input, which NEPI may move too ("Servos"). | `{}` | Up to 4 entries |

#### Inputs

Each entry in `control.inputs` is keyed by the input's name, 1 to 24 letters, digits or `_`, starting with a letter. The name is what the other keys use, and it is the key of the input's value in the status `out` object.

| Key | Meaning | Valid values |
|---|---|---|
| `slot` | The input's position in the list or tuple the robot's code hands on. | 0 to 255, each slot once |
| `min`, `max` | The input's raw range. | Integers, `min` < `max` |
| `neutral` | The input's resting value. | An integer, `min` ≤ `neutral` ≤ `max` |
| `deadzone` | How far from neutral the robot's own code starts to act on the input. The pilot holds the input while \|value − neutral\| ≥ `deadzone`. | An integer, 1 or more |
| `label` | Optional. A short name for messages to the pilot, such as `RB`. Defaults to the input's name. | Up to 24 printable characters |

Every key but `label` is required.

One rule decides whether the pilot holds an input, for every input: \|value − neutral\| ≥ `deadzone`. It reports the `pilot` and `servos` flags in the status, keeps a servo input the pilot's, and decides when the toggle input is pressed. Version 1 used strictly more than a dead zone one smaller for servo inputs (2999 for a stick, 0 for a button or D-pad). For the whole-number values controllers report, "≥ 3000" and "> 2999" pick the same values, and so do "≥ 1" and "> 0", so the result is the same.

#### Axes

Each entry in `control.axes` is keyed by its axis: `surge`, `sway`, `heave` or `yaw`. An axis left out stays the pilot's alone; NEPI never sends it and reports it as not controllable.

| Key | Meaning | Valid values |
|---|---|---|
| `kind` | `stick`: one input that moves both ways. `trigger_pair`: two inputs that each move one way from neutral, the first for positive demands and the second for negative ones. | `stick` or `trigger_pair` |
| `input` | For `stick`: its input. | An input whose `min` < `neutral` < `max` |
| `inputs` | For `trigger_pair`: `[positive, negative]`. | Two different inputs, each with `max` > `neutral` |
| `sign` | 1 or -1, so that a positive FRD demand moves the robot the FRD way: forward, right, down, clockwise seen from above. Measure it: push the input and watch which way the robot goes. | `1` or `-1` |
| `cap` | The largest demand the robot accepts on this axis, as a fraction of full travel. 0 keeps the axis at neutral. | A number from 0 to 1 |

All four keys (with `input` or `inputs`) are required. An input can belong to only one axis, and an axis input cannot be the toggle input, a servo input or a reserved input. Each side of an axis input that moves must have more travel than its dead zone.

#### Mapping

In NEPI control, each axis input carries NEPI's demand. The robot limits the demand to ±`cap`, multiplies it by `sign`, and maps it onto the part of the input's range the robot's code acts on, so the smallest demand lands just past the dead zone and a full demand at the end of the range. With `n` the neutral, `dz` the dead zone and `travel` the distance from neutral to the end the value moves toward:

```
stick(d)    = n                                                           if |d| < 0.01
            = n ± min(travel, round(dz + (|d| − 0.01) / 0.99 × (travel − dz)))   otherwise, + for d > 0
trigger(m)  = n                                                           if m < 0.01
            = n + min(travel, round(dz + (m − 0.01) / 0.99 × (travel − dz)))    otherwise
```

A `trigger_pair` axis with value `y` writes `trigger(y)` to its first input if `y > 0`, `trigger(−y)` to its second if `y < 0`, and leaves both at neutral if `y = 0`. While the demand is older than 250 ms, every axis input sits at neutral.

#### Takeover

An input's takeover point is `ceil(takeover_fraction × travel)` away from neutral, where travel is `max − neutral` above neutral and `neutral − min` below it. A side with no travel, such as below a trigger's neutral, never takes over. While NEPI control is on, any axis input at or past its takeover point ends it at once, and while it is off, the toggle input is ignored while one is. Each takeover point must lie past its input's dead zone, so that a pilot moving an input enough to act on it, but less than the takeover point, does not end NEPI control by accident.

#### Servos

Each entry in `control.servos` is keyed by the servo's name, 1 to 16 lowercase letters, digits or `_`, starting with a letter. NEPI shows one servo page per name.

| Key | Meaning | Default | Valid values |
|---|---|---|---|
| `input` | The input the robot's code moves the servo with. | required | An input in `control.inputs` that is not an axis input, the toggle input or a reserved input, and that no other servo uses |
| `min`, `neutral`, `max` | The range NEPI's value maps onto ([LINK_PROTOCOL.md](LINK_PROTOCOL.md) "Servos"). | the input's own | Integers, `min` ≤ `neutral` ≤ `max`, `min` < `max` |
| `deadzone` | The pilot holds the input, and wins over NEPI, while \|value − neutral\| ≥ `deadzone`. | the input's own | An integer, 1 or more |

A version 1 `NEPI_SERVO_n_DEADZONE` of `d` is a `deadzone` of `d + 1` here.

### telemetry

Optional. Leave the whole section out if the robot prints nothing NEPI should read; NEPI then gets no navigation values.

| Key | Meaning | Default | Valid values |
|---|---|---|---|
| `telemetry.format` | The line format. `csv_line`: one line of comma-separated values. | required | `csv_line` |
| `telemetry.prefix` | The line's first value, which marks it as telemetry. | `''` (no prefix value) | Up to 16 characters, no comma |
| `telemetry.trailer` | A regular expression the line's last value must match, such as a checksum. Empty means the line has no trailer value. | `''` | A Python regular expression of up to 64 characters |
| `telemetry.fields` | Every value between the prefix and the trailer, in order, each as `{name, type}`. | required | 1 to 64 entries. Names: 1 to 24 lowercase letters, digits or `_`, starting with a letter, each once. Types: `int` or `float` |
| `telemetry.nav` | Which fields carry NEPI's standard navigation values, as `group: {standard name: field name}`. | `{}` | The groups and names in the next table; each group whole or not at all |
| `telemetry.raw` | Extra fields to forward as they are, in the status `raw` object. Useful for bench diagnostics. | `[]` | Field names, each once |
| `telemetry.imu_mount` | Where the robot's IMU board's X and Y arrows point, such as `X_FORWARD_Y_LEFT` (flat, X forward). NEPI starts its `imu_mount` setting at this value. | `X_FORWARD_Y_LEFT` | `X_<direction>_Y_<direction>` with two perpendicular directions out of `FORWARD`, `BACK`, `LEFT`, `RIGHT`, `UP`, `DOWN` |

| Group | Standard names | Units |
|---|---|---|
| `depth` | `depth_m`, `depth_m_s` | m below the surface, positive down; m/s |
| `orientation` | `roll_deg`, `pitch_deg`, `yaw_deg`, `gyro_x_deg_s`, `gyro_y_deg_s`, `gyro_z_deg_s` | Z-Y-X angles of the IMU board, degrees; rates about the board's X, Y and Z arrows, deg/s |

A line counts as telemetry only when it has exactly the prefix, one value per field and the trailer, and every value reads as its type: an `int` is up to 10 digits with an optional minus sign, inside the 32-bit range; a `float` is the same with up to 6 digits after an optional point. Arduino prints `nan`, `inf` or `ovf` for a number it cannot show, and those lines are ignored. Ignored lines count as `nav_bad`. These limits keep the status inside its size limit.

### cameras

Optional. A list of up to 4 cameras that NEPI can show. NEPI lists each camera's address on its driver's `link_status` topic, for the user to add to NEPI's network stream camera driver by hand.

| Key | Meaning | Default | Valid values |
|---|---|---|---|
| `name` | The camera's name. | required | 1 to 16 lowercase letters, digits or `_`, starting with a letter, each once |
| `transport` | `http`: the robot's NEPI Connect service streams a USB camera with ustreamer, at `http://<robot>:<port>/stream`. `fastmjpg`: the robot's own FastMJPG command sends to this NEPI port; announced only. | required | `http` or `fastmjpg` |
| `port` | For `http`, the robot's port; for `fastmjpg`, NEPI's port. | required | 1 to 65535, each once |
| `device` | `http` only. The camera, as a `/dev/v4l/by-path/...-video-index0` path, which follows the USB port; empty means the first USB camera found. | `''` | A string |
| `resolution` | `http` only. One of the camera's MJPEG modes. | `1280x720` | `<width>x<height>` |
| `fps` | `http` only. The frame rate of that mode. | `30` | 1 to 120 |

## Worked Example: the MATE ROV

[vehicles/mate_coral_crusaders.yaml](vehicles/mate_coral_crusaders.yaml) describes the Coral Crusaders' MATE ROV. Its control Pi reads an Xbox controller and hands 18 values to the Teensy microcontroller 10 times a second. The team's script calls the four hooks in `nepi_link.py`, which is the `hooks` adapter.

- **Inputs.** All 18 values, by their evdev names, in the script's `gamepad_map` order. The sticks run from -32767 to 32767 with neutral 0, and the Teensy ignores anything inside ±3000, so their dead zone is 3000. The triggers run from 0 to 1023 with dead zone 40, the sketch's own constants. Buttons are 0 to 1 and the D-pad -1 to 1, each with dead zone 1. Labels such as `LY` and `RB` keep the pilot's messages in the team's words.
- **Toggle and reserved.** RB (`BTN_TR`) is the toggle input. LB (`BTN_TL`) is the Teensy's slow mode, so it is reserved.
- **Axes.** Surge on LY with sign -1, because LY pushed forward reads negative on the team's pad. Sway on LX, heave on RY. Yaw is a `trigger_pair`: LT for clockwise, RT for counter-clockwise. The caps hold surge, sway and heave at 0.2 and yaw at 0.1. A surge demand of 0.2 becomes LY = -8713.
- **Takeover.** At 0.25, the sticks take over at ±8192 and the triggers at 256, both well past their dead zones.
- **Servo inputs.** The 11 inputs left after the axis inputs, RB and LB are the ones a servo may use: RX, the D-pad, the thumb buttons, the four face buttons, Start and Select. The team asked for no servos, so `servos` is empty.
- **Telemetry.** The Teensy prints `$TNCTL`, 20 values and a `*FF` trailer. The first 8 are thruster powers (`int`), the rest decimals. `nav` maps depth and depth rate, the three angles and the three gyro rates to their standard names; the gyro columns are named after yaw, pitch and roll in the firmware, but they are the board's X, Y and Z axes, so `gyro_x_deg_s` is `yaw_deg_s`. The other 12 values go in `raw`. The IMU board lies on its side in this ROV, but its mount has not been measured, so `imu_mount` stays flat until it is.
- **Cameras.** The team's video reaches NEPI by FastMJPG from its own commands, so none are announced.

The worst-case vehicle message from this file is 506 bytes and the worst-case status 1035 bytes, both under the 1200-byte limit.

## Converting a Version 1 File

A version 1 `nepi_link.yaml` held flat `NEPI_*` keys. Start from [vehicles/mate_coral_crusaders.yaml](vehicles/mate_coral_crusaders.yaml) and carry each value over:

| Version 1 key | Vehicle file key |
|---|---|
| `NEPI_LAN_IP` | `network.nepi_ip` |
| `NEPI_STATUS_UDP_PORT`, `NEPI_DEMAND_UDP_PORT` | `network.status_port`, `network.demand_port` |
| `NEPI_ASSIST_BUTTON` | `control.toggle_input` |
| `NEPI_MAX_SURGE` (and the other axes) | `control.axes.surge.cap` |
| `NEPI_SURGE_SIGN` (and the other axes) | `control.axes.surge.sign` |
| `NEPI_TRIGGER_DEADZONE`, `NEPI_TRIGGER_MAX` | `deadzone` and `max` of `ABS_Z` and `ABS_RZ` in `control.inputs` |
| `NEPI_TAKEOVER_FRACTION` | `control.takeover_fraction` |
| `NEPI_SERVO_n_NAME`, `_INPUT` | the servo's name in `control.servos`, and its `input` |
| `NEPI_SERVO_n_MIN`, `_NEUTRAL`, `_MAX` | its `min`, `neutral`, `max` |
| `NEPI_SERVO_n_DEADZONE` | its `deadzone`, plus 1 |

Then name the file `nepi_connect_vehicle.yaml` and delete `nepi_link.yaml`.
