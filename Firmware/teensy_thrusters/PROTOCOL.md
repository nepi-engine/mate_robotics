# teensy_thrusters: Pi-to-Teensy Protocol, Version 1.0

This document specifies the serial protocol between the NEPI device (a Raspberry Pi 5) and a PJRC Teensy 4.1 that generates the servo-style pulses for Blue Robotics Basic ESCs (BESC30-R3), each driving one T200 thruster. Three files implement it and are meant to be bench-tested together:

| Side | File |
|---|---|
| Teensy firmware | `Firmware/teensy_thrusters/teensy_thrusters.ino` |
| NEPI discovery | `Drivers/svx_drivers/svx_teensy_thrusters_discovery.py` |
| NEPI node | `Drivers/svx_drivers/svx_teensy_thrusters_node.py` |

Hardware facts used here come from `Docs/tbd/pjrc_teensy_4_1.md` (the Teensy), `Docs/Motors/bluerobotics_basic_esc.md` (the ESC) and `Docs/Motors/bluerobotics_t200_thruster.md` (the thruster). This specification covers every item in the Teensy document's "To Be Defined" list; section 12 maps each item to the section that covers it.

## 1. Why this protocol exists

Neither the T200 nor the Basic ESC has a data link. The ESC takes one-way servo pulses on one signal wire and reports nothing back, so software cannot tell whether an ESC is connected, powered or armed (ESC doc, "The ESC cannot be detected from software"). The Teensy is the only device the Pi talks to. The protocol therefore carries pulse widths, not thrust, speed or RPM, and nothing in it claims to know what a thruster is doing.

## 2. Transport

- USB type "Serial" (CDC-ACM). The board appears on Linux as `/dev/ttyACM*` with VID `0x16C0`, PID `0x0483`, manufacturer "Teensyduino" and product "USB Serial". Any Teensyduino sketch shows those same IDs, so the host must confirm the firmware with the `ID` command (section 6.1) before it sends anything else.
- The baud rate is ignored. The host opens the port at 115200. **The host must never open the port at 134 baud**, because that rate is the request that reboots a running Teensy into its bootloader, and in the bootloader every pulse stops (Teensy doc, "134 baud is reserved").
- The host must assert DTR. pySerial does this by default on open, and so do common terminal programs (picocom, minicom, screen). Section 9 describes what DTR controls.
- The USB serial number is unique per board (Teensy doc, "Unique serial number") and is the key the NEPI driver uses to identify a board, not the `/dev/ttyACM` index.

## 3. Framing

Every message in either direction is one line of printable ASCII in the NMEA style:

```
$<BODY>*<HH>\n
```

- `$` starts a frame. A `$` received in the middle of a line throws away everything before it, so the receiver resynchronizes on the next frame.
- `<BODY>` is a two-letter uppercase command, optionally followed by comma-separated fields: `SA,1500,1500,...`. Spaces are not allowed anywhere in a frame.
- `<HH>` is the checksum: the XOR of every byte of `<BODY>` (everything strictly between `$` and `*`), written as two hexadecimal digits. The firmware sends uppercase and accepts either case.
- The line ends with `\n`. A `\r` before the `\n` is accepted and ignored.
- Maximum frame length is 120 characters from `$` to the last checksum digit.
- Integer fields are unsigned decimal, with no sign, no leading `+` and at most 10 digits. The channel mask is two hexadecimal digits.

Example: `$ST*` followed by the checksum of `ST`, which is `'S' ^ 'T'` = `0x53 ^ 0x54` = `0x07`. The frame is therefore `$ST*07\n`.

**USB packet behavior.** The firmware assembles frames byte by byte and acts only on a complete line, so a frame split across USB packets, or several frames in one packet, is handled correctly. This is required because `Serial.available()` counts only the first buffered packet, and a message the host writes in pieces can look shorter than it is (Teensy doc, "Receive buffering").

**Request and reply.** The protocol is strictly request and reply. The firmware sends exactly one reply line for every complete line it receives that contains a `$`, and it never sends anything unsolicited, including at boot. Lines that are empty or contain no `$` are ignored without a reply. After each reply the firmware calls `Serial.send_now()`, so the reply is not left waiting for the 3 to 5 ms USB transmit timeout (Teensy doc, "Transmit buffering").

## 4. Channels and pins

The firmware drives 8 thruster channels plus 1 reserved servo channel. The channel count is fixed when the firmware is compiled and is reported by `ID`.

| Channel | Use | Teensy pin | PWM timer group |
|---|---|---|---|
| 0 | Thruster | 2 | FlexPWM4.2 (shared with pin 3) |
| 1 | Thruster | 3 | FlexPWM4.2 (shared with pin 2) |
| 2 | Thruster | 4 | FlexPWM2.0 (pin 33 also on this timer; leave 33 unused) |
| 3 | Thruster | 5 | FlexPWM2.1 |
| 4 | Thruster | 6 | FlexPWM2.2 (shared with pin 9) |
| 5 | Thruster | 9 | FlexPWM2.2 (shared with pin 6) |
| 6 | Thruster | 22 | FlexPWM4.0 |
| 7 | Thruster | 23 | FlexPWM4.1 |
| 8 | Reserved: servo (Axon MAX MK2) | 10 | QuadTimer1.0 |

Pins and timer groups are taken from the Teensy doc's PWM timer table, which quotes PJRC's PWM page. The choice avoids:

- I2C pins 18/19 (Wire) and 24/25 (Wire2), and pins 0/1 (Serial1), which share PWM timers with those functions (Teensy doc, "Pin functions that matter for this kit");
- pins 41, 51, 53 and 54, where PJRC's two pages disagree (Teensy doc, "The two PJRC pages disagree");
- pin 13, which drives the onboard LED. The firmware uses that LED as a status light (section 10).

All of these pins are on the board's edge headers, so a solderless breadboard reaches them.

**One frame rate for every thruster group.** The six thruster timer groups are all FlexPWM. At start-up and on every accepted `CF`, the firmware calls `analogWriteFrequency(pin, rate_hz)` on each thruster pin, which sets that pin's whole timer group, and then `analogWriteResolution(16)`. FlexPWM groups compute their period the same way from the same 150 MHz bus clock (Teensy doc, "How the core picks the period"), so all six groups run at the identical configured rate. Two pins that share a group (2/3 and 6/9) share it by construction. The pulse width is set with a 16-bit duty value:

```
duty = round(pulse_us * rate_hz * 65536 / 1,000,000)
```

Resolution derived from the Teensy doc's calculation: about 0.43 µs per step at 50 Hz and about 0.053 µs per step at 400 Hz. The doc derives these from source, not measurement, so they must be measured on hardware (Teensy doc, Open Question 9).

**Reserved servo channel.** Channel 8 (pin 10) is reserved for the Axon servo in a later minor version. In version 1.0 the firmware drives pin 10 as a digital output held LOW (no pulses). No command addresses it: `SA` carries channels 0 to 7 only, and the `CF` mask covers bits 0 to 7 only. The watchdog never touches it. Its timer group (QuadTimer1.0) is separate from every thruster group, so a servo frame rate chosen later cannot change the thruster rate. The watchdog behavior for the servo channel (hold position or go limp) is deliberately left for the version that drives it.

## 5. Configuration and its defaults

The firmware holds one configuration in RAM. Nothing is stored in EEPROM, so every boot starts from these defaults, and the host sends `CF` every time it connects.

| Setting | Boot default | Allowed by `CF` |
|---|---|---|
| `rate_hz` | 200 | 50, 100, 200 or 400 |
| `watchdog_ms` | 200 | 50 to 1000 |
| `mask` (active thruster channels) | `FF` (all 8 active) | `01` to `FF` |
| `min_us` | 1100 | 800 to 2200 |
| `stop_us` | 1500 | 800 to 2200 |
| `max_us` | 1900 | 800 to 2200 |

Rules checked by `CF`: `min_us < stop_us < max_us`, and `max_us <= 1,000,000 / rate_hz - 100`, which keeps a pulse at least 100 µs shorter than its frame. The second rule only binds at 400 Hz (2500 µs frame), where it limits `max_us` to 2400, so the 2200 µs hard limit always satisfies it. The 800 to 2200 µs hard limits sit inside the 900 to 2235 µs window that the ESC's BLHeli_S firmware counts as in range (ESC doc, "Valid pulse window"); a custom ESC that needs a wider range is outside what this firmware supports.

Why the defaults: the Basic ESC's documented 1100/1500/1900 µs values, not its internal 1112/1512/1912 (ESC doc, "The driver should command the documented ... values"). 200 Hz because Blue Robotics "generally use a 200Hz update rate, and don't recommend below 50Hz", and the ESC accepts up to 400 Hz (ESC doc).

An **inactive** channel (its bit clear in `mask`) outputs no pulses at all: the pin is held LOW. An ESC that never sees a pulse never arms, so a channel the host has not declared cannot drive a thruster. At boot every channel is active at `stop_us`, because the host has not yet sent a mask.

## 6. Commands

Each command lists its request, its success reply and its specific errors. Section 7 covers error replies in general.

### 6.1 `ID`: identify

```
request:  $ID*hh
reply:    $ID,teensy_thrusters,<protocol>,<firmware>,<thrusters>,<servo_channel>,<serial>*hh
example:  $ID,teensy_thrusters,1.0,1.0.0,8,8,12345670*hh
```

| Field | Meaning |
|---|---|
| `teensy_thrusters` | Firmware name. The host refuses any other value. |
| `<protocol>` | Protocol version `MAJOR.MINOR` (section 11). |
| `<firmware>` | Firmware version `MAJOR.MINOR.PATCH`, independent of the protocol version. |
| `<thrusters>` | Number of thruster channels (8). |
| `<servo_channel>` | Index of the reserved servo channel (8). |
| `<serial>` | The board's unique number, computed exactly as the Teensy core computes its USB serial-number string: the low 24 bits of `HW_OCOTP_MAC0`, multiplied by 10 if below 10,000,000 (Teensy doc, "Unique serial number"). It matches the `serial_number` that pySerial reports. |

`ID` changes no state and does not feed the watchdog.

### 6.2 `SA`: set all thruster channels (also the keepalive)

```
request:  $SA,<p0>,<p1>,<p2>,<p3>,<p4>,<p5>,<p6>,<p7>*hh
reply:    $OK,SA*hh
```

Exactly 8 pulse widths in µs, one per thruster channel, in channel order. The frame is accepted only if every value passes:

- an active channel's value lies in `[min_us, max_us]`;
- an inactive channel's value equals `stop_us`.

If any value fails, the **whole frame** is rejected (`RANGE` or `INACTIVE`), no output changes, and the watchdog is not fed. The firmware never clamps an out-of-range value into range, because clamping turns a host bug into motion. An accepted `SA` applies every channel at once, moves the state to `RUN` and feeds the watchdog.

`SA` is rejected with `ARMING` while an arm hold runs (6.4), and with `NO_HOST` while DTR is low (section 9).

The host sends `SA` at a fixed keepalive rate well inside `watchdog_ms`, repeating the current command set even when nothing has changed. The NEPI node sends it every `watchdog_ms / 4`, limited to between 20 and 100 ms, so 50 ms (20 Hz) at the default 200 ms watchdog.

### 6.3 `SP`: stop all

```
request:  $SP*hh
reply:    $OK,SP*hh
```

Sets every active thruster channel to `stop_us` and moves the state to `IDLE`. It is accepted in every state, including while DTR is low, because a stop is never unsafe. During an arm hold it is acknowledged and the hold continues.

### 6.4 `AR`: arm hold

```
request:  $AR,<hold_ms>*hh       hold_ms: 2000 to 15000
reply:    $OK,AR*hh
```

Holds every active thruster channel at `stop_us` for `hold_ms` and rejects `SA` with `ARMING` until the hold ends; the state is `ARMING`. When the hold ends the state becomes `IDLE` and outputs stay at stop until the next accepted `SA`. A second `AR` during a hold restarts it with the new length.

Why: the ESC "will not drive until it has seen 1500 µs for a few seconds" after power-up, and Blue Robotics' example holds 1500 µs for 7 s (ESC doc, "How long to hold stop"). Without a hold, the host's next thrust command could reach an ESC that is still initializing. The host sends `AR` after the Teensy boots and whenever the ESCs may have lost power. The NEPI node sends it at startup and whenever `ST` shows `cfg=0` (section 6.6). Its length is the driver's `arming_hold_s` option (default 7 s).

### 6.5 `CF`: configure

```
request:  $CF,<rate_hz>,<watchdog_ms>,<mask>,<min_us>,<stop_us>,<max_us>*hh
example:  $CF,200,200,3F,1100,1500,1900*hh
reply:    $OK,CF*hh
```

Field ranges and rules are in section 5. `<mask>` is two hex digits; bit *n* set means thruster channel *n* is active. If any field fails, the whole frame is rejected with `RANGE` and the old configuration stays in force. An accepted `CF`:

1. sets every active channel to the new `stop_us`, holds inactive channels LOW, and reprograms the timer frequency;
2. moves the state to `IDLE` (cancelling any arm hold);
3. sets the `cfg` flag that `ST` reports.

`CF` is accepted while DTR is low.

### 6.6 `ST`: status

```
request:  $ST*hh
reply:    $ST,<state>,<cfg>,<dtr>,<arm_ms_left>,<rate_hz>,<watchdog_ms>,<mask>,<min_us>,<stop_us>,<max_us>,<rx_errors>,<uptime_ms>,<p0>,...,<p7>*hh
example:  $ST,RUN,1,1,0,200,200,3F,1100,1500,1900,0,51234,1550,1500,1500,1500,1500,1500,0,0*hh
```

| Field | Meaning |
|---|---|
| `<state>` | `IDLE`, `ARMING`, `RUN` or `TRIPPED` (section 8). `ARMING` is the arm-hold state; `TRIPPED` is the watchdog-tripped flag. |
| `<cfg>` | `1` once a `CF` has been accepted since boot, else `0`. **`0` while the host believes it configured the board means the Teensy rebooted**, and the host must send `CF` and `AR` again. |
| `<dtr>` | `1` while DTR is asserted. |
| `<arm_ms_left>` | Milliseconds left in the arm hold, `0` outside `ARMING`. |
| `<rate_hz>` ... `<max_us>` | The configuration in force (section 5). |
| `<rx_errors>` | Count of lines rejected since boot, of every error code. Wraps at 2^32. |
| `<uptime_ms>` | `millis()` since boot. Wraps after about 49.7 days. |
| `<p0>` ... `<p7>` | Pulse width in µs each thruster channel is outputting now; `0` for an inactive channel (no pulses). |

There are no pushed messages; the host polls `ST` at whatever rate it wants. The NEPI node polls it at 2 Hz, the rate `MotorsDeviceIF` publishes `MotorsStatus` at. `ST` changes no state and does not feed the watchdog, so a host whose status loop runs while its command loop has died still trips the watchdog.

## 7. Errors

```
$ER,<code>,<cmd>*hh
```

`<cmd>` is the two-letter command if one could be read, otherwise `--`. A rejected frame never changes outputs, state or configuration, and never feeds the watchdog.

| Code | Cause |
|---|---|
| `CHECKSUM` | No `*`, not exactly two hex digits after it, or the checksum does not match. |
| `FORMAT` | Not of the form in section 3: bad characters (including spaces), a command that is not two uppercase letters, an empty or non-numeric field, or a number too long. |
| `OVERFLOW` | More than 120 characters without a line end. The firmware discards bytes up to the next `\n`, then replies once. |
| `UNKNOWN` | Valid frame, unknown command. |
| `FIELDS` | Wrong number of fields for the command. |
| `RANGE` | A field outside its allowed range or rule (sections 5, 6.2, 6.4). |
| `INACTIVE` | `SA` carried a value other than `stop_us` for an inactive channel. |
| `ARMING` | `SA` during an arm hold. Expected while arming; the NEPI node treats it as state, not failure. |
| `NO_HOST` | `SA` while DTR is low. |

## 8. State machine and the watchdog

| State | Outputs | Entered by |
|---|---|---|
| `IDLE` | Every active channel at `stop_us` | Boot; `SP`; accepted `CF`; end of an arm hold; DTR falling |
| `ARMING` | Every active channel at `stop_us`; `SA` rejected | `AR` |
| `RUN` | The last accepted `SA` values | Accepted `SA` |
| `TRIPPED` | Every active channel at `stop_us` | Watchdog timeout in `RUN` |

**Watchdog.** In `RUN`, if no accepted `SA` or `SP` arrives within `watchdog_ms` of the last one, every active channel goes to `stop_us` and the state becomes `TRIPPED`. It stays there until the next accepted `SA`, which applies its values and returns to `RUN`. Only `SA` and `SP` feed the watchdog. `ID`, `ST`, `CF`, `AR` and every rejected frame do not, so a host that keeps polling status but stops commanding still trips it.

**Why the watchdog is the only stop.** The ESC stops by itself after roughly 0.3 s without pulses (ESC doc, derived from the BLHeli_S source). This firmware never stops pulsing, because it always holds `stop_us` at minimum, so that ESC behavior never fires while the firmware runs. When the Pi, the NEPI node or the USB link dies, the firmware watchdog is the only thing that stops the thrusters.

**What the watchdog does not cover: a firmware hang.** The pulses are made by PWM hardware, which keeps outputting the last duty value even if the firmware's main loop stops. Version 1.0 does not enable the i.MX RT1062 hardware watchdog, so a hung firmware leaves the thrusters at their last command. The host detects this (no replies) and the NEPI node exits, but it cannot stop the pulses. The vehicle's power kill remains the stop for that case. Enabling the hardware watchdog is a listed follow-up.

## 9. Boot, connect and disconnect

**Before the firmware runs** (power-up, bootloader, programming, crash before `setup()`): every pin is an input with a keeper, so no pulses come out (Teensy doc, "Before the firmware runs"). An ESC without pulses never arms. The pull-downs in section 10 keep the signal lines LOW in this window.

**At boot**, the first instructions in `setup()` set the PWM resolution, the frame rate and `stop_us` (1500 µs) on every thruster channel, before USB is started. The firmware does not wait for a host (`while (!Serial)` is never used), so it pulses stop with or without one. The state is `IDLE` with `cfg=0`.

**When the host connects** (DTR rises), nothing moves. The host sends `ID`, then `CF`, then `AR`, then starts its `SA` keepalive.

**When DTR drops** (the host closed the port, or the process holding it died): every active channel goes to `stop_us`, any arm hold is cancelled, and the state becomes `IDLE`. While DTR stays low, `SA` is rejected with `NO_HOST`; `ID`, `ST`, `CF`, `SP` and `AR` are still answered. The configuration and the `cfg` flag are kept.

**If USB disconnects or the Teensy resets**, the board re-enumerates, possibly as a different `/dev/ttyACM*`. A Teensy reset stops the pulses until `setup()` runs again, and that gap is longer than the ESC's roughly 0.3 s signal-loss timeout, so the ESCs stop and need stop pulses again before they drive. The firmware's boot `stop_us` provides those, and the host's `AR` after reconnecting guarantees the hold.

## 10. Wiring

```
Teensy pin (2,3,4,5,6,9,22,23) ──┬─────────── ESC signal (white)
                                 │
                               10 kΩ  pull-down, at the Teensy end
                                 │
Teensy GND ──────────────────────┴─────────── ESC signal ground (black)
```

- **Signal and ground only.** The ESC's signal connector is a 3-position 0.1 in servo connector carrying ground, empty and signal; the middle position is empty, so it carries no power (ESC doc). The ESC has no BEC.
- **Common ground.** The ESC's signal ground must connect to a Teensy ground pin (ESC doc, "Signal level and ground").
- **3.3 V levels are fine.** The ESC accepts 3.3 to 5 V signals (ESC doc). Teensy pins are 3.3 V only and not 5 V tolerant (Teensy doc), so never connect anything that could drive a signal line to 5 V.
- **Pull-down on every signal line (recommended).** At startup the ESC firmware checks whether its input "is high for more than 15ms" and, if so, jumps to its bootloader instead of running (ESC doc, derived from the BLHeli_S source). Teensy pins default to inputs with keeper resistors (Teensy doc), which can hold a line at whatever level it last saw, and an unpowered or rebooting Teensy leaves the line floating. A 10 kΩ resistor from each signal line to ground holds it LOW whenever the Teensy is not driving it. Running firmware never holds a line high for more than one pulse (at most 2.2 ms), and an inactive or reserved channel is driven LOW.
- **Power backfeed.** If the Teensy is also powered from the vehicle on VIN while USB goes to the Pi, cut the VUSB-VIN pads first, or current can flow back into the Pi (Teensy doc, "Power").
- **Pin 13 LED** shows state: off with no host (DTR low), fast blink while arming, slow blink when the watchdog has tripped, on otherwise.

## 11. Versioning

- **Protocol version `MAJOR.MINOR`, currently 1.0.** `MINOR` increments for additive changes only: a new command, or new fields appended to the end of a reply. A host must ignore reply fields beyond the ones it knows. `MAJOR` increments for any change to an existing command's fields, order, meaning or ranges.
- **The host refuses a different major version.** NEPI discovery launches no node for a board whose `ID` reports a protocol major other than the one the driver was written for, and the node repeats the check when it connects. The firmware cannot check the host's version, so the host enforces compatibility. An older host that sends a newer firmware a frame it does not understand gets `FIELDS` or `UNKNOWN` and causes no motion.
- **Firmware version `MAJOR.MINOR.PATCH`** tracks the sketch itself and appears in the NEPI device info. A firmware change that does not change the protocol bumps only this.
- **Reserved for later minor versions:** servo-channel commands for channel 8, and sensor reads for the I2C sensors the Teensy doc lists (BNO085, Bar depth sensor, MS8607). Neither exists in 1.0.

## 12. Coverage of the Teensy doc's "To Be Defined" list

| Item | Covered in |
|---|---|
| 1. Command set | Section 6. Thruster pulses are covered; sensor reads are reserved (section 11) because no sensor is wired to the Teensy yet. |
| 2. Framing and checksum | Sections 3 and 7 |
| 3. Identify reply | Section 6.1 |
| 4. Telemetry rate | Section 6.6: polled, no push; host chooses the rate (NEPI uses 2 Hz) |
| 5. Command watchdog | Section 8; re-arm in 6.4; the servo channel's watchdog behavior is deferred to the version that drives it (section 4) |
| 6. Connect and disconnect behavior | Section 9 |
| 7. Channel and pin map | Section 4 |
| 8. Versioning | Section 11 |

## 13. Debugging with a serial terminal

```
picocom --omap crlf --echo /dev/ttyACM0   # asserts DTR; Enter sends \r\n; baud is irrelevant (never 134)
$ID*0D                                    # identify   ('I'^'D' = 0x0D)
$ST*07                                    # status
$SP*03                                    # stop all
```

Checksums for the other commands are easiest to compute with a one-liner:

```
python3 -c "import functools,sys; b=sys.argv[1]; print('\$%s*%02X' % (b, functools.reduce(lambda a,c: a^ord(c), b, 0)))" "SA,1550,1500,1500,1500,1500,1500,1500,1500"
```

Type `$SA` frames by hand only with the propellers clear or in water, and remember the watchdog: a hand-typed `SA` runs for `watchdog_ms` and then stops.
