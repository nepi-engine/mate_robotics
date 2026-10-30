# teensy_thrusters: Pi-to-Teensy Protocol, Version 1.2

This document specifies the serial protocol between the NEPI device (a Raspberry Pi 5) and a PJRC Teensy 4.1 that generates the servo-style pulses for Blue Robotics Basic ESCs (BESC30-R3), each driving one T200 thruster, and for up to four hobby servos, and that reads a Blue Robotics Bar02 or Bar30 depth sensor. These files implement it and are meant to be bench-tested together:

| Side | File |
|---|---|
| Teensy firmware | `Firmware/teensy_thrusters/teensy_thrusters.ino` |
| NEPI discovery | `Drivers/svx_drivers/Teensy/svx_teensy_thrusters_discovery.py` |
| NEPI thruster node | `Drivers/svx_drivers/Teensy/svx_teensy_thrusters_node.py` |
| NEPI servo node (one per servo) | `Drivers/svx_drivers/Teensy/svx_teensy_thrusters_servo_node.py` |
| NEPI depth sensor node | `Drivers/svx_drivers/Teensy/svx_teensy_thrusters_depth_node.py` |

How to compile the firmware and flash it from the NEPI device is in `FLASHING.md` in this folder.

Hardware facts used here come from `Docs/tbd/pjrc_teensy_4_1.md` (the Teensy), `Docs/Motors/bluerobotics_basic_esc.md` (the ESC), `Docs/Motors/bluerobotics_t200_thruster.md` (the thruster), `Docs/svx/axon_max_mk2_servo.md` (the servo) and `Docs/npx/bluerobotics_bar_depth_sensor.md` (the depth sensor, "the Bar doc").

**What 1.1 adds to 1.0.** Four servo channels and the commands that drive them (`SC`, `SL`, `SS`, `SO`, `SV`, sections 6.7 to 6.11), one field appended to the `ID` reply, and the rules for several host processes sharing the port (section 2.1). Every 1.0 command, reply and behavior is unchanged, so a 1.0 host still drives the thrusters of a 1.1 board. This specification covers every item in the Teensy document's "To Be Defined" list; section 12 maps each item to the section that covers it.

**What 1.2 adds to 1.1.** A Bar depth sensor on the Teensy's I2C port and the three commands that read it (`DC`, `DV`, `DP`, sections 6.12 to 6.14). The firmware does not touch I2C until a host sends `DC`, so a host that never does gets exactly the 1.1 behavior, and every 1.1 command, reply and behavior is unchanged. The `ID` reply is unchanged too: a host knows the depth commands exist from the protocol version.

## 1. Why this protocol exists

Neither the T200 nor the Basic ESC has a data link. The ESC takes one-way servo pulses on one signal wire and reports nothing back, so software cannot tell whether an ESC is connected, powered or armed (ESC doc, "The ESC cannot be detected from software"). The Teensy is the only device the Pi talks to. The protocol therefore carries pulse widths, not thrust, speed or RPM, and nothing in it claims to know what a thruster is doing.

## 2. Transport

- USB type "Serial" (CDC-ACM). The board appears on Linux as `/dev/ttyACM*` with VID `0x16C0`, PID `0x0483`, manufacturer "Teensyduino" and product "USB Serial". Any Teensyduino sketch shows those same IDs, so the host must confirm the firmware with the `ID` command (section 6.1) before it sends anything else.
- The baud rate is ignored. The host opens the port at 115200. **The host must never open the port at 134 baud**, because that rate is the request that reboots a running Teensy into its bootloader, and in the bootloader every pulse stops (Teensy doc, "134 baud is reserved").
- The host must assert DTR. pySerial does this by default on open, and so do common terminal programs (picocom, minicom, screen). Section 9 describes what DTR controls.
- The USB serial number is unique per board (Teensy doc, "Unique serial number") and is the key the NEPI driver uses to identify a board, not the `/dev/ttyACM` index.

### 2.1 Several host processes on one port

The NEPI driver runs one process for the thrusters and one per servo, and they all open the same `/dev/ttyACM*`. The firmware cannot tell them apart; the rules that keep them from corrupting each other are host-side:

- **One transaction at a time.** A host holds an exclusive advisory lock (`flock(LOCK_EX)`) on its open port from just before it writes a request until it has read the reply or given up. No process opens the port with pySerial's `exclusive=True` while it runs, because that holds the same lock for the life of the open and would lock every other process out.
- **Open under the lock too.** Opening a tty with pySerial flushes its input queue, and that queue is shared by every process holding the port, so a process that opens the port while another is mid-transaction throws that process's reply away. The nodes therefore take the same lock (on a short-lived second descriptor) around the open itself.
- **Match the reply to the request.** A reply that arrives after its requester gave up can be read by the next process. Every reply names its command (`OK,<cmd>`, `ER,<code>,<cmd>`, `ID`, `ST`, `SV`, `DV`, `DP`), so a host discards any reply that does not name the command it sent and keeps reading until its own timeout.
- **DTR is shared.** DTR stays asserted while any process holds the port open and falls only when the last one closes it (section 9). One process exiting therefore does not stop the thrusters through DTR; the thruster watchdog does that within `watchdog_ms` if the thruster process was the one that died.

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

The firmware drives 8 thruster channels and 4 servo channels. Both counts are fixed when the firmware is compiled and are reported by `ID`. Thruster commands number thruster channels 0 to 7; servo commands number servos 0 to 3.

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
| servo 0 | Servo | 10 | QuadTimer1.0 |
| servo 1 | Servo | 11 | QuadTimer1.2 |
| servo 2 | Servo | 12 | QuadTimer1.1 |
| servo 3 | Servo | 14 | QuadTimer3.2 |

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

### 4.1 Servo channels

Each servo pin is the only PWM pin on its own QuadTimer channel, and none is on a thruster timer, so the servo frame rate (`SC`, section 6.7) can never change the thruster rate. All four servo groups are set to the one servo rate. Pins 10, 11 and 12 are also SPI0 and pin 14 is also Serial3 TX; this firmware uses neither. Pin 14's QuadTimer3 module also owns pins 18 and 19, but as separate channels, so using 14 for PWM leaves I2C on 18/19 untouched.

- **Off at boot.** Every servo pin is a digital output held LOW (no pulses) from `setup()` until the host sends `SS` for it. A hobby servo with no pulses is limp, so nothing moves at power-up.
- **No arming.** Servos need no arm hold; `AR` does not touch them.
- **Not on the thruster watchdog.** Servo commands never feed the thruster watchdog, and a watchdog trip never moves a servo. A servo keeps its last pulse until `SS`, `SO`, `SL` or a host loss changes it, because a gripper that lets go when the thrusters time out would drop what it holds.
- **Host loss.** When DTR falls (section 9), servos keep their pulses if `host_loss` is `HOLD` (the default) and go off if it is `LIMP`.

The servos are slow, non-thrust actuators, so a servo held at its last position is not a runaway the way a thruster held at its last command is. That is why servos have no watchdog of their own.

### 4.2 Depth sensor port

The Bar depth sensor goes on `Wire`, I2C port 0: SDA on pin 18, SCL on pin 19. The sensor's address, 0x76, is fixed, so it is the only Bar sensor on that bus (Bar doc, "Bus and address").

- **Why `Wire`.** Section 4 already keeps the thruster and servo outputs off pins 18 and 19, so `Wire` costs no output this firmware uses. Pin 14's QuadTimer3 module also owns 18 and 19, but as separate channels (section 4.1). It is also where the MATE bench already wired its Bar02 (`Nepi_Connect/BENCH_SETUP_PLAN.md`, "Bar02 wired to the Teensy").
- **Bus speed.** 100 kHz, the `Wire` default and the speed Blue Robotics' Arduino library runs at. The sensor allows 400 kHz, but the Bar sensor's cable is about 30 cm and its board has no pull-ups (Bar doc), so the slower edge rate leaves more margin.
- **Untouched until asked.** `Wire.begin()` runs on the first accepted `DC` with a non-zero ratio, never at boot. Until then pins 18 and 19 stay inputs, as in 1.1.
- **One more 0x76 part.** The Adafruit MS8607 in this kit also answers at 0x76 (Bar doc, "Address conflict"). On this bus it would fail the calibration check and read as `BAD_PROM` (section 6.13); it belongs on another bus.

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

### 5.1 Servo configuration and its defaults

Also held in RAM only. Every servo node sends `SC` and its own `SL` when it connects.

| Setting | Boot default | Allowed by `SC` / `SL` |
|---|---|---|
| `servo_rate_hz` (all servos) | 50 | 50, 100, 200 or 333 |
| `host_loss` (all servos) | `HOLD` | `HOLD` or `LIMP` |
| per-servo `min_us` | 1000 | 500 to 2500 |
| per-servo `max_us` | 2000 | 500 to 2500 |

Rules: `min_us < max_us`, and `max_us <= 1,000,000 / servo_rate_hz - 100`. That rule only binds at 333 Hz (3003 µs frame), where 2500 µs still fits. The 500 to 2500 µs hard limits are the Axon MAX MK2's range (servo doc) and the common hobby-servo range. 50 Hz is the default because a servo that "works with" standard robot-controller ports gets 50 Hz pulses (servo doc); faster rates suit digital servos rated for them, such as the DS3235's 50 to 330 Hz. The 1000 to 2000 µs boot range is deliberately narrow, so that a servo commanded before its node has pushed its calibration cannot be driven into its end stops.

### 5.2 Depth sensor configuration and its defaults

Also held in RAM only. The depth sensor node sends `DC` when it connects.

| Setting | Boot default | Allowed by `DC` |
|---|---|---|
| sampling | off | on, at one ratio below; `0` turns it off |
| oversampling ratio (OSR) | 1024 once turned on | 256, 512, 1024, 2048, 4096 or 8192 |

The six ratios are the sensor's own, and a higher one is quieter but slower (Bar doc, "Conversion timing"). The firmware waits each conversion's maximum time from the datasheet before reading it, using the Bar30's figure because it is the longer of the two models, plus about 5%: 0.65, 1.25, 2.4, 4.8, 9.5 or 19 ms. A sample is one pressure (D1) and one temperature (D2) conversion. A new sample starts at most every 20 ms, so the firmware samples at most 50 times a second at any ratio. That also limits how much of the time the sensor spends converting, which warms it (Bar doc, "Self-heating at high OSR").

## 6. Commands

Each command lists its request, its success reply and its specific errors. Section 7 covers error replies in general.

### 6.1 `ID`: identify

```
request:  $ID*hh
reply:    $ID,teensy_thrusters,<protocol>,<firmware>,<thrusters>,<servo_channel>,<serial>,<servo_count>*hh
example:  $ID,teensy_thrusters,1.2,1.2.0,8,8,12345670,4*hh
```

| Field | Meaning |
|---|---|
| `teensy_thrusters` | Firmware name. The host refuses any other value. |
| `<protocol>` | Protocol version `MAJOR.MINOR` (section 11). |
| `<firmware>` | Firmware version `MAJOR.MINOR.PATCH`, independent of the protocol version. |
| `<thrusters>` | Number of thruster channels (8). |
| `<servo_channel>` | Index of the first servo channel when thruster and servo channels are counted together (8). Kept from 1.0, where it named the one reserved servo channel; servo commands use servo numbers 0 to `servo_count - 1`, not this index. |
| `<serial>` | The board's unique number, computed exactly as the Teensy core computes its USB serial-number string: the low 24 bits of `HW_OCOTP_MAC0`, multiplied by 10 if below 10,000,000 (Teensy doc, "Unique serial number"). It matches the `serial_number` that pySerial reports. |
| `<servo_count>` | Number of servo channels (4). Added in 1.1; a 1.0 reply has no eighth field, and a host treats that as 0 servos. |

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

There are no pushed messages; the host polls `ST` at whatever rate it wants. The NEPI node polls it at 2 Hz, the rate `MotorsDeviceIF` publishes `MotorsStatus` at. `ST` changes no state and does not feed the watchdog, so a host whose status loop runs while its command loop has died still trips the watchdog. `ST` reports thrusters only; servos are in `SV` (6.11).

### 6.7 `SC`: servo configure

```
request:  $SC,<rate_hz>,<host_loss>*hh       host_loss: HOLD or LIMP
example:  $SC,50,HOLD*hh
reply:    $OK,SC*hh
```

Sets the frame rate of every servo channel and what servos do when DTR falls (sections 4.1 and 5.1). It is rejected with `RANGE` if the rate is not allowed or if any servo's current `max_us` would not fit the new frame. A rate change reprograms the servo timers and recomputes every running servo's duty, so no servo moves. An accepted `SC` sets the `scfg` flag that `SV` reports. `SC` is accepted while DTR is low. Every servo node sends the same values, taken from the driver's options, so repeated `SC` frames are harmless.

### 6.8 `SL`: servo limits

```
request:  $SL,<n>,<min_us>,<max_us>*hh       n: 0 to servo_count - 1
example:  $SL,0,500,2500*hh
reply:    $OK,SL*hh
```

Sets servo *n*'s allowed pulse range (rules in 5.1). It is the firmware's copy of the servo's calibration, the way the Maestro driver pushes its pulse endpoints to the board's own clamp, so a host bug cannot drive a servo past the endpoints the operator calibrated. If servo *n* is outputting a pulse outside the new range, it goes **off**, not to the nearest end: moving it would be motion nobody commanded. Accepted while DTR is low.

### 6.9 `SS`: set servo

```
request:  $SS,<n>,<pulse_us>*hh
example:  $SS,0,1500*hh
reply:    $OK,SS*hh
```

Drives servo *n* at `pulse_us`. A value outside servo *n*'s `[min_us, max_us]` is rejected with `RANGE` and nothing changes; it is never clamped. Rejected with `NO_HOST` while DTR is low. Does not feed the thruster watchdog.

### 6.10 `SO`: servo off

```
request:  $SO,<n>*hh
reply:    $OK,SO*hh
```

Stops pulsing servo *n* (pin held LOW), so the servo goes limp. Accepted in every state, including while DTR is low.

### 6.11 `SV`: servo status

```
request:  $SV*hh
reply:    $SV,<scfg>,<rate_hz>,<host_loss>,<p0>,<min0>,<max0>,<p1>,<min1>,<max1>,<p2>,<min2>,<max2>,<p3>,<min3>,<max3>*hh
example:  $SV,1,50,HOLD,1500,500,2500,0,1000,2000,0,1000,2000,0,1000,2000*hh
```

| Field | Meaning |
|---|---|
| `<scfg>` | `1` once an `SC` has been accepted since boot, else `0`. **`0` while a servo node believes it configured the board means the Teensy rebooted**: its servo is off and its limits are back at the boot defaults, so the node sends `SC` and `SL` again. |
| `<rate_hz>`, `<host_loss>` | The servo configuration in force. |
| `<pN>` | Pulse width servo *N* is outputting now; `0` when off. |
| `<minN>`, `<maxN>` | Servo *N*'s allowed range. |

`SV` changes no state.

### 6.12 `DC`: depth sensor configure

```
request:  $DC,<osr>*hh       osr: 256, 512, 1024, 2048, 4096 or 8192; 0 turns sampling off
example:  $DC,1024*2C
reply:    $OK,DC*hh
```

A non-zero `<osr>` starts the firmware sampling the depth sensor in the background at that ratio (section 5.2); the first `DC` also starts `Wire` (section 4.2). `0` stops sampling. A ratio that is not one of the six, and not `0`, is rejected with `RANGE` and nothing changes. An accepted `DC`:

1. from off, moves the state to `SEARCHING`, and the sensor is probed on the next pass of the firmware's loop;
2. with sampling already on, changes only the ratio, which applies from the next conversion. A conversion in progress keeps its own wait, and mixing ratios within one sample is harmless, because the ratio sets the resolution of D1 and D2, not their scale;
3. sets the `dcfg` flag that `DV` reports.

`DC` is accepted while DTR is low. It never touches the thrusters, the servos or the watchdog.

**Probing.** While the state is `SEARCHING` or `BAD_PROM`, the firmware sends the sensor its reset command, waits 10 ms as Blue Robotics' libraries do, and reads the seven calibration words (Bar doc, "Startup and Initialization"). A sensor passes when the words' CRC-4 matches the top four bits of word 0 and no coefficient (words 1 to 6) is 0 or 0xFFFF, since an all-zero PROM also passes the CRC. Passing moves the state to `OK` and counts one detection. If nothing acknowledges the reset, the state is `SEARCHING`; if something does but its words fail, `BAD_PROM`. Either way the next probe is one second later.

**Sampling.** In `OK`, each sample is: convert D1, wait, read it; convert D2, wait, read it. A read that fails, or that returns 0 (the sensor's sign of a read that was early or overlapped a conversion), discards the sample, counts one error, and waits one full 8192-ratio conversion before the next command, in case the failed command started a conversion. Three failures in a row move the state back to `SEARCHING`, which re-probes once a second.

### 6.13 `DV`: depth sensor value

```
request:  $DV*12
reply:    $DV,<dcfg>,<state>,<osr>,<detects>,<seq>,<age_ms>,<d1>,<d2>,<errors>*hh
example:  $DV,1,OK,1024,1,1532,12,6465444,8077636,0*hh
```

| Field | Meaning |
|---|---|
| `<dcfg>` | `1` once a `DC` has been accepted since boot, else `0`. **`0` while the depth sensor node believes it configured the board means the Teensy rebooted**: sampling is off, and the node sends `DC` again. |
| `<state>` | `OFF` (no `DC` yet, or the last one was `0`), `SEARCHING` (nothing acknowledges at 0x76, or not probed yet), `BAD_PROM` (something acknowledges but its calibration words fail the check) or `OK` (sampling). |
| `<osr>` | The ratio in force; `0` when `OFF`. |
| `<detects>` | How many times a sensor has passed the probe since boot. It changes when a sensor is re-detected, which may be a different sensor, so the host reads `DP` again whenever it changes. |
| `<seq>` | How many samples have completed since boot. The host tells a new sample from a repeat by this. Wraps at 2^32. |
| `<age_ms>` | Milliseconds since the reported sample completed; `0` before the first sample. A sample is kept, and keeps ageing, through failed reads and a lost sensor, so a host must read `<state>` and `<age_ms>`, not just the values. |
| `<d1>`, `<d2>` | The latest sample's raw 24-bit pressure (D1) and temperature (D2) readings; `0` before the first sample. |
| `<errors>` | Failed reads while `OK` since boot (section 6.12, Sampling). Wraps at 2^32. |

The values are raw because turning them into pressure and temperature needs the calibration words and the sensor model, and the model cannot be read from the chip (Bar doc, "Identification and Detection"). The host does the conversion with the Bar doc's first- and second-order formulas. `DV` changes no state.

### 6.14 `DP`: depth sensor calibration

```
request:  $DP*14
reply:    $DP,<detects>,<w0>,<w1>,<w2>,<w3>,<w4>,<w5>,<w6>*hh
example:  $DP,1,32768,46372,43981,29059,27842,31553,28165*hh
```

The seven calibration words (decimal, 0 to 65535) read at the latest detection, and the `<detects>` count they belong to: word 0 holds the CRC and version bits, and words 1 to 6 are the coefficients C1 to C6 (Bar doc, "PROM layout"). The example carries the Bar02 datasheet coefficients, with the CRC they compute to (8) in word 0's top four bits. All zero before the first detection. A host pairs these words only with `DV` samples that report the same `<detects>`, so a sensor swapped between the two requests is never converted with the other sensor's calibration. `DP` changes no state.

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
| `RANGE` | A field outside its allowed range or rule (sections 5, 5.1, 6.2, 6.4, 6.7 to 6.10, 6.12), including a servo number that does not exist. |
| `INACTIVE` | `SA` carried a value other than `stop_us` for an inactive channel. |
| `ARMING` | `SA` during an arm hold. Expected while arming; the NEPI node treats it as state, not failure. |
| `NO_HOST` | `SA` or `SS` while DTR is low. |

## 8. State machine and the watchdog

| State | Outputs | Entered by |
|---|---|---|
| `IDLE` | Every active channel at `stop_us` | Boot; `SP`; accepted `CF`; end of an arm hold; DTR falling |
| `ARMING` | Every active channel at `stop_us`; `SA` rejected | `AR` |
| `RUN` | The last accepted `SA` values | Accepted `SA` |
| `TRIPPED` | Every active channel at `stop_us` | Watchdog timeout in `RUN` |

**Watchdog.** In `RUN`, if no accepted `SA` or `SP` arrives within `watchdog_ms` of the last one, every active channel goes to `stop_us` and the state becomes `TRIPPED`. It stays there until the next accepted `SA`, which applies its values and returns to `RUN`. Only `SA` and `SP` feed the watchdog. `ID`, `ST`, `CF`, `AR` and every rejected frame do not, so a host that keeps polling status but stops commanding still trips it.

**Why the watchdog is the only stop.** The ESC stops by itself after roughly 0.3 s without pulses (ESC doc, derived from the BLHeli_S source). This firmware never stops pulsing, because it always holds `stop_us` at minimum, so that ESC behavior never fires while the firmware runs. When the Pi, the NEPI node or the USB link dies, the firmware watchdog is the only thing that stops the thrusters.

**What the depth sensor can do to the watchdog: delay it, never stop it.** The firmware checks the watchdog, DTR and the arm hold first in every pass of its loop, and services the depth sensor last, one or two short I2C transactions at a time; it never waits for a conversion. A healthy transaction takes well under a millisecond at 100 kHz, and reading the calibration words about 2 ms. A faulty bus is bounded by the Teensy 4 `Wire` library, whose master calls give up after about 50 ms plus a 16 ms wait for the bus to go idle, with a bus-clearing attempt (`WireIMXRT.cpp`, unchanged for master transfers since 2023, so in Teensyduino 1.59). One loop pass can therefore be delayed by about 70 ms, which can postpone a watchdog trip by that much but cannot skip it. After three failures in a row the sensor is only re-probed once a second, so a dead bus costs at most one such delay a second. A delay never causes a false trip either: the queued `SA` frames are handled at the start of the next pass, before the watchdog is checked. Sampling is off until a host sends `DC`, so a board whose host never asks for depth runs exactly as in 1.1.

**What the watchdog does not cover: a firmware hang.** The pulses are made by PWM hardware, which keeps outputting the last duty value even if the firmware's main loop stops. Version 1.0 does not enable the i.MX RT1062 hardware watchdog, so a hung firmware leaves the thrusters at their last command. The host detects this (no replies) and the NEPI node exits, but it cannot stop the pulses. The vehicle's power kill remains the stop for that case. Enabling the hardware watchdog is a listed follow-up.

## 9. Boot, connect and disconnect

**Before the firmware runs** (power-up, bootloader, programming, crash before `setup()`): every pin is an input with a keeper, so no pulses come out (Teensy doc, "Before the firmware runs"). An ESC without pulses never arms. The pull-downs in section 10 keep the signal lines LOW in this window.

**At boot**, the first instructions in `setup()` set the PWM resolution, the frame rate and `stop_us` (1500 µs) on every thruster channel, before USB is started. The firmware does not wait for a host (`while (!Serial)` is never used), so it pulses stop with or without one. The state is `IDLE` with `cfg=0`.

**When the host connects** (DTR rises), nothing moves. The host sends `ID`, then `CF`, then `AR`, then starts its `SA` keepalive.

**When DTR drops** (the last host process closed the port or died; section 2.1): every active channel goes to `stop_us`, any arm hold is cancelled, and the state becomes `IDLE`. Servos keep their pulses if `host_loss` is `HOLD` and go off if it is `LIMP`. Depth sampling carries on, since reading a sensor moves nothing. While DTR stays low, `SA` and `SS` are rejected with `NO_HOST`; every other command is still answered. The configuration and the `cfg`, `scfg` and `dcfg` flags are kept.

**Depth sensor at boot and on reset.** Sampling is off at boot, with `dcfg=0`, so after a Teensy reset `DV` reports `dcfg` 0 and `OFF`, and the depth sensor node sends `DC` again. The sensor itself needs no re-arming: its reset and calibration read are part of every probe (section 6.12).

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

**Servos.**

```
Teensy pin (10,11,12,14) ──┬─────────── servo signal
                           │
                         10 kΩ  pull-down (recommended)
                           │
Teensy GND ────────────────┴─────────── servo ground ──┬── servo supply GND
                                                       │
servo supply + (e.g. 6 V) ──────────────── servo + ────┘ (servo power, not from the Teensy)
```

- **Power the servos from their own supply**, never from a Teensy pin or its 3.3 V output. The Axon MAX MK2 runs on 4.8 to 8.4 V with 6.0 V recommended (servo doc), and the Basic ESCs provide no 5 V (ESC doc). Size the supply for stall current; a 35 kg-class servo can draw over 2 A stalled.
- **Common ground** between the servo supply, the servo and the Teensy.
- **The pull-down keeps the line LOW** while the Teensy is off or booting, so the servo stays limp instead of twitching on a floating line.
- **Do not wire the Axon's analog position output to a Teensy pin** until its voltage has been measured: the scale is not published (servo doc) and Teensy pins are 3.3 V only.

**Depth sensor.**

```
Teensy 3V (3.3 V out) ───────────────────── Bar red    (Vin, 2.5 to 5.5 V)
Teensy pin 18 (SDA) ──┬──────────────────── Bar white  (SDA)
                      └── 4.7 kΩ ── 3.3 V     pull-up (recommended)
Teensy pin 19 (SCL) ──┬──────────────────── Bar green  (SCL)
                      └── 4.7 kΩ ── 3.3 V     pull-up (recommended)
Teensy GND ──────────────────────────────── Bar black  (GND)
```

- **Power it from the Teensy's 3.3 V pin** (or any 2.5 to 5.5 V supply sharing the Teensy's ground). The sensor chip draws at most 1.25 mA, while converting (Bar doc), far inside the 250 mA the Teensy's 3.3 V output supplies (Teensy doc).
- **3.3 V logic only.** The sensor's SDA and SCL are 3.3 V lines, and 5 V logic damages it (Bar doc). Teensy pins are 3.3 V, so they connect directly; the pull-ups must go to 3.3 V, never 5 V.
- **Pull-ups.** The Bar board's 2016 schematic shows no pull-ups, so the host must provide them (Bar doc, Open Question 5). `Wire` enables the Teensy's own pull-ups, but PJRC calls them very weak and recommends external 1 to 4.7 kΩ resistors (Teensy doc, "I2C pull-ups"). The MATE bench's Bar02 answered on these pins wired with only its four leads (`Nepi_Connect/BENCH_SETUP_PLAN.md`), but resistors are cheap insurance on a 30 cm cable.
- **Keep the cable as supplied.** "A few centimeters of jumper wire is fine, but anything longer can cause unstable or complete communication failure" (Bar doc).
- **The back of the sensor is not waterproof.** It mounts through a pressure-housing wall (Bar doc, "Sealing"); the Teensy stays inside the housing.

## 11. Versioning

- **Protocol version `MAJOR.MINOR`, currently 1.2.** `MINOR` increments for additive changes only: a new command, or new fields appended to the end of a reply. A host must ignore reply fields beyond the ones it knows. `MAJOR` increments for any change to an existing command's fields, order, meaning or ranges.
- **The host refuses a different major version.** NEPI discovery launches no node for a board whose `ID` reports a protocol major other than the one the driver was written for, and the node repeats the check when it connects. The firmware cannot check the host's version, so the host enforces compatibility. An older host that sends a newer firmware a frame it does not understand gets `FIELDS` or `UNKNOWN` and causes no motion.
- **Firmware version `MAJOR.MINOR.PATCH`** tracks the sketch itself and appears in the NEPI device info. A firmware change that does not change the protocol bumps only this.
- **Servos need 1.1.** The NEPI driver launches servo nodes only for a board whose `ID` reports protocol 1.1 or later and a `servo_count` of at least the number asked for. A 1.0 board still gets its thruster node.
- **The depth sensor needs 1.2.** The NEPI driver launches a depth sensor node only for a board whose `ID` reports protocol 1.2 or later, and logs that the board needs reflashing otherwise. Older boards still get their thruster and servo nodes.
- **Reserved for later minor versions:** reads for the other I2C sensors the Teensy doc lists (BNO085, MS8607).

## 12. Coverage of the Teensy doc's "To Be Defined" list

| Item | Covered in |
|---|---|
| 1. Command set | Section 6. Thruster pulses (6.2 to 6.6), servos (6.7 to 6.11) and the Bar depth sensor (6.12 to 6.14) are covered; reads for the other I2C sensors are reserved (section 11). |
| 2. Framing and checksum | Sections 3 and 7 |
| 3. Identify reply | Section 6.1 |
| 4. Telemetry rate | Section 6.6: polled, no push; host chooses the rate (NEPI uses 2 Hz) |
| 5. Command watchdog | Section 8; re-arm in 6.4; servos are deliberately not on the watchdog and hold their position (section 4.1) |
| 6. Connect and disconnect behavior | Section 9 |
| 7. Channel and pin map | Section 4 |
| 8. Versioning | Section 11 |

## 13. Debugging with a serial terminal

```
picocom --omap crlf --echo /dev/ttyACM0   # asserts DTR; Enter sends \r\n; baud is irrelevant (never 134)
$ID*0D                                    # identify   ('I'^'D' = 0x0D)
$ST*07                                    # status
$SP*03                                    # stop all
$SV*05                                    # servo status ('S'^'V' = 0x05)
$DC,1024*2C                               # start depth sampling at OSR 1024
$DV*12                                    # depth sensor state and latest raw sample
$DP*14                                    # depth sensor calibration words
$DC,0*1B                                  # stop depth sampling
```

`DV` shows raw readings; turning them into mbar and deg C takes the calibration words and the formulas in the Bar doc. Do not type frames at the port while a NEPI node of the board is running: its replies are matched to its own requests, and a frame typed alongside can steal one (section 2.1).

Checksums for the other commands are easiest to compute with a one-liner:

```
python3 -c "import functools,sys; b=sys.argv[1]; print('\$%s*%02X' % (b, functools.reduce(lambda a,c: a^ord(c), b, 0)))" "SA,1550,1500,1500,1500,1500,1500,1500,1500"
```

Type `$SA` frames by hand only with the propellers clear or in water, and remember the watchdog: a hand-typed `SA` runs for `watchdog_ms` and then stops.
