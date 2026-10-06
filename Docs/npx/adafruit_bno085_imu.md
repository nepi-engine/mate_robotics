# Adafruit 9-DOF Orientation IMU Fusion Breakout, BNO085 (Product 4754)

## Summary

This Adafruit board carries a CEVA BNO085. The BNO085 is a single package containing an accelerometer, gyroscope, magnetometer and an ARM Cortex-M0+ that runs CEVA's SH-2 sensor-fusion firmware [1][4]. Software talks to it in one of four modes, selected by two pins at reset [3][4]:

- I2C, at address 0x4A or 0x4B
- UART at 3 Mbit/s
- UART-RVC: a fixed 100 Hz, 115200 baud stream of heading and acceleration
- SPI

Except in UART-RVC mode, data arrives as SHTP-framed SH-2 reports that the host must first enable one by one [4].

What a driver author most needs to know:

- **Orientation outputs differ in their use of the magnetometer.** Rotation Vector uses it for absolute heading. Game Rotation Vector does not, and drifts in yaw [4].
- **The I2C interface depends on clock stretching.** Adafruit documents it as troublesome on the Raspberry Pi [2][16][18].
- **Calibration settings are lost at reset.** Calibration data persists only if saved [4].

## At a Glance

| Item | Value | Source |
|---|---|---|
| Sensor | CEVA BNO085: accelerometer (±8 g), gyroscope (±2000 °/s), magnetometer, Cortex-M0+ with SH-2 firmware | [4] |
| Host interfaces | I2C, UART (SHTP), UART-RVC, SPI; chosen by pins P0/P1 at reset | [3][4] |
| I2C address | 0x4A default; 0x4B with DI pulled high | [2][4] |
| I2C speed | 100 or 400 kbit/s (400 kHz max); host must support clock stretching | [4] |
| UART | SHTP 3 Mbit/s 8N1; UART-RVC 115200 8N1 output-only at 100 Hz | [4] |
| SPI | Mode 3, up to 3 MHz | [4] |
| Board supply (VIN) | 3 to 5 V (onboard 3.3 V regulator) | [2] |
| Logic level | SCL/SDA level-shifted for 3 to 5 V; 10 kΩ pull-ups | [2] |
| Connector | 2 × STEMMA QT (JST SH 4-pin), plus 0.1 in header pins | [1][2] |
| Max rates | Rotation Vector and Game Rotation Vector 400 Hz; accelerometer 500 Hz; magnetometer 100 Hz; gyro-integrated rotation vector 1000 Hz | [4] |
| Board size | 25.6 × 22.7 × 4.6 mm; 2.5 g | [1] |

## Physical and Electrical Interface

**Board pins.** Functions are from Adafruit's guide [2][3]. The on-board circuit details are from Adafruit's published board design [15].

| Pin | Function | On-board circuit |
|---|---|---|
| VIN | Power in, 3 to 5 V; "give it the same power as the logic level of your microcontroller" | Regulator input; also feeds both STEMMA QT connectors |
| 3Vo | 3.3 V regulator output, up to 100 mA | AP2112K-3.3 regulator |
| GND | Ground | — |
| SCL | I2C clock. UART mode: sensor RX (connect to host TX). SPI mode: SCK | BSS138 level shifter, 10 kΩ pull-ups |
| SDA | I2C data. UART mode: sensor TX (connect to host RX). SPI mode: MISO | BSS138 level shifter, 10 kΩ pull-ups |
| INT | "Interrupt/Data Ready - Active Low"; required for stable SPI | Wired directly to the chip, not level-shifted |
| BT (BOOT) | Bootloader entry (expert use) | 10 kΩ pull-up, buffered |
| P0 / P1 | Interface mode select; both pulled low by default (I2C) | 10 kΩ pull-downs; solder jumpers to VIN |
| RST | Reset, active low | 10 kΩ pull-up, buffered |
| DI | I2C address select (high = 0x4B); SPI MOSI | 10 kΩ pull-down, buffered |
| CS | SPI chip select | 10 kΩ pull-up, buffered |

**Level shifting and pull-ups.**

- Adafruit states SCL and SDA are "level shifted so you can use 3-5V logic", each with a 10 kΩ pull-up [2].
- The other inputs pass through a 74AHC4050 buffer [15].
- INT has no shifter, so it is a 3.3 V output on any host [15].

**Clock.** A 32.768 kHz crystal provides the reference clock [1][2]. CEVA states the internal clock "must not be used with the UART-SHTP or UART-RVC interfaces" [4].

**Chip electrical** [4]:

- **VDD** 2.4 to 3.6 V and **VDDIO** 1.7 to 3.6 V. On this board both come from the 3.3 V regulator [15].
- **Operating temperature** −40 to 85 °C.
- **Chip current** with 6- or 9-axis fusion at 100 Hz: about 11 mA (3.50 mA VDDIO + 7.50 mA VDD), measured on SPI.

Adafruit publishes no current figure for the whole board.

**Board revision.** Adafruit updated the board's silkscreen on 22 Dec 2023 [1]. The published board design predates that change [15], so whether the circuit also changed is not stated.

## Communication Protocol

### Interface selection

P0 and P1 select the mode. Both pins and the address pin are sampled at reset, so changing them needs a reset [3][4].

| P1 (PS1) | P0 (PS0) | Mode |
|---|---|---|
| Low | Low | I2C (default on this board) |
| Low | High | UART-RVC |
| High | Low | UART (SHTP) |
| High | High | SPI |

### I2C

- **Address.** The chip "answers to a 7-bit address of either 0x4A or 0x4B". The low bit comes from the SA0 pin (DI on this board), sampled at reset [2][4].
- **Speed.** Supports 100 kbit/s and 400 kbit/s, with SCL at 400 kHz maximum [4].
- **Clock stretching is mandatory.** "The master device MUST support clock stretching." [4]
- **No data waiting.** "If the BNO085 is polled and it has no data to send it will stretch the clock until it has data to send." [4]
- **No repeated starts.** Repeated starts are not supported, and each SHTP transfer must end with a STOP [4][6].
- **Pull-ups.** CEVA suggests 2 to 4 kΩ [4]; the board fits 10 kΩ [2].
- **Reading one packet** [4][6]:
  1. Wait for INT to go low.
  2. Read the 4-byte SHTP header to learn the length.
  3. Read again with that full length.

  Adafruit's Arduino library reads this way, by polling rather than waiting on INT [11].

### UART (SHTP)

- 3 Mbit/s, 8 data bits, 1 stop bit, no parity [4].
- The host must leave at least 100 µs between bytes it sends [4][6].
- Messages are framed with flag byte 0x7E, escape byte 0x7D, and a protocol ID byte [6].
- INT goes low just before the first byte of each transmission; the host can use it as a timestamp [4].

### UART-RVC

- **Output only.** The board sends packets on its SDA pin at 115200 baud, 8N1, 100 times per second [2][4].
- **Packet.** A fixed 19-byte packet, with little-endian 16-bit fields [4]:

| Bytes | Field | Units |
|---|---|---|
| 0–1 | Header 0xAA 0xAA | — |
| 2 | Index (counter 0–255) | — |
| 3–4 | Yaw | 0.01°, ±180°; "rotation around the Z-axis since reset" |
| 5–6 | Pitch | 0.01°, ±90° |
| 7–8 | Roll | 0.01°, ±180° |
| 9–14 | X, Y, Z acceleration | mg |
| 15–17 | Reserved | — |
| 18 | Checksum: sum of bytes 2 to 17 | — |

- **Applying the angles.** Rotations are applied in the order yaw, pitch, roll [4].
- **Calibration in this mode.** CEVA says planar-ZGO calibration is enabled [4]. The accelerometer and magnetometer dynamic calibration that is on by default in every other mode is not [4].

### SPI

- **Bus settings.** SPI mode 3 (CPOL = 1, CPHA = 1), clock up to 3 MHz [4].
- **Mode-pin timing.** Both mode pins must be high from before reset until after the first INT assertion [4].
- **WAKE.** CEVA says PS0 doubles as WAKE and "must be connected to a GPIO" [4]. Adafruit's wiring diagram instead ties P0 and P1 to 3 V [2].
- **Required pins.** Adafruit warns that INT and RST "are required for a good SPI connection" [3].

### SHTP framing and channels

Every transfer starts with a 4-byte header [4][6]:

| Byte | Field |
|---|---|
| 0 | Length LSB |
| 1 | Length MSB; bit 15 marks a continuation of a previous transfer |
| 2 | Channel |
| 3 | Sequence number (separate per channel and direction) |

- **Length.** The length includes the 4 header bytes [4][6].
- **Fragmentation.** The BNO08x can send fragmented messages but does not accept them [4].

| Channel | Use | Source |
|---|---|---|
| 0 | SHTP command channel (advertisement, error list) | [4][6] |
| 1 | Executable: write 1 = reset, 2 = on, 3 = sleep; read 1 = reset complete | [4] |
| 2 | Sensor hub control (SH-2 commands and responses) | [4] |
| 3 | Normal input sensor reports | [4] |
| 4 | Wake input sensor reports | [4] |
| 5 | Gyro-integrated rotation vector | [4] |

### SH-2 commands used by a driver

**Set Feature (0xFD).** Enables a report and sets its rate. Report interval 0 turns the sensor off [4][5].

| Byte(s) | Field |
|---|---|
| 0 | 0xFD |
| 1 | Report ID |
| 2 | Feature flags |
| 3–4 | Change sensitivity |
| 5–8 | Report interval in µs (uint32, little-endian) |
| 9–12 | Batch interval (µs) |
| 13–16 | Sensor-specific configuration |

CEVA's worked example enables the accelerometer every 60 ms, with the SHTP header [4]:

```
15 00 02 <seq> FD 01 00 00 00 60 EA 00 00 00 00 00 00 00 00 00 00
```

The Get Feature Response (0xFC) may report a different period than requested [4].

**Other control reports.**

- **Product ID.** Request with `F9 00`; the response is 0xF8 (16 bytes) [4][5].
- **Commands.** Command Request 0xF2; Command Response 0xF1 [5].

**Common report header.** Every input report starts with: report ID, sequence number, status, delay [4][5].

- Status bits 1:0 give accuracy: 0 unreliable, 1 low, 2 medium, 3 high.
- The delay has 100 µs resolution.
- Each batch of reports is preceded by a 0xFB timebase report [4].

## Identification and Detection

- **I2C probe.** An I2C scan should show 0x4A, or 0x4B with DI high. Adafruit's example shows `i2cdetect -y 1` finding 0x4A [16].
- **Bootloader address.** If the BOOT pin is low at reset, the chip comes up in its bootloader at 0x28 or 0x29 instead [4].
- **Product ID.** Send the Product ID Request (`F9 00`) on channel 2. The 0xF8 response carries [4][5]:
  - the reset cause (power-on, internal, watchdog, external, other)
  - the software major and minor version
  - the software part number, build number and patch
- **More than one Product ID response.** CEVA's driver expects several, "Most products supply 4 product ids" [14].
- **What the Adafruit library checks.** Adafruit's CircuitPython library confirms the device by sending 0xF9 and waiting for 0xF8. It retries up to 3 times with resets, then raises "Could not read ID" [10].
- **BNO080 vs BNO085.** Adafruit sells the board as "BNO085 (BNO080)" [1]. CEVA states that code written for the BNO080 works unchanged on the BNO085 [9].
- **Other boards may default to 0x4B.** SparkFun's BNO08x boards and library default to 0x4B, not 0x4A [23]. Code written for one board may need its address changed for the other.

## Startup and Initialization

1. **Reset.** Pull RST low (minimum pulse 10 ns) [4]. Adafruit's libraries hold it high 10 ms, low 10 ms, then high 10 ms [10][11].
2. **Wait for the chip.** It takes about 90 ms of internal initialization plus 4 ms of configuration before INT asserts [4].
3. **Read the startup messages.** In order [4]:
   - the SHTP advertisement packet;
   - a reset-complete message on channel 1;
   - an unsolicited initialization response on channel 2.

   In UART mode the chip sends its advertisement when it is ready to communicate [4].
4. **Enable reports.** "The BNO08X starts up with all sensors disabled". Enable each wanted report with Set Feature [4].
5. **After every reset, start again.** Re-enable reports after any reset, whether commanded, watchdog or brown-out. Adafruit's example checks `wasReset()` and re-enables its reports [3][11].

## Data and Commands

**Reports** (units and Q-points from SH-2 [5]; maximum rates from the datasheet [4])

| Report | ID | Units | Q-point | Max rate |
|---|---|---|---|---|
| Accelerometer (includes gravity) | 0x01 | m/s² | 8 | 500 Hz |
| Gyroscope, calibrated | 0x02 | rad/s | 9 | 400 Hz |
| Magnetic field, calibrated | 0x03 | µT | 4 | 100 Hz |
| Linear acceleration (gravity removed) | 0x04 | m/s² | 8 | 400 Hz |
| Rotation Vector | 0x05 | unit quaternion + heading accuracy estimate (rad) | 14 (accuracy 12) | 400 Hz |
| Gravity | 0x06 | m/s² | 8 | 400 Hz |
| Gyroscope, uncalibrated | 0x07 | rad/s + bias | 9 | Not stated |
| Game Rotation Vector | 0x08 | unit quaternion | 14 | 400 Hz |
| Geomagnetic Rotation Vector | 0x09 | unit quaternion + accuracy (rad) | 14 (accuracy 12) | 90 Hz |
| Magnetic field, uncalibrated | 0x0F | µT + hard-iron bias | 4 | Not stated |
| Stability classifier | 0x13 | state (on table, stationary, stable, motion) | — | Not stated |
| ARVR-stabilized Rotation Vector | 0x28 | quaternion + accuracy | 14 / 12 | Same as RV |
| ARVR-stabilized Game Rotation Vector | 0x29 | quaternion | 14 | Same as GRV |
| Gyro-integrated rotation vector | 0x2A | quaternion + angular velocity (channel 5, no report ID byte) | 14 / 10 | 1000 Hz |

**Reading the fixed-point values.** A value with Q-point n is the raw signed integer divided by 2^n [5].

**Rate limits.**

- The delivered rate can be 0.9 to 2.1 times the requested rate [4].
- "All sensors cannot operate at their maximum rate simultaneously" [4].

**Which orientation output to use**

| Output | Sensors used | Heading | Drift | Source |
|---|---|---|---|---|
| Rotation Vector (0x05) | Accelerometer + gyroscope + magnetometer | Referenced to magnetic north and gravity | "The magnetometer provides correction in yaw to reduce drift" | [4] |
| Game Rotation Vector (0x08) | Accelerometer + gyroscope ("no magnetometer") | No heading reference; roll and pitch referenced to gravity | "will likely drift in yaw"; 0.5°/min heading drift | [4] |
| Geomagnetic Rotation Vector (0x09) | Accelerometer + magnetometer (gyroscope excluded) | Magnetic north and gravity | Less responsive; "More errors in the presence of varying magnetic fields" | [4] |

CEVA recommends the Rotation Vector "in a relatively stable magnetic field", and the Game Rotation Vector "in an unstable magnetic field" [4].

**Accuracy (CEVA simulation, external crystal)** [4]:

| Output | Accuracy |
|---|---|
| Rotation Vector | Dynamic error 3.5°, static error 2.0° |
| Game Rotation Vector | Non-heading error 2.5° dynamic, 1.5° static; heading drift 0.5°/min |
| Geomagnetic Rotation Vector | Dynamic error 4.5°, static error 3.0° |

CEVA adds: "In practice the rotation vector is typically accurate to 5˚ and the geomagnetic rotation vector to 10˚." [4]

## Configuration and Calibration

**Dynamic calibration.** Each sensor's calibration is enabled with the ME Calibration command (0x07 via Command Request 0xF2): accelerometer, gyroscope and magnetometer flags [5].

- **Defaults.** "by default the accelerometer and magnetometer calibration are enabled for all interface modes except UART-RVC". Gyroscope calibration is not on by default [4].
- **Not persistent.** "the calibration settings do not persist across resets of the BNO08X." A driver must re-send them after every reset [4].
- **Gyroscope calibration can be fooled.** "it is possible to fool the calibration algorithms through slow horizontal rotations". CEVA advises disabling gyroscope calibration if there is not enough tremor [4].

**Calibration procedure** (CEVA application note) [7]:

1. **Accelerometer.** Hold the device in 4 to 6 orientations for about 1 s each.
2. **Gyroscope.** Leave it still for about 2 to 3 s.
3. **Magnetometer.** Rotate about 180° and back in roll, pitch and yaw, about 2 s per axis.
   - The Magnetic Field report must be enabled at 50 Hz during this step.
   - Continue until the magnetometer accuracy status is 2 or 3.
4. **Save.** Send Save DCD.
5. **Where.** "calibration within the final assembled device is essential" [7].

**Saving calibration on the chip** [4][5]:

- **Save DCD.** Command 0x06 saves the dynamic calibration data (DCD) to flash.
- **Periodic saving.** The chip also stores DCD to RAM every 5 seconds, and at a non-power-up reset it persists the last RAM copy to flash.
- **Stopping periodic saves.** Command 0x09 disables them. It does not block Save DCD.
- **What survives a power cycle.** Only DCD already written to flash.
- **Clearing.** Clear DCD and Reset (0x0B) clears it.

**Tare and mounting orientation** [4][5][8]:

- **Tare.** Command 0x03, Tare Now, zeroes chosen axes against a chosen rotation vector.
- **Persist Tare.** Writes the result to flash.
- **Tare the Rotation Vector only after north is resolved.** For the Rotation Vector, magnetic north must be resolved first, "Otherwise when the magnetometer calibrates the heading will change." [4]
- **Game Rotation Vector tares are not kept.** "Persist tare does not apply to the Game Rotation Vector." [4]
- **Mounting orientation.** It can be stored in FRS record 0x2D3E as a unit quaternion with Q30 coordinates [4].

## Failure Modes and Safety

- **Raspberry Pi I2C.**
  - Adafruit's guide carries a warning: "The BNO085 seems to work best on the Raspberry Pi with an I2C clock frequency of 400kHz", set with `dtparam=i2c_arm_baudrate=400000` [2].
  - Adafruit's clock-stretching guide says the issue "is simply how Pi's handle (or don't) I2C clock stretching" and uses the BNO085 as its example [16].
  - Adafruit's CircuitPython-on-Raspberry-Pi page says the opposite: slow the clock to `dtparam=i2c_arm_baudrate=10000` for the BNO085 [17].
  - Adafruit's own pages therefore disagree. The device-specific guide [2] and a third-party report that 10 kHz failed while 400 kHz worked [20] favor 400 kHz. Test both on the vehicle's Pi (Open Question 2).
  - Other workarounds Adafruit documents:
    - a software (bit-banged) I2C bus [16];
    - UART or UART-RVC instead of I2C [19].
- **Adafruit's list of troublesome chips.** Adafruit lists the BNO085 as using clock stretching, violating "I2C protocol timing in some cases", and that it "sometimes needs to be reset" [18]. Its guide adds that it "does not work well with I2C multiplexers" [3]. This matters if a multiplexer is used to separate the kit's two 0x76 sensors: keep the BNO085 off the multiplexer.
- **Service INT promptly.** If the host does not respond to INT within about 10 ms, the BNO085 times out and retries. "Frequent delays will cause process starvation", and some outputs will have errors. CEVA advises handling INT within one-tenth of the fastest sensor period [4].
- **Magnetic disturbance.**
  - The magnetometer field "is distorted by the proximity of ferrous or magnetic material". CEVA names "speakers, magnets etc." as hard-iron sources [4].
  - Without calibration, "the heading reported by BNO08X will be highly suspect" [4].
  - CEVA's calibration note says calibration done near interfering objects bakes them into the saved data [7].
  - Neither CEVA nor Adafruit addresses motors or thrusters specifically. The T200 thrusters contain coated permanent magnets and draw up to 32 A each at 20 V [24], so magnetic heading near them must be tested (Open Question 3).
- **Library quirks found in Adafruit's code** (verified there, but not tested on hardware):
  - The CircuitPython library scales the Geomagnetic Rotation Vector quaternion with Q12 [10], while SH-2 defines it as Q14 [5].
  - Adafruit's guide shows `serial.Serial("/dev/serial0", 115200)` for UART-SHTP mode [2]. That rate is wrong for that mode: the datasheet gives 3 Mbit/s [4], and the library examples were corrected to 3,000,000 [21].
  - Open reports describe "Unprocessable Batch bytes" errors from the CircuitPython library on Raspberry Pi boards [22].

## Software and Libraries

| Name | Use | Language | License | Link |
|---|---|---|---|---|
| Adafruit CircuitPython BNO08x | I2C, UART and SPI driver; runs on Linux through Blinka (`pip3 install adafruit-circuitpython-bno08x`) | Python | MIT | [2][10] |
| Adafruit CircuitPython BNO08x RVC | UART-RVC packet parser | Python | MIT | [13] |
| Adafruit BNO08x (Arduino) | Driver built on CEVA's sh2 code; one sensor per program | C/C++ | BSD 3-clause; bundled sh2 files Apache-2.0 | [11] |
| Adafruit BNO08x RVC (Arduino) | UART-RVC parser | C++ | BSD | [12] |
| CEVA sh2 | CEVA's reference SH-2/SHTP host driver; host supplies a hardware layer | C | Apache-2.0 (file headers) | [14] |
| SparkFun BNO08x Arduino Library | THIRD-PARTY driver on CEVA sh2 | C/C++ | MIT | [23] |

## Related Parts

- [Raspberry Pi 5](../tbd/raspberry_pi_5.md): a possible I2C or UART host; its RP1 I2C controller's clock-stretching behavior is unverified.
- [PJRC Teensy 4.1](../tbd/pjrc_teensy_4_1.md): a possible host on one of its I2C ports or a hardware UART, which would avoid the Pi I2C issue.
- [Blue Robotics T200 thruster](../tbd/bluerobotics_t200_thruster.md): its magnets and motor currents may disturb magnetic heading [4][24].
- [Blue Robotics Bar depth sensor](bluerobotics_bar_depth_sensor.md): may share an I2C bus; if a multiplexer is used (see that document), keep the BNO085 off it [3].
- [Adafruit MS8607 PHT sensor](../tbd/adafruit_ms8607_pht_sensor.md): same multiplexer note as the Bar sensor.

## Open Questions

1. **Which interface is wired: I2C, UART, UART-RVC or SPI, and to which host?**
   - Why it matters: the driver, the framing and the reliability problems all depend on it [3][4].
   - How to get it: a wiring decision, recorded with the P0/P1 jumper state and the host port.
2. **Is I2C reliable on the vehicle's host at the chosen clock?**
   - Why it matters: Adafruit gives conflicting advice (400 kHz vs 10 kHz) [2][17], and the Pi 5's I2C controller is not covered by those guides.
   - How to get it: on the host, run `i2cdetect -y <bus>`. Then stream Rotation Vector at the planned rate for 30 minutes at each candidate `i2c_arm_baudrate` (or the equivalent bus overlay baudrate). Count read errors and unexpected resets.
3. **Is magnetic heading usable near the thrusters?**
   - Why it matters: this decides between the Rotation Vector, which uses the magnetometer, and the Game Rotation Vector, which drifts in yaw [4].
   - How to get it: with the IMU mounted in place, log calibrated magnetic field and Rotation Vector heading while stepping each thruster from stop to full in water. Watch the reported accuracy status and accuracy estimate.
4. **Mounting orientation.**
   - Why it matters: the outputs are in the chip's frame until reoriented [4].
   - How to get it: record the board orientation relative to the vehicle axes. Then either rotate in software or write the FRS 0x2D3E orientation record [4].
5. **Calibration plan.**
   - Why it matters: calibration enable settings are lost at reset, while saved calibration data persists [4]. The team must decide when to calibrate, whether to Save DCD, and whether to disable periodic saves.
   - How to get it: a team procedure, using CEVA's calibration motions inside the assembled vehicle [7].
6. **Report set and rates.**
   - Why it matters: rates are quantized and not all sensors can run at maximum together [4].
   - How to get it: decide which reports and rates are needed, then read back the Get Feature Response to confirm the actual period.
7. **Which board revision?**
   - Why it matters: the published board design predates the Dec 2023 revision [1][15].
   - How to get it: check the silkscreen on the received board.

## Sources

1. "Adafruit 9-DOF Orientation IMU Fusion Breakout - BNO085 (BNO080)" (product 4754), Adafruit. https://www.adafruit.com/product/4754. Used for: board features, crystal, connectors, size and weight, Dec 2023 revision.
2. "Adafruit 9-DOF Orientation IMU Fusion Breakout - BNO085" (Learn guide), Adafruit. https://learn.adafruit.com/adafruit-9-dof-orientation-imu-fusion-breakout-bno085. Used for: pin functions, regulator and level shifting, addresses, Raspberry Pi 400 kHz note, SPI wiring, UART snippets, install command.
3. "Adafruit 9-DOF Orientation IMU Fusion Breakout - BNO085" (Learn guide PDF), Adafruit. https://cdn-learn.adafruit.com/downloads/pdf/adafruit-9-dof-orientation-imu-fusion-breakout-bno085.pdf. Used for: mode table, multiplexer warning, INT/RST required for SPI, single-sensor Arduino limit.
4. "BNO08X Data Sheet" 1000-3927 v1.17, CEVA. https://www.ceva-ip.com/wp-content/uploads/BNO080_085-Datasheet.pdf. Used for: interfaces, I2C/UART/RVC/SPI details, SHTP channels, Set Feature, startup, rates, accuracy, rotation vector behavior, calibration and DCD, tare, magnetic disturbance, INT timing.
5. "SH-2 Reference Manual" 1000-3625 v1.9, CEVA. https://www.ceva-ip.com/wp-content/uploads/SH-2-Reference-Manual.pdf. Used for: report IDs, units and Q-points, command IDs, Save DCD, periodic DCD, tare commands.
6. "Sensor Hub Transport Protocol" 1000-3535 v1.10, CEVA (hosted by Core Electronics). https://core-electronics.com.au/attachments/localcontent/Sensor-Hub-Transport-Protocol_28557225508.pdf. Used for: SHTP header, continuation, I2C STOP rule, UART framing and byte spacing.
7. "BNO080/BNO085 Sensor Calibration Procedure" 1000-4044 v1.3, CEVA (hosted by xdevs). https://xdevs.com/doc/CEVA/BNO080-BNO085-Sesnor-Calibration-Procedure.pdf. Used for: calibration motions, 50 Hz magnetometer, Save DCD, calibrate in final assembly, environment warning.
8. "BNO080/BNO085 Tare Function Usage Guide" 1000-4045 v1.2, CEVA (hosted by xdevs). https://xdevs.com/doc/CEVA/BNO080-BNO085-Tare-Function-Usage-Guide.pdf. Used for: tare command bytes.
9. "BNO080/BNO085 Migration Guide" v1.6, CEVA (hosted by xdevs). https://xdevs.com/doc/CEVA/BNO080-BNO085-Migration-Guide.pdf. Used for: BNO080 code runs unchanged on BNO085.
10. "Adafruit_CircuitPython_BNO08x", Adafruit (GitHub). https://github.com/adafruit/Adafruit_CircuitPython_BNO08x. Used for: reset timing, product-ID check, quaternion scaling quirk, MIT license.
11. "Adafruit_BNO08x" (Arduino), Adafruit (GitHub). https://github.com/adafruit/Adafruit_BNO08x. Used for: header-then-body I2C reads, reset handling, licenses.
12. "Adafruit_BNO08x_RVC" (Arduino), Adafruit (GitHub). https://github.com/adafruit/Adafruit_BNO08x_RVC. Used for: RVC parser, license.
13. "Adafruit_CircuitPython_BNO08x_RVC", Adafruit (GitHub). https://github.com/adafruit/Adafruit_CircuitPython_BNO08x_RVC. Used for: Python RVC parser, license.
14. "sh2", CEVA (GitHub). https://github.com/ceva-dsp/sh2. Used for: reference host driver, multiple product IDs, Apache-2.0 headers.
15. "Adafruit-BNO08x-PCB", Adafruit (GitHub). https://github.com/adafruit/Adafruit-BNO08x-PCB. Used for: regulator, level shifter, buffer, pull-ups and pull-downs, INT wiring, crystal circuit.
16. "Raspberry Pi I2C Clock Stretching Fixes", Adafruit. https://learn.adafruit.com/raspberry-pi-i2c-clock-stretching-fixes. Used for: Pi clock-stretching cause, BNO085 400 kHz fix, software I2C, `i2cdetect` example.
17. "CircuitPython Libraries on Linux and Raspberry Pi: I2C Clock Stretching", Adafruit. https://learn.adafruit.com/circuitpython-on-raspberrypi-linux/i2c-clock-stretching. Used for: the conflicting 10 kHz recommendation.
18. "Troublesome chips" (I2C Addresses list), Adafruit (GitHub). https://github.com/adafruit/I2C_Addresses/blob/main/troublesome_chips.md. Used for: BNO085 clock stretching, protocol timing, "sometimes needs to be reset".
19. "I2C Unknwon Report Type" (issue #9), Adafruit (GitHub). https://github.com/adafruit/Adafruit_CircuitPython_BNO08x/issues/9. Used for: Adafruit advice to slow I2C or use UART instead on a Raspberry Pi.
20. "Library throws key error exception with Learning Guide example code" (issue #16), Adafruit (GitHub; THIRD-PARTY user report). https://github.com/adafruit/Adafruit_CircuitPython_BNO08x/issues/16. Used for: 10 kHz failed and 400 kHz worked.
21. "UART bardrate wrong in examples" (issue #46), Adafruit (GitHub). https://github.com/adafruit/Adafruit_CircuitPython_BNO08x/issues/46. Used for: UART-SHTP examples corrected to 3,000,000 baud.
22. "Unprocessable Batch bytes" (issue #49), Adafruit (GitHub; THIRD-PARTY reports). https://github.com/adafruit/Adafruit_CircuitPython_BNO08x/issues/49. Used for: open library error on Raspberry Pi boards.
23. "SparkFun_BNO08x_Arduino_Library", SparkFun (THIRD-PARTY, GitHub). https://github.com/sparkfun/SparkFun_BNO08x_Arduino_Library. Used for: SparkFun default address 0x4B, MIT license.
24. "T200 Thruster" (R2), Blue Robotics. https://bluerobotics.com/store/thrusters/t100-t200-thrusters/t200-thruster-r2-rp/. Used for: coated permanent magnets in the rotor, up to 32 A at 20 V.
