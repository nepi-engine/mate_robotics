# Blue Robotics Basic ESC (BESC30-R3)

## Summary

The Basic ESC R3 is a 30 A, 7 to 26 V, bidirectional brushless motor speed controller sized for the T200 thruster. It runs the open-source BLHeli_S firmware [1][6]. Software commands it with standard RC servo pulses on one signal wire:

- 1100 µs is full reverse.
- 1500 µs is stop.
- 1900 µs is full forward.

There is a ±25 µs deadband around stop, and updates are accepted at up to 400 Hz [1][2].

The most important facts for a driver author:

- The ESC is one-way. It reports nothing back.
- It will not drive until it has seen a stop pulse after power-up.
- Its source code shows that after signal loss it stops the motor and must be re-armed with a stop pulse [1][2][15].

## At a Glance

| Item | Value | Source |
|---|---|---|
| Interface | One-way RC/servo PWM pulse on one signal wire | [1][2] |
| Pulse mapping | 1100 µs full reverse, 1500 µs stop/initialize, 1900 µs full forward | [1] |
| Deadband | 1500 µs ± 25 µs | [1] |
| Max update rate | 400 Hz | [1] |
| Signal voltage | 3.3 to 5 V | [1] |
| Signal connector | 3-position 0.1 in (2.54 mm) servo connector: ground, empty, signal | [1] |
| Supply | 7 to 26 V (2S to 6S) | [1] |
| Current | 30 A continuous (depends on cooling) | [1] |
| Feedback / telemetry | None; status only as beeps through the motor | [1][19] |
| BEC (5 V output) | None | [12] |
| Firmware | BLHeli_S 16.6, open source (GPL v3) | [6][13][14] |
| Weight | 16.3 g | [1] |

## Physical and Electrical Interface

| Lead | Wires | Termination | Source |
|---|---|---|---|
| Power | Red positive (7 to 26 V), black ground; 14 AWG | #6 spade terminals | [1][3] |
| Motor | Blue phase A, green phase B, white phase C; 16 AWG | Tinned ends | [1][3] |
| Signal | Black ground, white signal (PWM) | 3-position 0.1 in servo connector; the middle position is empty, so the connector carries no power | [1][3] |

**Signal level and ground.** The signal input accepts 3.3 to 5 V logic [1]. The signal ground (black) must connect to a ground pin on the device that generates the pulses [2].

**No 5 V output.** The Basic ESC has no BEC, so it cannot power a receiver or servo [12].

**Heat.**

- Most heat comes from the MOSFETs under the blue aluminum heat spreader. That spreader must be exposed to air or attached to a larger heat sink [1].
- Do not cover it with insulating adhesives such as silicone sealant [1].
- Size from the drawing: about 32.0 by 17.1 mm [4].

**Not waterproof.** The R3 product page gives no water or depth rating [1]. Blue Robotics staff wrote in 2017 that the Basic ESC "must be inside a watertight enclosure" [22]. Treat it as a dry-enclosure part.

**Input capacitance.** Blue Robotics staff stated the R3 has "about 110µF of capacitance in the form of surface mount capacitors" on its input [18].

## Communication Protocol

**Signal type.** The ESC uses "the same signals commonly used by RC receivers or to control servos" [2]: a pulse whose width sets the command, repeated at the frame rate.

**Pulse mapping (bidirectional mode, factory setting)**

| Pulse width | Meaning | Source |
|---|---|---|
| 1100 µs | Full throttle reverse | [1][5] |
| 1100 to below 1475 µs | Reverse, increasing toward 1100 | [2] |
| 1475 to 1525 µs | Deadband (stopped) | [1][2][11] |
| 1500 µs | Initialize / stop | [1][5] |
| above 1525 to 1900 µs | Forward, increasing toward 1900 | [2] |
| 1900 µs | Full throttle forward | [1][5] |

**Factory endpoints are offset by 12 µs.**

- The factory BLHeliSuite settings are PPM minimum 1112 µs, center 1512 µs and maximum 1912 µs [7][8].
- Blue Robotics staff explained that these offsets make the ESC "more correctly register physical inputs at the desired pulse-durations (1100/1500/1900)". They suggested handling any remaining constant offset with output trims in the control software [25].
- The driver should command the documented 1100/1500/1900 µs values [1], not the internal ones [25].

**Thrust is not linear in pulse width.** In Blue Robotics' T200 test data, measured with this ESC, zero thrust spans about 1460 to 1540 µs at 10 V and 1476 to 1528 µs at 20 V. That band combines the deadband with the test's 300 RPM cut-off [27]. See [the T200 document](bluerobotics_t200_thruster.md) for the tables.

**Update rate.**

- The maximum accepted update rate is 400 Hz [1].
- Blue Robotics staff wrote that they "generally use a 200Hz update rate, and don't recommend below 50Hz" [23].
- Blue Robotics' Arduino example uses the Arduino Servo library at 50 Hz [10].

**Valid pulse window (derived from the firmware source).** The BLHeli_S source counts pulses outside "900-2235us" as out of range. After enough of them it treats the input as invalid [15]. Pulses must stay inside 1100 to 1900 µs anyway.

**Other input protocols.**

- Blue Robotics states the Basic ESC is "Oneshot 125 capable" [6].
- The BLHeli_S manual says the firmware also auto-detects Oneshot42, Multishot and DShot [9]. Blue Robotics does not document those for this product.
- All of these are one-way. Blue Robotics staff note these protocols "don't support sending back information about the motor or ESC state" [19].

**Bidirectional operation.**

- In bidirectional mode, center throttle is zero; above center is forward and below is reverse [9].
- When the command crosses stop, the firmware brakes before it reverses. "Startup power … is used to limit the power applied during direction reversal." [9]

## Identification and Detection

**The ESC cannot be detected from software.** The signal wire is input-only, and the ESC sends nothing back [1][19]. A driver cannot tell whether an ESC is connected, powered, armed, or which revision it is. It must take every ESC channel from configuration.

**Audible identification.**

- At power-up with a motor connected, an R3 plays three rising tones [1].
- R1 and R2 ESCs used SimonK firmware [21]. Mixed sets of R1/R2/R3 ESCs give different startup beeps [18].

**Firmware identity.** The firmware and its settings can be read only with the BLHeliSuite configuration tool, over the signal wire. The stock R3 identifies as layout "R_H_15" running BLHeli_S 16.6, on an EFM8BB21 microcontroller [6][7][8].

## Startup and Initialization

**Arming sequence** (R3) [1][2]:

1. Power the ESC with the motor connected. It plays three rising tones.
2. Send a stop signal (1500 µs) "for a few seconds". The ESC plays one tone when it detects any throttle signal, and a second tone when it detects the stop signal.
3. Only after that will it accept commands from 1100 to 1900 µs.

Blue Robotics: "The ESC must be initialized before it can accept any other throttle signals" [2]. If the two tones do not play, the stop signal was not sent properly or not for long enough [28].

**How long to hold stop.** Blue Robotics says "a few seconds" [1]. Its Arduino example holds 1500 µs for 7 s ("delay(7000); // delay to allow the ESC to recognize the stopped signal") [10]. No firmware minimum is published.

**Power-up order.** In Blue Robotics' Arduino example, if the controller starts before the ESC is powered, "the ESC will miss the initialization step and won't start" [10]. The firmware protocol must therefore keep sending stop until the ESC has armed. A one-off burst at controller boot is not enough.

**No accidental throttle calibration.** With bidirectional operation selected, "programming by TX is disabled" [9]. The source jumps past the throttle-programming path in bidirectional mode [15]. A non-stop pulse at power-up therefore does not start a calibration; the ESC waits for stop.

**Signal held high at power-up (derived from the firmware source).**

- At startup the firmware checks whether the input "is high for more than 15ms". If so, it jumps to its bootloader [15].
- A signal line held high, for example by a microcontroller pin driven high or pulled up before its firmware runs, can therefore leave the ESC in bootloader mode instead of running.
- This is a reading of the source, not a Blue Robotics statement (Open Question 4).

## Data and Commands

| Command or output | Units and range | Resolution and rate | Source |
|---|---|---|---|
| Throttle command (pulse width) | 1100 to 1900 µs, 1500 µs stop | Up to 400 Hz; 200 Hz typical in Blue Robotics use | [1][23] |
| Telemetry | None | — | [1][19] |
| Status | Beeps through the motor | Not machine-readable | [1] |

**Behavior notes.**

- **Open loop.** There is no speed or current regulation; Blue Robotics calls the Basic ESC "an open loop control system" [27].
- **Smoothing.** The response is smoothed by a low-pass filter set through the "Startup Power" setting, which Blue Robotics sets to 50% by default. Staff warn that speeding it up "will significantly increase the stress on the thrusters" [25].
- **Measured lag (third-party).** One user measured about 110 ms from a 1500 to 1700 µs step to first motor phase activity from standstill [24].
- **Motor drive frequency.** The ESC's own motor PWM frequency is fixed at 24 kHz [9].

## Configuration and Calibration

**Factory settings.** Read with BLHeliSuite; Blue Robotics publishes the settings file [7][8].

| Setting | Factory value |
|---|---|
| Motor direction | Bidirectional Rev. |
| PPM minimum / center / maximum | 1112 / 1512 / 1912 µs |
| Startup power | 0.50 |
| Motor timing | Medium |
| Temperature protection | 140 °C |
| Brake on stop | Off |
| Beacon delay | Infinite |

**Persistence.** These settings are stored in the ESC's firmware, so they survive power cycles. They change only when someone reconfigures or reflashes the ESC with BLHeliSuite [6][9].

**Reconfiguring and reflashing.** Done "through the PWM signal wire using a programming tool like the Turnigy USB Linker, the AfroESC Programmer, or an Arduino", with BLHeliSuite [6]. In bidirectional mode the BLHeli_S manual requires the minimum and maximum throttle to be at least 70 µs apart [9].

**Reversing motor direction.** Use one method per ESC:

| Method | How | Source |
|---|---|---|
| Swap wires | Swap any two of the three motor wires | [1] |
| Firmware setting | Change "Motor Direction" between bidirectional fwd and bidirectional rev in BLHeliSuite | [8][9] |
| Software | Mirror the command about 1500 µs in the controller | [29] |

The software method keeps the hardware identical across ESCs, but the reversal must then be recorded in the driver's configuration.

**Calibration.** There is no throttle calibration to perform in bidirectional mode [9]. The 12 µs factory offset described under Communication Protocol is the only known offset [25].

## Failure Modes and Safety

**Signal loss.**

- Blue Robotics does not publish the R3's signal-loss timeout. A staff member wrote "I would expect that it's on the order of tens of milliseconds" but was not sure [23].
- The BLHeli_S source (Rev 16.6 era) shows this logic [15][16]:
  - Each valid pulse reloads a timeout counter to 10.
  - A timer interrupt, commented "Happens every 32ms", decrements it.
  - When it reaches zero while running, the firmware turns motor power off and goes back to waiting for a signal.
- From the source, that is a timeout of roughly 10 × 32 ms ≈ 0.3 s. This is derived from the code, not a manufacturer value.
- After a timeout the ESC must detect the signal and be re-armed with stop before it drives again [15].
- A host-side watchdog must therefore stop commanding thrust well before 0.3 s if it wants a controlled stop. After a dropout it must send stop to re-arm, not resume mid-throttle (Open Question 1).

**Power loss or brown-out.** On reset the firmware replays its power-on beeps and the full detect-and-arm sequence [15]. After any power interruption the ESC needs stop again before it will drive. This is from the source, not a Blue Robotics statement.

**Over-temperature.** Power is derated from the MCU temperature [1][9]:

| MCU temperature | Power limit |
|---|---|
| ≥ 140 °C | 75% |
| ≥ 145 °C | 50% |
| ≥ 150 °C | 25% |
| ≥ 155 °C | 0% |

**Stall.** If the motor tries to start but fails for a few seconds, the ESC stops trying until throttle returns to zero [9].

**Current margin.**

- The ESC is rated 30 A continuous, depending on cooling [1].
- A T200 at full throttle draws 31 to 32 A at 20 V [27]. Blue Robotics staff describe about 30 A at 20 V as "near the limit of the Basic ESCs capabilities" [26].
- Running at 20 V full throttle leaves no margin.

**Regenerative braking voltage spikes.**

- The R3's "Damped Light" regenerative braking is always on, with no way to turn it off [20].
- On direction changes and decelerations it pushes energy back into the supply. That can produce voltage spikes that trip a bench power supply's over-current protection [20].
- Blue Robotics staff say the R1 and R2 did not do this [20].
- The BLHeli_S manual confirms "All codes use damped light mode." [9]

**Reverse-polarity and over-voltage protection.** None is stated for the R3 [1]. Treat the 26 V maximum as absolute.

## Software and Libraries

| Name | Use | Language | License | Link |
|---|---|---|---|---|
| BLHeli_S (Rev 16.6 shipped) | ESC firmware, SiLabs EFM8 | 8051 assembly | GPL v3 | [13][14][15] |
| Blue Robotics BLHeliSuite settings file | Factory configuration for this ESC | INI | Not stated | [7] |
| BLHeli_S manual (SiLabs Rev16.x) | Firmware behavior, protocols, protections | PDF | — | [9] |
| BLHeliSuite | Read and write settings; flash firmware | Windows application | Not checked | [6][13] |
| bluerobotics/br-esc-examples | Arduino sketches driving the ESC with the Servo library (its README's wire colours are for the pre-R3 ESC) | C++ (Arduino) | MIT | [17] |

**Revision history.** The R3 was released on 9 Nov 2017 [18]. The R1 and R2 used SimonK firmware [21]. The R2 had I2C support in its custom firmware, but the R3's microcontroller does not support I2C [19]. Mixing R3 with older R1/R2 ESCs on one vehicle is fine [18].

## Related Parts

- [Blue Robotics T200 thruster](bluerobotics_t200_thruster.md): the motor this ESC drives; has the published PWM, thrust and current tables measured with this ESC.
- [PJRC Teensy 4.1](pjrc_teensy_4_1.md): the likely pulse source; its firmware must implement arming, the update rate and a stop-on-timeout watchdog.
- [Raspberry Pi 5](raspberry_pi_5.md): explains why the pulses likely come from the Teensy rather than the Pi.

## Open Questions

1. **Actual signal-loss behavior on these units.**
   - Why it matters: the driver's watchdog must be designed around the real timeout and re-arm behavior. Only a source-derived ~0.3 s figure exists [15][23].
   - How to get it: with a thruster in water at a low command such as 1600 µs, cut the pulse train in firmware. Time the stop with a scope on one motor phase, or by audio. Then resume commands without a stop pulse and record whether it drives.
2. **Update rate to use.**
   - Why it matters: it must be between 50 and 400 Hz [1][23], and it sets the timer configuration on whatever generates the pulses.
   - How to get it: a team decision. Blue Robotics' typical 200 Hz is a reasonable default [23].
3. **Are all units at factory settings?**
   - Why it matters: a changed Motor Direction or endpoint would silently flip or scale a thruster [8][9].
   - How to get it: read each ESC with BLHeliSuite through a USB linker, and compare against the published settings file [7].
4. **State of the signal line at ESC power-up.**
   - Why it matters: a line held high for more than 15 ms sends the ESC to its bootloader (from the source [15]). The pulse source's pin state before its firmware starts is therefore important.
   - How to get it: check the pin's power-up state (see the Startup section of [the Teensy document](pjrc_teensy_4_1.md)). Decide whether to add a pull-down on each signal line, and confirm by powering ESC and controller in every order.
5. **Number of ESCs and their location.**
   - Why it matters: one ESC per thruster, and they need a dry enclosure and cooling for their heat spreaders [1][22].
   - How to get it: the vehicle design.
6. **Supply voltage, fusing and peak current.**
   - Why it matters: 30 A continuous per ESC, and the T200 reaches about 32 A at 20 V [1][27].
   - How to get it: the power-system design; record the voltage and per-ESC fuse rating.
7. **Arming procedure ownership.**
   - Why it matters: someone must send stop for "a few seconds" after every ESC power-up and after every signal loss [1][15].
   - How to get it: decide whether the microcontroller firmware does this on its own or the host commands it, and write it into the protocol specification.

## Sources

1. "Basic ESC" (BESC30-R3), Blue Robotics. https://bluerobotics.com/store/thrusters/speed-controllers/besc30-r3/. Used for: technical details (voltage, current, signal voltage, pulse mapping, deadband, 400 Hz), wires and connectors, quick start, beeps, thermal derating, heat spreader, direction swap.
2. "Thruster User Guide", Blue Robotics. https://bluerobotics.com/learn/thruster-usage-guide/. Used for: RC-style PWM, signal ground, pulse ranges, initialization requirement.
3. "Basic ESC R3 wiring diagram" (PNG), Blue Robotics. https://bluerobotics.com/wp-content/uploads/2018/10/BESC30-R3-diagram.png. Used for: wire colours and functions.
4. "Basic ESC R3 drawing" (PNG), Blue Robotics. https://bluerobotics.com/wp-content/uploads/2018/10/BESC-R3-Drawing.png. Used for: dimensions.
5. "Thruster usage guide PWM signal" (PNG), Blue Robotics. https://bluerobotics.com/wp-content/uploads/2022/04/thruster-usage-guide-PWM-signal.png. Used for: pulse mapping graphic.
6. "Basic ESC R3 Firmware Files and Customization", Blue Robotics. https://bluerobotics.com/learn/basic-esc-r3-firmware-files-and-customization/. Used for: BLHeli_S 16.6, open-source firmware, flashing tools, SiLabs MCU, Oneshot125.
7. "BLHeli_BRBasicESC-R_H_15-Rev.16.6-Multi_170921.ini", Blue Robotics. https://cad.bluerobotics.com/BLHeli_BRBasicESC-R_H_15-Rev.16.6-Multi_170921.ini. Used for: factory settings, MCU and layout.
8. "Basic ESC default configuration" (PNG), Blue Robotics. https://bluerobotics.com/wp-content/uploads/2025/02/BasicESC_default_config.png. Used for: factory settings as displayed in BLHeliSuite.
9. "BLHeli_S manual SiLabs Rev16.x", Steffen Skaug (hosted by Blue Robotics). https://bluerobotics.com/wp-content/uploads/2018/10/BLHeli_S-manual-SiLabs-Rev16.x.pdf. Used for: bidirectional behavior, protocols, TX programming disabled, protections, damped light, motor PWM frequency, throttle-range rule.
10. "Basic ESC R3 Example Code for Arduino", Blue Robotics. https://bluerobotics.com/learn/basicesc-r3-example-code-for-arduino/. Used for: 50 Hz Servo library, 7 s stop hold, power-up order.
11. "Control the Basic ESC with the Arduino Serial Monitor", Blue Robotics. https://bluerobotics.com/learn/controlling-basic-esc-with-the-arduino-serial-monitor/. Used for: ±25 µs deadband statement.
12. "Control the Basic ESC with an RC Transmitter", Blue Robotics. https://bluerobotics.com/learn/control-the-basic-esc-with-an-rc-transmitter/. Used for: no BEC.
13. "BLHeli" repository, Steffen Skaug (bitdump, GitHub). https://github.com/bitdump/BLHeli. Used for: firmware source home, BLHeliSuite reference, license.
14. "BLHeli COPYING", bitdump (GitHub). https://raw.githubusercontent.com/bitdump/BLHeli/master/COPYING. Used for: GPL v3 license text.
15. "BLHeli_S.asm" at Rev 16.6-era commit 3156526, bitdump (GitHub). https://raw.githubusercontent.com/bitdump/BLHeli/3156526f317800f186ec5a24799ae2663f493cec/BLHeli_S%20SiLabs/BLHeli_S.asm. Used for: timeout counter and 32 ms interrupt, valid pulse window, 15 ms high-line bootloader entry, re-arm after signal loss or reset, TX programming skipped in bidirectional mode.
16. "BLHeli_S.asm" master (Rev 16.7), bitdump (GitHub). https://raw.githubusercontent.com/bitdump/BLHeli/master/BLHeli_S%20SiLabs/BLHeli_S.asm. Used for: cross-check that the timeout logic is unchanged.
17. "br-esc-examples", Blue Robotics (GitHub). https://github.com/bluerobotics/br-esc-examples. Used for: example code, MIT license.
18. "New Product! Basic ESC R3", Blue Robotics Forum (Blue Robotics staff). https://discuss.bluerobotics.com/t/new-product-basic-esc-r3/1709. Used for: R3 release date, mixing revisions, input capacitance.
19. "Basic ESC R3 with i2c", Blue Robotics Forum (Blue Robotics staff). https://discuss.bluerobotics.com/t/basic-esc-r3-with-i2c/8775. Used for: R2 vs R3 firmware and MCU, no I2C, one-way protocols.
20. "New Basic ESC r3 on power supply (no batteries)", Blue Robotics Forum (Blue Robotics staff). https://discuss.bluerobotics.com/t/new-basic-esc-r3-on-power-supply-no-batteries/2066. Used for: always-on regenerative braking and voltage spikes.
21. "Using Another ESC", Blue Robotics Forum (Blue Robotics staff). https://discuss.bluerobotics.com/t/using-another-esc/7343. Used for: SimonK firmware on R1/R2.
22. "When will BlueESC be available?", Blue Robotics Forum (Blue Robotics staff). https://discuss.bluerobotics.com/t/when-will-blueesc-be-available/839. Used for: Basic ESC must be in a watertight enclosure.
23. "Basic ESC safety time to stop motors", Blue Robotics Forum (Blue Robotics staff). https://discuss.bluerobotics.com/t/basic-esc-safety-time-to-stop-motors/12470. Used for: timeout not published, 200 Hz typical, not below 50 Hz.
24. "Response time and minimum rotation speed for T200 with Basic ESC", Blue Robotics Forum (third-party measurement). https://discuss.bluerobotics.com/t/response-time-and-minimum-rotation-speed-for-t200-with-basic-esc/11112. Used for: ~110 ms response lag.
25. "Basic ESC response time / speed ramp", Blue Robotics Forum (Blue Robotics staff). https://discuss.bluerobotics.com/t/basic-esc-response-time-speed-ramp/11359/2. Used for: 1112/1512/1912 rationale, startup-power filtering.
26. "R3 ESC - does it allow me to use 24 volt batteries?", Blue Robotics Forum (Blue Robotics staff). https://discuss.bluerobotics.com/t/r3-esc-does-it-allow-me-to-use-24-volt-batteries/2084/2. Used for: ~30 A at 20 V near the ESC limit.
27. "T200 Public Performance Data 10-20V September 2019" (xlsx), Blue Robotics. https://cad.bluerobotics.com/T200-Public-Performance-Data-10-20V-September-2019.xlsx. Used for: zero-thrust band, full-throttle current, open-loop statement, ESC and firmware used in testing.
28. "Autonomous t200 Thruster", Blue Robotics Forum (Blue Robotics staff). https://discuss.bluerobotics.com/t/autonomous-t200-thruster/11465/2. Used for: missing arming tones mean the stop signal was not held.
29. "BlueROV2 Heavy 6-DOF model", Blue Robotics Forum (Blue Robotics staff). https://discuss.bluerobotics.com/t/bluerov2-heavy-6-dof-model/13065/2. Used for: reversing direction by reversing the PWM signal.
