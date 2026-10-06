# Blue Robotics T200 Thruster (R2)

## Summary

The T200 is an underwater thruster built around a fully flooded, three-phase, sensorless brushless outrunner motor. Its three cable conductors are motor phases, so a sensorless brushless ESC is required to run it [1][4]. Software never talks to the thruster: it commands an ESC with a pulse width, and the ESC drives the motor [4][5].

The most important facts for a driver author:

- There is no feedback of any kind.
- Thrust is a nonlinear, direction-asymmetric function of pulse width and supply voltage, with a zero-thrust band around 1500 µs [2].
- The motor must not run dry for more than 10 seconds [1].

## At a Glance

| Item | Value | Source |
|---|---|---|
| Motor | Three-phase sensorless brushless outrunner, fully flooded | [1] |
| Drive | One sensorless brushless ESC per thruster | [1] |
| Operating voltage | 7 to 20 V; 12 to 16 V recommended; optimized for 16 V | [1] |
| Full-throttle thrust, forward / reverse | 3.71 / 2.92 kgf at 12 V; 5.25 / 4.1 kgf at 16 V; 6.7 / 5.05 kgf at 20 V | [1] |
| Full-throttle current (power) | 17 A (205 W) at 12 V; 24 A (390 W) at 16 V; 32 A (645 W) at 20 V | [1] |
| Minimum thrust | 0.02 kgf, limited by the ESC used | [1] |
| Command interface | None on the thruster; command the ESC with 1100 to 1900 µs pulses | [2][5] |
| Feedback | None; the ESC is open loop | [2] |
| Cable | 3 conductors, 16 AWG (R2), bare tinned ends; blue, green, white | [1][4] |
| Maximum tested depth | 300 m (seawater), stated from the March 2025 revision | [1] |

## Physical and Electrical Interface

**Cable.**

- The R2 uses cable BR-100997 with 16 AWG conductors [1].
- The R2 release on 12 Feb 2021 changed the conductors from 18 AWG to 16 AWG, which is the visible difference from R1 [1].
- The three conductors end in tinned wire, with no connector [4]. Their colours are blue, green and white [4].
- The standard R2 drawing shows 880 mm ±40 of jacketed cable plus 130 mm ±25 of exposed conductors [3].
- The penetrator that fits is the WetLink M10-6.5mm-LC [1].

**Wiring to the ESC.** Connect the three thruster wires to the ESC's three motor wires. The order does not matter for operation; swapping any two reverses rotation [4][5]. For the earlier 18 AWG cable, Blue Robotics advised keeping the thruster-to-ESC cable to about 2 m or less [14].

**Size and weight.**

| Item | Value | Source |
|---|---|---|
| Length | 113 mm (table); 113.4 mm (drawing) | [1][3] |
| Diameter | 100 mm (table); 97.3 mm (drawing) | [1][3] |
| Propeller diameter | 76 mm | [1] |
| Weight in air, standard 1 m cable | 427 g | [1] |
| Weight in water, standard 1 m cable | 239 g | [1] |

The table and the drawing disagree on diameter. Use the drawing for mechanical fit, since it is the dimensioned source.

**Mounting.** M3x0.5 holes on a 19 mm spacing, plus one M6x1.0 insert added 14 Nov 2024 [1][3].

**Supply and current.**

- Rated range: 7 to 20 V. Operating at 12 to 16 V is recommended, and going over 20 V "is not within the rating" [1].
- Source disagreement: a 2018 Blue Robotics forum post said the T200 "is rated to run at 6-20V" [9]. The current product page's 7 to 20 V is the newer statement and the one to use.
- Full-throttle current reaches 32 A at 20 V [1][2].

**Construction.** The motor has "encapsulated motor windings and stator as well as coated magnets and rotor". The water cools the motor and lubricates the plastic bushings [1].

**Motor constants.** Blue Robotics does not publish Kv, pole count or winding resistance. Asked about them, a staff member replied that they "don't have any more information than what is on our Technical Details tab" [10]. A third-party forum user measured 14 poles and a stator resistance of about 0.1 Ω [10]. That is not a manufacturer value.

## Communication Protocol

The thruster has no data protocol. It is three motor phases driven by the ESC [1][4]. The command path is: controller → pulse width → ESC → three-phase drive. The pulse-width rules (1100 to 1900 µs, 1500 µs stop, deadband, arming, 400 Hz maximum) belong to the ESC and are in [the Basic ESC document](bluerobotics_basic_esc.md) [5].

### Published PWM to thrust and current data

Blue Robotics publishes bollard (static) thrust, current, RPM and power against ESC pulse width at 10, 12, 14, 16, 18 and 20 V. The data runs in 4 µs steps from 1100 to 1900 µs [2]. The product page links this file as the raw data behind its specifications [1].

Test conditions, from the file's "READ ME FIRST" sheet [2]:

- **ESC:** Blue Robotics Basic ESC R3 with stock settings and BLHeli_S 16.6.
- **Thruster:** a single standard T200 made in August 2019, with the 1 m, 18 AWG retail cable. That date makes it an R1 unit; R2 (16 AWG) was released in Feb 2021 [1].
- **Conditions:** bollard thrust, static, water inlet velocity zero.
- **Supply:** voltage sensed at the ESC power input.
- **Zero cut-off:** "all values below 300 RPM are dropped to zero", so the zero-thrust band below includes that cut-off as well as the ESC deadband.
- **Sign convention:** 1100 to 1500 µs is reverse and 1500 to 1900 µs is forward.

A 2019 Blue Robotics forum post says there had been "no significant performance changes to the T200 since launch" [13]. No R2-specific data set was found.

The three tables below are extracted from the published file at 100 µs points [2].

**Thrust (kgf; negative is reverse)**

| PWM (µs) | 10 V | 12 V | 14 V | 16 V | 18 V | 20 V |
|---|---|---|---|---|---|---|
| 1100 | -2.31 | -2.90 | -3.52 | -4.07 | -4.59 | -5.04 |
| 1200 | -1.54 | -1.95 | -2.35 | -2.71 | -2.96 | -3.13 |
| 1300 | -0.80 | -1.02 | -1.24 | -1.44 | -1.56 | -1.58 |
| 1400 | -0.24 | -0.32 | -0.40 | -0.48 | -0.53 | -0.53 |
| 1500 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 |
| 1600 | 0.29 | 0.39 | 0.50 | 0.60 | 0.63 | 0.64 |
| 1700 | 0.97 | 1.28 | 1.55 | 1.82 | 1.92 | 1.92 |
| 1800 | 1.91 | 2.46 | 2.99 | 3.42 | 3.72 | 3.96 |
| 1900 | 2.93 | 3.71 | 4.52 | 5.25 | 6.02 | 6.72 |

**Current (A)**

| PWM (µs) | 10 V | 12 V | 14 V | 16 V | 18 V | 20 V |
|---|---|---|---|---|---|---|
| 1100 | 13.6 | 17.0 | 20.7 | 24.3 | 28.2 | 32.2 |
| 1200 | 7.1 | 8.9 | 10.6 | 11.9 | 13.3 | 14.7 |
| 1300 | 2.6 | 3.3 | 3.9 | 4.4 | 4.8 | 5.2 |
| 1400 | 0.4 | 0.5 | 0.6 | 0.7 | 0.8 | 0.8 |
| 1500 | 0.0 | 0.0 | 0.0 | 0.0 | 0.0 | 0.0 |
| 1600 | 0.4 | 0.5 | 0.7 | 0.8 | 0.8 | 0.8 |
| 1700 | 2.7 | 3.3 | 3.9 | 4.6 | 4.7 | 5.1 |
| 1800 | 7.1 | 8.9 | 10.6 | 11.9 | 13.1 | 14.5 |
| 1900 | 13.6 | 16.9 | 20.4 | 23.8 | 27.6 | 31.2 |

**Summary per supply voltage**

| Supply | Max forward thrust | Max reverse thrust | Max current | Max power | Zero-thrust band in the data |
|---|---|---|---|---|---|
| 10 V | 2.93 kgf at 1900 µs | 2.31 kgf at 1100 µs | 13.6 A at 1104 µs | 136 W | 1460 to 1540 µs |
| 12 V | 3.71 kgf at 1900 µs | 2.92 kgf at 1104 µs | 17.1 A at 1104 µs | 205 W | 1464 to 1536 µs |
| 14 V | 4.53 kgf at 1896 µs | 3.52 kgf at 1100 µs | 20.7 A at 1100 µs | 290 W | 1468 to 1532 µs |
| 16 V | 5.25 kgf at 1900 µs | 4.07 kgf at 1100 µs | 24.3 A at 1104 µs | 389 W | 1472 to 1528 µs |
| 18 V | 6.02 kgf at 1900 µs | 4.59 kgf at 1100 µs | 28.2 A at 1100 µs | 508 W | 1472 to 1528 µs |
| 20 V | 6.72 kgf at 1900 µs | 5.04 kgf at 1100 µs | 32.2 A at 1100 µs | 643 W | 1476 to 1528 µs |

What a driver author should take from the tables [2]:

- **Thrust is not linear in pulse width.** It is roughly quadratic away from neutral.
- **Reverse is weaker than forward.** At the same pulse offset, reverse gives about 75 to 80% of forward thrust at full throttle.
- **Thrust depends strongly on voltage.** A thrust-to-pulse map must use the actual battery voltage, or the vehicle's thrust will drift as the battery drains.

## Identification and Detection

- **No electronic presence check.** The thruster has no ID, sensor or data line; its cable carries only motor phases [1][4]. Software cannot tell whether a thruster is connected, which model it is, or whether it is turning. A driver must take the thruster's presence and position from configuration.
- **ESC tones.** The only presence check Blue Robotics documents is audible. An ESC with a motor connected plays three rising tones at power-up [5]. A driver cannot read this.
- **Telling revisions apart.** R2 has 16 AWG conductors and R1 has 18 AWG [1]. Units made from 14 Mar 2023 carry a laser-marked serial number [1].
- **Propeller handedness.** Propellers are labelled "CW" and "CCW". The direction can also be read from the leading edges of the blades, viewed from the front nose cone [4]. Each thruster ships with a CW propeller installed and a CCW propeller in the box [1].

## Startup and Initialization

The thruster has no startup sequence of its own. Arming is done by the ESC, which must see a stop pulse before it will drive. The details are in [the Basic ESC document](bluerobotics_basic_esc.md) [4][5].

Blue Robotics' quick start [1]:

1. Connect the thruster wires to the ESC phase wires.
2. Power the ESC.
3. Connect the ESC signal.
4. Send a signal.

## Data and Commands

| Direction | What | Units and range | Source |
|---|---|---|---|
| Command (through the ESC) | Pulse width | 1100 to 1900 µs; 1500 µs stop; up to 400 Hz | [5] |
| Output | Thrust | Per the tables above; up to 6.72 kgf forward and 5.04 kgf reverse at 20 V | [2] |
| Output | Supply current | Per the tables above; up to 32.2 A at 20 V | [2] |
| Feedback | None | — | [2] |

Blue Robotics describes the Basic ESC as "an open loop control system", with no direct relationship between PWM input and RPM [2]. A given pulse width produces a given thrust only under the published conditions: static water, the stated voltage, and the stated ESC and firmware [2].

## Configuration and Calibration

**Definitions** [4]:

- The front of the thruster is the side where the cable enters the nose cone.
- Forward thrust means water enters at the front and leaves through the back of the nozzle.
- Reverse gives "slightly lower force and efficiency".

**Reversing direction.** Use only one of these methods per thruster:

| Method | What it does | Source |
|---|---|---|
| Swap any two of the three phase wires | Reverses rotation | [4][5] |
| Mirror the command about 1500 µs in software | Reverses rotation | [7][8] |
| Fit the opposite-handed propeller | Reverses the water flow for the same signal | [6] |

Blue Robotics says of the propeller and wire methods that "you only need to do one, not both" [6]. Software reversal plays the same role.

**Counter-rotating pairs.** CW and CCW propellers are used on pairs of thrusters on the same axis so that their torques cancel [4][6].

**Calibration.** The thruster has nothing to calibrate electrically [1][4]. Thrust calibration means choosing a pulse-to-thrust map, for example by interpolating the published tables at the measured supply voltage [2]. The published data is from an R1 unit in a test tank, so on-vehicle thrust will differ (Open Question 7).

## Failure Modes and Safety

- **Dry running.** "Do not operate the thruster for more than 10 seconds outside of water (while dry). The bearings require water for lubrication and may be damaged." [1][4] Any bench test of the driver needs the thrusters in water, or the ESC outputs held at stop.
- **Noise.** Clicking or noise is normal, especially when dry [1][4].
- **Over-voltage.** Above 20 V is outside the rating [1]. A 2018 Blue Robotics forum post warned of "imminent danger of overheating and burning out the thruster in minutes or less" at 27 to 28 V [9].
- **High current.**
  - Full throttle at 20 V draws about 32 A [1][2].
  - A Blue Robotics forum post notes this is "near the limit of the Basic ESCs capabilities" [9].
  - The vehicle's total current is the sum across all thrusters at their commanded pulse (Open Question 5).
- **Magnets.**
  - The rotor carries coated permanent magnets [1].
  - "Iron particles may be pulled from the water and collect inside the rotor on the magnets". Blue Robotics says to clear them regularly to prevent corrosion [1].
  - Abrasive particles can scratch the rotor coating until the rotor corrodes and seizes [4].
- **Magnetic interference with a compass.**
  - Blue Robotics' founder wrote, about the earlier T100 of the same design family, that the motors' field "should be fairly small since they have a flux ring on the outside of the rotor". He pointed to high-current wiring as the more likely cause of the compass interference being discussed [11].
  - No T200 field-strength figure is published [1]. Both the magnets and the up-to-32 A phase and supply currents sit near any IMU on the vehicle. See [the BNO085 document](../npx/adafruit_bno085_imu.md).
- **Propeller breakage.**
  - A Blue Robotics change notice (PCN THR-2026-001) returned the propeller to 100% polycarbonate, reversing a 10% glass-filled change "that resulted in increased propeller breakage reports" [12].
  - The glass-filled material was introduced on 21 May 2024 [1].
- **Threadlocker.** Most threadlockers attack polycarbonate and will damage the thruster [1][4].
- **Debris.** Avoid drawing seaweed or other objects into the thruster [1].
- **Phase faults.** Blue Robotics' troubleshooting compares the resistance of each phase-wire pair (blue/green, blue/white, green/white). The three should match within about 0.1 to 0.2 Ω [4].

## Software and Libraries

The thruster runs no software. Command code belongs to the ESC (see [the Basic ESC document](bluerobotics_basic_esc.md)).

| Name | Use | Language | License | Link |
|---|---|---|---|---|
| T200 public performance data (xlsx) | Thrust, current, RPM and power vs pulse width at 10 to 20 V | Spreadsheet | Not stated | [2] |
| bluerobotics/br-esc-examples | Arduino sketches that drive Blue Robotics ESCs and thrusters with the Servo library | C++ (Arduino) | MIT | [15] |

## Related Parts

- [Blue Robotics Basic ESC](bluerobotics_basic_esc.md): required to drive each thruster; owns the pulse-width command, arming and signal-loss behavior.
- [PJRC Teensy 4.1](pjrc_teensy_4_1.md): the likely source of the ESC pulse trains.
- [Raspberry Pi 5](raspberry_pi_5.md): explains why the pulse trains likely come from the Teensy rather than the Pi.
- [Adafruit BNO085 IMU](../npx/adafruit_bno085_imu.md): its magnetometer heading may be affected by the thrusters' magnets and wiring currents [1][11]; testing is an open question in both documents.

## Open Questions

1. **How many thrusters, and where?**
   - Why it matters: the driver needs a channel per thruster, and the control layer needs each one's position and thrust axis.
   - How to get it: the vehicle design; recorded as a thruster table in this repo.
2. **Orientation and propeller handedness per position.**
   - Why it matters: the sign of each thruster's command depends on its mounting direction, its propeller (CW or CCW), and whether two phase wires were swapped [4][6].
   - How to get it: inspect each installed thruster (propeller label, blade edge [4]). Then give each one a short in-water pulse at 1550 µs and record the direction of flow.
3. **Supply voltage.**
   - Why it matters: thrust and current depend strongly on voltage [2]. The thrust map should use the measured voltage.
   - How to get it: the battery choice (nominal and minimum voltage), and whether the voltage is measured and fed to software.
4. **Which revision, and when built?**
   - Why it matters: R1 vs R2 conductor gauge, and the glass-filled propeller window (21 May 2024 to 7 May 2026) [1][12].
   - How to get it: inspect conductor gauge, serial marking and purchase date.
5. **Current or power limit the vehicle must stay under.**
   - Why it matters: several thrusters at full throttle can draw well over 100 A at 16 V [2]. The driver or control layer may need to cap total commanded current.
   - How to get it: the battery, fuse and tether power design, written down as a number.
6. **Distance from thrusters and power wiring to the IMU.**
   - Why it matters: magnetic heading may be unusable if the IMU is too close [11].
   - How to get it: the vehicle layout. Then test by logging magnetometer output while stepping each thruster through its range in water.
7. **On-vehicle thrust vs published data.**
   - Why it matters: the data is static, from an R1 unit with 18 AWG cable [2]. Different cable length, nozzle obstructions and vehicle speed all change thrust.
   - How to get it: a bollard test of one installed thruster on a force gauge at two or three pulse widths.
8. **Cable length between thruster and ESC.**
   - Why it matters: longer cable adds voltage drop [2]. Blue Robotics advised about 2 m or less for the older 18 AWG cable [14].
   - How to get it: the vehicle layout.

## Sources

1. "T200 Thruster" (R2), Blue Robotics. https://bluerobotics.com/store/thrusters/t100-t200-thrusters/t200-thruster-r2-rp/. Used for: technical details, voltage range, thrust and current at 12/16/20 V, materials, revision history, dry-run warning, magnets, FAQ.
2. "T200 Public Performance Data 10-20V September 2019" (xlsx), Blue Robotics. https://cad.bluerobotics.com/T200-Public-Performance-Data-10-20V-September-2019.xlsx. Used for: all PWM, thrust and current tables, test conditions, open-loop note.
3. "T200 standard drawing" (PNG), Blue Robotics. https://bluerobotics.com/wp-content/uploads/2021/02/T200-standard-drawing.png. Used for: dimensions, cable lengths.
4. "Thruster User Guide", Blue Robotics. https://bluerobotics.com/learn/thruster-usage-guide/. Used for: wiring, wire colours, direction definitions, propeller identification, arming, maintenance, troubleshooting.
5. "Basic ESC" (BESC30-R3), Blue Robotics. https://bluerobotics.com/store/thrusters/speed-controllers/besc30-r3/. Used for: pulse range and update rate, power-up tones, phase swap reverses direction.
6. "Changing thruster direction", Blue Robotics Forum (Blue Robotics staff). https://discuss.bluerobotics.com/t/changing-thruster-direction/12828. Used for: propeller swap vs wire swap, counter-rotation.
7. "BlueROV2 Heavy 6-DOF model", Blue Robotics Forum (Blue Robotics staff). https://discuss.bluerobotics.com/t/bluerov2-heavy-6-dof-model/13065/2. Used for: reversing by swapping phases or reversing the PWM signal.
8. "T200 thruster, anti clock wise or clockwise", Blue Robotics Forum (Blue Robotics staff). https://discuss.bluerobotics.com/t/t200-thruster-anti-clock-wise-or-clockwise/12170. Used for: reversal by wiring or by software setting.
9. "R3 ESC - does it allow me to use 24 volt batteries?", Blue Robotics Forum (Blue Robotics staff). https://discuss.bluerobotics.com/t/r3-esc-does-it-allow-me-to-use-24-volt-batteries/2084/2. Used for: 6–20 V statement (2018), over-voltage risk, ~30 A near the ESC limit.
10. "Full caracteristics of the T100 and T200", Blue Robotics Forum (staff reply; third-party measurement). https://discuss.bluerobotics.com/t/full-caracteristics-of-the-t100-and-t200/5226. Used for: motor constants not published; third-party pole count and resistance.
11. "Frustrated. Any suggestions?", Blue Robotics Forum (founder reply, T100 era). https://discuss.bluerobotics.com/t/frustrated-any-suggestions/305. Used for: flux-ring statement on motor field, wiring as interference source.
12. "Product Change Notifications", Blue Robotics. https://bluerobotics.com/product-change-notifications/. Used for: PCN THR-2026-001 propeller material reversal.
13. "T200 performance charts", Blue Robotics Forum (Blue Robotics staff). https://discuss.bluerobotics.com/t/t200-performance-charts/6031. Used for: no significant performance change since launch.
14. "When will BlueESC be available?", Blue Robotics Forum (Blue Robotics staff). https://discuss.bluerobotics.com/t/when-will-blueesc-be-available/839. Used for: about 2 m maximum thruster-to-ESC cable with 18 AWG.
15. "br-esc-examples", Blue Robotics (GitHub). https://github.com/bluerobotics/br-esc-examples. Used for: example code, MIT license.
