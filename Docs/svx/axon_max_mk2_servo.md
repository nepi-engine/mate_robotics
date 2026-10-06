# Axon Robotics Axon MAX MK2 Servo

## Summary

The Axon MAX MK2 is a programmable, standard-size hobby servo with a brushless motor, stainless steel gears and an aluminum case. Software commands it with a 500 to 2500 µs PWM pulse, and a fourth wire carries an analog position output back [1][4][9].

The most important fact for a driver author: the servo's behavior is set with the Axon Programmer MK2 and stored in the servo. That includes its travel limits, neutral, direction, signal-loss response, and even whether it runs as a position servo or a continuous-rotation servo [2]. A unit cannot be assumed to be at factory defaults.

Axon publishes no water rating. The IP67 figure comes only from distributors, and IP67 covers temporary immersion, not continuous use at depth [12][17].

## At a Glance

| Item | Value | Source |
|---|---|---|
| Command signal | Hobby-servo PWM, 500 to 2500 µs | [1] |
| Travel | "360° max range" (Axon); "350°" max and 0.175°/µs (distributor) | [1][12] |
| Signal logic level | 3.3 to 5 V (distributor) | [12] |
| Supply voltage | 4.8 to 8.4 V; 6.0 V recommended | [1][5] |
| Stall current | 3.6 A at 4.8 V up to 4.6 A at 8.4 V | [1] |
| Torque / speed | 28 kgf·cm, 0.140 s/60° at 4.8 V; 45 kgf·cm, 0.085 s/60° at 8.4 V | [1] |
| Wiring | 3-pin servo connector (300 mm, 22 AWG) plus an additional analog output | [1] |
| Position feedback | Analog output on a 4th wire; electrical scale not published | [4][12] |
| Modes | Servo mode, continuous rotation (CR) mode; swerve mode "coming soon" | [1] |
| Configuration | Axon Programmer MK2 with Axon Software MK2 only | [2] |
| Environmental rating | None from Axon; "IP67" from distributors | [8][12][13] |

## Physical and Electrical Interface

**Supply voltage.** 4.8 to 8.4 V [1]. Axon's quickstart recommends 6.0 V operation [5].

**Performance by supply voltage** (Axon's table) [1]:

| Supply | Idle current | Stall current | Speed | Torque |
|---|---|---|---|---|
| 4.8 V | 280 mA | 3600 mA | 0.140 s/60° | 28 kgf·cm |
| 6.0 V | 320 mA | 4000 mA | 0.115 s/60° | 34 kgf·cm |
| 7.4 V | 360 mA | 4300 mA | 0.100 s/60° | 39 kgf·cm |
| 8.4 V | 400 mA | 4600 mA | 0.085 s/60° | 45 kgf·cm |

Axon warns that its servos' stall current is "greater than the REV Robotics Control hub and Expansion hub can supply". It recommends lowering the servo's power limit when running from such sources [5]. Any supply for this servo must handle several amps of stall current.

**Mechanical.**

- **Weight.** Axon gives 85 g [1]; the goBILDA and ServoCity distributor pages give 78 g [12][13]. The manufacturer figure is the authoritative one.
- **Build.** Billet aluminum case (anodized black) and heat-treated stainless steel gears [1]. A brushless motor [9].
- **Output spline.** H25T [1].
- **Internal position sensor.** Distributors list a "Digital Encoder" [12]. Axon does not state the sensor type.
- **Dimensions.** From a distributor drawing [14]:
  - Case: 40 by 20 mm; 54 mm across the mounting tabs; mounting holes 48 mm apart.
  - Height: 38.8 mm from base to case top.

**Wiring.**

- **Cable.** A 3-pin servo connector on a 300 mm, 22 AWG cable, plus a separate analog output [1].
- **Connector.** Distributors list it as a "3-Pos TJC8 Servo Connector [MH-FC]" [12].
- **Signal wire colour.** Axon's programmer guide identifies the signal wire as grey [2].
- **Power, ground and analog wire colours, and the analog connector pinout.** Not published (Open Question 5).

**Signal level.** Distributors list the pulse amplitude as 3.3 to 5 V [12]. Axon does not state it. A 3.3 V pulse source is within the distributor's figure.

## Communication Protocol

**Signal.** Standard hobby-servo PWM: a pulse whose width sets the command, repeated at the frame rate. Axon's quickstart describes the servos as working with standard robot-controller servo ports out of the box [5]. No other runtime protocol is documented [1][2].

**Pulse mapping in servo (position) mode**

| Item | Value | Source |
|---|---|---|
| Pulse range | 500 to 2500 µs | [1][12] |
| Travel over that range | Up to 360° (Axon) / 350° (distributor) | [1][12] |
| Travel per µs | 0.175°/µs (distributor) | [12] |
| Center | 1500 µs: the programmer software's test slider is centered at 1500 µs, and it is the midpoint of the range | [3] |
| Direction | Increasing pulse width turns clockwise; programmable (distributor) | [12] |
| Deadband | 2 µs (distributor) | [12] |

- **Derived from the distributor figure:** 1000 to 2000 µs gives about 175° of travel at 0.175°/µs [12]. That assumes the travel limits are at their default values, which are not published (Open Question 2).
- **Programmable limits and neutral.** The left and right limits and the neutral position are programmable [2]. The real angle-to-pulse mapping on a given unit depends on its stored settings.

**Continuous-rotation (CR) mode.**

- **Pulse range.** Distributors give the CR pulse range as 1050 to 1950 µs [12].
- **Not published.** The stop pulse, the speed-versus-pulse curve and the direction convention in CR mode (Open Question 3).
- **How CR mode is set.** It is a separate firmware image that the programmer flashes onto the servo [2][11]. In CR mode the pulse width commands speed, not position [1][2].

**Frame rate.** Axon publishes no maximum or recommended frame rate [1][2].

- The programmer has a "Refresh rate" parameter [2]. Its documentation screenshot shows "3.0 ms" and "333 Hz" with no servo connected [3]. What this parameter controls is not documented.
- The robot-controller servo ports Axon says the servo works with send "variable length pulses every 20 ms", i.e. 50 Hz (third-party description) [5][16]. Inference from these two facts: 50 Hz works.

**Analog position output.** Axon: "The analog output from Axon servos allows you to read the position of the servo with high precision in realtime" [4]. The same page says "Docs coming soon!" [4]. Its voltage range, scale and update rate are not published (Open Question 4).

## Identification and Detection

- **No identification over the control wire.** The servo has no documented way to identify itself to the controller. The PWM wire is input-only [1][4]. A driver cannot detect whether the servo is connected from the PWM side.
- **Possible presence check.** If the analog output is wired to an ADC, a plausible voltage there could serve as one. That is untested (Open Question 4).
- **With the programmer.** Axon Software MK2 shows the servo's model name, manufacturer and firmware when the servo is connected through the Programmer MK2 [3]. "Model name" is a writable parameter [2].
- **Telling MK2 from MAX+.**
  - The Programmer MK2 works with the MAX MK2 and MINI MK2, but not with the MAX+, MINI+ or MICRO+ [2].
  - The older Servo Programmer is the reverse [6].
  - The MAX+ is discontinued and was replaced by the MAX MK2 [7].

## Startup and Initialization

- **Startup sound.** The servo plays a startup sound, which can be disabled [1][2].
- **Soft start.** Programmable [2]. The documentation screenshot shows "Level 1" [3].
- **Mode at boot.** The servo boots in whichever firmware mode (Servo or CR) was last flashed [2].
- **Out of the box.** Axon: "All servos come configured ready to use out of the box." [5]
- **Not documented.** What the servo does when powered with no PWM signal present (Open Question 7).

## Data and Commands

| Direction | What | Units and range | Source |
|---|---|---|---|
| Command | Pulse width | 500 to 2500 µs; position in servo mode, speed in CR mode | [1][2] |
| Command (CR mode) | Pulse width | 1050 to 1950 µs (distributor) | [12] |
| Feedback | Analog position voltage | Scale not published | [4] |

No other runtime commands or outputs are documented [1][2].

**Checking the driver's output with the programmer.** The Programmer MK2 can "display information about a PWM signal" when the signal is connected to its bottom-row pins with Signal Test enabled [2]. That is a convenient way to check the pulse width and frame rate the driver actually produces.

## Configuration and Calibration

**What the programmer can change.** Settings are read and written with the Axon Programmer MK2 and Axon Software MK2. The adjustable parameters are [2]:

- Left and right limit, neutral position
- Power limit, sensitivity, soft start, damping
- Model name, refresh rate
- 3-stage overload protection
- Lose PPM protection (signal loss)
- Power-on sound enable/disable
- Inversion

**Programmer settings**

| Setting | What it does | Values shown in Axon's documentation screenshot (no servo connected) | Source |
|---|---|---|---|
| Left / right limit | Travel limits | L 90, R 90 | [2][3] |
| Neutral position | Center offset | 0 | [2][3] |
| Power limit | Output power cap | 10% | [2][3] |
| Sensitivity | Deadband | Low / Medium / High / Ultra High options | [2][3] |
| Soft start | Startup acceleration limit | Level 1 | [2][3] |
| Damping | Servo loop damping | 0 | [2][3] |
| Refresh rate | Not documented | 3.0 ms (333 Hz) | [2][3] |
| 3-stage overload protection | Reduces power after timed stalls | 5.0 s / 60%, 5.0 s / 40%, 10.5 s / 20% | [2][3] |
| Lose PPM protection | Behavior when the PWM signal is lost | Release | [2][3] |
| Power-on sound | Enable / disable | — | [1][2] |
| Inversion | Switches CCW and CW | — | [2] |

- **Screenshot values are not defaults.** The values above come from a screenshot taken with no servo connected. Axon does not say they are factory defaults, and it publishes no factory-default list [2][3].
- **Restoring defaults.** The software has a "Default" button [2].
- **Older-model definitions, for context only.** Axon's documentation for the previous Servo Programmer (MAX+ era) defines the signal-loss options as [6]:
  - "Release: Acts as if the servo wasn't powered"
  - "Hold: Holds the last position"
  - "Neutral: Goes to the middle position"

  The MK2 documentation does not list its own options in text [2].

**Read, write and persistence.** "Read" loads the servo's current settings into the software, and "Write" stores them to the servo [2]. Because settings are written to the servo, they live on it. Axon does not explicitly say they survive power cycles (Open Question 2).

**Changing mode.** To switch between Servo and CR mode, download the matching firmware file and use "UPF" in Axon Software MK2 to flash it [2]. Axon provides "Axon MAX MK2 Servo Mode" and "Axon MAX MK2 CR Mode" firmware files [11].

**Programmer supply limits.** Axon says an alarm sounds and the programmer disables itself above 9 V DC [2]. goBILDA's programmer page says it disables above 8.4 V [15]. Keep the programmer supply at 8.4 V or below to satisfy both.

**Calibration.** Beyond setting neutral and limits [2], the servo has no calibration procedure.

## Failure Modes and Safety

- **Signal loss.**
  - The response is set by the "Lose PPM protection" parameter [2], so it can differ unit to unit.
  - Axon's screenshot shows "Release" [3]. On the older model, Release meant the servo behaves as if unpowered, Hold keeps the last position, and Neutral returns to center [6].
  - A driver cannot know which behavior a unit has without reading it with the programmer (Open Question 2).
- **Stall and overheating.**
  - Stall current reaches 4.6 A at 8.4 V [1].
  - 3-stage overload protection reduces power after timed stalls [2].
  - Axon's older-model documentation warns that at 100% power the servo can overheat and "will shut down to protect itself from burning out" [6]. It also says "There is no 3-stage Overload Protection in CR Mode" [6]. Whether these statements apply to the MK2 is not stated.
- **Unknown configuration.** Limits, neutral, inversion, power cap, signal-loss behavior and even the operating mode may have been changed by a previous user [2]. A unit flashed with CR firmware treats pulse width as speed, so a position command would spin it continuously [1][2].
- **Water.**
  - Axon's documentation contains no water, IP or temperature rating [8].
  - The goBILDA and ServoCity distributor pages list "IP67" [12][13]. IP67 means temporary immersion to 1 m for 30 minutes [17].
  - No source rates this servo for continuous submersion, for pressure at depth, or for salt water. On an underwater vehicle it may need a housing or other protection (Open Question 6).
- **Beta software.** Axon Software MK2 "is in beta" [2].

## Software and Libraries

There is no manufacturer driver library. Any code that can produce standard servo pulses can command the servo; see [the Teensy 4.1 document](../tbd/pjrc_teensy_4_1.md) for pulse generation on the likely controller.

| Name | Use | Platform / language | License | Link |
|---|---|---|---|---|
| Axon Software MK2 v1.1.2 | Read, write and reset servo settings; flash mode firmware; PWM signal test | Windows executable (.exe); beta | Not stated | [2][10] |
| Axon MAX MK2 Servo Mode / CR Mode firmware | Firmware images flashed to select the mode | .sfm files for the programmer | Not stated | [11] |
| Axon Programmer MK2 | USB-C programmer hardware for the above | — | — | [2] |

## Related Parts

- [PJRC Teensy 4.1](../tbd/pjrc_teensy_4_1.md): the likely source of this servo's PWM pulses. The servo's 500 to 2500 µs range and frame rate may differ from the ESCs', which matters if they share a timer.
- [Raspberry Pi 5](../tbd/raspberry_pi_5.md): could drive this servo from one of its hardware PWM channels instead, if the wiring puts it there.
- [Blue Robotics Basic ESC](../tbd/bluerobotics_basic_esc.md): also takes servo pulses, but over a different range (see that document), so the two channel types need separate limits in the driver.

## Open Questions

1. **What does the servo move on the vehicle?**
   - Why it matters: it sets the travel range to command, the safe direction, and what "stop" or "neutral" should mean.
   - How to get it: the vehicle design (for example a gripper or camera tilt), recorded in this repo.
2. **What settings are programmed into the unit?**
   - Why it matters: limits, neutral, inversion, power limit, sensitivity, signal-loss behavior and refresh rate all change the pulse-to-motion mapping and the failure behavior [2]. Whether settings persist across power cycles is also not stated.
   - How to get it: connect the servo to the Programmer MK2, press "Read" in Axon Software MK2 and record every value [2]. Power-cycle and read again to confirm persistence.
3. **Which mode is flashed: Servo or CR?**
   - Why it matters: CR mode turns pulse width into speed [2]. The CR stop pulse and speed curve are not published.
   - How to get it: the software shows the firmware when connected [3]. If CR, measure the stop pulse by stepping the command around 1500 µs.
4. **Analog output: wired, and what scale?**
   - Why it matters: it is the only feedback [4]. The driver needs volts-per-degree and the range.
   - How to get it: with the servo powered, command 500, 1500 and 2500 µs. Measure the analog wire with a multimeter against servo ground at each. Also confirm the wire colour and connector.
5. **Wire colours and pin order** of power, ground and the analog output.
   - Why it matters: wiring the harness without damage.
   - How to get it: inspect the cable and confirm with a multimeter (continuity to the connector pins) before powering.
6. **Water exposure.**
   - Why it matters: Axon publishes no rating, and IP67 (distributor) covers only temporary immersion to 1 m [12][17].
   - How to get it: decide whether the servo sits inside a dry housing or in the water. If in the water, at what depth and for how long, and whether that needs a supplier statement or a pressure test.
7. **Power-up behavior with no PWM signal.**
   - Why it matters: during boot the controller produces no pulses. Whether the servo holds, goes limp or moves determines whether it is safe.
   - How to get it: power the servo with the signal wire disconnected, then with the controller held in reset, and observe.
8. **Supply voltage and current.**
   - Why it matters: torque, speed and stall current all depend on voltage (4.8 to 8.4 V; up to 4.6 A stall) [1].
   - How to get it: the power-system design; record the voltage and current limit for the servo rail.
9. **Accepted frame rate.**
   - Why it matters: Axon publishes none.
   - How to get it: drive the servo at 50 Hz and at the ESC frame rate if it shares a timer group. Confirm smooth motion, and verify the produced signal with the programmer's Signal Test [2].

## Sources

1. "Axon MAX MK2", Axon Robotics documentation. https://docs.axon-robotics.com/servos/max. Used for: features, modes, voltage, PWM range, 360°, wiring, analog output, performance table, programmer compatibility.
2. "Programmer MK2", Axon Robotics documentation. https://docs.axon-robotics.com/servos/programmer. Used for: adjustable parameters, Read/Write/Default, mode change via UPF, signal test, grey signal wire, 9 V alarm, beta status, compatibility.
3. "Programmer MK2 software screenshot" (image embedded in the Programmer MK2 page), Axon Robotics. https://2269588985-files.gitbook.io/~/files/v0/b/gitbook-x-prod.appspot.com/o/spaces%2F9eh1a50wToy49XgPSa8i%2Fuploads%2FpFkGon4WQU7eHDp160c3%2FImage.png?alt=media&token=7f15cf49-9f90-4c7a-b88c-66cece2a54e2. Used for: 1500 µs centered slider, refresh-rate display, setting values shown with no servo connected, servo-information fields.
4. "Analog Output", Axon Robotics documentation. https://docs.axon-robotics.com/servos/analog-output. Used for: position output on the extra wire; "Docs coming soon!".
5. "Servo Quickstart", Axon Robotics documentation. https://docs.axon-robotics.com/servos/quickstart.md. Used for: ready to use out of the box, 6.0 V recommended, stall-current warning, compatibility with standard servo ports.
6. "Servo Programmer" (archive, MAX+ era), Axon Robotics documentation. https://docs.axon-robotics.com/archive/programmer.md. Used for: older-model setting definitions (Release/Hold/Neutral), thermal shutdown, no overload protection in CR mode, incompatibility with MK2.
7. "Axon MAX+" (archive), Axon Robotics documentation. https://docs.axon-robotics.com/archive/max.md. Used for: MAX+ discontinued and replaced by the MAX MK2.
8. "Axon documentation full text", Axon Robotics. https://docs.axon-robotics.com/llms-full.txt. Used for: confirming no water, IP or temperature rating appears in Axon's documentation.
9. "Axon MAX MK2", Axon Robotics store. https://axon-robotics.com/products/max. Used for: brushless motor, stainless gearbox, billet case.
10. "Axon Software MK2 v1.1.2" installer, Axon Robotics (download link from the programmer page; headers checked, not run). https://cdn.shopify.com/s/files/1/0657/9182/0008/files/Axon_Software_MK2_v1.1.2.exe?v=1758604512. Used for: software version and Windows executable type.
11. "Axon MAX MK2 Servo Mode" and "Axon MAX MK2 CR Mode" firmware, Axon Robotics (download links from the programmer page; headers checked). https://cdn.shopify.com/s/files/1/0657/9182/0008/files/Axon_MAX_MK2_Servo_Mode.sfm?v=1758606422 and https://cdn.shopify.com/s/files/1/0657/9182/0008/files/Axon_MAX_MK2_CR_Mode.sfm?v=1758606422. Used for: mode firmware files.
12. "Axon MAX Servo MK2", goBILDA (THIRD-PARTY distributor). https://www.gobilda.com/axon-max-servo-mk2/. Used for: 78 g, 3.3–5 V pulse amplitude, 2 µs deadband, 500–2500 / CR 1050–1950 µs, 350°, 0.175°/µs, clockwise direction, digital encoder, connector, IP67, 4th-wire feedback.
13. "Axon MAX Servo MK2", ServoCity (THIRD-PARTY distributor). https://www.servocity.com/axon-max-servo-mk2/. Used for: same specification table as [12], including IP67.
14. "2004-0025-0002 dimension drawing", ServoCity (THIRD-PARTY). https://cdn11.bigcommerce.com/s-f6vfspkkjf/images/stencil/1734w/products/5338/32056/b%252F221%252F2004-0025-0002-REV0-Schematic__69223__04709.1783712106.png?c=1. Used for: case dimensions and mounting-hole spacing.
15. "Axon Servo Programmer MK2", goBILDA (THIRD-PARTY distributor). https://www.gobilda.com/axon-servo-programmer-mk2/. Used for: programmer 8.4 V limit statement.
16. "SDK Servos", Game Manual 0 (THIRD-PARTY community guide). https://gm0.org/en/latest/docs/software/adv-control-system/sdk-servos.html. Used for: robot-controller servo ports send a pulse every 20 ms (50 Hz).
17. "Decoding IP Ratings: IP67 and IP68", Electropages (THIRD-PARTY). https://www.electropages.com/blog/2023/09/decoding-ip-ratings-comprehensive-guide-ip67-and-ip68-connector. Used for: IP67 meaning temporary immersion to 1 m for 30 minutes.
