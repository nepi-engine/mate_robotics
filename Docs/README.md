# MATE Standard Kit: Hardware Reference for Driver Authors

This folder holds one hardware reference per part in the MATE standard hardware kit. Each document is enough to start writing that part's software driver, and every fact in it links to its source.

## Parts

| Part | Folder | Document | Interface to software | Status |
|---|---|---|---|---|
| PJRC Teensy 4.1 microcontroller | tbd | [pjrc_teensy_4_1.md](tbd/pjrc_teensy_4_1.md) | USB serial (CDC-ACM) to the host; generates PWM, hosts I2C | Needs pin map and the host-to-Teensy protocol |
| Raspberry Pi 5 | tbd | [raspberry_pi_5.md](tbd/raspberry_pi_5.md) | Host computer: USB, I2C, UART, 4 hardware PWM channels | Needs bus and overlay plan |
| Blue Robotics T200 thruster (R2) | tbd | [bluerobotics_t200_thruster.md](tbd/bluerobotics_t200_thruster.md) | None; three motor phases driven by an ESC | Needs thruster count, layout and supply voltage |
| Blue Robotics Basic ESC (BESC30-R3) | tbd | [bluerobotics_basic_esc.md](tbd/bluerobotics_basic_esc.md) | One-way servo PWM, 1100 to 1900 µs | Needs pulse source and watchdog design; signal-loss timeout unmeasured |
| Axon Robotics Axon MAX MK2 servo | svx | [axon_max_mk2_servo.md](svx/axon_max_mk2_servo.md) | Servo PWM, 500 to 2500 µs, plus an analog position output | Needs settings read from the unit, and its job on the vehicle |
| DeepWater Exploration exploreHD (400 m) | idx | [dwe_explorehd_camera.md](idx/dwe_explorehd_camera.md) | USB 2.0 UVC (H.264, MJPEG, YUY2) | Needs `v4l2-ctl` and `lsusb` capture from a real unit; serial uniqueness unconfirmed |
| Adafruit BNO085 IMU (4754) | npx | [adafruit_bno085_imu.md](npx/adafruit_bno085_imu.md) | I2C, UART, UART-RVC or SPI | Needs interface choice and an I2C reliability test on the host |
| Blue Robotics Bar depth sensor (Bar30 or Bar02) | npx | [bluerobotics_bar_depth_sensor.md](npx/bluerobotics_bar_depth_sensor.md) | I2C | Needs model identified and a bus without the MS8607 |
| Adafruit MS8607 PHT sensor (4716) | tbd | [adafruit_ms8607_pht_sensor.md](tbd/adafruit_ms8607_pht_sensor.md) | I2C (two addresses) | Needs purpose, alarm thresholds and a bus without the Bar sensor |

The `tbd` folder holds parts that have no driver category assigned yet.

## Read First

### Kit layout

What each part is driven by, as confirmed in the part documents:

- **Pulse-driven parts with no data link of their own.**
  - The T200 thrusters have only motor phases and are driven by Basic ESCs.
  - Each ESC takes one-way servo pulses from 1100 to 1900 µs, at up to 400 Hz, and reports nothing back. See [the ESC document](tbd/bluerobotics_basic_esc.md).
  - The Axon servo also takes servo pulses (500 to 2500 µs), but it has a fourth wire carrying an analog position output. See [the servo document](svx/axon_max_mk2_servo.md).
  - None of these can be detected by software; their presence comes from configuration.
- **I2C sensors.**
  - The BNO085 IMU also offers UART, UART-RVC and SPI.
  - The Bar depth sensor and the MS8607 are I2C only.
- **USB devices.**
  - The exploreHD camera is a standard UVC camera.
  - The Teensy 4.1 appears as a CDC-ACM serial port.
- **Why the pulses likely come from the Teensy.**
  - The Pi 5 has exactly 4 independent hardware PWM channels on its header. A Pi 5-only overlay adds up to 4 PIO-assisted PWM outputs. The older DMA-based pigpio library does not run on the Pi 5. See [the Pi 5 document](tbd/raspberry_pi_5.md).
  - So 8 ESCs plus a servo (9 pulse trains) do not fit on the Pi, and 6 ESCs plus a servo fit only by mixing hardware and PIO outputs.
  - The Teensy 4.1 has 35 PWM pins in 22 independent frequency groups. See [the Teensy document](tbd/pjrc_teensy_4_1.md).
  - The likely design is: the Teensy generates every pulse train and may also host the I2C sensors, and the Pi 5 talks to it over USB serial. That is likely, not decided.

### I2C address map

| Address | Part | Changeable? |
|---|---|---|
| 0x40 | MS8607 humidity die | No |
| 0x4A (0x4B with DI pulled high) | BNO085 IMU | Yes, by pin, latched at reset |
| **0x76** | **MS8607 pressure/temperature die** | **No** |
| **0x76** | **Bar30 / Bar02 depth sensor** | **No** |

**CONFLICT at 0x76.** The MS8607 and the Bar sensor cannot share an I2C bus. Both also pass the same PROM CRC check, so a wrong-bus mistake can go unnoticed. The fixes are:

- Put them on separate buses. The Teensy has 3 I2C ports and the Pi 5 has several I2C controllers.
- Or use an I2C multiplexer, set to an address other than 0x76.

Adafruit warns that the BNO085 does not work well behind a multiplexer, so keep it off any multiplexer. See [the Bar document](npx/bluerobotics_bar_depth_sensor.md), [the MS8607 document](tbd/adafruit_ms8607_pht_sensor.md) and [the BNO085 document](npx/adafruit_bno085_imu.md).

### Open decisions that block more than one driver

1. **Wiring and pin map.** Which host each part connects to, and the full list of:
   - Teensy and Pi pins
   - I2C buses
   - the multiplexer, if any

   This blocks the Teensy, Pi 5, ESC, servo, BNO085, Bar and MS8607 drivers.
2. **The Pi-to-Teensy protocol.** The command set, framing and checksum, identify reply, telemetry rate, and the command watchdog that stops every thruster when commands stop. Nothing exists yet. See the To Be Defined list in [the Teensy document](tbd/pjrc_teensy_4_1.md). This blocks the Teensy, ESC, servo, and any sensor the Teensy hosts.
3. **Which Bar model: Bar30 or Bar02.** The two use different conversion math, and library auto-detection can misclassify a Bar30. This blocks the Bar driver and the I2C bus plan.
4. **Thruster count and layout.** The number of thrusters, their positions, orientations, propeller handedness, supply voltage, and the vehicle's current limit. This blocks the ESC and T200 drivers, the Teensy pin map, and the IMU's magnetic-heading decision.
5. **How the 0x76 conflict is resolved.** Separate buses or a multiplexer. This blocks the Bar, MS8607 and BNO085 drivers.
