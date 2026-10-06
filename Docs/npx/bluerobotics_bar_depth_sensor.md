# Blue Robotics Bar Depth/Pressure Sensor (Bar30 and Bar02)

## Summary

The Bar sensors are Blue Robotics' sealed-face, bulkhead-mounted water pressure sensors [1]. There are two models, and the kit does not say which was bought, so this document covers both:

- The Bar30 uses the TE MS5837-30BA and reads up to 30 bar, about 300 m of water.
- The Bar02 uses the MS5837-02BA and reads up to 2 bar, about 10 m, at much finer resolution.

Software talks to either one over I2C at the fixed address 0x76 [1][2]. It reads factory calibration words, triggers pressure and temperature conversions, and computes calibrated values.

The most important fact for a driver author: the two models use different conversion math and output units [10][11]. Running the wrong model's formula gives readings off by about 20×. Neither model has an ID register that cleanly says which it is.

## At a Glance

| Item | Bar30 | Bar02 | Source |
|---|---|---|---|
| Sensor chip | MS5837-30BA | MS5837-02BA | [1] |
| Interface | I2C, address 0x76 (fixed) | I2C, address 0x76 (fixed) | [1][2] |
| Pressure range | 0 to 30 bar | 0.3 to 1.2 bar; 0.01 to 2 bar extended | [1] |
| Depth range | 0 to 295.6 m | 0 to 10 m | [1] |
| Max mechanical pressure | 50 bar | 10 bar | [1] |
| Resolution (OSR 8192) | 0.2 mbar (2 mm of fresh water) | 0.016 mbar (0.16 mm of fresh water) | [1] |
| Pressure accuracy | ±200 mbar absolute (0–45 °C) | ±2 mbar relative (0–60 °C) | [1] |
| Temperature accuracy | ±4 °C | ±2 °C | [1] |
| Board supply (Vin) | 2.5 to 5.5 V | 2.5 to 5.5 V | [1] |
| I2C logic | 2.5 to 3.6 V (3.3 V logic; 5 V logic damages the sensor) | same | [1][2] |
| Max I2C clock | 400 kHz | 400 kHz | [10][11] |
| Connector | 4-pin JST GH (R2); was DF13 on R1 | same | [1] |
| Raw pressure output unit | 0.1 mbar | 0.01 mbar | [10][11] |
| Current revision | BAR30-SENSOR-R2 (9 May 2022) | BAR02-SENSOR-R2 (11 May 2022) | [1] |

## Physical and Electrical Interface

**Connector and pinout.** 4-position JST GH [1][5]:

| Pin | Wire | Signal |
|---|---|---|
| 1 | Red | Vin, 2.5 to 5.5 V |
| 2 | Green | SCL, 3.3 V logic |
| 3 | White | SDA, 3.3 V logic |
| 4 | Black | GND |

The R2 revision (May 2022) changed the connector from DF13 to JST GH [1].

**Logic level.** "These lines use 3.3 V logic … Do not try to connect the sensor's SDA and SCL lines directly to a 5 V device; this will damage the sensor." [2]

**Board circuit.** Blue Robotics' schematic, dated 2016 and linked for both models, shows [6][3]:

- Vin passes through a diode into a MIC5365 regulator that makes 3.3 V for the sensor.
- SDA and SCL run straight to the sensor, with no level shifter and no pull-up resistors.

The original README lists reverse-polarity protection [7]. Whether the R2 board still matches the 2016 schematic is not stated (Open Question 5). The host must supply the I2C pull-ups.

**Sensor chip electrical.** Both datasheets give [10][11]:

- **Supply:** 1.5 to 3.6 V.
- **Input thresholds:** high at 80 to 100% of VDD, low at 0 to 20% of VDD.
- **Current:** 1.25 mA peak during a conversion; 0.01 µA typical in standby.

At the board's 3.3 V supply, an input high must therefore be at least 2.64 V. That figure is derived from the datasheet thresholds.

**Mechanical** [1][2][4]:

- M10 x 1.5 thread, 14 g. Installed through a 10.0 to 10.2 mm clearance hole or an M10 x 1.5 tapped hole.
- Sealed by an O-ring under the bulkhead, against a flat, smooth surface.
- The current drawing shows a 295 mm ±20 cable.

**Sealing.** "The Bar sensor is not fully waterproof. The front side is sealed and can be exposed to water, but the back side (where the wires exit) is not sealed—water will enter if submerged." [1] It must be mounted through a pressure-housing wall.

**Wiring length.** "A few centimeters of jumper wire is fine, but anything longer can cause unstable or complete communication failure." [2]

## Communication Protocol

**Bus and address.** I2C at 0x76 (binary 1110110), up to 400 kHz [10][11]. The address "is hardcoded and cannot be changed, so only one Bar sensor can be connected to a single I²C bus at a time". Blue Robotics suggests a second bus or a hardware I2C multiplexer [2].

**Commands.** Each command is one byte written to the sensor [10][11].

| Command | Byte |
|---|---|
| Reset | 0x1E |
| Convert D1 (pressure), OSR 256 / 512 / 1024 / 2048 / 4096 / 8192 | 0x40 / 0x42 / 0x44 / 0x46 / 0x48 / 0x4A |
| Convert D2 (temperature), same OSR order | 0x50 / 0x52 / 0x54 / 0x56 / 0x58 / 0x5A |
| ADC read (then read 3 bytes, MSB first) | 0x00 |
| PROM read word n (then read 2 bytes, MSB first) | 0xA0 + 2·n, n = 0 to 6 |

**Conversion timing.**

- Both models support all six oversampling ratios (OSR), including 8192 [10][11].
- Wait at least the maximum conversion time before reading: "Maximum values must be used to determine waiting times in I2C communication" [10][11].

| OSR | Bar30 max (ms) | Bar02 max (ms) | Bar30 pressure resolution (mbar RMS) | Bar02 pressure resolution (mbar RMS) |
|---|---|---|---|---|
| 256 | 0.60 | 0.56 | 1.57 | 0.11 |
| 512 | 1.17 | 1.10 | 0.84 | 0.062 |
| 1024 | 2.28 | 2.17 | 0.54 | 0.039 |
| 2048 | 4.54 | 4.32 | 0.38 | 0.028 |
| 4096 | 9.04 | 8.61 | 0.28 | 0.021 |
| 8192 | 18.08 | 17.2 | 0.20 | 0.016 |

Sources for the table: Bar30 figures [10], Bar02 figures [11].

**Read rules** [10][11]:

- An ADC read with no conversion done, or repeated, returns 0.
- An ADC read sent during a conversion returns 0, and "the final result will be wrong".
- Starting a new conversion during one already running also gives incorrect results.

**PROM layout** [10][11]:

| Word | Address | Contents |
|---|---|---|
| 0 | 0xA0 | Bits 15:12 = CRC-4; bits 11:5 = product version; bits 4:0 = factory |
| 1 | 0xA2 | C1: pressure sensitivity (SENS_T1) |
| 2 | 0xA4 | C2: pressure offset (OFF_T1) |
| 3 | 0xA6 | C3: temperature coefficient of pressure sensitivity (TCS) |
| 4 | 0xA8 | C4: temperature coefficient of pressure offset (TCO) |
| 5 | 0xAA | C5: reference temperature (T_REF) |
| 6 | 0xAC | C6: temperature coefficient of the temperature (TEMPSENS) |

**CRC-4 check.** The datasheets give this C code; both Blue Robotics libraries implement it line for line [8][9][10][11]. Compare the result with `(word0 & 0xF000) >> 12`.

```c
unsigned char crc4(unsigned int n_prom[])   // n_prom: 8 words, n_prom[7] used as scratch
{
    int cnt; unsigned int n_rem = 0; unsigned char n_bit;
    n_prom[0] = ((n_prom[0]) & 0x0FFF);     // CRC bits replaced by 0
    n_prom[7] = 0;
    for (cnt = 0; cnt < 16; cnt++) {
        if (cnt % 2 == 1) n_rem ^= (unsigned short)((n_prom[cnt >> 1]) & 0x00FF);
        else              n_rem ^= (unsigned short)(n_prom[cnt >> 1] >> 8);
        for (n_bit = 8; n_bit > 0; n_bit--) {
            if (n_rem & (0x8000)) n_rem = (n_rem << 1) ^ 0x3000;
            else                  n_rem = (n_rem << 1);
        }
    }
    n_rem = ((n_rem >> 12) & 0x000F);       // final 4-bit remainder is the CRC
    return (n_rem ^ 0x00);
}
```

### First-order calculation

D1 is the raw pressure and D2 the raw temperature, both 24-bit. C1 to C6 are the PROM words. Formulas are from the datasheets [10][11]:

| Variable | Bar30 (MS5837-30BA) | Bar02 (MS5837-02BA) |
|---|---|---|
| dT | `D2 - C5 * 2^8` | `D2 - C5 * 2^8` |
| TEMP | `2000 + dT * C6 / 2^23` | `2000 + dT * C6 / 2^23` |
| OFF | `C2 * 2^16 + (C4 * dT) / 2^7` | `C2 * 2^17 + (C4 * dT) / 2^6` |
| SENS | `C1 * 2^15 + (C3 * dT) / 2^8` | `C1 * 2^16 + (C3 * dT) / 2^7` |
| P | `(D1 * SENS / 2^21 - OFF) / 2^13` | `(D1 * SENS / 2^21 - OFF) / 2^15` |
| P unit | 0.1 mbar (39998 = 3999.8 mbar) | 0.01 mbar (110002 = 1100.02 mbar) |
| TEMP unit | 0.01 °C (1981 = 19.81 °C) | 0.01 °C (2000 = 20.00 °C) |

**Integer sizes.** Use signed 32-bit for dT and TEMP, and signed 64-bit for OFF and SENS [10][11].

**Unit-test vectors from the datasheets:**

| | C1–C6 | D1 / D2 | Expected results |
|---|---|---|---|
| Bar30 [10] | 34982, 36352, 20328, 22354, 26646, 26146 | 4958179 / 6815414 | dT = −5962, TEMP = 1981, OFF = 2381323464, SENS = 1145816755, P = 39998 |
| Bar02 [11] | 46372, 43981, 29059, 27842, 31553, 28165 | 6465444 / 8077636 | dT = 68, TEMP = 2000, OFF = 5764707214, SENS = 3039050829, P = 110002 |

### Second-order compensation

Apply these after the first-order step. TEMP is the first-order value in 0.01 °C, so "20 °C" means TEMP < 2000 [10][11].

**Bar30 (MS5837-30BA)** [10]:

```
if TEMP < 2000:                       # low temperature
    Ti    = 3 * dT^2 / 2^33
    OFFi  = 3 * (TEMP - 2000)^2 / 2^1
    SENSi = 5 * (TEMP - 2000)^2 / 2^3
    if TEMP < -1500:                  # very low temperature
        OFFi  = OFFi  + 7 * (TEMP + 1500)^2
        SENSi = SENSi + 4 * (TEMP + 1500)^2
else:                                 # high temperature
    Ti    = 2 * dT^2 / 2^37
    OFFi  = 1 * (TEMP - 2000)^2 / 2^4
    SENSi = 0
OFF2  = OFF - OFFi
SENS2 = SENS - SENSi
TEMP2 = (TEMP - Ti) / 100                              # °C
P2    = (((D1 * SENS2) / 2^21 - OFF2) / 2^13) / 10     # mbar
```

**Bar02 (MS5837-02BA)** [11]:

```
if TEMP < 2000:                       # low temperature only
    Ti    = 11 * dT^2 / 2^35
    OFFi  = 31 * (TEMP - 2000)^2 / 2^3
    SENSi = 63 * (TEMP - 2000)^2 / 2^5
else:
    Ti = OFFi = SENSi = 0             # no correction at or above 20 °C
OFF2  = OFF - OFFi
SENS2 = SENS - SENSi
TEMP2 = (TEMP - Ti) / 100                              # °C
P2    = (((D1 * SENS2) / 2^21 - OFF2) / 2^15) / 100    # mbar
```

**Libraries agree with the datasheets.** Blue Robotics' Python and Arduino libraries use the same constants and branches as above [8][9]. The Bar30 formulas are also unchanged from TE's 2019 datasheet revision [12].

**TE's own generic C driver differs, so do not use it as a reference.** Its high-temperature offset term and its pressure scaling do not match its own datasheet [13].

### Depth equation

Both Blue Robotics libraries compute depth in meters as [8][9]:

```
depth_m = (P_Pa - 101300) / (fluid_density * 9.80665)
```

- **P_Pa** is the compensated pressure in pascals, which is mbar × 100.
- **101300 Pa** is a fixed surface pressure.
- **9.80665 m/s²** is standard gravity.

**Fluid density.**

| Library | Fresh water | Salt water | Default |
|---|---|---|---|
| Python [8] | `DENSITY_FRESHWATER = 997` kg/m³ | `DENSITY_SALTWATER = 1029` kg/m³ | Fresh water |
| Arduino [9] | 997 kg/m³ (header comment) | 1029 kg/m³ | Salt water |

The two Blue Robotics libraries therefore default differently. Blue Robotics' depth specifications instead assume 1 bar atmospheric pressure and 1000 kg/m³ [1].

**Surface reference.** The libraries use the fixed 101300 Pa. The Arduino source comment says: "In order to calculate the correct depth, the actual atmospheric pressure should be measured once in air, and that value should subtracted for subsequent depth calculations." [9]

## Identification and Detection

**Probe.** An ACK at 0x76 followed by a PROM read with a valid CRC-4 shows that a compatible pressure sensor is present [10][11]. The CRC proves the data is intact, not which part it is. Blue Robotics' Arduino library notes that "The MS5637 has the same address as the MS5837 and will also pass the CRC check" [9]. The [MS8607 document](../tbd/adafruit_ms8607_pht_sensor.md) describes another 0x76 part in this kit.

**PROM word 0 does not identify the model.** Bits 11:5 hold a product-version code, but it is 0000000 on both the 30BA01 and the 02BA01 variants [10][11]. A TE statement relayed in a third-party ArduPilot discussion: "Register 0 of PROM is used to differentiate different versions from a same pressure range. Not for differentiating pressure ranges (02bar VS 30bar)." [15]

**What the libraries actually do: guess from C1.** Blue Robotics' Python library [8]:

| C1 (pressure sensitivity) | Model chosen |
|---|---|
| 26000 to 37000 | Bar30 |
| above 37000, up to 49000 | Bar02 |
| anything else | Unknown |

The Arduino library uses the same thresholds [9].

**The guess can be wrong.** TE production data relayed in the ArduPilot discussion shows 30BA01 C1 values from 27035 to 41918, and 02BA01 values from 40238 to 45033 [15]. A Bar30 with C1 above 37000 would therefore be classified as a Bar02 by these libraries.

**Plausibility check.** This check is derived from the datasheet test vectors [10][11]. The wrong model's formula gives readings off by about 20×:

- The Bar02 example through Bar30 math gives about 22000 mbar.
- The Bar30 example through Bar02 math gives about 200 mbar.

So a reading in air near 1013 mbar confirms the configured model. A driver should take the model from configuration and check it this way at startup (Open Question 1).

## Startup and Initialization

1. **Power up.** The supply must ramp continuously. A power-on reset needs VDD below 0.1 V for at least 200 ms [10][11].
2. **Reset.** Send Reset (0x1E). It "shall be sent once after power-on to make sure that the calibration PROM gets loaded into the internal register" [10][11]. The datasheets give no reset duration. Blue Robotics' libraries wait 10 ms [8][9].
3. **Read and check the PROM.** Read words 0 to 6 (0xA0 to 0xAC) and check the CRC-4 [10][11]. If the CRC fails, the Python library returns False and prints "PROM read error, CRC failed!" [8].
4. **Set the model.** From configuration (see Identification and Detection).
5. **Measure each cycle:**
   1. Convert D1 at the chosen OSR, wait the maximum conversion time, then ADC read.
   2. Convert D2, wait, then ADC read.
   3. Compute first-order, then second-order values.

**SDA stuck low.** If the sensor does not respond after power-on, it may be holding SDA in an acknowledge state. "The only way to get the MS5837 to function is to send several SCLs followed by a reset sequence or to repeat power on reset." [10][11]

## Data and Commands

| Output | Units | Range | Resolution | Source |
|---|---|---|---|---|
| Pressure (Bar30) | mbar | 0 to 30 bar | 0.2 mbar RMS at OSR 8192 | [1][10] |
| Pressure (Bar02) | mbar | 0.3 to 1.2 bar; 0.01 to 2 bar extended | 0.016 mbar RMS at OSR 8192 | [1][11] |
| Temperature | °C | In water 2 to 40 °C (sensor rated −20 to 85 °C in air) | 0.0022 °C (Bar30) / 0.002 °C (Bar02) RMS at OSR 8192 | [1][10][11] |
| Depth (computed) | m | 0 to 295.6 m (Bar30) / 0 to 10 m (Bar02) | Bar30 2 mm; Bar02 0.16 mm of fresh water | [1][8] |

**Maximum rate.** One pressure plus one temperature conversion at OSR 8192 takes at least 18.08 + 18.08 ms on the Bar30 [10]. That is about 27 complete readings per second before I2C overhead. At OSR 256 the conversions take about 1.2 ms. These figures are derived from the conversion-time table.

**Temperature.** The temperature output is the sensor chip's own temperature [10]. Blue Robotics staff note that the daily drying requirement below "does not apply to the temperature accuracy" [16].

## Configuration and Calibration

**Chip settings.** The sensor has no configuration registers. The OSR is chosen in each conversion command [10][11]. Calibration is factory-programmed in PROM and cannot be changed [10][11].

**Driver settings:**

- **Model:** Bar30 or Bar02.
- **OSR:** 256 to 8192. The Python library defaults to 8192 [8]; the Arduino library is fixed at 8192 [9].
- **Fluid density:** for example 997 or 1029 kg/m³ [8].
- **Surface reference pressure** (see below).

**Self-heating at high OSR.** ArduPilot (third-party) uses OSR 1024 "to reduce the self-heating effect of the sensor", citing advice from the chip maker that some sensors are sensitive to it [14].

**Zero depth.** Blue Robotics' library comment recommends measuring atmospheric pressure once in air and subtracting it [9]. For a vehicle, the driver should capture a surface reading at startup or on command, and use it in place of the fixed 101300 Pa (Open Question 3).

**Bar02 accuracy depends on zeroing.** Its accuracy specification assumes "autozero at one pressure point" [11].

## Failure Modes and Safety

- **Gel and submersion limits.**
  - "These sensors use gel-covered sensing elements that must be dried for at least 2 hours each day to maintain accuracy." [1]
  - "Do not leave the sensor submerged for more than 24 hours. Prolonged submersion will cause readings to drift and may permanently damage the sensor." [2]
  - TE warns: "cleaning might damage the sensor." [10][11]
- **Over-pressure.** Pressure beyond the maximum mechanical pressure (50 bar Bar30, 10 bar Bar02) can cause permanent damage [1]. A Bar02 taken deeper than about 10 m is out of its operating range [1].
- **5 V logic damages the sensor** [2].
- **Address conflict.** Only one 0x76 device can be on a bus [2]. The Adafruit MS8607 in this kit also uses 0x76 for its pressure die [17], so the two cannot share a bus. The fix is a separate bus or a multiplexer [2][18]. Adafruit's TCA9548A multiplexer sits at an address from 0x70 to 0x77 [18], so it must not be set to 0x76.
- **Wrong model.** Wrong-model math gives readings off by about 20× (see Identification and Detection). Also, the Python library's `read()` raises an error if the model is unknown [8].
- **Bad reads.** An ADC result of 0 means the read was early, repeated, or overlapped a conversion. Discard it [10][11].
- **Over-range output.** The datasheets give calculation limits (Bar30 0 to 30 bar; Bar02 10 to 2000 mbar) but do not say what the output does beyond them [10][11].

## Software and Libraries

| Name | Use | Language | License | Link |
|---|---|---|---|---|
| ms5837-python | Blue Robotics' Python library: `init()`, `read(oversampling)`, `pressure()`, `temperature()`, `depth()`, `setFluidDensity()`, model auto-detect; uses smbus2 | Python | MIT | [8] |
| BlueRobotics_MS5837_Library | Blue Robotics' Arduino library; fixed OSR 8192; default density 1029 | C++ (Arduino) | MIT | [9] |
| TE MS5837_Generic_C_Driver | TE's reference C driver; Bar30 math only, with discrepancies against the datasheet | C | MIT | [13] |
| ArduPilot AP_Baro_MS5611 | THIRD-PARTY flight-controller driver supporting both models | C++ | Not checked | [14] |

## Related Parts

- [Adafruit MS8607 PHT sensor](../tbd/adafruit_ms8607_pht_sensor.md): also uses I2C address 0x76 [17], so it must be on a different bus or multiplexer channel.
- [Adafruit BNO085 IMU](adafruit_bno085_imu.md): another I2C sensor that may share a bus; read that document before putting it behind a multiplexer.
- [PJRC Teensy 4.1](../tbd/pjrc_teensy_4_1.md): a possible host; see that document for its I2C ports.
- [Raspberry Pi 5](../tbd/raspberry_pi_5.md): a possible host, with several I2C controllers that can be enabled.

## Open Questions

1. **Which model was bought, Bar30 or Bar02?**
   - Why it matters: the conversion math, output units, range and resolution all differ [10][11]. The libraries' C1-based guess can misclassify a Bar30 [15].
   - How to get it:
     - Check the purchase record, or the part label (BAR30 or BAR02) [1].
     - Confirm with the plausibility check: in air, the configured model's math should read about 1000 mbar.
     - On the host, `i2cdetect -y <bus>` should show 0x76.
2. **Which water density to use?**
   - Why it matters: depth is inversely proportional to density. Fresh (997) vs salt (1029) water differ by about 3% [8].
   - How to get it: the operating environment (pool, lake, sea). Make it a driver setting, not a constant.
3. **How is zero depth set?**
   - Why it matters: the libraries' fixed 101300 Pa ignores weather and altitude [9]. The sensor's mounting height on the vehicle also offsets depth.
   - How to get it: decide whether the driver captures a surface reading at power-up, on an operator command, or both. Record the sensor's vertical offset from the vehicle's reference point.
4. **Which I2C bus, and how is the 0x76 conflict resolved?**
   - Why it matters: it cannot share a bus with the MS8607 [2][17].
   - How to get it: a wiring decision (separate host buses, or a multiplexer channel) recorded in this repo.
5. **Does the R2 board have I2C pull-ups?**
   - Why it matters: the 2016 schematic shows none [6], so the host must provide them. The host's own pull-ups may or may not be enough.
   - How to get it: with power off, measure the resistance from SDA and from SCL to Vin on the sensor connector.
6. **Operating duty cycle vs the 24-hour submersion limit.**
   - Why it matters: readings drift and the sensor can be damaged if the vehicle stays submerged too long without drying [1][2].
   - How to get it: the operations plan.
7. **Oversampling and read rate.**
   - Why it matters: it trades resolution for speed and self-heating [10][14].
   - How to get it: a team decision. Start at OSR 4096 or 8192, then measure the reading noise in still water.

## Sources

1. "Bar High-Resolution Depth/Pressure Sensors", Blue Robotics. https://bluerobotics.com/store/sensors-cameras/sensors/bar-depth-pressure-sensor/. Used for: model specifications, pinout, logic and supply ranges, revision history, sealing note, gel and drying warnings, density assumption, library links.
2. "Bar High-Resolution Depth/Pressure Sensors Guide", Blue Robotics. https://bluerobotics.com/learn/bar-sensors-guide/. Used for: 3.3 V logic and 5 V damage, fixed address and multiplexer advice, wiring length, 24-hour submersion limit, mounting.
3. "PCB for Bar Depth/Pressure Sensors", Blue Robotics. https://bluerobotics.com/store/sensors-cameras/sensors/pcb-for-bar-depth-pressure-sensors/. Used for: schematic applies to both models.
4. "Bar dimension drawing" (PNG), Blue Robotics. https://bluerobotics.com/wp-content/uploads/2016/01/BAR-VP-drawing.png. Used for: dimensions, cable length, thread.
5. "Bar JST GH pinout" (PNG), Blue Robotics. https://bluerobotics.com/wp-content/uploads/2016/01/bar-i2c-jst-gh-pinout.png. Used for: pin numbers, colours, voltage labels.
6. "BAR30-SENSOR schematic" (2016), Blue Robotics (GitHub). https://raw.githubusercontent.com/bluerobotics/Bar30-Pressure-Sensor/master/BAR30-SENSOR-Schematic.pdf. Used for: regulator, diode, no level shifter or pull-ups drawn.
7. "Bar30-Pressure-Sensor README" (R1), Blue Robotics (GitHub). https://raw.githubusercontent.com/bluerobotics/Bar30-Pressure-Sensor/master/README.md. Used for: reverse-polarity protection.
8. "ms5837-python", Blue Robotics (GitHub). https://github.com/bluerobotics/ms5837-python. Used for: Python API, model constants, C1 thresholds, density constants and default, depth equation, OSR delays, CRC code, MIT license.
9. "BlueRobotics_MS5837_Library", Blue Robotics (GitHub). https://github.com/bluerobotics/BlueRobotics_MS5837_Library. Used for: Arduino defaults (1029 density, OSR 8192), atmospheric-reference comment, MS5637 CRC note, thresholds, MIT license.
10. "MS5837-30BA" datasheet REV C6 02/2025, TE Connectivity. https://www.te.com/commerce/DocumentDelivery/DDEController?Action=showdoc&DocId=Data+Sheet%7FMS5837-30BA%7FB1%7Fpdf%7FEnglish%7FENG_DS_MS5837-30BA_B1.pdf%7FCAT-BLPS0017. Used for: Bar30 commands, timing, resolution, PROM, CRC-4, first- and second-order math, test vector, electrical limits, read rules, reset and recovery.
11. "MS5837-02BA" datasheet REV A12 09/2026, TE Connectivity. https://www.te.com/commerce/DocumentDelivery/DDEController?Action=srchrtrv&DocNm=MS5837-02BA01&DocType=Data+Sheet&DocLang=English&DocFormat=pdf&PartCntxt=CAT-BLPS0059. Used for: Bar02 commands, timing, resolution, PROM, CRC-4, first- and second-order math, test vector, autozero note.
12. "MS5837-30BA(26)" datasheet REV C2 12/2019, TE Connectivity (hosted by Farnell). https://www.farnell.com/datasheets/2917217.pdf. Used for: confirming identical Bar30 math in an older revision.
13. "MS5837_Generic_C_Driver" source (ms5837.c), TE Connectivity (GitHub). https://github.com/TEConnectivity/MS5837_Generic_C_Driver. Used for: discrepancies against the datasheet, MIT license.
14. "AP_Baro_MS5611.cpp", ArduPilot (THIRD-PARTY, GitHub). https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Baro/AP_Baro_MS5611.cpp. Used for: OSR 1024 self-heating note, support for both models.
15. "AP_Baro: Support MS5837-02BA" (PR #29122), ArduPilot (THIRD-PARTY, GitHub). https://github.com/ArduPilot/ardupilot/pull/29122. Used for: relayed TE statement that word 0 does not distinguish ranges, TE production C1 statistics.
16. "BAR30 sensor drifting when being submersed continuously", Blue Robotics Forum (Blue Robotics staff). https://discuss.bluerobotics.com/t/bar30-sensor-drifting-when-being-submersed-continuously-will-bar02-and-celcius-sensor-work/7483. Used for: drying requirement does not apply to temperature accuracy.
17. "Adafruit TE MS8607 PHT Sensor" (Learn guide), Adafruit. https://learn.adafruit.com/adafruit-te-ms8607-pht-sensor. Used for: MS8607 pressure sensor at I2C address 0x76.
18. "Adafruit TCA9548A 1-to-8 I2C Multiplexer Breakout: Overview", Adafruit. https://learn.adafruit.com/adafruit-tca9548a-1-to-8-i2c-multiplexer-breakout/overview. Used for: same-address devices need a multiplexer; multiplexer address range 0x70 to 0x77.
