# Safety: What Our Code Enforces and What Stays Yours

2026-11-02, from the NEPI team. You named current caps and a stable deadband as reasons to do the last steps yourselves, and both are yours to set. This file gives you what we know to set them: what our code guarantees, what it cannot do, the ten places in rov-26 that matter for safety, and the deadband arithmetic.

## Summary

- Our code can only change the controller inputs your Pi script encodes, and only after the pilot presses RB. It caps every axis on the ROV Pi, hands control back the moment the pilot pushes a stick or trigger past a quarter of its travel, and goes neutral or ends NEPI control when demands stop.
- It cannot stop a thruster. The Teensy acts on the last frame it received until a new one arrives. Current limiting, the ESC deadband, watchdogs on the Teensy, and the supply switch are all yours.
- Several rov-26 behaviors change what NEPI's demands do on the ROV. The worst, finding 1, turns any LT press into a full-rate yaw request. Check each against your current code before the first powered test.

Citations of your code point to github.com/CoralCrusaders/rov-26 at commit `1af4984`. Read each one as "rov-26 does this; check whether your current code still does".

## Terms

- **NEPI**: the edge-computing software on your NEPI computer. A NEPI **driver** connects one device. Ours for the ROV is an **RBX** (robot) driver; **IDX** drivers handle cameras. **Discovery** is the driver step that finds a device and starts its node. **NavPose** is NEPI's navigation and pose data. The **RUI** is NEPI's browser interface.
- **nepi_link, NEPI control, pilot mode, takeover point, status, demand**: as defined in [SYSTEM_GUIDE.md](SYSTEM_GUIDE.md#terms).
- **Cap**: the largest demand nepi_link passes on for one axis, as a fraction of full stick or trigger travel.
- **ESC deadband**: the pulse range around 1500 µs where the ESC does not turn the motor. For the Blue Robotics Basic ESC it is 1475 to 1525 µs ([Docs/Motors/bluerobotics_basic_esc.md](../Docs/Motors/bluerobotics_basic_esc.md)).
- **Watchdog**: code that acts when expected data stops arriving.
- **Current limit**: code that scales thruster power down so the estimated total current stays under a budget.

## What Our Code Enforces

The numbers live in [SYSTEM_GUIDE.md](SYSTEM_GUIDE.md#control-modes-and-handover-rules); this table says where each rule is enforced.

| # | Rule | Where it is enforced |
|---|---|---|
| 1 | **Caps.** Every demand is limited to its `NEPI_MAX_*` key on the ROV Pi, after NEPI and before your sketch. Nothing on NEPI can exceed it. 0 turns an axis off. | `clampDemand` in `NepiLink.mergeLocked`; range checked in `validate_config` ([nepi_link.py](../Nepi_Connect/pi/nepi_link.py)) |
| 2 | **Takeover.** Any of LY, LX, RY, LT or RT at or past the takeover point ends NEPI control on that pass, and the pilot's input goes through at once. | `takeoverInputLocked` and `updatePilotLocked`; `NEPI_TAKEOVER_FRACTION` |
| 3 | **RB hands over only when it is safe to.** RB starts NEPI control only while NEPI is ready and no movement input is past the takeover point. NEPI can never start it: `active` true only marks NEPI ready. | `toggleAssistLocked`, `nepiReadyLocked`, `handleDemandLocked` |
| 4 | **Stale demands go neutral.** A demand older than 0.25 s leaves every NEPI-owned input at 0. | `demandFreshLocked`, `DEMAND_STALE_SEC` |
| 5 | **Silence ends NEPI control.** 2 s without an accepted demand hands the ROV back. | `checkTimeoutLocked`, `ASSIST_TIMEOUT_SEC` |
| 6 | **NEPI stops sending when it stops hearing.** The driver sends no demand once 0.5 s pass without a valid status, so rules 4 and 5 then act on the ROV Pi. | `MateRovLink.linkUpLocked`, `nextDemandLocked`, `LINK_TIMEOUT_SEC` ([rbx_mate_rov_link_node.py](../Drivers/rbx_drivers/rbx_mate_rov_link_node.py)) |
| 7 | **Neutral on no command.** With no teleop command newer than 0.5 s, or after Stop, the demand is 0 on every axis, and the Teensy's own holds act. A handover always starts from neutral. | `nextDemandLocked`, `TELEOP_CMD_TIMEOUT_SEC`, `go_stop` |
| 8 | **Teleop only during NEPI control,** and only while `teleop_movement_enabled` is `TRUE` | `MateRovLinkNode.teleopControlsReady`, `MateRovLink.is_nepi_control` |
| 9 | **NEPI can hand back but never take.** Set State PILOT and a driver shutdown each send one `active` false demand, which ends NEPI control at once. | `setStateInd`, `release_to_pilot`, `cleanup_actions` |
| 10 | **Servos only in NEPI control, the pilot wins per input, and neutral after a handback.** Not used in the team config. | `mergeLocked` servo branch; `MateRovLink.set_servo`; `nextDemandLocked` drops the values |
| 11 | **Only NEPI's address is obeyed.** Demands from any other address, malformed demands and out-of-order demands are dropped and counted (`demand_bad`). | `parse_demand`, `handleDemandLocked` |
| 12 | **nepi_link never stops your script.** Every public function catches every error and passes the pilot's inputs through. An empty `NEPI_LAN_IP` or an invalid config value turns the link off entirely. | `merge_inputs` and the other module functions; `validate_config` |
| 13 | **The pilot always sees the state.** The topside tag reads `NEPI:ON`, `WAIT`, `OFF` or `NOTREADY`. | `topside_tag` |

## What Our Code Cannot Do

- **It cannot stop a thruster.** It only changes the tuple your script encodes. A frame already sent stays in effect on the Teensy until the next one. If your Pi script, the ROV Pi or the USB link stops, nothing of ours reaches the Teensy (finding 4).
- **Its timeouts act only when your main loop runs.** Rules 4 and 5 change what `merge_inputs` returns on its next call. A main loop that blocks also freezes the last frame, NEPI's demand included ([PI_INTEGRATION.md](PI_INTEGRATION.md#what-nepi_link-assumes-about-your-code), row 10).
- **A cap is a fraction of stick travel, not of thrust or current.** What it means in thrust depends on your sketch's gains, slow mode and current limiter (below).
- **Neutral is only as steady as your holds.** In NEPI control with no command, your depth and heading holds keep the ROV in place. Findings 3 and 6 can make them misbehave.
- **The link has no password.** nepi_link obeys any datagram from NEPI's address. Keep your switch closed: no guest devices and no Wi-Fi bridge onto it.

## What Stays Yours

| Item | Where it lives | Notes |
|---|---|---|
| Current limits | Your sketch: `AMP_LIMIT`, `HOR_AMP_LIMIT`, `VERT_AMP_LIMIT`, `limit_current()` (`rov-26 new_ctl.ino:16-18, 521-550`) | NEPI's caps sit on top of these and never replace them. Finding 2. |
| ESC deadband | The ESCs; your sketch's pulse mapping (`THRUSTER_POWER`, `new_ctl.ino:33`) | Decides the smallest demand that does anything (below) |
| Kill switch and supply | Your ROV's power system | On our bench, the supply switch was the only stop that worked in every case. Ctrl+C on the scripts does not stop a thruster. |
| Firmware watchdogs | Your sketch | rov-26 has none for link loss or an IMU reset (findings 3 and 4) |
| Thruster pin map, reversals, signs | Your sketch and wiring | We never saw your pin map; the `NEPI_*_SIGN` keys follow from it ([TEST_PLAN.md](TEST_PLAN.md#measuring-the-signs-and-the-imu-mount)) |
| Depth hold and heading hold | Your sketch | NEPI relies on them whenever its demand is 0 |
| IMU orientation in the firmware | Your sketch | The driver's `imu_mount` fixes only what NEPI sees (finding 6) |
| The caps themselves | `NEPI_MAX_*` in your `nepi_link.yaml` | Ours are starting values only |

## Choosing the Caps

A cap of 0.2 lets NEPI move a stick to at most 8713 of 32767, which is 27% of full travel. A yaw cap of 0.1 lets it pull a trigger to at most 129 of 1023 ([SYSTEM_GUIDE.md](SYSTEM_GUIDE.md#demand-to-stick-mapping)). In rov-26, that becomes:

| Axis at its default cap | rov-26 result | Source |
|---|---|---|
| Surge 0.2 | 53 µs from 1500 on each horizontal thruster, 26 in slow mode | `new_ctl.ino:465-470` |
| Sway 0.2 | 42 µs, 21 in slow mode | `new_ctl.ino:472-477` |
| Heave 0.2 | A depth-rate request of 0.27 m/s | `new_ctl.ino:426-427` |
| Yaw 0.1 | A yaw-rate request of 22.7 deg/s, or the full rate with finding 1 | `new_ctl.ino:481-482` |
| All four at once | At most about 1641 µs on one thruster, about 1.4 A, computed in air. With finding 1, the yaw term saturates instead and the current limiter holds every horizontal near 1617 or 1383 µs. | Our simulation of `new_ctl.ino:411-550` |

- A cap below an axis's smallest useful demand (next section) makes that axis do nothing.
- Raise caps one step at a time, and only after the axis has passed its tests at the lower cap.
- Until you have checked finding 1, set `NEPI_MAX_YAW: 0`.

## The ESC Deadband and the Smallest Useful Demand

For an open-loop axis, the smallest demand that turns a thruster at all follows from four facts. To redo it for your current sketch, put your own constants in.

1. The pulse is 1500 µs plus the thruster's power value (`THRUSTER_POWER`, `rov-26 new_ctl.ino:33`). The Basic ESC does not turn inside 1475 to 1525 µs, so a thruster turns only when the power's magnitude is at least 26. On our bench, a power of 34 crept and 17 did not turn.
2. A stick's power on each horizontal thruster is `int(stick / 32767 x gain x mult x 400)` (`new_ctl.ino:465-477, 507-510`):
   - gain is `FOR_GAIN` 0.5 for surge and `LAT_GAIN` 0.4 for sway (`:35-36`);
   - mult is 1.0, or 0.5 in slow mode (`:39-40`);
   - 400 is `ESC_MAGNITUDE` (`:26`);
   - the cast truncates toward 0.
3. So the stick must reach 26 x 32767 / (400 x gain x mult), rounded up.
4. nepi_link's `stick_value` gives that stick for a demand of 0.01 + 0.99 x (stick - 3000.5) / 29767 or more.

| Axis | Mode | Stick needed | Smallest useful demand | Default cap |
|---|---|---|---|---|
| Surge | Normal | 4260 | 0.052 | 0.2 |
| Surge | Slow | 8520 | 0.194 | 0.2 |
| Sway | Normal | 5325 | 0.087 | 0.2 |
| Sway | Slow | 10650 | 0.265 | 0.2, so NEPI has no sway in slow mode |

Heave and yaw are closed loops in rov-26: the stick or trigger asks for a rate, and the Teensy sets power from the error.

- **Heave.** In air, where the measured depth rate is 0, the vertical power is 400 x the requested rate. So the smallest nonzero heave demand (0.01, RY 3000) already gives 36 µs.
- **Yaw.** In air, the horizontal power is 400 x 0.005 x the requested rate (`yaw_pwr_controller`, `new_ctl.ino:207`). So it needs a request of 13 deg/s: LT 74 of 1023, a yaw demand of about 0.044.
- **In water** both depend on how the ROV moves, so measure them ([TEST_PLAN.md](TEST_PLAN.md#stage-3-supply-on-thrusters-in-air-low-caps)).

## rov-26 Findings

Each finding is in rov-26 at `1af4984`. For each one, check whether your current code still has it.

1. **Any LT press asks for the full yaw rate.**
   - **Where.** `NORMALIZE_TRIGGER(x)` does not parenthesize `x` (`new_ctl.ino:32`, `tuning.ino:32`). So `NORMALIZE_TRIGGER(input_data.ABS_LT - input_data.ABS_RT)` (`new_ctl.ino:482`, `tuning.ino:499`) expands to `LT - RT / TRIGER_MAGNITUDE`.
   - **Effect.** RT alone works as intended. LT alone asks for LT x 180 deg/s. On our bench, LT 129 gave a yaw target of 23220.
   - **With NEPI.** This affects the pilot's own LT, and every positive NEPI yaw, which nepi_link sends on LT. The yaw then runs at whatever the current limiter allows.
   - **Check.** Pull LT a little and read `$TNCTL` column 19 (`yaw_rate_target_deg_s`). Thousands means the bug is there.
   - **Fix.** `#define NORMALIZE_TRIGGER(x) ((double)(x) / (double)(TRIGER_MAGNITUDE))`. Until it is fixed, set `NEPI_MAX_YAW: 0`.
2. **Thrust is not monotonic in stick travel, and combined inputs can reach about 17 A.**
   - **Where.** `limit_current()` scales pulses linearly against the summed current estimate of a whole group (`new_ctl.ino:513-550`), using a table that is far from linear (`AMP_LIST`, `:21`).
   - **RY alone,** computed in air: 60% stick gives about 1730 µs (4.8 A each), but full stick only about 1618 µs (0.8 A each).
   - **Combined.** LY, LX and a trigger together can drive one horizontal thruster to about 1890 µs (about 17 A). The normalization allows each thruster up to 400 µs, and the 20 A estimate allows it too. Slow mode does not scale the yaw term.
   - **The fuse.** Your fuse comment says 25 A (`:16`).
3. **A BNO085 reset hangs the loop.**
   - **Where.** `read_imu()` loops until it has both a gyro and a game-rotation event (`new_ctl.ino:381-396`). Neither sketch calls `wasReset()` or re-enables the reports.
   - **Effect.** If the IMU resets, `loop()` never returns. No new frame is read, `$TNCTL` stops, and the thrusters hold their last pulse.
   - **With NEPI.** Its `tn_age_ms` keeps growing and its NavPose goes stale.
   - **Fix.** A bounded wait, plus `wasReset()` and re-enabling the reports.
4. **Nothing stops a thruster when the link drops.**
   - **Where.** Your Pi script resends its last frame every 0.1 s whatever happens upstream (`rpi/modes/main.py:84-88`). The Teensy keeps its last inputs until a new frame arrives (`new_ctl.ino:325-362`).
   - **Effect.** Ctrl+C on the Pi script does not stop a thruster; the supply switch does. On our bench, Ctrl+C left the Teensy on its last command.
   - **With NEPI.** If the Pi script or the USB link stops during NEPI control, the last frame keeps NEPI's last demand in effect.
   - **Related.** The frame is copied into `input_data` before its terminator and checksum are checked (`new_ctl.ino:347` vs `:348-360`), so a frame that fails them is still used.
   - **Fix.** A Teensy-side timeout that zeroes the movement inputs after, say, 0.5 s with no valid frame. Read into a buffer and copy only after the checks pass.
5. **rov-26's `tuning.ino` and its Pi scripts do not match.**
   - **Where.** `tuning.ino` reads a 313-byte frame: 20 PID gains as doubles, then the 18 inputs as 64-bit integers (`tuning.ino:127-166`). Both Pi scripts send 154 bytes, because the encoder takes exactly 18 values (`rpi/modes/nmea_encode_c_ext/nmea_encode.c:8`).
   - **Effect.** The sketch uses the bytes before checking them (`tuning.ino:364` vs `:365-377`), and loads gains from them on every loop (`:659-682`).
   - **Rule.** Never run that pairing with `power_thrusters()` enabled.
   - **With NEPI.** If your current Pi script sends gains as well, merge only the 18 inputs ([PI_INTEGRATION.md](PI_INTEGRATION.md#what-nepi_link-assumes-about-your-code), row 11).
6. **With the IMU on its side, heading is unreliable.**
   - **Where.** The sketch's quaternion conversion assumes a flat board (`new_ctl.ino:279-291`). On its side, pitch reads near ±90°, where yaw and roll stop being separate angles.
   - **Our simulation.** Heading held still: with the ROV level and ±0.5° of wobble, the reported yaw was off by a median of 93°. With a steady 3° trim and ±1° of wobble, it was off by a median of 10° and up to 26°.
   - **Effect.** Whenever no trigger is pressed (the pilot's or NEPI's yaw at 0), the heading hold acts on that yaw (`:484-491`), so a jumping yaw becomes a yaw command.
   - **With NEPI.** The driver's `imu_mount` corrects what NEPI sees ([NEPI_SETUP.md](NEPI_SETUP.md#choosing-imu_mount)), but not what your Teensy uses.
   - **Fix,** in your firmware. Rotate the quaternion by the mount before the conversion. Or store the mount in the BNO085 (`sh2_setReorientation()`, FRS record 0x2D3E), which rotates the gyro axes too, so the yaw rate moves to `gyroscope.z`.
   - **Also check** that the hold corrects rather than runs away. With target 0, `yaw_rate_controller.compute(shortest_angle(yaw, yaw_target))` asks for a rate of yaw minus target (`:298-304, 491`), and which way that turns depends on the mount.
7. **The Teensy will not start without both sensors.** Both sketches loop in `setup()` until a Bar02 and a BNO085 answer (`new_ctl.ino:227-256`, `tuning.ino:244-273`). The thruster pins are attached only after that (`new_ctl.ino:268-275`). The "Failed to initialize" lines printed meanwhile count as `tn_bad`.
8. **The repo's depth-sensor library does not compile, and the replacement is slow.**
   - **Where.** `rov-26/MS5837/MS5837.h` is the newer Blue Robotics version (it declares `init(TwoWire &wirePort = Wire)`, `begin()` and `MS5837_UNRECOGNISED`), while `MS5837.cpp` is the older one.
   - **The replacement.** The Arduino library manager's "BlueRobotics MS5837 Library" compiles. With it, our bench saw one `$TNCTL` line every 2.27 s.
   - **What NEPI needs.** By our estimate, NEPI stabilization needs at least 20 new readings a second, with a 95th-percentile age of 100 ms or less ([DRIVER_HANDOFF.md](../Nepi_Connect/DRIVER_HANDOFF.md) "Targets"). Rows 1, 2 and 6 of its "Problems to fix" are firmware suggestions for you.
9. **A telemetry line must hold exactly 20 values.** nepi_link drops any `$TNCTL` line that is not 20 values plus a `*XX` field (`parse_teensy_line`).
   - rov-26's `tuning.ino` already prints 25 (it adds four controller values and `millis()`, `tuning.ino:601-656`), so its lines would all be dropped.
   - If your current sketch prints more, for example PHT values, `TN_NAMES` in `nepi_link.py` and `TN_VALUES` in the driver node must change together. Send us the column order and we will make that change.
10. **The topside script accepts one connection only.** `topside/modes/main.py:20` calls `accept()` once, so after restarting the Pi script (for example after editing `nepi_link.yaml`), restart the topside script first.

## Rules for the First Powered Tests

1. One person holds the supply switch, and nobody touches a propeller while the supply is on.
2. Run every stage of [TEST_PLAN.md](TEST_PLAN.md) in order. Do not supply the ESCs until stages 1 and 2 pass.
3. Start with `NEPI_MAX_YAW: 0` and the other caps at 0.1 or lower. Raise one cap at a time.
4. Test one axis at a time, at the smallest useful demand first.
5. A T200 must not run more than 10 s out of water ([Docs/Motors/bluerobotics_t200_thruster.md](../Docs/Motors/bluerobotics_t200_thruster.md)).
6. Before each powered run, check that RB, a stick past the takeover point, and stopping NEPI each hand the ROV back.
7. Shutdown order: supply off first, then the Pi script, then the topside script.
8. If anything moves that you did not expect, the supply switch comes first and questions second.
