# Test Plan: From the Desk to the Water

2026-11-02, from the NEPI team. Four stages, each with steps, the machine each command runs on, and what has to be true to move on. Run them in order. [SAFETY.md](SAFETY.md) has the rules for every powered step, and [SYSTEM_GUIDE.md](SYSTEM_GUIDE.md) the handover rules these tests check.

## Summary

| Stage | Hardware | What it proves |
|---|---|---|
| 1. Desk | Any computer; no Teensy, no ROV | Our code passes its own tests, your config is valid, and the handover rules behave as documented |
| 2. ESC supply off | Your ROV Pi script, the Teensy, and a fake NEPI | Your hooked script merges correctly with your real sketch, every exit works, and the stick signs |
| 3. Supply on, thrusters in air, low caps | Plus the ESC supply and the real NEPI driver | Thrusters respond the right way, the smallest useful demands, the IMU mount, and the NEPI exits |
| 4. In water, on the tether | The whole ROV | Signs and exits in the water, then a careful first hold |

What we have run, and what we have not, is in [README.md](README.md#status). The exit tests marked **new** below have not been run by us under the current RB-toggle design.

## Terms

- **NEPI**: the edge-computing software on your NEPI computer. A NEPI **driver** connects one device. Ours for the ROV is an **RBX** (robot) driver, started by its **discovery** step once it hears your ROV Pi. **IDX** drivers handle cameras. **NavPose** is NEPI's navigation and pose data. The **RUI** is NEPI's browser interface.
- **nepi_link, NEPI control, pilot mode, takeover point, status, demand**: as defined in [SYSTEM_GUIDE.md](SYSTEM_GUIDE.md#terms).
- **fake NEPI**: [fake_nepi.py](../Nepi_Connect/tools/fake_nepi.py), which stands in for NEPI ([PI_INTEGRATION.md](PI_INTEGRATION.md#testing-on-the-desk-with-fake_nepipy)).
- **`$TNCTL` column N**: the Nth value of a Teensy telemetry line. Columns 1 to 4 are the vertical thrusters and 5 to 8 the horizontal ones, in µs from 1500.

## Stage 1: Desk, No Teensy

**Needs.** Any computer with Python 3.8 or newer and a mate_robotics checkout. No ROV, Teensy or NEPI.

**1.1 Unit tests.** `-B` keeps Python from writing cache files into the repo.

```bash
# [topside] or any computer, from a mate_robotics checkout
cd Nepi_Connect/pi && python3 -B -m unittest -v test_nepi_link
cd ../tools && python3 -B -m unittest -v test_rbx_mate_rov_link
```

*Pass:* both end in `OK`, with the counts in [README.md](README.md#status).

**1.2 Your config.** Load the file through nepi_link's own loader and validator:

```bash
# [topside] from a mate_robotics checkout; use the path of the nepi_link.yaml you will put on the ROV Pi
cd Nepi_Connect/pi && python3 -B -c "import nepi_link as n; c, m = n.load_config_file('../../Handoff/nepi_link_team.yaml'); print(m); print(n.validate_config(c)[1])"
```

*Pass:* the second line reads `On: NEPI 192.168.68.103, status to UDP 5700, demands on UDP 5701, triggers 40 to 1023, takeover at stick 8192 and trigger 256, no servos`. Anything that ends in `link off` names the bad key.

**1.3 The handover rules, with nepi_link and the fake NEPI on one computer.** An interactive Python session stands in for your main loop. Both sides use 127.0.0.1, so nothing leaves the computer.

```bash
# [topside] terminal 1, from a mate_robotics checkout: a desk copy of the config that names this computer as NEPI
cd Nepi_Connect/pi && sed 's|^NEPI_LAN_IP: .*|NEPI_LAN_IP: 127.0.0.1|' ../../Handoff/nepi_link_team.yaml > /tmp/nepi_link_desk.yaml
NEPI_LINK_CONFIG=/tmp/nepi_link_desk.yaml python3 -B
```

```bash
# [topside] terminal 2, from a mate_robotics checkout: NEPI ready, asking for surge 0.1
python3 Nepi_Connect/tools/fake_nepi.py --pi 127.0.0.1 hold --surge 0.1 --seconds 600
```

In terminal 1, type these lines one at a time. `p` is the 18 pilot inputs. Slot 1 is LY, 13 is RB and 14 is LT.

```python
import nepi_link
nepi_link.start()                      # prints "nepi_link: On: NEPI 127.0.0.1, ..."
p = [0] * 18
nepi_link.topside_tag()                # ',NEPI:OFF': NEPI ready, pilot mode
p[13] = 1; nepi_link.merge_inputs(p)   # press RB ...
p[13] = 0; nepi_link.merge_inputs(p)   # ... and release it: NEPI control on
nepi_link.topside_tag()                # ',NEPI:ON'
nepi_link.merge_inputs(p)[1]           # -5706: surge 0.1, NEPI_SURGE_SIGN -1
p[1] = -5000; nepi_link.merge_inputs(p)[1]   # -5706 still: under the takeover point, the pilot's LY is ignored
p[1] = -9000; nepi_link.merge_inputs(p)[1]   # -9000: past the takeover point, NEPI control ends at once
nepi_link.topside_tag()                # ',NEPI:OFF'
p[13] = 1; nepi_link.merge_inputs(p)   # RB while LY is held past the takeover point ...
p[13] = 0; nepi_link.merge_inputs(p)   # ... is ignored: prints "RB ignored: release LY first"
p[1] = 0
```

Then, still in terminal 1:

| Step | Do | Expect |
|---|---|---|
| a | RB press and release again | `,NEPI:ON`; terminal 2 prints `NEPI control on (pilot_button)` |
| b | RB press and release once more | `,NEPI:OFF`; terminal 2 prints `NEPI control off (pilot_button)` |
| c | RB again, then Ctrl+C in terminal 2 | Terminal 2 sends `active` false for 0.5 s; the tag goes `,NEPI:NOTREADY` and the reason is `nepi_off` |
| d | Restart terminal 2's command, RB again, then **Ctrl+\\** in terminal 2 | A fake NEPI restarted within 1 s has its first demands dropped as out of order (`Demand dropped: seq 0 is not after ...`), so wait a second before RB. Within 0.25 s of Ctrl+\\, `nepi_link.merge_inputs(p)[1]` is 0 and the tag is `,NEPI:WAIT`. After 2 s the tag is `,NEPI:NOTREADY`, NEPI control is off (`timeout`), and the pilot's inputs pass through. |
| e | RB with terminal 2 stopped | Nothing; prints `RB ignored: NEPI is not connected` |
| f | Terminal 2 with `--yaw 0.1`, RB, then `nepi_link.merge_inputs(p)[14]` | 129 on LT, and RT (slot 15) 0 |

nepi_link prints each kind of message at most once per 10 s, so a repeated "RB ignored" line can be held back. The tag and the merged values are the test.

*Pass:* every row as listed. This exercises the rules in [SYSTEM_GUIDE.md](SYSTEM_GUIDE.md#control-modes-and-handover-rules) with nothing that can move. Type `nepi_link.stop()` and `exit()` to finish.

## Stage 2: ESC Supply Off

**Needs.**
- Your ROV Pi with your current Pi script hooked ([PI_INTEGRATION.md](PI_INTEGRATION.md)).
- The Teensy running the sketch you fly, and the topside with your controller.
- **The ESC supply switched off**, so no thruster can move. Judge everything by the `$TNCTL` values and the status lines.

**Setup.** For this stage the fake NEPI runs on the topside, so point `NEPI_LAN_IP` at the topside. Remember to set it back.

```bash
# [ROV Pi] in your Pi script folder: name the topside as NEPI for this stage only
sed -i 's|^NEPI_LAN_IP: .*|NEPI_LAN_IP: 192.168.68.12|' nepi_link.yaml
```

1. Start the topside script first, then your Pi script ([SAFETY.md](SAFETY.md#rov-26-findings), finding 10).
2. Check that your Pi script prints `nepi_link: On: NEPI 192.168.68.12, ...` before its main loop.
3. On the topside, in a second terminal, start the fake NEPI:

```bash
# [topside] second terminal, from a mate_robotics checkout
python3 Nepi_Connect/tools/fake_nepi.py --pi 192.168.68.22 hold --surge 0.1 --seconds 600
```

**Steps.**

| # | Do | Expect | Tested by us |
|---|---|---|---|
| 2.1 | Nothing | The topside line ends in `,NEPI:OFF,*FF`. fake_nepi prints status lines with `ready yes`, and `tn` with your eight thruster values. | Yes |
| 2.2 | Fly the sticks as usual | Your `$TNCTL` values follow exactly as without nepi_link | Yes |
| 2.3 | RB | `assist ON pilot_button`, `out_ly -5706`, tag `,NEPI:ON`. Columns 5 to 8 show your sketch's response: -34 in rov-26 normal mode, -17 in slow mode, the same sign your pilot's forward stick gives. | Yes |
| 2.4 | **New:** push LY slowly past a quarter of its travel | NEPI control ends (`pilot_stick`) as LY passes 8192, and `out_ly` follows your stick at once | No |
| 2.5 | **New:** RB to hand over, then RB again | `pilot_button` both ways | No |
| 2.6 | **New:** hold LX past a quarter, press RB | Ignored; the ROV Pi prints `RB ignored: release LX first` | No |
| 2.7 | **New:** RB to hand over, then Ctrl+\\ on fake_nepi | `,NEPI:WAIT` and every movement output 0 within 0.25 s; `,NEPI:NOTREADY` and `timeout` after 2 s. Watch the reason with `fake_nepi.py --pi 192.168.68.22 watch`. | No |
| 2.8 | **New:** RB to hand over, then Ctrl+C on fake_nepi | NEPI control ends at once, as `nepi_off` | No |
| 2.9 | **New:** RB with fake_nepi stopped | Ignored; tag `,NEPI:NOTREADY` | No |
| 2.10 | **New:** send junk to the demand port from the topside: `echo junk \| nc -u -w1 192.168.68.22 5701` | `demand_bad` rises by 1 and nothing else changes (assumes `nc` is installed) | No |
| 2.11 | Stick signs: [Part A below](#part-a-signs-with-no-thrust) | Each `NEPI_*_SIGN` decided | Surge only |

*Pass:* every row as listed. RB and the takeover hand back within one serial frame (0.1 s).

**After the stage**, put your NEPI's address back:

```bash
# [ROV Pi] in your Pi script folder
sed -i 's|^NEPI_LAN_IP: .*|NEPI_LAN_IP: 192.168.68.103|' nepi_link.yaml
```

## Stage 3: Supply On, Thrusters in Air, Low Caps

**Needs.**
- Stage 2 passed.
- One person on the supply switch.
- Caps lowered: `NEPI_MAX_SURGE`, `NEPI_MAX_SWAY` and `NEPI_MAX_HEAVE` at 0.1, and `NEPI_MAX_YAW` at 0 until finding 1 is checked ([SAFETY.md](SAFETY.md#rules-for-the-first-powered-tests)).
- The ROV out of the water and held still, propellers clear. A T200 runs at most 10 s in air.

**3.1 With the fake NEPI** (`NEPI_LAN_IP` at the topside, as in stage 2):

| # | Do | Expect |
|---|---|---|
| 3.1a | `hold --surge 0.05`, RB, for up to 10 s; supply off | All four horizontal thrusters turn the way the pilot's "forward" turns them |
| 3.1b | The same with `--sway 0.1` and `--heave 0.05` | They match the pilot's "right" and "down" |
| 3.1c | Raise `--surge` in steps of 0.01 from 0.02 until a thruster turns | The smallest useful surge. Compare it with [SAFETY.md](SAFETY.md#the-esc-deadband-and-the-smallest-useful-demand); repeat in slow mode and for sway. |
| 3.1d | **New:** each of stage 2's exits (2.4 to 2.8) once, with thrust | The thrusters return to the pilot's command at once (RB, takeover), go neutral within 0.25 s, and stop being NEPI's after 2 s (timeout) |

**3.2 With the real NEPI** (`NEPI_LAN_IP` at 192.168.68.103, the driver installed and enabled, [NEPI_SETUP.md](NEPI_SETUP.md)):

| # | Do | Expect | Tested by us |
|---|---|---|---|
| 3.2a | Nothing | The topside tag reads `,NEPI:OFF`. The RUI Robots page shows `mate_rov_link_robot` in state `PILOT`, and NavPose depth changes with the sensor. | Depth: yes |
| 3.2b | RB, then RUI Teleop W for a moment | State `NEPI`; the horizontal thrusters push forward. Mind that E and D move left and counter-clockwise ([NEPI_SETUP.md](NEPI_SETUP.md#teleop-from-the-robots-page)). | Thruster moved: yes; direction: no |
| 3.2c | **New:** RUI Set State PILOT during NEPI control | NEPI control ends at once (`nepi_off`), and the state shows `PILOT` | No |
| 3.2d | **New:** disable the driver during NEPI control | NEPI control ends at once (`nepi_off`) or, at the latest, after 2 s (`timeout`) | No |
| 3.2e | **New:** stop your Pi script while watching `link_status` | `link_up` goes false within 0.5 s. Restart the topside script, then the Pi script: the link comes back without restarting NEPI. | No |
| 3.2f | **New:** start the driver before the Pi script | The driver starts once the Pi script sends status | No |
| 3.2g | IMU mount: [Part B below](#part-b-the-imu-mount) | `imu_mount` set | No |
| 3.2h | Heading hold: triggers released, turn the ROV a little by hand | `$TNCTL` column 19 opposes the turn (the hold corrects) and does not follow it (runs away) | No |
| 3.2i | 10 minutes idle with the link up | `status_bad` on `link_status` stays 0 | No |
| 3.2j | Telemetry rate: `sample_link_rates.py` (below) | Record the line interval and the 95th-percentile age; see what NEPI needs in [SYSTEM_GUIDE.md](SYSTEM_GUIDE.md#what-nepi-needs-from-teensy-telemetry) | Yes, on our bench sketch |

```bash
# [topside] copy the rate tool into the NEPI container
scp -P 2222 -i ~/.ssh/nepi_default_ssh_key Nepi_Connect/tools/sample_link_rates.py nepi@192.168.68.103:
```

```bash
# [NEPI] in a container shell where rostopic works; its built-in namespace is our bench device, so pass yours
python3 sample_link_rates.py /nepi/<device_id>/mate_rov_link_robot 60
```

*Pass:* every row as listed, with each sign confirmed by thrust direction.

## Stage 4: In Water, on the Tether

**Needs.**
- Stage 3 passed.
- The ROV in a pool on its tether, and the pilot's hands on the controller.
- Caps at 0.1 for surge, sway and heave, and 0.05 for yaw once finding 1 is fixed (0 until then).

| # | Do | Expect |
|---|---|---|
| 4.1 | Hover in pilot mode, then RB with no NEPI command | The ROV holds depth and heading on your own holds, as when the pilot lets go |
| 4.2 | RUI Teleop or `rostopic pub` on `set_teleop_velocity`, one axis at a time, small and brief | Each command moves the ROV the way the ROS axes say: `linear.x` + forward, `linear.y` + left, `linear.z` + up, `angular.z` + counter-clockwise seen from above ([SYSTEM_GUIDE.md](SYSTEM_GUIDE.md#the-chain)) |
| 4.3 | **New:** every exit from stage 3 once more, in water | The pilot gets the ROV back at once, and the ROV does not jump at the handover |
| 4.4 | A first stabilization process, if you have one, at the low caps for 60 s | Less drift than with NEPI control off, judged against a marker in the video. No oscillation that grows. |
| 4.5 | Raise one cap one step | Repeat 4.2 to 4.4 for that axis |

```bash
# [NEPI] a brief, small surge in NEPI control; Ctrl+C stops it, and the demand goes neutral 0.5 s later
rostopic pub -r 5 /nepi/<device_id>/mate_rov_link_robot/rbx/set_teleop_velocity geometry_msgs/Twist '{linear: {x: 0.1}}'
```

*Pass:* every row as listed. After that, how far to take the caps and how to tune a hold is your call.

## Measuring the Signs and the IMU Mount

### Part A: signs with no thrust

Your pilots already fly the ROV correctly, so each sign only has to make NEPI's positive demand produce the same input value as the pilot's matching stick direction. Use stage 2's setup, with `fake_nepi.py ... watch` in a second topside terminal to see `out_ly`, `out_lx`, `out_ry`, `out_lt` and `out_rt`.

| Key | Pilot input to read (pilot mode) | NEPI demand to compare (NEPI control) | Set the sign to |
|---|---|---|---|
| `NEPI_SURGE_SIGN` | LY pushed the way your pilots fly forward; note the sign of `out_ly` | `hold --surge 0.1` | Keep it if `out_ly` has the same sign as the pilot's; flip it if not |
| `NEPI_SWAY_SIGN` | LX pushed the way your pilots move right; `out_lx` | `hold --sway 0.1` | The same rule, on `out_lx` |
| `NEPI_HEAVE_SIGN` | RY pushed the way your pilots dive (go deeper); `out_ry` | `hold --heave 0.1` (heave is positive down) | The same rule, on `out_ry` |
| `NEPI_YAW_SIGN` | The trigger your pilots use to turn clockwise seen from above (turn right) | `hold --yaw 0.1` (positive is clockwise) | Keep it if NEPI's value appears on that same trigger (`out_lt` or `out_rt`); flip it if it appears on the other |

Change `nepi_link.yaml`, then restart the topside script and your Pi script. On our bench, rov-26 code with a stock Xbox pad gave `NEPI_SURGE_SIGN` -1: LY pushed forward reads negative. The other three we have not measured.

Part A trusts your pilots' own directions. Stage 3 confirms each sign with thrust, and stage 4 in the water.

### Part B: the IMU mount

1. Look at the BNO085 board in the ROV and note where its printed X and Y arrows point. Pick `imu_mount` from the table in [NEPI_SETUP.md](NEPI_SETUP.md#choosing-imu_mount) and set it on the robot's settings panel.
2. With the ROV level and still, the RUI NavPose orientation reads pitch and roll near 0. Pitch or roll near ±90° means the wrong row.
3. Turn the ROV slowly 90° counter-clockwise, seen from above, on a flat surface. Yaw rises by about 90°, and the yaw rate is positive while it turns.
4. Tip the nose down: pitch goes positive. Lower the right side: roll goes positive.
5. Unplug the IMU's power for a moment, if your wiring allows it. Check that `$TNCTL` keeps arriving (finding 3).

*Pass:* steps 2 to 4 within a few degrees. If steps 3 and 4 have the wrong sign, try the row with the opposite Y direction. Our code was checked for all 24 mounts against the sketch's own conversion and 2-decimal print (within 0.02°), but never with a real BNO085.
