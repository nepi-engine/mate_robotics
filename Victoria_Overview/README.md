# Victoria Overview: NEPI on the Coral Crusaders ROV

2026-11-14, from the NEPI team, for Victoria Zhang and the Coral Crusaders.

Start here. This folder holds every document we wrote for you, and this page says which one to read at each step. The work has two parts:

- **The link.** NEPI Connect brings your ROV's telemetry and video into NEPI, and lets NEPI fly the ROV gently after the pilot presses RB. Our side is built and bench-tested. Integrating it with your code is yours.
- **Stabilization.** NEPI holds the ROV in place from the camera, using your optical flow tracker. The plan has five phases, and only its foundation exists so far.

Code stays in the app and driver folders where NEPI builds it. The links below open it.

## The Plan in Brief

```
 ROV camera
     | FastMJPG
     v
 idx_net_stream driver: a NEPI camera
     | sensor_msgs/Image
     v
 your optical flow app
     | flow_data topic (Phase 1)
     v
 auto_move Stabilization <---- orientation topic (Phase 2) ------+
     | the Optical Flow method, every 0.1 s (Phase 3)            |
     | set_teleop_velocity                                       |
     v                                                           |
 rbx_nepi_connect driver ----------------------------------------+
     | demands, only during NEPI control after the pilot's RB
     v
 ROV Pi (nepi_link) --> your Teensy --> thrusters
```

1. **The link.** Add nepi_link to your Pi script, then run test stages 1 to 3.
2. **Video.** Make your ROV camera a NEPI camera, so NEPI apps can read it.
3. **Phase 1, flow data.** Your optical flow app publishes one data message per processed frame, on a new topic.
4. **Phase 2, inputs.** auto_move reads that topic and the ROV's orientation, and shows both in its status. It commands nothing.
5. **Phase 3, dry run.** The Optical Flow method works out its commands and shows what it would send. It still sends nothing.
6. **Phase 4, bench.** With the ESC supply off, check the signs and every safety gate against your `$TNCTL` lines.
7. **Phase 5, water.** Test stage 4 on the tether: one axis at a time at low caps, then tuning.

Steps 1 and 2 can run side by side, and Phase 1 needs only step 2. Phase 2 needs the robot driver connected to your ROV Pi. Phases 3 and 4 need NEPI control working between your ROV Pi and your NEPI, and the engine update in the next section. Phase 5 needs test stages 1 to 3 passed.

## Where Things Stand

As of 2026-11-14.

| Part | Where | State |
|---|---|---|
| nepi_link and the robot driver | [Nepi_Connect/pi/](../Nepi_Connect/pi/), [Drivers/rbx_drivers/](../Drivers/rbx_drivers/) | Built. Their 108 and 60 unit tests pass, and our bench ran them with rov-26 code. Not yet run with your current code or in water. [INTEGRATION_HANDOFF.md](INTEGRATION_HANDOFF.md#status) has the detail. |
| Video into NEPI | [Drivers/idx_drivers/](../Drivers/idx_drivers/) | Built. The `fastmjpg://` mode worked under NEPI on our bench with one camera. |
| Your optical flow app | [Apps/nepi_app_optical_flow/](../Apps/nepi_app_optical_flow/) | Yours. It publishes an annotated image and JSON telemetry, but no topic a controller can use. Plan section 2.2 says why, and Phase 1 adds one. |
| Stabilization foundation | [Apps/nepi_app_auto_move/](../Apps/nepi_app_auto_move/) | Written, not yet run on a NEPI. A Stabilization section with a Method selector and an Enable toggle, and a loop that runs the selected method every 0.1 s. Its one method, Optical Flow, commands nothing. |
| The command auto_move will send | The NEPI engine | Added 2026-11-13 as `ConnectRBXDeviceIF.set_teleop_velocity`. Your NEPI needs an engine update that includes it before Phase 3. The first lines of plan section 9 show how to check. |

Our NEPI-side code for this work, the robot driver, auto_move and the engine command, is written for NEPI 3.4.

## What to Read, and When

| When | Read | Done when |
|---|---|---|
| First | This page, then [INTEGRATION_HANDOFF.md](INTEGRATION_HANDOFF.md): terms, every file we built, what we tested, and the values only you can set | |
| Before you touch the ROV | [SYSTEM_GUIDE.md](SYSTEM_GUIDE.md), then [SAFETY.md](SAFETY.md) | You know the handover rules, what our code enforces and what stays yours |
| Step 1, the link | [PI_INTEGRATION.md](PI_INTEGRATION.md) with your vehicle file, [mate_coral_crusaders.yaml](../Nepi_Connect/vehicles/mate_coral_crusaders.yaml); then [NEPI_SETUP.md](NEPI_SETUP.md); then [TEST_PLAN.md](TEST_PLAN.md) stages 1 to 3 | Stage 3 passes |
| Step 2, video | [VIDEO.md](VIDEO.md): ["Our Method"](VIDEO.md#our-method-a-second-output-to-nepi), or ["Running Both at Once"](VIDEO.md#running-both-at-once) if you keep your relay | The ROV camera shows as a NEPI camera, and your optical flow app can select it |
| Before Phase 1 | [OPTICAL_FLOW_STAB_PLAN.md](OPTICAL_FLOW_STAB_PLAN.md) sections [1](OPTICAL_FLOW_STAB_PLAN.md#1-goal-and-scope), [7](OPTICAL_FLOW_STAB_PLAN.md#7-phased-plan) and [10](OPTICAL_FLOW_STAB_PLAN.md#10-rules-for-this-work): the chain, the phases and the rules | |
| Phase 1 | Plan [section 2](OPTICAL_FLOW_STAB_PLAN.md#2-part-1-publish-the-flow-data-optical-flow-app); deploy with [section 9](OPTICAL_FLOW_STAB_PLAN.md#9-deploy-and-verify), item 1 | [2.8](OPTICAL_FLOW_STAB_PLAN.md#28-phase-1-acceptance) passes |
| Phase 2 | First [install the foundation](#before-phase-2-install-the-foundation). Then plan [section 3](OPTICAL_FLOW_STAB_PLAN.md#3-part-2-get-the-flow-data-and-the-navpose-into-the-method-auto_move); deploy with section 9, item 2 | [3.5](OPTICAL_FLOW_STAB_PLAN.md#35-phase-2-acceptance) passes, including the orientation topic check |
| Phase 3 | Plan [section 4](OPTICAL_FLOW_STAB_PLAN.md#4-part-3-what-the-optical-flow-method-does) and [section 5](OPTICAL_FLOW_STAB_PLAN.md#5-part-4-sending-the-commands-the-automoveif-change); the engine check at the top of section 9 | Plan section 7, row 3 |
| Phase 4 | Plan [4.4](OPTICAL_FLOW_STAB_PLAN.md#44-the-sign-chain), [4.5](OPTICAL_FLOW_STAB_PLAN.md#45-the-gates) and [8.4](OPTICAL_FLOW_STAB_PLAN.md#84-bench-gates-and-in-water); [SAFETY.md](SAFETY.md) again | Plan section 7, row 4 |
| Phase 5 | [TEST_PLAN.md stage 4](TEST_PLAN.md#stage-4-in-water-on-the-tether), whose row 4.4 is this hold; [SAFETY.md, "Rules for the First Powered Tests"](SAFETY.md#rules-for-the-first-powered-tests); plan [4.7](OPTICAL_FLOW_STAB_PLAN.md#47-the-control-law-and-starting-gains) for tuning | Plan section 7, row 5 |
| When something fails | Plan [section 6](OPTICAL_FLOW_STAB_PLAN.md#6-safety-and-failure-behavior): what NEPI does on each failure | |

Reference documents stay beside the code they describe:

- [Nepi_Connect/VEHICLE_FILE.md](../Nepi_Connect/VEHICLE_FILE.md): every vehicle file key. Current.
- [Nepi_Connect/LINK_PROTOCOL.md](../Nepi_Connect/LINK_PROTOCOL.md): the wire contract, version 2. Current.
- [Nepi_Connect/README.md](../Nepi_Connect/README.md), [BENCH_SETUP_PLAN.md](../Nepi_Connect/BENCH_SETUP_PLAN.md) and [DRIVER_HANDOFF.md](../Nepi_Connect/DRIVER_HANDOFF.md): our working notes. Parts describe earlier designs ([INTEGRATION_HANDOFF.md, "About Our Older Notes"](INTEGRATION_HANDOFF.md#about-our-older-notes)).
- [Apps/nepi_app_optical_flow/README.md](../Apps/nepi_app_optical_flow/README.md): your app.
- [Docs/README.md](../Docs/README.md): notes on the ROV's parts, such as the T200, the Basic ESC, the BNO085 and the Bar02.

## Before Phase 2: Install the Foundation

auto_move's stabilization foundation changed its status message and its RUI page, so its first install on your NEPI needs a catkin build and an RUI build. Phases 2 to 4 need neither: they change only auto_move's `api/` and `sdk/` code, which plan section 9 live-syncs. After the install, check in the RUI:

1. The Stabilization box shows Method, set to Optical Flow, and Enable.
2. Enable turns on and off, and the status message follows it.
3. After auto_move restarts, Enable is off. It never turns itself on.

If you have not built an app or the RUI on your NEPI before, ask us first.

## Who Does What

- **You:** your Pi script and firmware, the values in your vehicle file, your optical flow app (Phase 1), the auto_move changes (Phases 2 to 4), and every test.
- **Us:** the NEPI engine and its update for Phase 3, any change to our side of this repo when something does not behave as written here, and answers. Send us what you saw, such as a status line, a `$TNCTL` line or a log line, and we answer from that.

## Answers We Need From You

[INTEGRATION_HANDOFF.md](INTEGRATION_HANDOFF.md#values-only-you-can-set) lists the values only you can set. Plan [section 12](OPTICAL_FLOW_STAB_PLAN.md#12-open-questions-for-the-team) collects the questions stabilization waits on:

1. Which camera and mount, forward or down, and its horizontal field of view.
2. Whether heave and yaw stay with your Teensy's depth and heading holds.
3. Who owns the Robots page Teleop while Enable is on.
4. Whether the ROV's orientation topic appears, with a time that advances (the 3.5 check).
5. When the sway cap can rise from 0.1 to 0.2.
6. Any lag between the image and the orientation (the 8.3 test).

## Notes

- `nepi_disp.sh` still expects the scripts your commit `aff0f84` removed along with `Apps/nepi_app_robot_stab` ([VIDEO.md](VIDEO.md#your-method-the-relay-through-nepi)).
- The plan's line numbers are from 2026-11-12 and 13 and will drift. Search for the quoted code if a line has moved. Names marked **(proposed)** do not exist yet.
- This folder replaced `Handoff/` on 2026-11-14. Its README is now [INTEGRATION_HANDOFF.md](INTEGRATION_HANDOFF.md), and the plan moved here from `Apps/nepi_app_auto_move/`.

The NEPI team
