# auto_move optical flow stabilization: build plan

For the MATE developer who will make the Optical Flow stabilization method hold the ROV in place.
Written 2026-11-12 against mate_robotics `46f5c2a` plus the stabilization foundation in auto_move, uncommitted at the time: [api/auto_move_if.py](../Apps/nepi_app_auto_move/api/auto_move_if.py), [scripts/auto_move_app_node.py](../Apps/nepi_app_auto_move/scripts/auto_move_app_node.py), [msg/NepiAppAutoMoveStatus.msg](../Apps/nepi_app_auto_move/msg/NepiAppAutoMoveStatus.msg), [rui/NepiAppAutoMove.js](../Apps/nepi_app_auto_move/rui/NepiAppAutoMove.js) and [sdk/nepi_auto_move_stab.py](../Apps/nepi_app_auto_move/sdk/nepi_auto_move_stab.py). The plan only: no code has been changed.
Revised 2026-11-13 with the user's decisions: navpose comes from the robot's NPX orientation topic (3.2), and commands go through a new engine method, `ConnectRBXDeviceIF.set_teleop_velocity` (5.1, section 11). That method was added to the NEPI engine the same day.

**How to read the references.** Line numbers are from 2026-11-12 and will drift as you edit, so search for the quoted code if a line has moved. Links open the mate_robotics files they name. A path with no app folder in front, such as `api/auto_move_if.py` or `deploy_app.sh`, is in `Apps/nepi_app_auto_move/`. Five files are cited so often that they have short names: "the IF" is [api/auto_move_if.py](../Apps/nepi_app_auto_move/api/auto_move_if.py), so `IF:1413` is its line 1413; "the stab module" is [sdk/nepi_auto_move_stab.py](../Apps/nepi_app_auto_move/sdk/nepi_auto_move_stab.py); "the flow node" is [Apps/nepi_app_optical_flow/scripts/optical_flow_app_node.py](../Apps/nepi_app_optical_flow/scripts/optical_flow_app_node.py); "the tracker" is [Apps/nepi_app_optical_flow/scripts/optical_flow.py](../Apps/nepi_app_optical_flow/scripts/optical_flow.py); "the driver" is [Drivers/rbx_drivers/rbx_nepi_connect_node.py](../Drivers/rbx_drivers/rbx_nepi_connect_node.py). Engine files are cited by short name. Their full paths inside the nepi_engine_ws checkout are:

| Short name | Path in nepi_engine_ws |
|---|---|
| `connect_device_if_rbx.py`, `device_if_rbx.py`, `device_if_npx.py`, `system_if.py`, `connect_system_if.py`, `node_if.py` | `src/nepi_engine/nepi_api/src/nepi_api/` |
| `nepi_sdk.py`, `nepi_nav.py`, `nepi_controls.py` | `src/nepi_engine/nepi_sdk/src/nepi_sdk/` |
| `NavPose.msg`, `NavPoseOrientation.msg`, `DeviceRBXStatus.msg`, `DataHeader.msg`, `Targets.msg` | `src/nepi_interfaces/msg/` |

Every name marked **(proposed)** does not exist yet. Code blocks are sketches for you to adapt, and say so.

---

## 1. Goal and scope

Today the Stabilization section of auto_move has a Method selector, an Enable toggle, and a loop that runs the selected method every 0.1 s. The only method, Optical Flow, is a placeholder that commands nothing ([stab:117–141](../Apps/nepi_app_auto_move/sdk/nepi_auto_move_stab.py#L117-L141)). This plan takes it to a method that holds the ROV in place from the camera. It answers three questions in order: how the optical flow app publishes data the method can use (section 2), how that data and the ROV's navpose reach the method (section 3), and what the method computes and how auto_move turns it into thruster demands (sections 4 and 5).

### The chain

```
 ROV camera --FastMJPG/UDP--> IDX net stream driver        frames stamped on arrival, NEPI clock
                                    | sensor_msgs/Image
                                    v
                       optical flow app (app_optical_flow)   one estimate per processed frame
                                    | <base>/app_optical_flow/flow_data   (proposed)
                                    v
 auto_move (app_auto_move)  flowDataCb (proposed) -----> newest flow sample
                            stabOrientationCb (proposed) -> orientation history (proposed)
                            stabProcessCb, every 0.1 s ---> stab_optical_flow() in the stab module
                                    | result['cmd'] (proposed)
                                    v
                       rbx_if.set_teleop_velocity() (engine method, 5.1)
                                    | <rbx namespace>/set_teleop_velocity, geometry_msgs/Twist
                                    v
 rbx_nepi_connect driver (RBXRobotIF + NepiConnectLink) --demand every 0.1 s--> ROV Pi (nepi_link) --> Teensy --> thrusters
        ^ status + nav every 100 ms
        | NPXDeviceIF getNavPoseCb, 10 Hz --> <npx navpose>/orientation (NavPoseOrientation) --> auto_move stabOrientationCb
```

| Piece | Owner | Rate |
|---|---|---|
| Camera, ROV Pi, Teensy | The team | Camera frame rate unknown; the flow app reports what it achieves (`fps`) |
| IDX net stream driver, rbx_nepi_connect driver | mate_robotics `Drivers/` | Driver demand every 0.1 s ([driver:140](../Drivers/rbx_drivers/rbx_nepi_connect_node.py#L140)); navpose 10 Hz ([driver:977](../Drivers/rbx_drivers/rbx_nepi_connect_node.py#L977)) |
| Optical flow app | mate_robotics `Apps/nepi_app_optical_flow` (you, Part 1) | Once per processed frame, newest frame only |
| auto_move: subscriptions, loop, method, commands | mate_robotics `Apps/nepi_app_auto_move` (you, Parts 2 to 4) | Loop every `STAB_PROCESS_INTERVAL_SEC` = 0.1 s ([IF:216](../Apps/nepi_app_auto_move/api/auto_move_if.py#L216)) |
| RBXRobotIF, NPXDeviceIF, NavPoseIF, ConnectRBXDeviceIF | NEPI engine | Not yours to change. `ConnectRBXDeviceIF.set_teleop_velocity` was added for this work (5.1); the NEPI side builds and deploys it before Phase 3 (section 9) |

Two facts about the ROV shape everything below. It supports no goto at all: its RBXRobotIF gets `None` for every goto function ([driver:1101–1131](../Drivers/rbx_drivers/rbx_nepi_connect_node.py#L1101-L1131)), so `set_teleop_velocity` is its only motion command. And NEPI never takes the ROV: only the pilot's RB hands it over, and the toggle, any stick past the takeover point, or 2 s without a demand hands it back ([driver:33–40](../Drivers/rbx_drivers/rbx_nepi_connect_node.py#L33-L40)).

### What "works when it is turned on" means

With NEPI control on (the pilot pressed RB) and Enable on, the ROV holds position on the axes the camera can observe and the operator has switched on. When Enable goes off, tracking is lost, or the flow or navpose data goes stale, NEPI commands neutral on those axes within one 0.1 s cycle, and the Teensy's own depth and heading holds keep the ROV where it is. The operator can read in `stab_msg` what the method is doing and why.

### Out of scope

- The rest of the optical flow app's 3.4 port, beyond what the data path needs. Section 2.7 lists it.
- Other stabilization methods.
- How the driver or the engine selects the RBX navpose source. The method reads the NPX orientation topic directly and does not depend on it (3.2).
- Any change to robot-side code: `Nepi_Connect/pi`, the Pi scripts, or the Teensy firmware.

---

## 2. Part 1: publish the flow data (optical flow app)

### 2.1 What the app publishes today

- `~telemetry`: `std_msgs/String` holding JSON, latched, queue 2 ([flow node:90](../Apps/nepi_app_optical_flow/scripts/optical_flow_app_node.py#L90)). Every publish carries `status`, `camera_topic`, `available_topics`, `k1`, `mask_foreground` and `fps` ([flow node:167–169](../Apps/nepi_app_optical_flow/scripts/optical_flow_app_node.py#L167-L169)). A processed frame adds `t`, `x`, `y`, `features`, `inliers`, `processing_ms`, `width` and `height` ([flow node:228–232](../Apps/nepi_app_optical_flow/scripts/optical_flow_app_node.py#L228-L232)).
- `~annotated_image`: the corrected frame with the trail drawn on it, `bgr8` ([flow node:91](../Apps/nepi_app_optical_flow/scripts/optical_flow_app_node.py#L91), [:213–221](../Apps/nepi_app_optical_flow/scripts/optical_flow_app_node.py#L213-L221)).

The node is plain rospy and imports nothing from nepi_sdk or nepi_api ([flow node:11](../Apps/nepi_app_optical_flow/scripts/optical_flow_app_node.py#L11), [:241](../Apps/nepi_app_optical_flow/scripts/optical_flow_app_node.py#L241)). The package has no `msg/` folder, and its [CMakeLists.txt](../Apps/nepi_app_optical_flow/CMakeLists.txt) generates no messages.

### 2.2 Why the method cannot use that

1. **JSON in a String has no contract.** Nothing checks a key at build time, and a rename breaks consumers silently. The RUI page parses it by hand ([NepiAppOpticalFlow.js:89–92](../Apps/nepi_app_optical_flow/rui/NepiAppOpticalFlow.js#L89-L92)).
2. **It is latched and republished without a sample.** The 2 s timer publishes telemetry with no new estimate ([flow node:96](../Apps/nepi_app_optical_flow/scripts/optical_flow_app_node.py#L96), [:158](../Apps/nepi_app_optical_flow/scripts/optical_flow_app_node.py#L158)). A consumer that judges freshness by arrival would trust a camera that stopped.
3. **`t` is the wall clock at processing** (`time.time()`, [flow node:228](../Apps/nepi_app_optical_flow/scripts/optical_flow_app_node.py#L228)), not the image's stamp, so it cannot be lined up with navpose.
4. **`x` and `y` are swapped and rounded:** `x = dy`, `y = -dx` ([flow node:207–208](../Apps/nepi_app_optical_flow/scripts/optical_flow_app_node.py#L207-L208)), to 0.01 px. Nothing says what they mean for the robot.
5. **Nobody can tell a fresh estimate from a held one.** Every rejection path in the tracker returns the previous value unchanged ([tracker:115](../Apps/nepi_app_optical_flow/scripts/optical_flow.py#L115), [:123](../Apps/nepi_app_optical_flow/scripts/optical_flow.py#L123), [:140](../Apps/nepi_app_optical_flow/scripts/optical_flow.py#L140), [:148](../Apps/nepi_app_optical_flow/scripts/optical_flow.py#L148), [:159](../Apps/nepi_app_optical_flow/scripts/optical_flow.py#L159)). `last_inlier_count` does not tell you either: on the max-jump path ([tracker:154–159](../Apps/nepi_app_optical_flow/scripts/optical_flow.py#L154-L159)) it still holds the full, passing count.
6. **A reference reset looks like a return to the start.** A reset puts `x` and `y` back to 0 ([flow node:191–195](../Apps/nepi_app_optical_flow/scripts/optical_flow_app_node.py#L191-L195), [:205](../Apps/nepi_app_optical_flow/scripts/optical_flow_app_node.py#L205)). It happens on `reset_reference`, a k1 change, a mask change, a camera change and a resolution change ([flow node:114](../Apps/nepi_app_optical_flow/scripts/optical_flow_app_node.py#L114), [:120](../Apps/nepi_app_optical_flow/scripts/optical_flow_app_node.py#L120), [:126](../Apps/nepi_app_optical_flow/scripts/optical_flow_app_node.py#L126), [:148](../Apps/nepi_app_optical_flow/scripts/optical_flow_app_node.py#L148), [:191](../Apps/nepi_app_optical_flow/scripts/optical_flow_app_node.py#L191)).
7. **Half the fit is thrown away.** `cv2.estimateAffinePartial2D` fits rotation, uniform scale and translation ([tracker:133–138](../Apps/nepi_app_optical_flow/scripts/optical_flow.py#L133-L138)), but only the displacement of the frame centre is kept ([tracker:152–153](../Apps/nepi_app_optical_flow/scripts/optical_flow.py#L152-L153)).

### 2.3 The data contract

**The tracker's sign, measured.** On 2026-11-12 a textured synthetic 640×480 image was shifted with `cv2.warpAffine` and fed to `OpticalFlow(nfeatures=700, smooth=1.0, track_incrementally=True)`, exactly as the node builds it ([flow node:203–204](../Apps/nepi_app_optical_flow/scripts/optical_flow_app_node.py#L203-L204)). Content moved right 10 px gave `(dx, dy)` = (+10.00, −0.00); down 10 px gave (−0.00, +10.03); right 12 and down 7 gave (+12.00, +7.05); left 8 and up 5 gave (−8.01, −5.00), each with 80 inliers. So `(dx, dy)` is how far the image content moved since the reference, in OpenCV image axes: +x right, +y down. The telemetry's `x = dy` is therefore content moving down, and `y = -dx` is content moving left. Three shifts of 5 px in a row returned 5, 10 and 15 (accumulation works), and a blank frame after them returned 15 with 0 inliers (the held value). Rotating the content 3° about the centre left `(dx, dy)` near 0, and the fit's `atan2(A[1,0], A[0,0])` came out −2.99°: in image axes, a positive fit angle is clockwise as displayed.

**The message (proposed):** `msg/NepiAppOpticalFlowData.msg` in the optical flow app, a new `msg/` folder. It carries image-axis values, not robot axes. The camera mount is unconfirmed, and mapping image motion to robot motion is the method's job (section 4). The telemetry's swapped `x` and `y` stay as they are for the RUI page.

```
# Sketch: msg/NepiAppOpticalFlowData.msg (proposed)
# One message per processed frame on <app>/flow_data. Not latched.
# Every value is relative to the current reference frame.

# source_topic: the camera image topic. source_timestamp: that image's header
# stamp (s, NEPI clock), the time to align against.
nepi_interfaces/DataHeader data_header

uint32 reference_id    # changes every time the reference resets
bool accepted          # False: shift, scale and rotation are the last accepted values, held

float32 shift_x_px     # content moved since the reference, at the frame centre, + right
float32 shift_y_px     # + down (OpenCV image axes)
float32 scale          # content scale since the reference; above 1: the content grew
float32 rotation_deg   # content rotation since the reference, + counter-clockwise as displayed

int32 features         # ORB features found in this frame
int32 inliers          # RANSAC inliers among the 80 best matches; 0 on most rejection paths
uint32 width_px        # size of the image the shift is measured in
uint32 height_px
```

| Field | Why the method needs it | When it changes |
|---|---|---|
| `data_header.source_timestamp` | Aligning the sample with navpose (derotation), and the D term's time base | Every message |
| `data_header.source_topic` | `stab_msg` names the camera the hold uses | On a camera change |
| `reference_id` | Recapture the hold point on a reset, instead of driving back to the old 0 | On every reset; the first message of a reference has shift 0 |
| `accepted` | Ignore held estimates, and declare tracking lost when they run on | Every message |
| `shift_x_px`, `shift_y_px` | The error | On accepted frames only |
| `scale`, `rotation_deg` | Not used by the first method. See below. | On accepted frames only |
| `features`, `inliers` | The quality gate, and the operator's readout | Every message |
| `width_px`, `height_px` | Focal length in pixels for derotation (section 4.2) | On a resolution change, with a reset |

`DataHeader` is the NEPI convention for "computed from this source at this time". `Targets.msg` uses it (line 4), and auto_move already reads `data_header.source_topic` from targets and obstacles ([IF:1385](../Apps/nepi_app_auto_move/api/auto_move_if.py#L1385), [:1404](../Apps/nepi_app_auto_move/api/auto_move_if.py#L1404)). Its fields are `process_name`, `process_namespace`, `process_timestamp`, `source_topic` and `source_timestamp` (`DataHeader.msg:3–8`).

**Add `scale` and `rotation_deg` now.** The fit already computes them, so they cost two multiplies per frame. Adding fields later costs a second message change and a catkin build of both packages. A later method can use them for what translation cannot see:

- `scale` grows as the camera closes on the scene. With a forward camera that is the one signal for surge (scale ≈ Z0 / (Z0 − Δx) for range Z0). With a downward camera it signals altitude change.
- `rotation_deg` is the heading change for a downward camera, about −Δyaw for small angles.

Do not add `fps` or `processing_ms`. The method measures the sample interval itself from `source_timestamp`, and the telemetry keeps both for the page.

### 2.4 Tracker and node changes (Phase 1)

The tracker gets three public attributes beside `last_feature_count` and `last_inlier_count` ([tracker:62–63](../Apps/nepi_app_optical_flow/scripts/optical_flow.py#L62-L63)). The node gets a reference counter and the publisher. Both are additive: the telemetry, the annotated image and the RUI page keep working unchanged.

```python
# Sketch (fragment), tracker. In __init__, beside last_feature_count:
        self.last_accepted = False
        self.scale = 1.0
        self.rotation_deg = 0.0

# In compute_displacement_px, beside "self.last_inlier_count = 0" (tracker:105):
        self.last_accepted = False

# On the accepted path, after the max-jump check passes (tracker:161), before smoothing.
# Sign from the 2026-11-12 test: a fit angle of +3 deg is content turned clockwise as displayed.
        scale_step = float(np.hypot(transform[0, 0], transform[1, 0]))
        rotation_step_deg = -float(np.degrees(np.arctan2(transform[1, 0], transform[0, 0])))
        if self.track_incrementally:
            self.scale *= scale_step
            self.rotation_deg += rotation_step_deg
        else:
            self.scale = scale_step
            self.rotation_deg = rotation_step_deg
        self.last_accepted = True
```

In the node, add `self.reference_id = 0` in `__init__`, and add one to it where a new tracker is built ([flow node:198–205](../Apps/nepi_app_optical_flow/scripts/optical_flow_app_node.py#L198-L205)). On that frame, set `dx = dy = 0.0` and `accepted = True`: the reference frame is its own exact estimate. On the other frames, keep `dx, dy` from [flow node:207](../Apps/nepi_app_optical_flow/scripts/optical_flow_app_node.py#L207) and read `accepted`, `scale` and `rotation_deg` from the tracker.

### 2.5 Publishing it

**Topic (proposed):** `flow_data` under the node's namespace, so `<base>/app_optical_flow/flow_data`. The node name is fixed by [the params file](../Apps/nepi_app_optical_flow/params/optical_flow_app_params.yaml) (`node_name: app_optical_flow`) and by `init_node` ([flow node:241](../Apps/nepi_app_optical_flow/scripts/optical_flow_app_node.py#L241)). The topic is not latched, has queue 1, and gets one message per processed frame, including the reference frame. Nothing is published on the 2 s timer.

**Add it before the rest of the port.** The port (2.7) is several days of work and touches the RUI page. The data topic needs none of it, and Parts 2 to 4 need only the topic. Write the new lines through nepi_sdk now, so that they already follow the 3.4 rule. The old rospy lines are the port's job. When the port lands, the publisher moves into the node's NodeClassIF `PUBS_DICT` under the key `optical_flow_data_pub` (proposed), as auto_move registers `status_pub` ([IF:493–500](../Apps/nepi_app_auto_move/api/auto_move_if.py#L493-L500)). The topic and the message stay the same, so auto_move does not change.

```python
# Sketch (fragment), flow node. Imports:
from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_app_optical_flow.msg import NepiAppOpticalFlowData

# In __init__, beside telemetry_pub (flow node:90):
        self.reference_id = 0
        self.flow_pub = nepi_sdk.create_publisher('~flow_data', NepiAppOpticalFlowData, queue_size = 1)

# In receive_frame (flow node:160), record when the frame arrived:
            self.pending_frame = message
            self.pending_time = nepi_utils.get_time()

# In process_frames, after publish_telemetry (flow node:228-232).
# source_topic and receive_time are read under the lock with the frame.
                image_stamp_s = nepi_sdk.sec_from_timestamp(message.header.stamp)
                if image_stamp_s <= 0.0:
                    image_stamp_s = receive_time   # unstamped source: use arrival time
                flow_msg = NepiAppOpticalFlowData()
                flow_msg.data_header.process_name = 'app_optical_flow'
                flow_msg.data_header.process_namespace = nepi_sdk.get_node_namespace()
                flow_msg.data_header.process_timestamp = nepi_utils.get_time()
                flow_msg.data_header.source_topic = source_topic
                flow_msg.data_header.source_timestamp = image_stamp_s
                flow_msg.reference_id = self.reference_id
                flow_msg.accepted = accepted
                flow_msg.shift_x_px, flow_msg.shift_y_px = float(dx), float(dy)
                flow_msg.scale, flow_msg.rotation_deg = float(scale), float(rotation_deg)
                flow_msg.features = int(self.tracker.last_feature_count)
                flow_msg.inliers = int(self.tracker.last_inlier_count)
                flow_msg.width_px, flow_msg.height_px = int(outgoing.width), int(outgoing.height)
                nepi_sdk.publish_pub(self.flow_pub, flow_msg)
```

`nepi_sdk.create_publisher` and `publish_pub` are thin wrappers that never raise (`nepi_sdk.py:1741`, `:1752`). `sec_from_timestamp` is what NavPoseIF uses for the same job (`system_if.py:573`). The IDX net stream driver stamps each frame with its NEPI arrival time ([idx_net_stream_driver.py:380](../Drivers/idx_drivers/idx_net_stream_driver.py#L380)), so on the ROV camera the image stamp and NavPose times share one clock. Check on the device that the stamp is not 0 (section 9).

### 2.6 Build files

Follow auto_move's [CMakeLists.txt:12–51](../Apps/nepi_app_auto_move/CMakeLists.txt#L12-L51). Its comment explains why `sensor_msgs`, `geometry_msgs` and `nav_msgs` must be listed even though the message does not use them: they are `nepi_interfaces`' message dependencies, and without them the build fails with "Messages depends on unknown pkg".

```cmake
# Sketch: ../nepi_app_optical_flow/CMakeLists.txt, replacing its lines 3-4. The install lines stay.
find_package(catkin REQUIRED COMPONENTS
  rospy message_generation std_msgs sensor_msgs geometry_msgs nav_msgs nepi_interfaces)
add_message_files(FILES NepiAppOpticalFlowData.msg)
generate_messages(DEPENDENCIES std_msgs sensor_msgs geometry_msgs nepi_interfaces)
catkin_package(CATKIN_DEPENDS message_runtime nepi_interfaces)
```

```xml
<!-- Sketch: ../nepi_app_optical_flow/package.xml, added after the buildtool_depend -->
  <build_depend>message_generation</build_depend>
  <build_depend>std_msgs</build_depend>
  <build_depend>sensor_msgs</build_depend>
  <build_depend>geometry_msgs</build_depend>
  <build_depend>nav_msgs</build_depend>
  <build_depend>nepi_interfaces</build_depend>
  <build_export_depend>nepi_interfaces</build_export_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>nav_msgs</exec_depend>
  <exec_depend>nepi_interfaces</exec_depend>
  <exec_depend>message_runtime</exec_depend>
```

### 2.7 The consumer side, and a robot without the app

No connect class is needed for this work. auto_move already consumes another app's message, `Obstacles`, without one. It imports the message behind a guard ([IF:58–64](../Apps/nepi_app_auto_move/api/auto_move_if.py#L58-L64)) and adds the subscriber to its own `SUBS_DICT` only when the import worked ([IF:700–711](../Apps/nepi_app_auto_move/api/auto_move_if.py#L700-L711)). Its [package.xml:36–40](../Apps/nepi_app_auto_move/package.xml#L36-L40) explains why it declares no dependency on that app. Do the same for the flow message (section 3.1). On a robot without the optical flow app, auto_move then builds and starts unchanged, and the Optical Flow method reports "No flow source: the optical flow app is not installed". A connect class in the optical flow app's `api/` would not help here: CMake installs `api/*.py` into `nepi_api`, so that file would be missing on the same robots and need the same guard. Write `api/connect_app_optical_flow.py` (proposed), after [api/connect_app_auto_move.py](../Apps/nepi_app_auto_move/api/connect_app_auto_move.py), when another node needs to drive the app's controls.

**The rest of the 3.4 port,** listed only. Each item names the auto_move code that shows the 3.4 way:

- Plain rospy to nepi_sdk: `nepi_sdk.init_node` and `nepi_sdk.spin` ([scripts/auto_move_app_node.py:45](../Apps/nepi_app_auto_move/scripts/auto_move_app_node.py#L45), [:88](../Apps/nepi_app_auto_move/scripts/auto_move_app_node.py#L88)), and a NodeClassIF holding the pubs and subs ([IF:713–722](../Apps/nepi_app_auto_move/api/auto_move_if.py#L713-L722)).
- The camera input: a `ConnectColorImageIF` selector ([IF:1063–1070](../Apps/nepi_app_auto_move/api/auto_move_if.py#L1063-L1070)), replacing the `get_published_topics` scan and `set_image_topic`. Keep the large receive buffer (`buff_size` on `nepi_sdk.create_subscriber`, `nepi_sdk.py:1715`).
- The annotated image: a `ColorImageIF` data product, as the image pub node publishes its overlay ([scripts/auto_move_app_img_pub_node.py:515](../Apps/nepi_app_auto_move/scripts/auto_move_app_img_pub_node.py#L515)).
- k1, mask and reset as controls in a `ControlsIF` ([IF:730–739](../Apps/nepi_app_auto_move/api/auto_move_if.py#L730-L739)).
- The JSON telemetry as a typed status message, after [msg/NepiAppAutoMoveStatus.msg](../Apps/nepi_app_auto_move/msg/NepiAppAutoMoveStatus.msg).
- The RUI page reads that status and mounts `NepiIFControls` for the controls, as the Stabilization section does ([rui/NepiAppAutoMove.js:686–714](../Apps/nepi_app_auto_move/rui/NepiAppAutoMove.js#L686-L714)). Its PID inputs are wired to nothing ([NepiAppOpticalFlow.js:192](../Apps/nepi_app_optical_flow/rui/NepiAppOpticalFlow.js#L192)). Remove them, because the gains live in auto_move.
- The [README](../Apps/nepi_app_optical_flow/README.md): the class is registered through the params `RUI_DICT`, not in `Nepi_IF_Apps.js` (README lines 22–23), and the tracker no longer lives in `nepi_app_robot_stab` (line 5).

### 2.8 Phase 1 acceptance

On the device, with the ROV camera streaming:

1. `rostopic list | grep flow_data` shows `<base>/app_optical_flow/flow_data`.
2. `rostopic hz` on it matches the `fps` the RUI page shows.
3. `rostopic echo -n 1` on it shows a nonzero `data_header.source_timestamp` within a second of `process_timestamp`, and `source_topic` names the camera.
4. Slide a printed texture right in front of the camera: `shift_x_px` goes positive. Slide it down: `shift_y_px` goes positive.
5. Press Reset reference on the page: `reference_id` goes up by one, and the shifts return to 0.
6. Cover the lens: `accepted` goes false, `inliers` drops, and the shifts stop changing.
7. The RUI page still plots and the annotated image still streams.

---

## 3. Part 2: get the flow data and the navpose into the method (auto_move)

### 3.1 Find the flow source and keep the newest sample

There are three ways to find the source: a fixed node name, a selector, or discovery by message type (`nepi_sdk.find_topics_by_msg`, `nepi_sdk.py:1842`). **Use the fixed name.** The app runs one instance, and its node name is fixed by its params. A selector would add an RUI box and a second choice an operator can get wrong. The sample's `source_topic` still tells the operator which camera the hold is using.

```python
# Sketch (fragment), the IF. At module level, after the Obstacles guard (IF:58-64):
try:
    from nepi_app_optical_flow.msg import NepiAppOpticalFlowData
    HAS_FLOW_MSG = True
except ImportError:
    NepiAppOpticalFlowData = None
    HAS_FLOW_MSG = False

OPTICAL_FLOW_NODE_NAME = 'app_optical_flow'   # its params node_name
FLOW_DATA_TOPIC = 'flow_data'
STAB_NAVPOSE_HISTORY_LEN = 20                 # 2 s of navpose at 10 Hz

# In __init__, beside the all_obstacles block (IF:700-711):
        if HAS_FLOW_MSG == True and self.has_stab == True:
            flow_namespace = nepi_sdk.create_namespace(self.base_namespace, OPTICAL_FLOW_NODE_NAME)
            self.stab_flow_topic = nepi_sdk.create_namespace(flow_namespace, FLOW_DATA_TOPIC)
            self.SUBS_DICT[self.node_if_prefix + 'flow_data'] = {
                'namespace': flow_namespace,
                'topic': FLOW_DATA_TOPIC,
                'msg': NepiAppOpticalFlowData,
                'qsize': 1,
                'callback': self.flowDataCb,
                'callback_args': ()
            }

    def flowDataCb(self, msg):
        flow_dict = {'receive_time_s': nepi_utils.get_time(),
                     'image_time_s': float(msg.data_header.source_timestamp),
                     'source_topic': msg.data_header.source_topic,
                     'accepted': (msg.accepted == True)}
        for key in ['reference_id', 'features', 'inliers', 'width_px', 'height_px']:
            flow_dict[key] = int(getattr(msg, key))
        for key in ['shift_x_px', 'shift_y_px', 'scale', 'rotation_deg']:
            flow_dict[key] = float(getattr(msg, key))
        self.stab_flow_dict = flow_dict
        if flow_dict['accepted'] == True:
            self.stab_flow_accepted_dict = flow_dict
```

New attributes, with class defaults beside the stabilization block ([IF:336–350](../Apps/nepi_app_auto_move/api/auto_move_if.py#L336-L350)), all **(proposed)**: `stab_flow_topic = ''`, `stab_flow_dict = None`, `stab_flow_accepted_dict = None`, `stab_navpose_list = ()`, `stab_method_state_dict = {}`. The subscriber key `auto_move_flow_data` and the callback `flowDataCb` are also proposed. The key carries the `auto_move_` domain prefix, as the 2026-07 registry decision requires.

**Threading.** `flowDataCb` runs on its own subscriber thread. It builds a new dict and replaces the attributes whole. That is the rule `robot_velocity_dict` already follows ([IF:251–252](../Apps/nepi_app_auto_move/api/auto_move_if.py#L251-L252)), so the stabilization thread can read either attribute once, with no lock, and get a consistent sample. Keep two slots because the loop runs at 10 Hz and samples can arrive faster. Without the second slot, the newest accepted sample could be overwritten by a held one between cycles. The callback must never take `stab_lock`. A method holds that lock for its whole cycle ([IF:2101–2108](../Apps/nepi_app_auto_move/api/auto_move_if.py#L2101-L2108)), and a subscriber waiting on it would stall rospy's delivery.

### 3.2 The navpose: the NPX orientation topic

auto_move already follows the selected robot's combined NavPose. `getRobotNavPoseTopic` reads `navpose_topic` from the RBX status ([IF:1277–1293](../Apps/nepi_app_auto_move/api/auto_move_if.py#L1277-L1293)). The updater re-resolves it every second and resubscribes on a change ([IF:1151–1156](../Apps/nepi_app_auto_move/api/auto_move_if.py#L1151-L1156)), and drops it on a robot change ([IF:1137–1142](../Apps/nepi_app_auto_move/api/auto_move_if.py#L1137-L1142)). `robotNavPoseCb` ([IF:1413](../Apps/nepi_app_auto_move/api/auto_move_if.py#L1413)) keeps only a velocity report. Nothing in the app reads attitude, rates or depth today. Leave all of that as it is: the velocity report and `get_robot_velocity_dict()` ([IF:856](../Apps/nepi_app_auto_move/api/auto_move_if.py#L856)) are untouched.

**Read the robot's NPX orientation topic instead (decided 2026-11-13).** The combined NavPose cannot serve stabilization, for two reasons:

- **Its `timestamp` is the time NEPI published it,** not when the ROV measured it. `NPXDeviceIF` calls `publish_navpose(navpose_dict)` with no timestamp (`device_if_npx.py:729`), and `nepi_nav.convert_navpose_dict2msg` then stamps `nepi_utils.get_time()` (`nepi_nav.py:1253–1268`, called at `system_if.py:730`). When the newest telemetry is older than `NAV_MAX_AGE_MS` = 500, the driver hands back its previous dict unchanged ([driver:1290–1304](../Drivers/rbx_drivers/rbx_nepi_connect_node.py#L1290-L1304)), and that dict goes out with a fresh timestamp. Frozen values look new.
- **It reaches auto_move only if the RBX device picks the driver's own NPX as its navpose source,** which is the open 3.4 risk of engine commit `2be5ec8` (section 11, item 5).

The NPX device also publishes one topic per navpose group it has (`device_if_npx.py:637–646`). The orientation one is `<node>/<dev>/npx/navpose/orientation`, type `NavPoseOrientation` ([NEPI_SETUP.md](NEPI_SETUP.md), topic table, line 194; topic name at `system_if.py:332–338`). Its float64 `timestamp` is the driver's read time: `publish_navpose` copies the dict's `time_orientation` into it (`system_if.py:668–679`), and the driver sets that to when the robot read it, on the NEPI clock ([driver:1305](../Drivers/rbx_drivers/rbx_nepi_connect_node.py#L1305), [:952](../Drivers/rbx_drivers/rbx_nepi_connect_node.py#L952), docstring at [:932](../Drivers/rbx_drivers/rbx_nepi_connect_node.py#L932)). A frozen value keeps its old timestamp, so its age grows. The topic does not go through the RBX navpose source selection at all. The device's node-level NavPoseIF, which republishes the combined NavPose, publishes no group topics (`device_if.py:332`, with the defaults at `system_if.py:170–171`). `ConnectNavPoseIF` (`connect_system_if.py:499`) was the other option the run weighed; it reads the combined NavPose, so it has the same timestamp problem.

What navpose adds, detailed in 4.2: roll, pitch and yaw to take camera rotation out of the flow (derotation), and to stop when the ROV tilts or turns fast. Depth is not read (4.2).

**Finding the topic.** The driver's RBX and NPX share one device namespace, `<node>/<dev>/rbx` and `<node>/<dev>/npx` ([NEPI_SETUP.md](NEPI_SETUP.md), topic table, lines 191–195). So drop `/rbx` from the selected robot's namespace, add `/npx/navpose/orientation`, and subscribe only if `nepi_sdk.find_topics_by_msg('NavPoseOrientation')` (`nepi_sdk.py:1842`) lists that name. A robot whose driver hosts no NPX has no such topic, and the method then runs without navpose. Resolve it in `updateRobotState` while nothing is subscribed, after the navpose re-resolve, and drop it on a robot change, as `subscribeRobotTopics` and `unsubscribeRobotTopics` do for the combined NavPose ([IF:1295–1330](../Apps/nepi_app_auto_move/api/auto_move_if.py#L1295-L1330)).

**The fields kept.** Each snapshot has a validity flag, and a value is `None` when it is False, never 0.0, as `BLANK_ROBOT_VELOCITY_DICT` does ([IF:189–202](../Apps/nepi_app_auto_move/api/auto_move_if.py#L189-L202)).

```python
# Sketch (fragment), the IF. Import NavPoseOrientation beside NavPose (IF:36).
# Module level, beside BLANK_ROBOT_VELOCITY_DICT:
BLANK_STAB_NAVPOSE_DICT = {
    'receive_time_s': None,     # NEPI time the message arrived; readout only
    'timestamp_s': None,        # NavPoseOrientation.timestamp: the driver's READ time
    'orientation_valid': False,
    'roll_deg': None, 'pitch_deg': None, 'yaw_deg': None,                  # FLU, CCW yaw
    'roll_deg_per_sec': None, 'pitch_deg_per_sec': None, 'yaw_deg_per_sec': None,
}

# In updateRobotState: call self.unsubscribeStabOrientation() inside
# "if selection_changed == True:" (IF:1137), then, after the navpose re-resolve (IF:1154-1156):
        if self.has_stab == True and self.stab_orientation_topic == '':
            orientation_topic = self.getStabOrientationTopic()
            if orientation_topic != '':
                self.subscribeStabOrientation(orientation_topic)

    def getStabOrientationTopic(self):
        # <node>/<dev>/rbx -> <node>/<dev>/npx/navpose/orientation, if it is published
        if not self.rbx_namespace.endswith('/rbx'):
            return ''
        topic = self.rbx_namespace[:-len('/rbx')] + '/npx/navpose/orientation'
        if topic not in nepi_sdk.find_topics_by_msg('NavPoseOrientation'):
            return ''
        return topic

    def subscribeStabOrientation(self, orientation_topic):
        subs_dict = {
            'auto_move_stab_orientation_sub': {
                    'namespace': orientation_topic,
                    'msg': NavPoseOrientation,
                    'topic': '',
                    'qsize': 1,
                    'callback': self.stabOrientationCb,
                    'callback_args': ()
            },
        }
        self.msg_if.pub_info('Registering to robot orientation topic: ' + str(orientation_topic))
        self.stab_orientation_subs_if = NodeSubscribersIF(subs_dict = subs_dict, msg_if = self.msg_if)
        self.stab_orientation_topic = orientation_topic

    def unsubscribeStabOrientation(self):
        if self.stab_orientation_subs_if is not None:
            try:
                self.stab_orientation_subs_if.unregister_subs()
            except Exception as e:
                self.msg_if.pub_warn("Failed to unregister orientation sub: " + str(e))
            self.stab_orientation_subs_if = None
        self.stab_orientation_topic = ''
        self.stab_navpose_list = ()

    def stabOrientationCb(self, msg):
        nav_dict = copy.deepcopy(BLANK_STAB_NAVPOSE_DICT)
        nav_dict['receive_time_s'] = nepi_utils.get_time()
        nav_dict['timestamp_s'] = float(msg.timestamp)
        nav_dict['orientation_valid'] = True
        for key in ['roll_deg', 'pitch_deg', 'yaw_deg',
                    'roll_deg_per_sec', 'pitch_deg_per_sec', 'yaw_deg_per_sec']:
            nav_dict[key] = float(getattr(msg, key))
        self.stab_navpose_list = (self.stab_navpose_list + (nav_dict,))[-STAB_NAVPOSE_HISTORY_LEN:]
```

New names, all **(proposed)**: the `NavPoseOrientation` import, `BLANK_STAB_NAVPOSE_DICT`, the subscriber key `auto_move_stab_orientation_sub`, the methods `getStabOrientationTopic`, `subscribeStabOrientation`, `unsubscribeStabOrientation` and `stabOrientationCb`, and the attributes `stab_orientation_topic = ''` and `stab_orientation_subs_if = None`. They are private, so they are camelCase with no docstring. `stabOrientationCb` follows the threading rule in 3.1: it replaces `stab_navpose_list` whole and never takes `stab_lock`. Call `unsubscribeStabOrientation()` in `unregister()` ([IF:1029](../Apps/nepi_app_auto_move/api/auto_move_if.py#L1029)) too.

**The staleness rule.**

- A snapshot is usable while `time_s − timestamp_s` ≤ `STAB_OF_MAX_NAV_AGE_S` = 0.5 s (five periods at 10 Hz) and `orientation_valid` is True. Because `timestamp_s` is the read time, this also catches an NPX that keeps republishing frozen values.
- `timestamp_s` also picks the snapshot closest to an image's time (4.2). `receive_time_s` is for the readout only.
- Never use the `time_*` fields of the combined NavPose. They are `float32` (`NavPose.msg:37`), so a NEPI time near 1.8e9 s comes out in 128 s steps.

The read time cannot catch values the robot itself repeats inside fresh status messages. On this ROV that happens when the Teensy hangs on an IMU reset (SAFETY.md finding 3, [SAFETY.md](SAFETY.md#rov-26-findings)). `$TNCTL` stops then, and the thrusters hold their last pulse whatever NEPI sends, so no NEPI rule helps there. Section 6 lists it.

### 3.3 What reaches the method

The signature `method(stab_dict, robot_dict, controls_dict)` stays as it is ([stab:76](../Apps/nepi_app_auto_move/sdk/nepi_auto_move_stab.py#L76)). The new inputs travel as eight new `stab_dict` keys, all **(proposed)**, added where `updateStab` builds `stab_dict` today ([IF:2181–2186](../Apps/nepi_app_auto_move/api/auto_move_if.py#L2181-L2186)). Their exact definitions are the EXTENSION POINT text in 3.4. Their sources:

- `time_s` is `current_time`, already computed at [IF:2152](../Apps/nepi_app_auto_move/api/auto_move_if.py#L2152).
- `control_values` is `nepi_controls.get_values_dict(controls_dict)` (`nepi_controls.py:1015`).
- `flow_available`, `flow_topic`, `flow` and `flow_accepted` are `HAS_FLOW_MSG`, `stab_flow_topic`, `stab_flow_dict` and `stab_flow_accepted_dict` (3.1).
- `navpose_list` is `stab_navpose_list` (3.2).
- `method_state` is `stab_method_state_dict` (3.4).

`control_values` is needed because the stab module must not import `nepi_controls`. That module imports `nepi_sdk` (`nepi_controls.py:24`), which imports rospy, and the stab module's rule forbids that ([stab:96–98](../Apps/nepi_app_auto_move/sdk/nepi_auto_move_stab.py#L96-L98)). The live dict stores every value as a list of strings, as the planner's `_control_value` docstring explains ([sdk/nepi_auto_move.py:145–162](../Apps/nepi_app_auto_move/sdk/nepi_auto_move.py#L145-L162)). The IF has nepi_sdk anyway, so it converts once per cycle. Add `from nepi_sdk import nepi_controls` to the IF's imports.

### 3.4 Where a method's state lives

The stab module is pure, yet a hold needs a hold point, the last error and the last command between cycles. There are three options:

| Option | For | Against |
|---|---|---|
| **A. A per-run dict the IF passes in and recreates at every run start** | One line in the IF. The method reads and writes it in place. A test passes its own dict. The run-restart rule the EXTENSION POINT already defines resets it for free. | A method that raises halfway can leave it half-written, so a method writes its state at the end of a step. |
| B. The result hands state back, and the IF passes it in next cycle | A raise discards the cycle's changes | More plumbing, and every method has to return its state on every path |
| C. Module-level state | Nothing to change in the IF | Shared across callers and tests, survives a run restart, and breaks the pure-function rule |

**Use A.** In `updateStab`, set `self.stab_method_state_dict = {}` in both run-start branches, next to `self.stab_cycle_count = 0` ([IF:2156](../Apps/nepi_app_auto_move/api/auto_move_if.py#L2156), [:2168](../Apps/nepi_app_auto_move/api/auto_move_if.py#L2168)), and pass it as `stab_dict['method_state']`. The `method_state` name avoids a clash with the IF's own `stab_state` (off, running or error).

**The EXTENSION POINT text that changes.** Replace the `stab_dict` paragraph ([stab:82–90](../Apps/nepi_app_auto_move/sdk/nepi_auto_move_stab.py#L82-L90)) with:

```
#   stab_dict carries the loop's bookkeeping and the inputs the app gathered:
#       'method'         (str)   the method's registered display name
#       'cycle_count'    (int)   cycles since this run started, 0 on the first
#       'run_time_s'     (float) seconds since stabilization was turned on or the
#                                method last changed
#       'dt_s'           (float) seconds since the previous cycle, 0.0 on the first
#       'time_s'         (float) this cycle's NEPI time; every age is against it
#       'control_values' (dict)  {control name: native value} of the stab set
#       'flow_available' (bool)  the optical flow message package is installed
#       'flow_topic'     (str)   the flow data topic auto_move subscribes to
#       'flow'           (dict)  newest flow sample, or None
#       'flow_accepted'  (dict)  newest ACCEPTED flow sample, or None
#       'navpose_list'   (tuple) recent robot orientation snapshots, oldest first
#       'method_state'   (dict)  the method's own state, empty at every run start
#   A run restarts, and every counter and method_state with it, when Enable
#   goes on, when the method changes while on, and when a goto ends. A method
#   keeps state between cycles only in method_state, and writes it at the end
#   of a step, so a raise leaves the previous cycle's state.
```

and replace THE RESULT ([stab:105–114](../Apps/nepi_app_auto_move/sdk/nepi_auto_move_stab.py#L105-L114)) with the contract in 4.9.

### 3.5 Phase 2 acceptance

Build the inputs, but send nothing: no publisher yet. The method body for this phase only formats its inputs into `msg`. With the optical flow app and auto_move running, select the ROV in auto_move, choose Optical Flow, and turn Enable on:

1. `stab_msg` shows the flow `shift_x_px` and `shift_y_px`, the sample's age, inliers and `reference_id`, plus roll, pitch and yaw with their age by read time. Values change as you move a texture in front of the camera and tilt the ROV.
2. Disable the optical flow app: the flow age shown climbs past 0.3 s. Enable it again: the sample returns with a new `reference_id`.
3. Cover the lens: `flow` shows `accepted` false while `flow_accepted` stops changing.
4. Stop the ROV's telemetry while the driver keeps running (power the ROV Pi off): the orientation age climbs past 0.5 s even though the NPX keeps publishing. That is the frozen-data case the read time catches. Disable the driver: the messages stop and the age keeps climbing.
5. **The orientation topic check.** Check:
   - `rostopic list | grep npx/navpose/orientation` names `<node>/<dev>/npx/navpose/orientation`, and the auto_move log line `Registering to robot orientation topic: ...` (3.2, proposed) names the same one.
   - `rostopic echo -n 1` on it, twice, shows a `timestamp` that advances and is within a second of `date +%s` in the container, and roll, pitch and yaw that change when you tilt the ROV. `rostopic hz` shows about 10 Hz.

   If the topic is missing, the method runs without navpose, with Derotate off: record it for section 12. This check does not depend on the RBX navpose source (engine commit `2be5ec8`), which matters only to consumers of the combined NavPose (section 11, item 5).

---
## 4. Part 3: what the Optical Flow method does

### 4.1 What the camera can observe

The mount is unconfirmed (section 12). The method supports two. **Forward:** the optical axis along the bow, image right to starboard, image down to the keel. **Down:** the optical axis straight down, image top toward the bow, image right to starboard. The body frame is FLU, as in `NavPose.msg:31–35`: x forward, y left, z up. Positive roll lowers the right side, positive pitch puts the nose down, and positive yaw turns counter-clockwise seen from above. TEST_PLAN.md Part B checks exactly these signs ([TEST_PLAN.md](TEST_PLAN.md#part-b-the-imu-mount)). For small motions, with f the focal length in pixels, Z the range to the scene and h the height above the bottom, the frame-centre shift is:

| ROV motion | Forward camera | Down camera |
|---|---|---|
| Sway left, Δy | `shift_x = +f·Δy/Z` | `shift_x = +f·Δy/h` |
| Surge forward, Δx | none at the centre; `scale` grows | `shift_y = +f·Δx/h` |
| Heave up, Δz | `shift_y = +f·Δz/Z` | none at the centre; `scale` shrinks |
| Yaw left, Δψ | `shift_x = +f·tan Δψ` | none at the centre; `rotation_deg` ≈ −Δψ |
| Pitch nose down, Δθ | `shift_y = −f·tan Δθ` | `shift_y = −f·tan Δθ` |
| Roll right side down, Δφ | none at the centre; `rotation_deg` ≈ +Δφ | `shift_x = +f·tan Δφ` |

The couplings that matter:

- **Forward: yaw and sway** both move image x. A sway of Δy looks like a yaw of atan(Δy/Z). At 2 m, 1° of yaw looks like 3.5 cm of sway.
- **Forward: pitch and heave** both move image y.
- **Forward: surge is invisible to translation.** Moving along the optical axis expands the scene about the image centre, and the tracker reports the centre's displacement ([tracker:150–153](../Apps/nepi_app_optical_flow/scripts/optical_flow.py#L150-L153)). Only `scale` sees surge.
- **Down:** pitch looks like surge, and roll looks like sway.

### 4.2 What navpose adds: derotation

Derotation subtracts the shift that the ROV's own rotation caused, so only translation is left. The focal length is `f = (width_px / 2) / tan(hfov / 2)`. For 640 px at the default `stab_of_hfov_deg` of 110°, which is auto_move's own default for an unknown camera (`DEFAULT_WIDTH_DEG`, [IF:171](../Apps/nepi_app_auto_move/api/auto_move_if.py#L171)), f = 224 px, and 1° of rotation is 3.9 px. With Δ the snapshot's attitude minus the attitude stored at the hold point:

```
Forward:  error_x = (shift_x - hold_x) - f·tan(Δyaw)     error_y = (shift_y - hold_y) + f·tan(Δpitch)
Down:     error_x = (shift_x - hold_x) - f·tan(Δroll)    error_y = (shift_y - hold_y) + f·tan(Δpitch)
```

Each term cancels the matching row of the table in 4.1, so a pure rotation leaves an error of 0. The fixtures in 8.1 check this: a 5° yaw to the left shifts the forward image 19.6 px, the derotated error is 0.0, and with Derotate off the same turn commands sway −0.098. Wrap Δyaw to ±180°. The orientation topic carries no frame name: its angles are in the NPX NavPoseIF's `frame_nav` setting, `ENU` by default (`nepi_nav.py:143`, converted at `system_if.py:596–604`), whose counter-clockwise yaw is the sign used here. A `NED` setting would flip yaw alone (`NavPose.msg:33–35`), and the derotation bench test (8.3) shows it as a doubled error.

- **Angles, not rates.** The shift accumulates from the reference, so what has to come out is the attitude change since the hold point. Integrating rates over each frame interval would drift. Rates are used only as a gate.
- **The two clocks.** The image time is the frame's NEPI arrival time ([idx_net_stream_driver.py:380](../Drivers/idx_drivers/idx_net_stream_driver.py#L380)), and the orientation `timestamp` is the driver's read time (3.2). They share one clock, and both are late by amounts nobody has measured: the stream latency for the image, and the link latency for the orientation. The method uses the snapshot closest to the image time, within `STAB_OF_MAX_NAV_ALIGN_S` = 0.25 s. The derotation bench test (8.3) shows what is left. If the derotated error jumps during a turn and settles after it, the two lag by different amounts: record the difference rather than tuning a delay in the first version.
- **Depth does not help here.** Depth below the surface is not the range to what a forward camera sees. A down camera needs the height above the bottom, which would also need the bottom's depth. The plan does not read depth. If the operator wants it in `stab_msg`, the NPX's `<node>/<dev>/npx/navpose/depth` topic (`NavPoseDepth`) carries its read time the same way and can be added as in 3.2.

**The first version runs without navpose.** Derotate defaults off. Turn it on once the bench test passes and `imu_mount` is measured (TEST_PLAN.md Part B; the board lies on its side, [mate_coral_crusaders.yaml:168–172](../Nepi_Connect/vehicles/mate_coral_crusaders.yaml#L168-L172)). With Derotate off, the method holds on raw flow and still applies the tilt and rate gates whenever navpose is usable. With Derotate on and navpose missing, it commands neutral and says so. Expect derotation to matter for the forward mount: the Teensy's heading hold is unreliable with the IMU on its side ([SAFETY.md finding 6](SAFETY.md#rov-26-findings)), and without derotation, heading wander becomes sway commands.

### 4.3 Which axes the first version controls

| Mount | NEPI controls | Left at 0 for the Teensy |
|---|---|---|
| Forward | Sway | Surge (not observable), heave, yaw |
| Down | Surge and sway | Heave, yaw |

- **Surge and sway are open-loop power** ([mate_coral_crusaders.yaml:72–74](../Nepi_Connect/vehicles/mate_coral_crusaders.yaml#L72-L74)). Nothing closes a loop on them, which is what a position hold is for.
- **Heave:** while NEPI's heave is 0, the Teensy holds depth from its depth sensor ([:75–77](../Nepi_Connect/vehicles/mate_coral_crusaders.yaml#L75-L77)), a direct measurement. Image y mixes heave with pitch (forward) or does not see heave at all (down).
- **Yaw:** while NEPI's yaw is 0, the Teensy holds its heading. NEPI's smallest nonzero rate is about 7 deg/s ([:78–81](../Nepi_Connect/vehicles/mate_coral_crusaders.yaml#L78-L81)), too coarse for a fine hold, and the yaw cap stays at 0 until SAFETY.md finding 1 is fixed.
- **The ESC deadband sets the floor.** The smallest useful demand is 0.052 for surge and 0.087 for sway in normal mode, and sway does nothing in slow mode ([SAFETY.md](SAFETY.md#the-esc-deadband-and-the-smallest-useful-demand)). At stage 4's cap of 0.1, sway has only 0.087 to 0.1 to work with, so it acts nearly on/off until its cap is raised.

### 4.4 The sign chain

Worked example: forward camera, scene 2 m ahead, f = 224 px, the starting gains of 4.7. The ROV drifts 0.2 m to port.

| Step | Value | Source of the sign |
|---|---|---|
| 1. Image | Content moves right by f·Δy/Z = 224 × 0.2 / 2 = 22.4 px | 4.1 |
| 2. Tracker and message | `dx` = `shift_x_px` = +22.4 (the telemetry's `y` would read −22.4) | The 2.3 test: content right gives `dx` > 0 |
| 3. Error | `error_x` = 22.4 − 0 = +22.4 px, outside the 5 px deadband | Hold point at 0 |
| 4. FLU command | `y` = −0.005 × 22.4 = −0.112, clamped to −0.1, reached in two cycles by the slew limit: starboard | Method; fixtures in 8.1 |
| 5. Protocol | sway = −linear_y = +0.1, right | [driver:296–297](../Drivers/rbx_drivers/rbx_nepi_connect_node.py#L296-L297), [:320–321](../Drivers/rbx_drivers/rbx_nepi_connect_node.py#L320-L321) |
| 6. ROV | Under the 0.2 cap; the vehicle file's sway sign (+1, not yet measured, [mate_coral_crusaders.yaml:94](../Nepi_Connect/vehicles/mate_coral_crusaders.yaml#L94)) makes it the pilot's "right"; 0.1 is above the 0.087 floor, so the ROV pushes to starboard | TEST_PLAN.md Part A |

Down camera: a 0.1 m drift forward at h = 1 m moves the content down 22.4 px, so `error_y` = +22.4 and `x` = −0.1. The driver sends surge = linear_x = −0.1 (aft), and the measured surge sign −1 ([mate_coral_crusaders.yaml:93](../Nepi_Connect/vehicles/mate_coral_crusaders.yaml#L93)) turns that into the pilot's back stick.

**The bench sign test** (Phase 4, before a thruster turns). ESC supply off, ROV on the bench, camera streaming, the pilot's controller connected:

1. In pilot mode, push the stick for "right", and note which `$TNCTL` horizontal-thruster columns (5 to 8) go up and which go down. Do the same for "back".
2. Press RB (NEPI control on). Turn Enable on with Hold Sway on.
3. Slide a printed texture slowly to the right in front of the camera. That is what the ROV drifting to port looks like.
4. Expect `stab_msg` error x positive, `rostopic echo <rbx namespace>/set_teleop_velocity` showing `linear.y` negative, and the same `$TNCTL` pattern as the pilot's "right".
5. Down mount: turn Hold Surge on, and slide the texture toward the bottom of the image. Expect `linear.x` negative and the pilot's "back" pattern.

If step 4 or 5 shows the opposite pattern, stop. Do not flip a sign in the method: the method's sign is fixed by the tracker test and the FLU convention. Check the vehicle file's sway or surge sign with TEST_PLAN.md Part A first.

### 4.5 The gates

The method checks these in order, every cycle. "Nothing" means `cmd` is `None`: the method sends nothing, and the IF sends one neutral if it was commanding (5.2). "Neutral" means `cmd` is all zeros. With no axis switched on (the dry run), each Neutral row returns nothing instead, and its message starts with `DRY RUN`. Every row clears the hold point.

| Condition | Command | `stab_msg` starts with |
|---|---|---|
| Robot not connected | Nothing | `WAITING: no robot connected` |
| `teleop_control_mode_ready` not True: NEPI control off, or `teleop_movement_enabled` not `TRUE` ([driver:1270–1278](../Drivers/rbx_drivers/rbx_nepi_connect_node.py#L1270-L1278)) | Nothing | `WAITING for NEPI control` |
| Flow message package not installed | Neutral | `NEUTRAL: no flow source` |
| Newest accepted sample older than `STAB_OF_MAX_FLOW_AGE_S` = 0.3 s, with fresh held samples arriving | Neutral | `NEUTRAL: tracking lost` |
| No sample younger than 0.3 s at all: app stopped, camera frozen | Neutral | `NEUTRAL: no fresh flow` |
| Newest accepted sample has fewer than `STAB_OF_MIN_INLIERS` = 20 inliers | Neutral | `NEUTRAL: low inliers` |
| `reference_id` changed, or Mount, Derotate or an axis toggle changed | Neutral | `NEUTRAL: reference or settings changed` |
| Derotate on and no usable snapshot: none read within the last `STAB_OF_MAX_NAV_AGE_S` = 0.5 s (by its read time, so frozen telemetry counts as old), or none within 0.25 s of the image time | Neutral | `NEUTRAL: Derotate is on but navpose...` |
| Navpose usable and any angular rate above `STAB_OF_MAX_RATE_DEGPS` = 30 deg/s | Neutral | `NEUTRAL: rotating` |
| Navpose usable and roll or pitch more than `STAB_OF_MAX_TILT_DEG` = 10° from the hold point | Neutral | `NEUTRAL: tilted` |

- **The 0.3 s age limit** sits under the driver's 0.5 s command timeout ([driver:145](../Drivers/rbx_drivers/rbx_nepi_connect_node.py#L145)), so NEPI goes neutral on its own first, and it covers three frames at 10 fps. If the app runs below about 7 fps this gate trips constantly: raise the frame rate before raising the limit. Single held frames, such as blur, do not trip it as long as an accepted sample is younger than 0.3 s.
- **20 inliers** is a quarter of the tracker's 80 best matches. The tracker's own floor is 8, with a 0.35 ratio ([tracker:22–23](../Apps/nepi_app_optical_flow/scripts/optical_flow.py#L22-L23), [:121](../Apps/nepi_app_optical_flow/scripts/optical_flow.py#L121)).
- **The 10° and 30 deg/s limits** keep the hold inside the small-angle range the derotation assumes. They are guesses: check them against the Phase 5 data.

### 4.6 The hold point

The hold point is the shift (and, with navpose, the attitude) the method holds the ROV to.

- **Captured on the first cycle in which every gate passes with teleop ready.** That cycle commands neutral.
- **Cleared whenever teleop is not ready.** So if Enable is turned on before the pilot presses RB, the hold is captured at the handover, not where the ROV was when Enable went on. Every handover starts fresh.
- **Cleared by every gate, and recaptured at the first good cycle after it.** NEPI never drives back to a point it stopped tracking. Held frames younger than 0.3 s are not a gate, so a few blurred frames do not move the hold.
- **Cleared when `reference_id` changes**, and when Mount, Derotate or an axis toggle changes.
- **Gone at every run start**, because `method_state` is recreated then (3.4).

### 4.7 The control law and starting gains

For each controlled axis, with `e` = `error_x` for sway and `error_y` for surge:

1. **Deadband:** if |e| ≤ `stab_of_deadband_px`, the target is 0.
2. **P and D:** target = −(kp·e + kd·ė). ė comes from consecutive accepted samples, divided by the difference of their image times. It updates only when a new accepted sample arrives with 0 < Δt ≤ 0.5 s, and resets with the hold point.
3. **Clamp** to ±`stab_of_max_cmd`, at or below the vehicle cap. The ROV Pi caps again.
4. **Slew limit:** the command moves at most `STAB_OF_SLEW_PER_SEC` × dt (0.5/s, so 0.05 per cycle). A gate drops to neutral at once, with no slew.
5. **No I term in the first version.** If Phase 5 shows a steady offset, from a current for example, add `stab_of_ki` with anti-windup. Integrate only while the gates pass, |e| is outside the deadband and the output is not clamped. Limit the I term to half of `stab_of_max_cmd`, and reset it with the hold point.

Starting values. They are a starting point for tuning, not results:

- **kp 0.005 per px:** the 0.1 limit at 20 px of error. With f = 224 px, 20 px is 0.18 m at 2 m range, or 0.09 m at 1 m. Thrust only begins above the floors in 4.3: 17 px for sway and 10 px for surge. A pixel gain weakens with range on its own, because a farther scene gives fewer pixels per metre.
- **kd 0 at first.** When P alone overshoots, try 0.0025 s/px: a drift of 20 px/s then adds 0.05.
- **Deadband 5 px.** Set it above the drift Phase 2 shows with the ROV still for 60 s.
- **Max command 0.1:** the stage 4 cap ([TEST_PLAN.md stage 4](TEST_PLAN.md#stage-4-in-water-on-the-tether)). Raise it only with the vehicle cap.

### 4.8 The new controls

Nine controls, all **(proposed)**, added to `STAB_CONTROLS_DICT` after `stab_enabled` ([stab:39–54](../Apps/nepi_app_auto_move/sdk/nepi_auto_move_stab.py#L39-L54)), in the same init-dict form. Each name is unique against the app's existing controls: `max_step_m`, `standoff_m`, `clearance_m`, `hold_altitude`, `face_target`, `avoid_obstacles`, `stab_method` and `stab_enabled`. The RUI needs no edit, because `NepiIFControls` draws whatever controls the set reports ([rui/NepiAppAutoMove.js:698–702](../Apps/nepi_app_auto_move/rui/NepiAppAutoMove.js#L698-L702)).

| Name | Display name | Type | Default | Range | Saved |
|---|---|---|---|---|---|
| `stab_of_hold_sway` | Hold Sway | Toggle | False | | No: `'param': False` |
| `stab_of_hold_surge` | Hold Surge (Down mount) | Toggle | False | | No: `'param': False` |
| `stab_of_camera_mount` | Camera Mount | Selection | `Forward` | `Forward`, `Down` | Yes |
| `stab_of_hfov_deg` | Camera HFOV (deg) | Float | 110.0 | 20.0 to 180.0 | Yes |
| `stab_of_derotate` | Derotate | Toggle | False | | Yes |
| `stab_of_kp` | Kp (per px) | Float | 0.005 | 0.0 to 0.05 | Yes |
| `stab_of_kd` | Kd (s per px) | Float | 0.0 | 0.0 to 0.05 | Yes |
| `stab_of_deadband_px` | Deadband (px) | Float | 5.0 | 0.0 to 50.0 | Yes |
| `stab_of_max_cmd` | Max Command | Float | 0.1 | 0.0 to 0.2 | Yes |

The two axis toggles are what start motion once Enable is on, so they carry `'param': False`, the 2026-11 decision: every session starts as a dry run, and the operator switches on one axis at a time, as the in-water steps do. The rest change how the method moves, not whether it moves, so they are saved like the planner's controls. `round_value` follows `max_step_m`'s form: 4 for the gains, 2 for the rest. Defaults sit inside their bounds, because `create_controls_dict` silently clamps them otherwise ([sdk/nepi_auto_move.py:36–41](../Apps/nepi_app_auto_move/sdk/nepi_auto_move.py#L36-L41)).

### 4.9 The result contract

This is the definition the EXTENSION POINT asks the first real method to make ([stab:111–114](../Apps/nepi_app_auto_move/sdk/nepi_auto_move_stab.py#L111-L114)). Replace THE RESULT with:

```
# THE RESULT
#   Return a dict:
#     'msg' (str)  copied into stab_msg; up to four lines show in the RUI.
#     'cmd'        None: send nothing this cycle. If the app was commanding,
#                  it sends one neutral, then nothing.
#                  Or a dict of RATIOS from -1 to 1 in the body FLU frame of
#                  set_teleop_velocity (geometry_msgs/Twist):
#                    'x'   forward         -> linear.x
#                    'y'   LEFT            -> linear.y
#                    'z'   up              -> linear.z
#                    'yaw' counter-clockwise from above -> angular.z
#                  A missing key is 0.0. All zeros is neutral: the robot's own
#                  holds act. The app sends it every cycle while the robot is
#                  connected and teleop_control_mode_ready is True, so the
#                  driver's command timeout never fires during a hold.
#   A method that raises puts the loop in the error state; the app sends one
#   neutral, Enable stays on, and the next cycle retries.
```

Use `None` when the method has no business commanding: no robot, NEPI control off, or a dry run. That leaves the robot to its own holds and to the pilot. Use neutral when NEPI has the robot and the method is the reason it should not move. Neutral takes effect at once instead of after the driver's 0.5 s timeout, and keeps NEPI control alive.

### 4.10 The method, step by step

The sketch below ran against the fixtures of 8.1 on 2026-11-12, and again on 2026-11-13 after the navpose change (python3 -B, no ROS). It is a starting point to adapt, not reviewed production code. It replaces the placeholder body of `stab_optical_flow` ([stab:117–141](../Apps/nepi_app_auto_move/sdk/nepi_auto_move_stab.py#L117-L141)), and the registry entry stays ([stab:146–148](../Apps/nepi_app_auto_move/sdk/nepi_auto_move_stab.py#L146-L148)). The `STAB_OF_*` constants, `NEUTRAL_CMD`, `_wrapDeg`, `_navAt` and `_axisCmd` are **(proposed)**. The public method has a docstring. The private helpers are camelCase with a leading underscore and no docstring.

```python
# Sketch: added to sdk/nepi_auto_move_stab.py, which keeps importing only
# copy and math. Constants and helpers sit above stab_optical_flow().
import math

STAB_OF_MAX_FLOW_AGE_S = 0.3     # section 4.5 has the reason for each value
STAB_OF_MIN_INLIERS = 20
STAB_OF_MAX_NAV_AGE_S = 0.5
STAB_OF_MAX_NAV_ALIGN_S = 0.25
STAB_OF_MAX_TILT_DEG = 10.0
STAB_OF_MAX_RATE_DEGPS = 30.0
STAB_OF_SLEW_PER_SEC = 0.5

NEUTRAL_CMD = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0}


def _wrapDeg(angle_deg):
    return (angle_deg + 180.0) % 360.0 - 180.0


def _navAt(nav_list, image_time_s, now_s):
    if len(nav_list) == 0 or now_s - nav_list[-1]['timestamp_s'] > STAB_OF_MAX_NAV_AGE_S:
        return None
    best = min(nav_list, key=lambda nav: abs(nav['timestamp_s'] - image_time_s))
    if best['orientation_valid'] != True or abs(best['timestamp_s'] - image_time_s) > STAB_OF_MAX_NAV_ALIGN_S:
        return None
    return best


def _axisCmd(error_px, rate_pxps, last_cmd, values, dt_s):
    target = 0.0
    if abs(error_px) > float(values.get('stab_of_deadband_px', 5.0)):
        target = -(float(values.get('stab_of_kp', 0.005)) * error_px +
                   float(values.get('stab_of_kd', 0.0)) * rate_pxps)
    max_cmd = float(values.get('stab_of_max_cmd', 0.1))
    target = max(-max_cmd, min(max_cmd, target))
    step = STAB_OF_SLEW_PER_SEC * (dt_s if dt_s > 0.0 else 0.1)
    return max(last_cmd - step, min(last_cmd + step, target))


def stab_optical_flow(stab_dict, robot_dict, controls_dict):
    """Hold position from optical flow; see OPTICAL_FLOW_STAB_PLAN.md section 4.

    Returns:
        dict: 'msg' (str) and 'cmd' (None to send nothing, or body FLU ratios).
    """
    values = stab_dict.get('control_values') or {}
    state = stab_dict.get('method_state')
    if state is None:
        state = {}
    now_s = float(stab_dict.get('time_s', 0.0))
    mount = str(values.get('stab_of_camera_mount', 'Forward'))
    derotate = (values.get('stab_of_derotate', False) == True)
    hold_y = (values.get('stab_of_hold_sway', False) == True)
    hold_x = (values.get('stab_of_hold_surge', False) == True) and mount == 'Down'
    signature = (mount, derotate, hold_x, hold_y)
    gate_cmd = dict(NEUTRAL_CMD) if (hold_x or hold_y) else None

    # 1. Robot gates: send nothing, and forget the hold point.
    status_dict = robot_dict.get('status_dict') or {}
    if robot_dict.get('connected') != True:
        state.clear()
        return {'msg': 'WAITING: no robot connected', 'cmd': None}
    if status_dict.get('teleop_control_mode_ready') != True:
        state.clear()
        return {'msg': 'WAITING for NEPI control: the pilot presses RB '
                       '(teleop_movement_enabled must be TRUE)', 'cmd': None}

    # 2. Flow and navpose gates: neutral, and forget the hold point.
    flow = stab_dict.get('flow')
    sample = stab_dict.get('flow_accepted')
    reason = None
    if stab_dict.get('flow_available') != True:
        reason = 'no flow source: the optical flow app is not installed'
    elif sample is None or now_s - sample['receive_time_s'] > STAB_OF_MAX_FLOW_AGE_S:
        if flow is not None and now_s - flow['receive_time_s'] <= STAB_OF_MAX_FLOW_AGE_S:
            reason = 'tracking lost, inliers ' + str(flow['inliers'])
        else:
            reason = 'no fresh flow on ' + str(stab_dict.get('flow_topic'))
    elif sample['inliers'] < STAB_OF_MIN_INLIERS:
        reason = 'low inliers ' + str(sample['inliers'])
    nav = None
    hold = state.get('hold')
    if reason is None:
        nav = _navAt(stab_dict.get('navpose_list') or (), sample['image_time_s'], now_s)
        if hold is not None and (hold['reference_id'] != sample['reference_id'] or
                                 hold['signature'] != signature):
            reason = 'reference or settings changed, recapturing'
        elif derotate and nav is None:
            reason = 'Derotate is on but navpose is missing, stale or unaligned'
        elif nav is not None and max(abs(nav['roll_deg_per_sec']), abs(nav['pitch_deg_per_sec']),
                                     abs(nav['yaw_deg_per_sec'])) > STAB_OF_MAX_RATE_DEGPS:
            reason = 'rotating faster than ' + str(STAB_OF_MAX_RATE_DEGPS) + ' deg/s'
        elif (nav is not None and hold is not None and hold['nav'] is not None and
              max(abs(nav['roll_deg'] - hold['nav']['roll_deg']),
                  abs(nav['pitch_deg'] - hold['nav']['pitch_deg'])) > STAB_OF_MAX_TILT_DEG):
            reason = 'tilted more than ' + str(STAB_OF_MAX_TILT_DEG) + ' deg since the hold'
    if reason is not None:
        state.clear()
        prefix = 'NEUTRAL: ' if gate_cmd is not None else 'DRY RUN, would be neutral: '
        return {'msg': prefix + reason, 'cmd': gate_cmd}

    # 3. Hold point: captured on the first good cycle, which sends neutral.
    if hold is None:
        state['hold'] = {'x': sample['shift_x_px'], 'y': sample['shift_y_px'],
                         'reference_id': sample['reference_id'], 'signature': signature,
                         'nav': nav}
        state['last'] = None
        state['cmd'] = dict(NEUTRAL_CMD)
        return {'msg': 'HOLD captured, reference ' + str(sample['reference_id']), 'cmd': gate_cmd}

    # 4. Error in pixels, derotated when asked. Signs: section 4.2.
    error_x = sample['shift_x_px'] - hold['x']
    error_y = sample['shift_y_px'] - hold['y']
    if derotate:
        hfov_rad = math.radians(float(values.get('stab_of_hfov_deg', 110.0)))
        focal_px = (sample['width_px'] / 2.0) / math.tan(hfov_rad / 2.0)
        d_yaw = _wrapDeg(nav['yaw_deg'] - hold['nav']['yaw_deg'])   # ENU: CCW positive (4.2)
        d_pitch = nav['pitch_deg'] - hold['nav']['pitch_deg']
        d_roll = nav['roll_deg'] - hold['nav']['roll_deg']
        side_deg = d_yaw if mount == 'Forward' else d_roll
        error_x = error_x - focal_px * math.tan(math.radians(side_deg))
        error_y = error_y + focal_px * math.tan(math.radians(d_pitch))

    # 5. Rates from consecutive accepted samples, on image time.
    last = state.get('last')
    rate_x = state.get('rate_x', 0.0)
    rate_y = state.get('rate_y', 0.0)
    if last is not None and 0.0 < sample['image_time_s'] - last['t'] <= 0.5:
        dt_image = sample['image_time_s'] - last['t']
        rate_x = (error_x - last['x']) / dt_image
        rate_y = (error_y - last['y']) / dt_image
    if last is None or sample['image_time_s'] != last['t']:
        state['last'] = {'t': sample['image_time_s'], 'x': error_x, 'y': error_y}
    state['rate_x'] = rate_x
    state['rate_y'] = rate_y

    # 6. Commands for every observable axis; only the switched-on ones are sent.
    # Image x is sway for both mounts; image y is surge for Down only.
    dt_s = float(stab_dict.get('dt_s', 0.1))
    last_cmd = state.get('cmd') or dict(NEUTRAL_CMD)
    preview = dict(NEUTRAL_CMD)
    preview['y'] = _axisCmd(error_x, rate_x, last_cmd['y'], values, dt_s)
    if mount == 'Down':
        preview['x'] = _axisCmd(error_y, rate_y, last_cmd['x'], values, dt_s)
    state['cmd'] = preview
    cmd = dict(NEUTRAL_CMD)
    cmd['x'] = preview['x'] if hold_x else 0.0
    cmd['y'] = preview['y'] if hold_y else 0.0
    msg = ('HOLD %s: error x %.1f y %.1f px%s\ncmd surge %.3f sway(left) %.3f\n'
           'flow age %.2f s, inliers %d, ref %d\n' % (
               mount, error_x, error_y, ' (derotated)' if derotate else '', preview['x'],
               preview['y'], now_s - sample['receive_time_s'], sample['inliers'],
               sample['reference_id']))
    msg += ('nav r %.1f p %.1f y %.1f' % (nav['roll_deg'], nav['pitch_deg'], nav['yaw_deg'])
            if nav is not None else 'no navpose')
    if gate_cmd is None:
        return {'msg': 'DRY RUN, no axis on\n' + msg, 'cmd': None}
    return {'msg': msg, 'cmd': cmd}
```

## 5. Part 4: sending the commands (the AutoMoveIF change)

### 5.1 Where the command goes

`ConnectRBXDeviceIF` offers the goto commands, `go_stop` and, since 2026-11-13, `set_teleop_velocity` (`connect_device_if_rbx.py:632–734`). It re-registers every one of its publishers on the selected robot's namespace whenever the selection changes (`subscribe_topic`, `connect_device_if_rbx.py:883`, its publisher dict at `:921–1057`, registered at `:1074`).

**Decided and added 2026-11-13: `set_teleop_velocity` on `ConnectRBXDeviceIF`, called from auto_move.** It is an engine change, so it reaches the robot's NEPI computer only with an engine build and deploy, before Phase 3 (section 9). Because the publisher sits in the connect's per-robot dict, the command follows the robot selection with no code in auto_move, and auto_move has no publisher of its own to build or unregister. The engine change as made:

```python
# As added (fragment), connect_device_if_rbx.py. Twist is imported from geometry_msgs.msg (:32).
# In subscribe_topic's connect_topic_pubs_dict, after 'goto_velocity' (:1007-1012):
            'set_teleop_velocity': {
                'namespace': self.selected_topic,
                'topic': 'set_teleop_velocity',
                'msg': Twist,
                'qsize': 1,
            },

# After goto_velocity (:691-712):
    def set_teleop_velocity(self, linear_x, linear_y, linear_z, angular_z):
        """Send one teleop velocity command to the selected RBX device.

        Ratios from -1 to 1 in the body FLU frame, passed through unchanged to
        the device's set_teleop_velocity topic: x forward, y left, z up, and
        angular z counter-clockwise. The device acts on it only while its
        teleop controls are ready, and a driver may time a command out, so a
        caller holding a velocity sends it every cycle.

        Args:
            linear_x (float): Forward ratio, -1 to 1.
            linear_y (float): Left ratio, -1 to 1.
            linear_z (float): Up ratio, -1 to 1.
            angular_z (float): Counter-clockwise turn ratio, -1 to 1.
        """
        pub_name = 'set_teleop_velocity'
        msg = Twist()
        msg.linear.x = linear_x
        msg.linear.y = linear_y
        msg.linear.z = linear_z
        msg.angular.z = angular_z
        self.node_if.publish_pub(pub_name, msg)
```

It follows `goto_velocity` line for line, and the topic, message and frame are the ones `RBXRobotIF` already subscribes (`device_if_rbx.py:722`, `:1417–1428`). On the auto_move side the new names, all **(proposed)**, are the methods `sendStabCmd` and `sendStabNeutral` and the attribute `stab_cmd_sent = False`. They are private, so they are camelCase with no docstring.

### 5.2 When it publishes

```python
# Sketch (fragment), the IF. Both run with stab_lock held.
    def sendStabCmd(self, cmd_dict, robot_dict):
        status_dict = robot_dict.get('status_dict') or {}
        if (cmd_dict is None or robot_dict.get('connected') != True or
                status_dict.get('teleop_control_mode_ready') != True):
            self.sendStabNeutral()
            return
        values = [float(cmd_dict.get(key, 0.0)) for key in ['x', 'y', 'z', 'yaw']]
        if not all(math.isfinite(value) for value in values):
            raise ValueError('Stabilization command is not a finite number: ' + str(cmd_dict))
        values = [max(-1.0, min(1.0, value)) for value in values]
        if not hasattr(self.rbx_if, 'set_teleop_velocity'):
            raise RuntimeError('This engine has no ConnectRBXDeviceIF.set_teleop_velocity (plan section 11)')
        self.rbx_if.set_teleop_velocity(values[0], values[1], values[2], values[3])
        self.stab_cmd_sent = True

    def sendStabNeutral(self):
        if self.stab_cmd_sent == True and hasattr(self.rbx_if, 'set_teleop_velocity'):
            self.rbx_if.set_teleop_velocity(0.0, 0.0, 0.0, 0.0)
        self.stab_cmd_sent = False
```

On an engine built before the method was added, `sendStabCmd` raises a plain error that the loop shows as `stab_state` error, and `sendStabNeutral` does nothing, so nothing crashes.

- **Call `sendStabCmd(result.get('cmd'), robot_dict)`** right after `self.stab_msg = str(result['msg'])` ([IF:2192](../Apps/nepi_app_auto_move/api/auto_move_if.py#L2192)). Call `getRobotDict()` once, before `runStabMethod` ([IF:2188–2191](../Apps/nepi_app_auto_move/api/auto_move_if.py#L2188-L2191)), and pass the same dict to both. An exception from `sendStabCmd` lands in the existing error path ([IF:2196–2202](../Apps/nepi_app_auto_move/api/auto_move_if.py#L2196-L2202)), which must also call `sendStabNeutral()`.
- **Publish only while connected and `teleop_control_mode_ready` is True.** Otherwise the driver drops the command and rewrites its error message on every one (`device_if_rbx.py:1411–1416`).
- **Publish every cycle while the result carries a command.** At 10 Hz that is five times inside the driver's 0.5 s timeout, so a live hold never times out.
- **Send one neutral when stabilization stops, fails or loses its gate.** That covers Enable going off (the off branch, [IF:2161–2165](../Apps/nepi_app_auto_move/api/auto_move_if.py#L2161-L2165)), a method change, a raise, a result of `None`, and a goto starting. `sendStabNeutral` sends it only if a command went out since the last neutral, so it costs nothing when idle.
- **A robot change sends nothing.** The connect already points at the new robot by the time `updateRobotState` sees the change ([IF:1127–1137](../Apps/nepi_app_auto_move/api/auto_move_if.py#L1127-L1137)), so a neutral would go there. In the `selection_changed` branch, take `stab_lock`, set `stab_cmd_sent = False` and restart the run (`stab_cycle_count = 0`, a new `stab_method_state_dict`). The robot being left times out its last command itself: 0.5 s on this ROV ([driver:145](../Drivers/rbx_drivers/rbx_nepi_connect_node.py#L145)).
- **Neutral is `set_teleop_velocity(0, 0, 0, 0)`, a zero Twist, not `go_stop`.** On this driver `go_stop` also clears the stored command ([driver:1285–1288](../Drivers/rbx_drivers/rbx_nepi_connect_node.py#L1285-L1288)), but on other robots it is that driver's own stop. A zero Twist means "no teleop demand" on every robot.

### 5.3 Gotos and the Robots page Teleop

**Gotos.** No goto can run on this ROV, but auto_move also runs on robots that have them. While `goto_state` is `planning` or `moving`, `updateStab` does not run the method. It sets `stab_msg` to `Paused: a goto is running` and calls `sendStabNeutral()`. When the goto ends, it restarts the run (`stab_cycle_count = 0`, a new `stab_method_state_dict`), so the hold is recaptured where the goto left the robot.

**Robots page Teleop** publishes on the same topic, resending a held key every 200 ms ([driver:141–145](../Drivers/rbx_drivers/rbx_nepi_connect_node.py#L141-L145)). The driver keeps whichever command arrived last, so the two would take turns, and nothing arbitrates. Until the team decides who owns Teleop (section 12), the rule is: do not use Robots page Teleop while Enable is on. An optional, cheap warning: once a second in `updateRobotState` (not under the lock), call `nepi_sdk.find_publishers(topic)` (`nepi_sdk.py:1976`) and put a line in `stab_msg` when a node other than auto_move publishes there. rosbridge stays listed once the page has sent Teleop, so make it a warning, not a gate.

### 5.4 The teleop log line

`setTeleopVelocityCb` logs `Received teleop velocity message` with `pub_info` on every message, unthrottled (`device_if_rbx.py:1410`): ten lines a second into the driver's log and the RUI message stream while a hold runs. The app cannot throttle another node's log, and it cannot publish less often without risking the 0.5 s timeout. Live with it until the engine fix (section 11), and filter it out when reading logs.

### 5.5 Status

`stab_msg` carries everything these phases need, as four lines, on the 1 s status publish and on every transition. Add no status field. Each one costs a catkin build, an RUI edit and an RUI build, and the RUI already shows `stab_msg` ([rui/NepiAppAutoMove.js:706–710](../Apps/nepi_app_auto_move/rui/NepiAppAutoMove.js#L706-L710)). If Phase 5 wants plots, a `Twist stab_cmd` field would be the first to add.

---
## 6. Safety and failure behavior

The ROV-side rules referred to below are in [SAFETY.md, "What Our Code Enforces"](SAFETY.md#what-our-code-enforces).

| Situation | What the ROV does | What the operator sees |
|---|---|---|
| Enable turned off | One neutral at once. The Teensy holds depth and heading. NEPI keeps control, sending neutral, until the pilot presses RB or pushes a stick past the takeover point. | `stab_state` off |
| The pilot takes the ROV back | The pilot's input goes through at once (robot side). Teleop is no longer ready, so the method sends nothing and drops its hold point. | `WAITING for NEPI control` |
| The link drops | The driver stops sending demands after 0.5 s. The ROV Pi goes neutral 0.25 s after the last demand and ends NEPI control after 2 s (rules 4 to 6). | `WAITING` lines; the robot shows disconnected |
| auto_move crashes or restarts mid-hold | Commands stop. The driver turns the stale command into neutral after 0.5 s ([driver:145](../Drivers/rbx_drivers/rbx_nepi_connect_node.py#L145)) and keeps sending neutral demands, so the Teensy holds. | After the restart: Enable off, both axis toggles off |
| The optical flow app stops | Neutral within 0.3 s; hold point dropped | `NEUTRAL: no fresh flow` |
| The camera freezes | If frames stop: as above. If a source repeats the same picture with new stamps, the tracker sees no motion, and the ROV drifts unobserved. | Nothing in `stab_msg` in the first version; the operator watches the video |
| Tracking lost: blur, silt, featureless water | Held samples, then neutral once the newest accepted sample is 0.3 s old | `NEUTRAL: tracking lost` |
| A reference reset (button, k1, mask, camera, resolution) | Neutral for one cycle, then a new hold point where the ROV is | `NEUTRAL: reference or settings changed`, then `HOLD captured` |
| Navpose missing, stale, or frozen by a telemetry stall (read time older than 0.5 s) | Derotate off: holds on raw flow, without tilt and rate gates. Derotate on: neutral. | `no navpose`, or `NEUTRAL: Derotate is on but navpose...` |
| Navpose with wrong signs (`imu_mount` wrong) | Derotation doubles the rotation instead of removing it: sway commands during turns, bounded by Max Command and the ROV Pi's cap | Caught by the derotation bench test (8.3); keep Derotate off until it passes |
| The method raises | Error state, one neutral, retried next cycle | `stab_state` error, with the exception |
| Enable after an app restart | Enable and both axis toggles come up off (`'param': False`). Nothing moves until the operator turns on Enable and an axis, and the pilot presses RB. | All off |
| Robots page Teleop used during a hold | The two sources alternate, and the ROV jerks | Rule in 5.3: do not |
| Robot selection changed mid-hold | No neutral reaches the robot being left, because the connect already points at the new one. Its driver times out the last command (0.5 s on this ROV). The run restarts and the hold point is dropped. | `WAITING` or `HOLD captured` on the new robot |

Two things no NEPI code can stop: the Teensy acting on its last frame if the Pi script or the USB link stops (finding 4), and a frozen loop after an IMU reset (finding 3). The supply switch is the stop for both.

---

## 7. Phased plan

| Phase | Work | Acceptance |
|---|---|---|
| 1. Data topic | 2.4 to 2.6 in the optical flow app | 2.8 |
| 2. Inputs in `stab_msg`, no commands | 3.1 to 3.4; the method only formats its inputs | 3.5, including the orientation topic check |
| 3. Dry run | First the engine method (section 11, item 1), built and deployed. Then the method (4.10), the controls (4.8), and the commands and their rules (5.1 to 5.3). Both axis toggles off. | NEPI control on, Enable on, no axis: `stab_msg` starts `DRY RUN` and shows the error and the command it would send. A texture moved as in 4.4 gives the signs of 4.4. `rostopic echo <rbx namespace>/set_teleop_velocity` shows nothing from auto_move. With Derotate on, 8.3 passes. |
| 4. Bench, thrusters unable to move the ROV | ESC supply off. The sign test (4.4), then every gate row of 4.5 by hand (8.4). | Each `$TNCTL` pattern matches the pilot's; each gate shows its message and one neutral on the topic |
| 5. In water, one axis at a time, then tuning | TEST_PLAN.md stage 4 with Hold Sway only at caps of 0.1, then Hold Surge on a down mount, then 4.7's tuning | Stage 4 row 4.4: less drift than with NEPI control off over 60 s, judged against a marker in the video, and no oscillation that grows |

---

## 8. Testing

### 8.1 Offline harness for the method (no ROS)

Put it at `Apps/nepi_app_auto_move/test/stab_optical_flow_test.py` **(proposed)**, with a first-line comment `# NOT DEPLOYED: offline stabilization harness`. It never reaches the robot: `deploy_app.sh` live-syncs only `scripts/`, `sdk/`, `api/` and `rui/` ([deploy_app.sh:144–214](../Apps/nepi_app_auto_move/deploy_app.sh#L144-L214)), and CMake installs nothing from `test/`. The stab module imports only `copy` and `math`, so import it by path with `importlib`, with no stubs, and run with `python3 -B` so no `__pycache__` lands in the repo.

Build three fixture helpers: a flow sample (`receive_time_s`, `image_time_s`, shifts, `reference_id`, `inliers`, `accepted`, 640×480), a navpose snapshot in `BLANK_STAB_NAVPOSE_DICT` form, and a `robot_dict` with `connected` and `status_dict['teleop_control_mode_ready']`. Drive the method cycle by cycle with one `method_state` dict, and assert `cmd` and the first word of `msg`. The 2026-11-12 run of the 4.10 sketch gave:

| Case | Result |
|---|---|
| Forward, Hold Sway, 22.4 px drift (0.2 m at 2 m) | `HOLD captured`, then `y` −0.05, −0.1, −0.1 (slew, then clamp) |
| Down, both axes, content down 22.4 px (0.1 m forward at 1 m) | `x` −0.1 |
| Forward, Derotate on, yaw left 5°, content right 19.6 px | error x 0.0, command 0 |
| The same turn, Derotate off | `y` −0.098 |
| Down, Derotate on, pitch nose down 3°, content up | error y 0.0, command 0 |
| A held sample with 60 inliers (a max-jump rejection) | `NEUTRAL: tracking lost`, neutral |
| No axis on | `DRY RUN`, `cmd` None, preview sway −0.05 |
| Teleop not ready | `WAITING for NEPI control`, `cmd` None |
| `reference_id` changes | `NEUTRAL: reference or settings changed` |

Add cases for every other row of 4.5: navpose too old by its read time (including a snapshot republished with the same `timestamp_s`) or unaligned with Derotate on, a rate over 30 deg/s, tilt over 10°, low inliers, the flow package missing, and a yaw that wraps through ±180°.

### 8.2 Tracker sign test

Keep the 2.3 test as `Apps/nepi_app_optical_flow/test/tracker_sign_test.py` **(proposed)**. Build a textured image, shift it with `cv2.warpAffine`, and assert that content moving right gives `dx` > 0 and content moving down gives `dy` > 0, with at least 20 inliers. After 2.4, also assert that a 3° counter-clockwise rotation gives `rotation_deg` near +3 and a 1.05 zoom gives `scale` near 1.05. Run it after any tracker change. The method's signs rest on it.

### 8.3 Derotation bench test

ESC supply off, NEPI control on, Enable on, Derotate on, both axis toggles off (dry run), and the orientation topic passing 3.5:

1. Forward mount: yaw the ROV slowly by hand, 10° to 20° and back. The raw shift moves about 4 px per degree, while the derotated error in `stab_msg` stays within the deadband.
2. Slide the ROV sideways without turning it. The derotated error now follows the raw shift.
3. Down mount: do the same with pitch, then with surge.
4. If the derotated error doubles instead of cancelling, `imu_mount` or the yaw frame is wrong (TEST_PLAN.md Part B). If it spikes during a turn and settles after, the image and navpose lag by different amounts. Record that for section 12.

### 8.4 Bench gates and in water

On the bench (Phase 4), after the sign test, trip each gate by hand and watch `stab_msg` and `rostopic echo <rbx namespace>/set_teleop_velocity`:

- Cover the lens.
- Stop the optical flow app.
- Press Reset reference.
- Press RB to take the ROV back.
- Turn Enable off mid-hold, then turn an axis toggle off mid-hold.

Each shows its message, and the topic shows one zero Twist and then nothing, or neutral every cycle, as 4.5 says. In water (Phase 5), follow TEST_PLAN.md stage 4 and the [rules for the first powered tests](SAFETY.md#rules-for-the-first-powered-tests). Record `flow_data`, the NPX orientation topic and `set_teleop_velocity` with `rosbag record` for tuning.

---

## 9. Deploy and verify

**Engine first, before Phase 3.** The `ConnectRBXDeviceIF.set_teleop_velocity` change (section 11, item 1) is an nepi_engine change, built and deployed on the NEPI side like any engine change. Confirm it inside the container with `grep -n "def set_teleop_velocity" /opt/nepi/nepi_engine/lib/python3/dist-packages/nepi_api/connect_device_if_rbx.py`.

1. **Optical flow app, after 2.4 to 2.6.** The app has no deploy script. Copy auto_move's [deploy_app.sh](../Apps/nepi_app_auto_move/deploy_app.sh) into it **(proposed)**: it takes the app name from its own folder ([deploy_app.sh:87–88](../Apps/nepi_app_auto_move/deploy_app.sh#L87-L88)), and it takes the target address from `NEPI_IP` ([:12–15](../Apps/nepi_app_auto_move/deploy_app.sh#L12-L15)). The new message needs a catkin build. The build sync ([:93–123](../Apps/nepi_app_auto_move/deploy_app.sh#L93-L123)) puts the app into the device's build workspace, then build there as for any app. A live sync cannot make a message. The RUI page is unchanged, so no RUI build.
2. **auto_move, after Parts 2 to 4.** Every change is in `api/` and `sdk/`, which `deploy_app.sh` live-syncs into the running container ([:165–203](../Apps/nepi_app_auto_move/deploy_app.sh#L165-L203)). No message or RUI file changes, so neither a catkin build nor an RUI build is needed.
3. **Order.** Build the optical flow app first. auto_move tries the flow message import once, at start, so restart auto_move after that build. Otherwise it reports `no flow source`. Deploy auto_move's Part 4 after the engine change. Before that, `sendStabCmd` reports the missing method as `stab_state` error and sends nothing (5.2).
4. **Restart.** A live sync does not restart a node. Disable each app in the RUI, then enable it again.
5. **Confirm.** Inside the container:
   - `grep -n "STAB_OF_MAX_FLOW_AGE_S" /opt/nepi/nepi_engine/lib/python3/dist-packages/nepi_sdk/nepi_auto_move_stab.py`
   - `grep -n "def flowDataCb" /opt/nepi/nepi_engine/lib/python3/dist-packages/nepi_api/auto_move_if.py`
   - `python3 -c "from nepi_app_optical_flow.msg import NepiAppOpticalFlowData"`

   Then the RUI Stabilization box shows the nine new controls, and 2.8 passes.
6. **No deletes on the device.** The `sdk/` and `api/` syncs run without `--delete` ([:171](../Apps/nepi_app_auto_move/deploy_app.sh#L171), [:192](../Apps/nepi_app_auto_move/deploy_app.sh#L192)), so a file renamed or removed in the repo survives on the device. This plan adds no file to either folder.

---

## 10. Rules for this work

- **No rospy outside nepi_sdk.** New code uses `nepi_sdk` wrappers and `nepi_api` interfaces. The optical flow node's existing rospy calls are the port's job (2.7).
- **The stab module stays pure.** It imports only `copy` and `math`: no ROS, no `nepi_sdk`, no `nepi_controls`, no interface class. It never blocks: one step per call ([stab:96–103](../Apps/nepi_app_auto_move/sdk/nepi_auto_move_stab.py#L96-L103)).
- **Anything that can start motion carries `'param': False`:** Enable, Hold Sway and Hold Surge. Add no other persistence for them.
- **Unique names:**
  - Control names, across both of the app's control sets.
  - node_if keys, with the `auto_move_` prefix (the 2026-07 registry decision).
  - `api/` and `sdk/` basenames, which share one namespace with the engine's. This plan adds none.
  - RUI basenames, which are global across apps.
- **Messages stay in the app package** until a second consumer needs them (the 2026-08 decision).
- **Naming.** Public methods are snake_case with a docstring (`stab_optical_flow`). Private methods are camelCase, with or without a leading underscore, and no docstring (`flowDataCb`, `stabOrientationCb`, `_axisCmd`). ROS and timer callbacks end in `Cb`. Rename nothing that exists: `robotNavPoseCb`, `stabProcessCb` and `stabControlsUpdatedCb` are registered callbacks.
- **No `py_compile` in the repo.** It writes `__pycache__`, which a push would commit. Check syntax with `python3 -B -c "import ast; ast.parse(open(PATH).read())"`.
- **Never hand-edit** `Nepi_IF_Apps.js` or `NepiApps.js`. Apps register through their params `RUI_DICT`.

---

## 11. Engine follow-ups (not this developer's job)

1. **Done 2026-11-13: `set_teleop_velocity` on `ConnectRBXDeviceIF`.** The method follows `goto_velocity` (`connect_device_if_rbx.py:691`), and its publisher is in the per-robot dict (`:1007`); the code is in 5.1. It needs an engine build before Phase 3, and auto_move's Part 4 calls it.
2. **Throttle the teleop log line** (`device_if_rbx.py:1410`). Pass `throttle_s`, or log only state changes.
3. **Stamp NavPose with the acquisition time.** `publish_navpose` takes a `timestamp` argument but does not pass it to the message (`system_if.py:538`, `:730`). The NPX device should pass the driver's read time, so a consumer of the combined NavPose can tell held data from fresh. auto_move no longer needs this: the orientation topic it reads already carries the read time (3.2).
4. **Make the `time_*` fields of `NavPose.msg` float64.** As float32 they have 128 s steps.
5. **The RBX navpose source on drivers that host their own NPX** (engine commit `2be5ec8`). This plan no longer depends on it (3.2), but every consumer of the RBX's combined NavPose does.
6. **Promote `NepiAppOpticalFlowData` to nepi_interfaces** when a second consumer appears.

---

## 12. Open questions for the team

1. **Which camera and mount** should the method assume, forward or down, and what is that camera's horizontal field of view? The 110° default is a placeholder.
2. **Heave and yaw:** do they stay with the Teensy's holds, as this plan assumes, after finding 1 is fixed?
3. **Who owns Robots page Teleop** while Enable is on: the operator, or nobody?
4. **Does the ROV's NPX publish its orientation topic in 3.4,** with a read time that advances? Run the 3.5 check.
5. **Sway cap.** At 0.1, sway has only 0.087 to 0.1 of useful range (4.3). When can its cap go to 0.2?
6. **Lag.** If 8.3 shows the image and navpose lagging differently, how much, and is a delay setting worth adding?
