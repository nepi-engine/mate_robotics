# auto_move HUD overlay: build plan

For the MATE developer building the goal and error overlay in the auto_move image node.
Written 2026-10-15 against mate_robotics `9771fd3` plus the uncommitted work in progress in the image node. The plan only: no code has been changed.

**How to read the references.** Line numbers are from 2026-10-15 and will drift as you edit, so search for the quoted code if a line has moved. Links to mate_robotics files are relative to this document. "The node" always means [scripts/auto_move_app_img_pub_node.py](scripts/auto_move_app_img_pub_node.py), so `node:798` is line 798 of that file. Engine files are cited by short name. Their full paths inside the nepi_engine_ws checkout are:

| Short name | Path in nepi_engine_ws |
|---|---|
| `nepi_img.py` | `src/nepi_engine/nepi_sdk/src/nepi_sdk/nepi_img.py` |
| `nepi_nav.py` | `src/nepi_engine/nepi_sdk/src/nepi_sdk/nepi_nav.py` |
| `data_if.py` | `src/nepi_engine/nepi_api/src/nepi_api/data_if.py` |
| `device_if_rbx.py` | `src/nepi_engine/nepi_api/src/nepi_api/device_if_rbx.py` |
| `connect_device_if_rbx.py` | `src/nepi_engine/nepi_api/src/nepi_api/connect_device_if_rbx.py` |
| `DeviceRBXStatus.msg`, `GotoGoals.msg`, `GotoErrors.msg`, `NavPose.msg` | `src/nepi_interfaces/msg/` |
| `rbx_ardupilot_node.py` | `src/nepi_drivers/rbx_drivers/rbx_ardupilot_node.py` |

---

## 1. Goal and scope

In auto_move, an operator clicks a point in the camera image. The parent node ([api/auto_move_if.py](api/auto_move_if.py)) turns that click into a relative move and sends it to the selected robot, which NEPI calls an RBX device. The image node draws overlays on the camera frame (depth map, boxes, targets, click crosshair) and publishes the image the RUI shows.

This work adds a heads-up display to that image. It shows where the robot was told to go (its **goto goals**) and how far it still is from there (its **goto errors**). Later it also shows the robot's own attitude, heading and depth. The end state looks like a helicopter HUD in a flight game: a pitch ladder, a roll scale, a heading tape, a depth or altitude tape, and a goal marker drawn over the scene where the goal is.

The work comes in four steps:

| Phase | What appears on the image |
|---|---|
| 0 | The work already in the node runs: an attitude reticle and a Move Goal marker |
| 1 | A text panel: active command, move state, per-axis goal and error |
| 2 | Simple gauges: heading with a goal tick, depth or altitude bar with a goal tick, distance-to-go bar, attitude indicator |
| 3 | A helicopter-style HUD, with the Move Goal marker driven by real data |

All of the work goes in one file, the node. It needs no message change, no RUI change and no build, because `deploy_app.sh` live-syncs the node (section 9). The only item that needs other files is an optional operator on/off switch in the RUI (end of Phase 3).

The team is removing the mate copy of `nepi_img.py` (`SDK/nepi_img.py`), and nothing in this plan uses it. The node keeps importing the engine's `nepi_img` for the calls it already makes (section 6). Every new drawing helper is copied into the node as a private method.

---

## 2. Phase 0: make the current work run

### What is already in the node (uncommitted)

- **`overlay_attitude`** ([node:798–841](scripts/auto_move_app_img_pub_node.py#L798-L841)) is a byte-for-byte copy of the engine's `nepi_img.overlay_target` (`nepi_img.py:1104`). It draws a circle with four tick marks, plus optional text to the right. The only change from the original is that its text call became `nepi_img.overlay_text_list` ([node:840](scripts/auto_move_app_img_pub_node.py#L840)).
- **An attitude block** in `processOverlayImage` ([node:893–896](scripts/auto_move_app_img_pub_node.py#L893-L896)) draws that shape at pixel (10, 10).
- **A Move Goal block** ([node:899–920](scripts/auto_move_app_img_pub_node.py#L899-L920)) uses the image interface's crosshair API (`img_if`, a `ColorImageIF`). It adds a "Center" marker at (0°, 0°) and a "Move Goal" marker at `goal_error_degs`, both placed **by angle from the image center**. The image interface stores these markers and draws them on every frame it publishes (`data_if.py:3459–3543`). That makes it the natural way to put a goal on the image, and Phase 3 keeps it.

Right now the node cannot start. Fix the defects in the order below. None of them had been fixed as of 2026-10-15.

### (a) The class ends at line 798. Fix this first.

[node:798](scripts/auto_move_app_img_pub_node.py#L798): `def overlay_attitude` starts at column 0, between two methods, and Python ends `class AutoMoveImgPub` there. Every `def` after it is now a function nested inside `overlay_attitude` instead of a method: `processOverlayImage`, `publishImgData`, `getBlendRatio`, `getOverlayFontDims`, `applyDepthMapOverlay`, `applyBoxesOverlay`, `applyTargetsOverlay`, `applyCrosshairOverlay` and `shutdownCb`.

**What you see:** `__init__` registers `nepi_sdk.on_shutdown(self.shutdownCb)` ([node:285](scripts/auto_move_app_img_pub_node.py#L285)), so the node dies at startup with `AttributeError: ... no attribute 'shutdownCb'`. If it got further, `renderSourceFrame` would fail the same way on `self.processOverlayImage` ([node:785](scripts/auto_move_app_img_pub_node.py#L785)).

**Fix:** cut the whole function (node:798–841) and paste it back inside the class. Put it after `applyCrosshairOverlay`, which ends at [node:1161](scripts/auto_move_app_img_pub_node.py#L1161), with the other drawing helpers. Indent it one level and add `self`. Rename it `overlayAttitude`, because private methods use camelCase. Drop the docstring, because private methods carry none.

```python
    def overlayAttitude(self, cv2_img, x_px, y_px, color_rgb = (0, 255, 0), size_ratio = 0.5,
                        thickness_ratio = 0.5, size = None, thickness = None,
                        text_list = [], text_ratio = 0.5):
        # Copied from the engine's nepi_img.overlay_target. Draws in place.
        shape = cv2_img.shape
        ...   # rest of the body unchanged, one indent level deeper
        return cv2_img

    def shutdownCb(self):
```

Once the column-0 `def` is gone, the methods after it are back in the class without any other edit.

**Check that the class is whole again.** `python3 -m py_compile` passes even while the class is broken, so a compile check is not enough. List the class with `ast` instead. Run this from the mate_robotics root:

```bash
python3 - <<'EOF'
import ast
path = 'Apps/nepi_app_auto_move/scripts/auto_move_app_img_pub_node.py'
tree = ast.parse(open(path).read())
for node in tree.body:
    if isinstance(node, ast.ClassDef):
        print(node.name, [m.name for m in node.body if isinstance(m, ast.FunctionDef)])
    elif isinstance(node, ast.FunctionDef):
        print('MODULE-LEVEL FUNCTION:', node.name, 'line', node.lineno)
# self.<name> read somewhere but never assigned or defined in the class
assigned, read = set(), {}
for cls in [n for n in tree.body if isinstance(n, ast.ClassDef)]:
    for item in cls.body:
        if isinstance(item, ast.Assign):
            assigned.update(t.id for t in item.targets if isinstance(t, ast.Name))
        if isinstance(item, ast.FunctionDef):
            assigned.add(item.name)
for n in ast.walk(tree):
    if isinstance(n, ast.Attribute) and isinstance(n.value, ast.Name) and n.value.id == 'self':
        if isinstance(n.ctx, ast.Store):
            assigned.add(n.attr)
        else:
            read.setdefault(n.attr, n.lineno)
print('NEVER ASSIGNED:', sorted((v, k) for k, v in read.items() if k not in assigned))
EOF
python3 -m pyflakes Apps/nepi_app_auto_move/scripts/auto_move_app_img_pub_node.py
```

Today the script prints `MODULE-LEVEL FUNCTION: overlay_attitude line 798` and 13 names under NEVER ASSIGNED. That list covers defects (a), (b) and (c). pyflakes prints `918: undefined name 'OVERLAY_CROSSHAIR_COLOR'`, which is defect (d). When Phase 0 is done, you should see:

- `processOverlayImage`, `overlayAttitude` and `shutdownCb` in the class list
- no `MODULE-LEVEL FUNCTION` line
- `NEVER ASSIGNED: []`
- nothing from pyflakes about undefined names

### (g) The helper still reads as the engine original. Fold this into (a).

[node:799](scripts/auto_move_app_img_pub_node.py#L799) still carries the crosshair docstring, which the engine's `overlay_target` also has. Nothing else in the helper is broken. `nepi_img.overlay_text_list` accepts `size_ratio` (`nepi_img.py:1036–1037`), and `nepi_utils.check_ratio` exists. The rename and docstring removal in (a) is the whole fix.

### (b) It is called as a method that does not exist

[node:896](scripts/auto_move_app_img_pub_node.py#L896) calls `self.overlay_attitude(...)`, but `overlay_attitude` is a module function, not a method. **What you see:** an AttributeError on the first rendered frame. With (a) fixed and (c) not yet fixed, (c) fires first. **Fix:** call `self.overlayAttitude`. The snippet under (e) replaces the whole block.

### (c) Four attributes are read but never defined

These four are read but never defined:

- `show_attitude_enabled` ([node:894](scripts/auto_move_app_img_pub_node.py#L894))
- `draw_attitude` (node:895)
- `show_move_enabled` ([node:901](scripts/auto_move_app_img_pub_node.py#L901))
- `draw_crosshair` (node:902). It is assigned at node:912, but only after node:902 has already read it.

None of them is in the parent status message, so `statusCb` never sets them. **What you see:** an AttributeError in the render timer on the first frame, and the overlay image stops publishing.

**Fix:** give each one a default. The HUD switches belong to the node, so make them constants beside the existing colors ([node:66–70](scripts/auto_move_app_img_pub_node.py#L66-L70)):

```python
# HUD colours, RGB. The img_if crosshair API and the nepi_img drawing helpers
# take RGB and convert it themselves; the colours above are BGR.
ATTITUDE_COLOR_RGB = (0, 255, 0)
MOVE_GOAL_COLOR_RGB = (255, 0, 255)
CENTER_COLOR_RGB = (0, 0, 127)

# HUD layers. Set here: the parent status message carries no switch for them.
SHOW_ATTITUDE_ENABLED = True
SHOW_MOVE_GOAL_ENABLED = True
```

Then add class attributes after the status-derived block ([node:153–170](scripts/auto_move_app_img_pub_node.py#L153-L170)):

```python
    # HUD layers, from the constants above rather than the parent status
    show_attitude_enabled = SHOW_ATTITUDE_ENABLED
    show_move_enabled = SHOW_MOVE_GOAL_ENABLED

    # Last Move Goal state pushed to img_if. None makes the next render push it.
    draw_crosshair = None

    # Move Goal placeholder until Phase 3, in img_if crosshair degrees:
    # positive x is LEFT of centre, positive y is BELOW centre.
    goal_error_degs = [10.0, 10.0]
```

After the (e) fix, `draw_attitude` is not needed at all.

Also reset `draw_crosshair` whenever the image interface is replaced. While it is being constructed (`data_if.py:2913`), a new `ColorImageIF` reloads its saved crosshairs from the ROS param server, keeping every one except `click` (`data_if.py:5535–5546`). So a stale "Move Goal" can come back with the new interface while the node believes nothing is drawn. `unsubscribeImgTopic` already clears per-source state ([node:561–567](scripts/auto_move_app_img_pub_node.py#L561-L567)). Add one line there:

```python
        self.clearResult()
        # The next img_if restores saved crosshairs; make the render push its state.
        self.draw_crosshair = None
```

### (d) Undefined colour, and the failure is silent

[node:918](scripts/auto_move_app_img_pub_node.py#L918) uses `OVERLAY_CROSSHAIR_COLOR`, which is not defined. The file defines `CROSSHAIR_COLOR` ([node:70](scripts/auto_move_app_img_pub_node.py#L70)). **What you see:** the NameError is caught by the try/except at node:916–920. The Move Goal marker never draws, and an info-level `Draw Crosshair Failed ... is not defined` line repeats in the log every 5 s.

Do not switch it to `CROSSHAIR_COLOR`. That constant is BGR (node:66), while `add_crosshair_degs` takes RGB (`data_if.py:4739`) and the engine converts it, so the red BGR `(0, 0, 255)` would draw blue. Use the RGB constants from (c), and log the failure as a warning:

```python
            try:
                img_if.add_crosshair_degs(0, 0, name = 'Center', color_rgb = CENTER_COLOR_RGB)
                img_if.add_crosshair_degs(x_deg, y_deg, name = 'Move Goal', color_rgb = MOVE_GOAL_COLOR_RGB)
            except Exception as e:
                self.msg_if.pub_warn('Draw Crosshair Failed: ' + str([x_deg, y_deg]) +
                                     ' with exception: ' + str(e), throttle_s = 5.0)
```

### (e) The attitude block draws once, mostly off-screen

[node:894–896](scripts/auto_move_app_img_pub_node.py#L894-L896) has two problems.

1. It uses the state-change test from the img_if block below it (`if self.draw_attitude != draw_attitude`) but never stores the new value. That pattern is right for img_if settings, because the interface keeps them. It is wrong for cv2 drawing. The node draws on a fresh copy of every frame ([node:872](scripts/auto_move_app_img_pub_node.py#L872)) and nothing carries over, so the reticle must be drawn on every frame it should appear on.
2. At (10, 10) the reticle's center is 10 px from the corner. Its arms reach about 0.11 of the shorter image side from that center (52 px on a 480-line image), so most of it falls off the top-left. A test draw on a 640×480 frame kept 400 of its 1044 pixels.

**Fix:** draw it every frame while it is enabled, at a point where all of it fits. The image center is safe at any size. Phase 2 moves it.

```python
        if self.show_attitude_enabled == True:
            [img_h, img_w] = overlay_img.shape[:2]
            overlay_img = self.overlayAttitude(overlay_img, int(img_w / 2), int(img_h / 2),
                                               color_rgb = ATTITUDE_COLOR_RGB)
```

At the center it rings the img_if "Center" cross, which reads as a boresight.

### (f) The goal is a hard-coded placeholder

[node:899](scripts/auto_move_app_img_pub_node.py#L899), `self.goal_error_degs = [10,10]`, runs on every frame, so nothing else can ever set the value. Delete the line. The class default from (c) keeps the same test value, and Phase 3 replaces it with the real goal. Mind the signs: (10, 10) puts the marker 10° **left** of center and 10° **below** it (`data_if.py:3468`, `:3481`), not right and up.

### While you are in there

[node:844](scripts/auto_move_app_img_pub_node.py#L844) reads `img_node_dict` without taking `img_node_lock`. Every other reader takes the lock first: `needsImgCheck` at [node:575–577](scripts/auto_move_app_img_pub_node.py#L575-L577) and `publishImgData` at [node:946–949](scripts/auto_move_app_img_pub_node.py#L946-L949). Match them, and return `False` like the other early exits:

```python
        self.img_node_lock.acquire()
        img_if = self.img_node_dict.get('img_if', None)
        self.img_node_lock.release()
        if img_if is None:
            return False
```

### Phase 0 acceptance

- The `ast` check prints no `MODULE-LEVEL FUNCTION` line and `NEVER ASSIGNED: []`, and pyflakes reports no undefined names.
- Deployed (section 9), the node logs `Initialization Complete` and no AttributeError.
- On the overlay image you see three things: a green reticle ringing the center, a dark-blue Center cross, and a magenta Move Goal cross 10° left of and 10° below center.
- Disabling the app produces no traceback from `shutdownCb`.

The img_if markers only appear when the frame goes out through the image interface. If `img_if` is not ready, `publishImgData` falls back to a raw publish ([node:952–959](scripts/auto_move_app_img_pub_node.py#L952-L959)), which skips them.

---

## 3. The data

### Frames

- **Body frame:** axes fixed to the robot, with x forward, y left and z up, in meters.
- **ENU (East-North-Up):** a world frame. ENU yaw is measured counter-clockwise from East.
- **NED (North-East-Down):** a world frame. NED yaw is a compass heading, clockwise from North. NEPI's NED roll and pitch are the same numbers as its ENU roll and pitch. Only yaw converts, as `yaw_ned = 90 - yaw_enu` (`nepi_nav.py:1128–1132`, `:1489`).

### Topic and fields

Each RBX device publishes `DeviceRBXStatus` on `<rbx_namespace>/status`, latched, at **2 Hz** (`device_if_rbx.py:129`, `:568–573`, `:830`). The parent status carries the selected robot's namespace in `rbx_namespace` ([NepiAppAutoMoveStatus.msg:24](msg/NepiAppAutoMoveStatus.msg#L24)), which is `'None'` when no robot is selected.

| Field | Line in `DeviceRBXStatus.msg` | Meaning |
|---|---|---|
| `goals_current` (GotoGoals) | 64 | Where the current or last command was told to go |
| `errors_current` (GotoErrors) | 56 | How far from that goal, during and after the command |
| `errors_prev` | 60 | Errors of the command before. Not needed here. |
| `process_current` | 42 | Name of the active command, or `"None"` |
| `process_last` | 45 | Name of the last command |
| `ready` | 49 | False while a command runs |
| `cmd_success` | 68 | Whether the last command met its error bounds in time |
| `last_error_message` | 89 | Last error text |
| `navpose_topic` | 14 | Where the robot's NavPose is published (used in Phase 2) |

`GotoGoals` and `GotoErrors` have the same seven float32 fields: `x_m`, `y_m`, `z_m`, `heading_deg`, `roll_deg`, `pitch_deg`, `yaw_deg` (lines 3–9 of each `.msg`). `heading_deg` is never written, so it is always 0.

### Update lifecycle

- **Reset.** Every goto command first sets all seven goals and all seven errors to 0. So do set state, set mode, go home and stop. Examples: `device_if_rbx.py:1500–1501` (goto position), `:1110–1111` (set state), `:1137–1138` (set mode), `:1332–1333` (go home), `:1356–1357` (stop).
- **During a command.** `update_current_goals` and `update_current_errors` (`device_if_rbx.py:2155`, `:2118`) write only the axes the command uses. A `None` leaves that axis unchanged.
- **After it ends.** When the command completes or times out, the values stay as they were until the next reset. `process_current` returns to `"None"` and `process_last` names the command, so use `process_last` to interpret goals after the fact.
- **The first instant of a move.** goto_position writes its goals (`device_if_rbx.py:1869`) before its first error pass (`:1936`), which comes one 0.1 s loop later. A status sent in that window shows real goals with all-zero errors. Treat "command active, goals non-zero, errors all exactly 0" as **not measured yet**, not as **arrived**.
- **Rate.** Errors are recomputed every 0.1 s (`device_if_rbx.py:1879`) but reach the wire only at 2 Hz.

### Goal and error slots by command

**Goals** (where to go):

| Command (process name) | `x_m` | `y_m` | `z_m` | `roll_deg`, `pitch_deg` | `yaw_deg` | Written at |
|---|---|---|---|---|---|---|
| goto_pose (`GoTo Pose`) | 0 | 0 | 0 | Absolute target, NED, deg | Absolute target, NED, deg | `device_if_rbx.py:1693` |
| **goto_position** (`GoTo Position`), which auto_move uses | Commanded offset forward, m, body frame at command start | Offset left, m | Offset up, m | 0 | Absolute target yaw, ENU, deg. When yaw is held, this is the start yaw. | `:1869` |
| goto_location (`GoTo Location`) | m north, start to target | m east | m up | 0 | Target yaw, NED, 0–360 | `:2044` |
| ArduPilot takeoff | 0 | 0 | Climb height, m | 0 | 0 | `rbx_ardupilot_node.py:2311` |
| goto_velocity, go home, stop, set state, set mode | 0 | 0 | 0 | 0 | 0 | reset only |

**Errors** (how far off):

| Command | `x_m` | `y_m` | `z_m` | `roll_deg`, `pitch_deg` | `yaw_deg` | Sign | Written at |
|---|---|---|---|---|---|---|---|
| goto_pose | 0 | 0 | 0 | NED, deg, not wrapped | NED, deg, not wrapped | target − current | `device_if_rbx.py:1717`, `:1735` |
| **goto_position** | **Left** error, m (swapped and rotated, see section 4) | **Forward** error, m (swapped and rotated) | Up error, m (correct) | 0 | Size only, ENU, deg, no direction | current − target | `:1899`, `:1910–1911`, `:1935–1936` |
| goto_location | North error, m | East error, m (no cos(latitude)) | Up error, m | 0 | Size only, NED, deg | current − target | `:2069–2070`, `:2082–2083`, `:2107` |
| ArduPilot takeoff | 0 | 0 | Altitude error, m | 0 | 0 | target − current | `rbx_ardupilot_node.py:2344–2346` |
| goto_velocity and the no-target commands | 0 | 0 | 0 | 0 | 0 | none | reset only |

auto_move sends only two commands: goto_position or goto_velocity ([auto_move_if.py:1936](api/auto_move_if.py#L1936), [:1944](api/auto_move_if.py#L1944)). Each plan step is its own goto_position with `yaw_deg` defaulting to 0 ([auto_move_if.py:1947](api/auto_move_if.py#L1947)), which holds yaw. So during an auto_move step, `goals_current.yaw_deg` is the robot's ENU yaw at the start of that step.

### What the overlay sees in velocity mode

When `goto_velocity_enabled` is on, auto_move sends goto_velocity ([auto_move_if.py:1936](api/auto_move_if.py#L1936)). That command is open loop: the robot holds a velocity for a set time and then stops. `process_current` reads `GoTo Velocity` while it runs, and all goals and errors stay 0 throughout. On the RBX side, the only things to show are the command name and `ready`.

### The planned move in the parent status

The parent status (`NepiAppAutoMoveStatus`) already reaches the node through `statusCb` ([node:665](scripts/auto_move_app_img_pub_node.py#L665)). Its planned-move fields can fill in whenever the RBX goals are 0:

| Field | Line in [NepiAppAutoMoveStatus.msg](msg/NepiAppAutoMoveStatus.msg) | Meaning | HUD use |
|---|---|---|---|
| `rbx_namespace`, `rbx_connected`, `rbx_ready` | 24, 26, 28 | The selected robot | Which RBX status to subscribe to. Show a "no robot" state. |
| `has_click`, `click_depth_valid` | 122, 125 | Whether the goto values below come from a usable click | Gate the planned-goal marker |
| `goto_x_m`, `goto_y_m`, `goto_z_m` | 130–132 | Move requested by the last click: body frame, x forward, y left, z up, in meters, relative to the robot's pose at click time | Goal preview while idle. Fallback in velocity mode. |
| `goto_velocity_enabled` | 148 | Which kind of move a Goto sends | Label |
| `goto_x_mps` … `goto_yaw_degps`, `goto_duration_s` | 172–174, 178, 181 | The velocity command and its duration | Velocity-mode panel |
| `goto_state` | 192 | idle, planning, moving, complete or cancelled (lines 8–12) | Panel text. Also picks the goal source. |
| `goto_msg` | 194 | Readable progress text | Panel |
| `goto_step`, `goto_step_count` | 196–197 | Current plan step and plan length | Panel, e.g. "step 1/2" |

**How the planned move compares with the RBX position goal.** Both use the same body frame and units. The parent builds `goto_x_m` from the click at [auto_move_if.py:1553–1562](api/auto_move_if.py#L1553-L1562), and the RBX device writes the commanded offset as its goal (`device_if_rbx.py:1869`). They are equal when the plan has one step and the planner passes the move through unchanged. The planner (`sdk/nepi_auto_move.py`) was not reviewed for this plan. With more than one step, the RBX goal is the current step relative to where that step started, while `goto_x_m` is the whole click. `goto_x_m` also changes on every click, even while the robot is idle.

---

## 4. Data caveats

The defects below are in the engine. Do not fix them in this work. Work around them, and keep **every** goal-and-error-to-display rule in one method, `getHudValues` (Phase 1), so the later engine fix only has to change one method.

| Defect | Where | What it breaks in the HUD | What the overlay does for now |
|---|---|---|---|
| **Signs differ by command.** goto_pose and takeoff report target − current; goto_position and goto_location report current − target. | `device_if_rbx.py:1717`, `:1899`, `:2069`; `rbx_ardupilot_node.py:2344` | Any signed error, and any "goal = current ± error" | In `getHudValues`, convert every command to `remaining = target − current` |
| **goto_position errors are written in [1],[0],[2] order.** `x_m` holds the left error and `y_m` the forward error, while goals use natural order. | `device_if_rbx.py:1936`, `:1941` | Per-axis error text, and the error direction | Swap them back in `getHudValues` |
| **goto_position errors are rotated by the robot's heading.** `convert_point_enu2body` adds the yaw where it should subtract it (`nepi_nav.py:1517–1518`), so the horizontal error is turned by twice the start yaw. It is correct only at ENU yaw 0° or 180°. The goal rewritten on a stop command has the same defect. | `device_if_rbx.py:1935`, `:1885–1889` | Horizontal error direction, and any marker placed from the errors | Show the horizontal error as a **magnitude only**: a vector's length survives both the swap and the rotation. The up error is unaffected, because z passes through. Do not place the marker from the errors yet (Phase 3). |
| **Errors use the yaw at command start**, not the current yaw | `device_if_rbx.py:1935` | Direction drift if the robot turns mid-move | auto_move holds yaw, so there is nothing to do. Noted only. |
| **Yaw errors are `abs()`** for goto_position and goto_location, so they carry no turn direction | `:1911`, `:2083` | Turn direction on the heading gauge | Draw the goal tick from `goals.yaw_deg` and current heading from NavPose; the gauge then shows the direction. Show the yaw error only as an unsigned number. |
| **Angle errors are not wrapped to ±180** | `:1717`, `:1910`, `:2082` | A 350° error that is really −10° | Wrap every error the HUD shows to ±180, and every heading to 0–360 |
| **goto_location does not scale longitude by cos(latitude)** | `:2044`, `:2070` | East goal and east error are overstated by 1/cos(latitude), about 1.4× at 45° | Label location distances "approx". auto_move does not send goto_location. |
| **A −999 "hold this axis" input to goto_position is not honored.** The check compares the whole list to −999. | `:1901` | The error on a held axis | auto_move never sends −999. Nothing to do. |
| **`heading_deg` is never written** | | | Ignore the slot. Heading comes from NavPose (Phase 2). |

---

## 5. Getting the data into the image node

**Recommendation:** subscribe the node directly to the selected robot's `DeviceRBXStatus`.

- **Topic:** `nepi_sdk.create_namespace(self.rbx_namespace, 'status')`. This is the same name `ConnectRBXDeviceIF` subscribes to (`connect_device_if_rbx.py:853–857`).
- **Source of `rbx_namespace`:** the parent status. Have `statusCb` copy it, the same way it copies `image_topic` ([node:671](scripts/auto_move_app_img_pub_node.py#L671)).
- **When to (re)subscribe:** in `updaterCb` ([node:438](scripts/auto_move_app_img_pub_node.py#L438)), using the same wanted-versus-active comparison the node already uses for the image topic ([node:439–456](scripts/auto_move_app_img_pub_node.py#L439-L456)). Unsubscribe when the active topic differs from the wanted one, and subscribe when nothing is active.
- **How:** copy `subscribeImgTopic` and `unsubscribeImgTopic` ([node:484](scripts/auto_move_app_img_pub_node.py#L484), [node:537](scripts/auto_move_app_img_pub_node.py#L537)).
  - Subscribe with `nepi_sdk.create_subscriber(topic, DeviceRBXStatus, self.rbxStatusCb, queue_size = 1)`.
  - Keep the handle in its own dict under its own lock, not in `img_node_dict`, which is torn down whenever the image source changes.
  - Unregister in a try/except, and drop the cached status on unsubscribe.
- **How to store it:** `rbxStatusCb` converts the message with `nepi_sdk.convert_msg2dict`, as `imageStatusCb` does ([node:588–595](scripts/auto_move_app_img_pub_node.py#L588-L595)). It stores the dict whole under a lock, the way `setResult`/`getResult` handle the per-cycle snapshot ([node:393–410](scripts/auto_move_app_img_pub_node.py#L393-L410)), and the render reads one reference.

This keeps all the work in the node, needs no message change and no build, and delivers the full status at its native 2 Hz.

**Not chosen:** having the parent copy goals and errors into `NepiAppAutoMoveStatus`. The parent already holds the robot's status: `ConnectRBXDeviceIF.get_status_msg` returns it whole (`connect_device_if_rbx.py:263`). But this route needs edits to [api/auto_move_if.py](api/auto_move_if.py) and [msg/NepiAppAutoMoveStatus.msg](msg/NepiAppAutoMoveStatus.msg) plus a catkin build. It would also slow the data to the parent's 1 Hz status timer ([auto_move_if.py:710](api/auto_move_if.py#L710)).

Phase 2 adds a second subscription the same way: the robot's NavPose, at `DeviceRBXStatus.navpose_topic`. The parent already follows that field ([auto_move_if.py:1060–1065](api/auto_move_if.py#L1060-L1065), `getRobotNavPoseTopic` at [:1186](api/auto_move_if.py#L1186), `subscribeRobotTopics` at [:1204](api/auto_move_if.py#L1204)).

---

## 6. Copying helpers from nepi_img

**Conventions in the engine helpers:**

- They take RGB colors and convert to BGR themselves.
- They size everything from the shorter image side, `min(h, w)`.
- They rely on module globals: `OVERLAY_FONT`, which is `FONT_HERSHEY_SIMPLEX` (`nepi_img.py:64`), and `logger` (`:57`).

**Conventions in the node:**

- Its constants are BGR ([node:66–76](scripts/auto_move_app_img_pub_node.py#L66-L76)).
- It sizes from the longer side.
- It caches font metrics per frame size (`getOverlayFontDims`, [node:984](scripts/auto_move_app_img_pub_node.py#L984)).
- Its own `OVERLAY_FONT` is `FONT_HERSHEY_DUPLEX` ([node:73](scripts/auto_move_app_img_pub_node.py#L73)).

That name clash matters. A verbatim copy of `overlay_text` silently picks up the node's DUPLEX font. Decide which font you want and give it its own name, for example `HUD_FONT`.

**Copy rules:**

- Each copied helper becomes a private camelCase method of the class, with no docstring.
- `logger.log_warn(...)` becomes `self.msg_if.pub_warn(..., throttle_s = 5.0)`.
- Calls between copied helpers go through `self`.

| Engine source | Already in the node? | New name | Phase | Must come with it, or change while copying |
|---|---|---|---|---|
| `overlay_target`, `nepi_img.py:1104` | Yes, as `overlay_attitude` (identical copy) | `overlayAttitude` | 0 | Its text call becomes `self.overlayTextList` in Phase 1. Uses `nepi_utils.check_ratio`, already imported. |
| `overlay_crosshair`, `:1068` | No. The image interface already draws Center and Move Goal with the engine's copy. | not needed | n/a | n/a |
| `overlay_text`, `:972` | No | `overlayText` | 1 | `OVERLAY_FONT` → `HUD_FONT`, and logger → `msg_if`. Its `background_rgb` goes to cv2 unconverted, so it is really BGR: rename it `background_bgr` or convert it. Always pass both `scale` and `thickness`, because with `scale` set and `thickness` None it computes a float thickness that `cv2.putText` rejects. |
| `overlay_text_list`, `:1036` | No. The node calls the engine copy at node:840. | `overlayTextList` | 1 | Needs `overlayText` and `HUD_FONT`. Pass `scale` and `thickness` from `self.getOverlayFontDims`, so the fallback does not run on every frame. |
| `get_optimal_text_size`, `:955` | No | `getOptimalTextSize` (optional) | 1 | Needs `HUD_FONT` and calls `get_optimal_font_dims`. Calling `cv2.getTextSize` with the `getOverlayFontDims` values does the same job. |
| `get_optimal_font_dims`, `:947` | Called from the engine at [node:994](scripts/auto_move_app_img_pub_node.py#L994) | not copied | n/a | Keep the engine import. It is pure arithmetic. |
| `overlay_rectangle`, `:1062` | No | `overlayRectangle` | 2 | The engine version copies and blends the whole frame on every call. Change it to blend only the rectangle's region, in place, the way `applyDepthMapOverlay` does ([node:1017–1018](scripts/auto_move_app_img_pub_node.py#L1017-L1018)). Its color goes straight to cv2, so it is BGR. |
| `overlay_bounding_box`, `:1154` | The same drawing exists in `applyBoxesOverlay` ([node:1056](scripts/auto_move_app_img_pub_node.py#L1056)) | not needed | n/a | n/a |
| `overlay_box`, `:1149` | No | **do not copy** | n/a | It draws on a copy and returns None, so it never changes the image |

These engine imports stay, because they are not drawing helpers:

- `nepi_img.rosimg_to_cv2img` ([node:626](scripts/auto_move_app_img_pub_node.py#L626), [:780](scripts/auto_move_app_img_pub_node.py#L780))
- `nepi_img.cv2img_to_rosimg` ([node:953](scripts/auto_move_app_img_pub_node.py#L953))
- `nepi_img.get_optimal_font_dims` (node:994)

After Phase 1 these three are the node's only `nepi_img` calls. Nothing in this plan reads, edits or depends on `SDK/nepi_img.py`.

---

## 7. Phased plan

**The same shape in every phase.** Once per frame, `processOverlayImage` takes snapshots of the inputs. One method maps them to display values. Then `apply…Overlay` methods draw them in a fixed order, after the existing layers ([node:874–891](scripts/auto_move_app_img_pub_node.py#L874-L891)) and before `publishImgData` ([node:922](scripts/auto_move_app_img_pub_node.py#L922)). Keeping the mapping separate from the drawing is what makes the drawing testable without ROS (section 8).

```python
        # in processOverlayImage, after the click crosshair layer
        hud_dict = self.getHudValues(self.getRbxStatus(), self.getNavPose())
        if self.show_hud_text_enabled == True:
            self.applyHudTextOverlay(overlay_img, hud_dict)
        ...
```

### Phase 1: text panel

**Shows:**

- The active command: `process_current`, or `process_last` with "done" or "failed" from `cmd_success`
- Ready or busy
- `goto_state` with step i/n
- Per-axis goal and error, with frame labels
- Distance to go
- `last_error_message` when it is set

| Method | Signature | Copies the pattern of |
|---|---|---|
| `statusCb` (extend) | existing | [node:665–684](scripts/auto_move_app_img_pub_node.py#L665-L684). Add `rbx_namespace`, `rbx_connected`, `goto_state`, `goto_msg`, `goto_step`, `goto_step_count`, `goto_velocity_enabled`, `goto_x_m/y_m/z_m`, `click_depth_valid` and the velocity fields, with class defaults in the block at node:153–170. |
| `updaterCb` (extend) | existing | The wanted-versus-active image topic check, node:439–456 |
| `subscribeRbxTopic` | `(self, rbx_status_topic)` | `subscribeImgTopic`, node:484 |
| `unsubscribeRbxTopic` | `(self)` | `unsubscribeImgTopic`, node:537 |
| `rbxStatusCb` | `(self, msg)` | `imageStatusCb` node:588, plus `setResult` node:402 |
| `getRbxStatus` | `(self)` → dict or None | `getResult`, node:393 |
| `getHudValues` | `(self, rbx_status_dict, navpose_dict)` → dict | New. The one home for every section 4 rule. |
| `applyHudTextOverlay` | `(self, cv2_img, hud_dict)` | The `apply…Overlay` methods, node:1022–1161 |
| `overlayText`, `overlayTextList` | as copied | `nepi_img.py:972`, `:1036` (section 6) |

Also add the constant `SHOW_HUD_TEXT_ENABLED = True` and the class attribute `show_hud_text_enabled`, and the import `from nepi_interfaces.msg import DeviceRBXStatus`. Only the node changes.

**Acceptance:**

- With no robot selected, the panel shows "RBX: none" and nothing else changes.
- With a robot selected and idle, the command reads "None", goals and errors read 0, and the panel updates about twice a second.
- Click, then Goto in position mode. The panel shows "GoTo Position", busy, and "moving 1/1". The FWD/LEFT/UP goals match `goto_x_m/y_m/z_m` for a one-step plan. Distance to go falls below the robot's distance bound, and after completion the panel shows "GoTo Position done" with the goals still visible.
- Change the selected robot in the RUI. The panel follows within about 1 s (the `updaterCb` period), and the log shows the unsubscribe and subscribe pair.

### Phase 2: simple gauges

**New data: the robot's NavPose** (`NavPose.msg`):

- `heading_deg`, valid when `has_heading` (lines 25–27)
- `roll_deg`, `pitch_deg` and `yaw_deg` in ENU, valid when `has_orientation` (33–37)
- `altitude_m`, valid when `has_altitude` (59–61)
- `depth_m`, valid when `has_depth` (67–69)

Subscribe to `navpose_topic` from the RBX snapshot and re-subscribe when it changes. Use the Phase 1 subscribe/unsubscribe shape; the parent follows the same field at [auto_move_if.py:1186–1227](api/auto_move_if.py#L1186-L1227).

**Display values, all computed in `getHudValues`:**

- **Current heading:** `heading_deg` when `has_heading`. Otherwise use `90 − yaw_enu` (`nepi_nav.convert_yaw_enu2ned`, `nepi_nav.py:1489`), wrapped to 0–360.
- **Goal heading:** `goals.yaw_deg`, converted according to the command. goto_position reports ENU, so use `90 − yaw`. goto_pose and goto_location report NED, so use the value as is.
- **Vertical:** use `depth_m` when `has_depth` (MATE robots are underwater), otherwise `altitude_m`.
- **Vertical goal:** `current + remaining_up` for altitude, or `current − remaining_up` for depth. Here `remaining_up = −errors.z_m` for goto_position and goto_location, and `+errors.z_m` for takeoff. z is the only goto_position error axis with no defect.
- **Distance to go:** the length of `(errors.x_m, errors.y_m, errors.z_m)`.
- **Total distance:** the length of the goal's xyz. The bar shows `1 − togo / total`.
- **Attitude:** NavPose roll and pitch.

| Method | Signature | Copies the pattern of |
|---|---|---|
| `subscribeNavPoseTopic`, `unsubscribeNavPoseTopic` | `(self, navpose_topic)`, `(self)` | `subscribeImgTopic`, `unsubscribeImgTopic` |
| `navPoseCb`, `getNavPose` | `(self, msg)`, `(self)` | `rbxStatusCb`, `getRbxStatus` |
| `applyHeadingOverlay` | `(self, cv2_img, hud_dict)` | `applyCrosshairOverlay`: cv2 primitives sized per frame |
| `applyVerticalBarOverlay` | `(self, cv2_img, hud_dict)` | same |
| `applyDistanceBarOverlay` | `(self, cv2_img, hud_dict)` | same |
| `applyAttitudeOverlay` | `(self, cv2_img, hud_dict)` | Calls `overlayAttitude` inset at least `2·size + thickness` from every edge, because its arms reach `2·size` from its center. Draws a horizon bar inside its circle, tilted by roll and shifted by pitch. |
| `overlayRectangle` | `(self, cv2_img, top_left_px, bot_right_px, color_bgr, alpha)` | Copied from `overlay_rectangle`, blending only the region (section 6) |
| `getHudGeometry` | `(self, cv2_img)` → dict | `getOverlayFontDims` per-size cache, node:984–1000 |

Also add the constant `SHOW_HUD_GAUGES_ENABLED` and import `NavPose` from `nepi_interfaces.msg`. Only the node changes.

**Acceptance:**

- The heading readout matches the robot's navpose heading in the RUI to within 1°, and turning the robot by hand moves the gauge the right way.
- During an auto_move position step, the heading goal tick sits at the start heading, because yaw is held.
- The depth bar moves the right way when the robot changes depth, and its goal tick sits at the start depth plus the commanded vertical offset.
- The distance bar fills to near 100% when the command completes.
- The attitude reticle sits fully inside the frame at 640×480 and at 1920×1080 (check with the harness in section 8). Roll and pitch move the horizon bar the right way; confirm the signs on the robot.
- With no NavPose (topic empty), the gauges that need it are hidden and the text panel still works.

### Phase 3: helicopter-style HUD

**Layout:**

- A heading tape across the top, with a goal caret
- A roll scale arc under it
- A pitch ladder through the center, behind a fixed boresight (`overlayAttitude` at the center)
- A depth or altitude tape at the right edge, with a goal caret
- Distance-to-go and command text at the lower left
- The Move Goal marker drawn on the scene

Draw the pitch ladder **to scale with the scene**: one degree of pitch spans `img_h / height_deg` pixels, which is the same linear angle-to-pixel rule the img_if crosshairs use (`data_if.py:3480–3485`) and the parent uses for clicks ([auto_move_if.py:1553–1554](api/auto_move_if.py#L1553-L1554)). Then the horizon line sits on the real horizon when the camera faces forward.

These methods replace the Phase 2 heading and vertical gauges and take the same `hud_dict`:

| Method | Signature |
|---|---|
| `applyHeadingTapeOverlay` | `(self, cv2_img, hud_dict)` |
| `applyRollScaleOverlay` | `(self, cv2_img, hud_dict)` |
| `applyPitchLadderOverlay` | `(self, cv2_img, hud_dict)` |
| `applyVerticalTapeOverlay` | `(self, cv2_img, hud_dict)` |
| `getGoalDegs` | `(self, fwd_m, left_m, up_m)` → `[x_deg, y_deg]` or None |
| `updateMoveGoalMarker` | `(self, img_if, hud_dict)`. Replaces the Phase 0 block at node:899–920 and keeps its img_if calls. |

#### The Move Goal marker

**Which value places the goal relative to the robot's current pose:** the remaining offset `r = target − current`, in the body frame (forward, left, up).

- **After the engine fix** (section 11): `r = (−errors.x_m, −errors.y_m, −errors.z_m)` for goto_position. The marker then tracks the goal as the robot moves.
- **Today** the horizontal errors are swapped and rotated (section 4), so `r` cannot be read from them. Until the fix, pick the source by state:

| State | Place the marker from | Label it |
|---|---|---|
| A goto_position is running or has just ended | The commanded offset `(goals.x_m, goals.y_m, goals.z_m)`. That is the goal relative to where the step started: exact at the start of the step, and it drifts as the robot closes in. | Distance to go, from the length of the errors, which is correct today |
| Idle, with `has_click` and `click_depth_valid` | The planned move `(goto_x_m, goto_y_m, goto_z_m)` | "PLAN" |
| Velocity mode | The planned move | "OPEN LOOP", with no distance to go |
| goto_pose, takeoff, or nothing active | Nothing. Remove the marker with `remove_crosshair('Move Goal')`. | n/a |

**From `r = (fwd, left, up)` to img_if crosshair degrees:**

```python
    def getGoalDegs(self, fwd_m, left_m, up_m):
        if fwd_m <= 0.0:
            return None   # beside or behind the camera
        x_deg = math.degrees(math.atan2(left_m, fwd_m))                      # + is left of centre
        y_deg = -math.degrees(math.atan2(up_m, math.hypot(fwd_m, left_m)))   # + is below centre
        return [x_deg, y_deg]
```

The signs follow the img_if placement: a positive `x_deg` moves the marker left (`data_if.py:3468`) and a positive `y_deg` moves it down (`:3481`). This is the exact inverse of the parent's click-to-move conversion ([auto_move_if.py:1553–1562](api/auto_move_if.py#L1553-L1562)). So for a one-step plan the marker lands on the click crosshair, which makes a good acceptance check.

**Assumption, unconfirmed:** the camera looks along the body x axis, image right is body −y, image up is body +z, and the camera sits at the body origin. The parent makes the same assumption when it turns a click into a move: [auto_move_if.py:1553–1562](api/auto_move_if.py#L1553-L1562) applies no camera-to-body transform. So the marker agrees with how the goal was made, but no one has checked the camera mounting on the MATE robot. A camera mounted off-axis shifts the marker by the mounting angle. A camera offset from the robot's center adds parallax that grows as the goal gets closer.

**How the known defects affect the marker:**

- Placing it from the errors today would rotate it by twice the start heading. It would be correct only at ENU yaw 0° and 180°, and mirrored at 90°. Hence the goals-based placement above.
- The distance label is safe, because the swap and the rotation both leave a vector's length unchanged.
- For goto_location the goal is north/east/up from the start point, and the east value is overstated (no cos(latitude)). auto_move does not send goto_location, so leave that marker hidden.

**Goals outside the view.** The image interface clamps a crosshair to the frame edge (`data_if.py:3473–3476`, `:3486–3489`), so an angle beyond the field of view pins the marker to the edge. When `|x_deg| > width_deg / 2` or `|y_deg| > height_deg / 2`, or when `getGoalDegs` returns None, remove the img_if marker and draw an edge arrow in the node instead.

**Cost.** `add_crosshair_degs` is cheap when the value is unchanged. When the value changes, it publishes the interface's status and writes the overlay settings to the ROS param server (`data_if.py:4761–4769`). A marker that moved on every frame would do that at the 10 Hz publish rate. Round `x_deg` and `y_deg` to a step such as 0.5°, and call it only when the rounded value changes. Push enable and disable only on change, as the Phase 0 block already does.

**Field of view.** The node passes the source image's FOV to the image interface, defaulting to 100°×70°. The path is [node:857–863](scripts/auto_move_app_img_pub_node.py#L857-L863), then [node:961–967](scripts/auto_move_app_img_pub_node.py#L961-L967), then `data_if.py:3407–3408`. The parent computes the click bearing from the depth map's FOV first (`getSourceFovDeg`, [auto_move_if.py:1680](api/auto_move_if.py#L1680)). If the two differ, the marker lands off the click in proportion (see open questions).

#### Turning the HUD on and off

Use module constants backed by class attributes with defaults, read in `processOverlayImage`: `SHOW_HUD_TEXT_ENABLED`, `SHOW_HUD_GAUGES_ENABLED`, `SHOW_ATTITUDE_ENABLED` and `SHOW_MOVE_GOAL_ENABLED`. Every attribute then exists before the first frame, so defect (c) cannot come back.

**Optional, later: an operator switch in the RUI.** This one needs files beyond the node. The switch has to appear in the browser, and the node has no UI of its own. Carry it the way `show_crosshair_enabled` is carried:

- the parent default ([auto_move_if.py:328](api/auto_move_if.py#L328))
- the `set_show_crosshair` subscriber ([:624](api/auto_move_if.py#L624)) and `setShowCrosshairCb` ([:1454](api/auto_move_if.py#L1454))
- the status field ([auto_move_if.py:900](api/auto_move_if.py#L900), [NepiAppAutoMoveStatus.msg:109](msg/NepiAppAutoMoveStatus.msg#L109))
- the RUI checkbox ([rui/NepiAppAutoMove.js:387](rui/NepiAppAutoMove.js#L387), [:444–445](rui/NepiAppAutoMove.js#L444-L445))
- the node's `statusCb` ([node:680](scripts/auto_move_app_img_pub_node.py#L680))

Changes to `msg/` and `rui/` need a build.

**Acceptance:**

- Click right of center and Goto (a one-step plan, with the same FOV on both sides). The marker appears on the click crosshair and stays there while distance to go counts down.
- For a goal beside or behind the robot, there is no img_if marker, and an edge arrow shows the side.
- Pitch the robot nose-up: the ladder moves down, and the zero line stays on the real horizon with a forward camera.
- At a non-zero heading, the marker does not swing, because it is placed from goals. The raw per-axis error text, if you show it, does swing; that is the engine defect.
- Render time per frame stays well under the 100 ms that the 10 Hz cap allows ([node:94](scripts/auto_move_app_img_pub_node.py#L94)). Measure it with the harness.

---

## 8. Testing

### Offline render harness (no ROS)

Put the harness at `Apps/nepi_app_auto_move/test/hud_render_test.py`, with a first-line comment `# NOT DEPLOYED: offline HUD render harness`. It never reaches the robot:

- `deploy_app.sh` live-syncs only `scripts/`, `sdk/`, `api/` and `rui/` ([deploy_app.sh:143–209](deploy_app.sh#L143-L209)).
- [CMakeLists.txt:67–71](CMakeLists.txt#L67-L71) installs only the two named scripts, plus the `etc/`, `params/`, `sdk/`, `api/` and `rui/` folders.
- The build sync ([deploy_app.sh:94](deploy_app.sh#L94)) does copy the whole app folder into the build workspace's source tree, but nothing installs it from there.

The approach below was dry-run on 2026-10-15 against the current file. The node imported cleanly with stubs and drew without ROS. It also caught defect (a), because `applyCrosshairOverlay` was not a method.

1. **Stub the ROS imports.** Before importing the node, put stub modules in `sys.modules` for `nepi_sdk` (with `nepi_sdk`, `nepi_utils` and `nepi_img` attributes), `nepi_api.messages_if`, `nepi_api.node_if`, `nepi_api.system_if`, `nepi_api.data_if`, `sensor_msgs.msg`, `nepi_interfaces.msg` and `nepi_app_auto_move.msg`. The stub `nepi_img` needs only `get_optimal_font_dims` (copy `nepi_img.py:947–952`), and the stub `nepi_utils` needs only `check_ratio`.
2. **Import the node from its path:**
   ```python
   spec = importlib.util.spec_from_file_location('img_node', NODE_PATH)
   img_node = importlib.util.module_from_spec(spec)
   spec.loader.exec_module(img_node)
   ```
3. **Create the object without running `__init__`**, because `__init__` starts a ROS node: `hud = img_node.AutoMoveImgPub.__new__(img_node.AutoMoveImgPub)`. Then set `hud.msg_if` to a stub with `pub_info` and `pub_warn`.
4. **Build synthetic inputs:** an `rbx_status_dict` and a `navpose_dict` as plain dicts with the keys `convert_msg2dict` produces.
5. **Draw and save.** Call `hud.getHudValues(...)` and each `apply…Overlay` on `np.zeros((h, w, 3), np.uint8)` or on a sample frame. `cv2.imwrite` one PNG per scenario at 640×480, 1280×720 and 1920×1080.
6. **Assert on the `hud_dict` values**, because the mapping is where the bugs hide. For example, a goto_position at ENU yaw 90° with swapped and rotated errors must still give the right distance to go.

Scenarios to cover:

- Reset, with everything zero
- goto_position mid-move
- goto_position complete: `process_current` "None", `process_last` "GoTo Position"
- Velocity mode
- No RBX status
- No NavPose
- goto_position at yaw 90° (the rotation defect)
- A goal behind the robot
- Roll ±30° and pitch ±20°

### On the robot

| Scenario | How | Expect |
|---|---|---|
| Start and shutdown (Phase 0) | Enable the app, then disable it | `Initialization Complete` and no AttributeError. No traceback on shutdown. |
| Reset | Select a robot and send no command | All zeros, command "None" |
| Mid-move | Click, then Goto in position mode | Goals match `goto_x_m/y_m/z_m`, and distance to go falls |
| Completed | Wait for the move to end | Goals still shown, with "done" or "failed" from `cmd_success` |
| Velocity mode | Turn velocity mode on, then click and Goto | RBX goals and errors stay 0. The panel shows the planned move and velocity, labeled OPEN LOOP. |
| RBX disconnected | Stop the robot driver, or select no robot | Panel shows "RBX: none" within about 1 s, the marker is removed, and no exceptions appear |
| Rotation defect | Turn the robot to face north (ENU yaw about 90°), then goto_position 1 m forward | The raw per-axis errors point the wrong way. The HUD's distance to go and its marker are still right. |

---

## 9. Deploy and verify

1. **Deploy.** From the app folder, run `deploy_app.sh` (it needs `NEPI_REMOTE_SETUP` set; see the top of the script). It live-syncs `scripts/` into the running container at `/opt/nepi/nepi_engine/lib/nepi_app_auto_move/`, with `--delete` ([deploy_app.sh:143–150](deploy_app.sh#L143-L150)). A change to the node alone needs no build.
2. **Restart.** A live sync does not restart the node. Disable the auto_move app in the RUI, then enable it again. The parent launches the image node ([node:190–193](scripts/auto_move_app_img_pub_node.py#L190-L193)), so it starts again with the parent. Its watchdog also shuts it down once the parent's status stops, but only checks after the node's first minute ([node:469–482](scripts/auto_move_app_img_pub_node.py#L469-L482)).
3. **Confirm the new code is the code running.** Inside the container, run `grep -n "def overlayAttitude" /opt/nepi/nepi_engine/lib/nepi_app_auto_move/auto_move_app_img_pub_node.py`. In later phases, grep for the newest method name instead, such as `getHudValues`. Then check the node log for `Initialization Complete`.
4. **Builds.** Changes to `msg/` or `rui/` always need a build. Only the optional RUI switch touches them.

**An old mate `nepi_img.py` may still be on the device.** `SDK/deploy_sdk.sh` makes two copies, both without `--delete` ([deploy_sdk.sh:86–89](../../SDK/deploy_sdk.sh#L86-L89)):

- It copies the mate SDK folder over the device's installed engine SDK at `/opt/nepi/nepi_engine/lib/python3/dist-packages/nepi_sdk` ([deploy_sdk.sh:132–137](../../SDK/deploy_sdk.sh#L132-L137)).
- It also copies it into the build overlay folder `src/nepi_src/nepi_sdk`, which the engine build copies over its own nepi_sdk ([deploy_sdk.sh:94–112](../../SDK/deploy_sdk.sh#L94-L112)).

If that script was ever run, the mate `nepi_img.py` may stay on the device after it leaves the repo, until the engine nepi_sdk is rebuilt or redeployed. A rebuild only helps if the overlay folder in the build workspace no longer holds a `nepi_img.py`. On the production dev host, that folder held only `README.md` on 2026-10-15; the device's build workspace was not checked. The team reports that the two copies differ only in the logger import, so the risk is low. Still, if the image does not match what the source says it should draw, first diff the device's `nepi_img.py` against the engine source.

---

## 10. Rules for this work

- **Naming.** Everything new is private: camelCase with no docstring (`getHudValues`, `applyHudTextOverlay`). Subscriber callbacks end in `Cb` (`rbxStatusCb`, `navPoseCb`). Copied helpers are renamed as follows:

  | Engine helper | Name in the node |
  |---|---|
  | `overlay_target` | `overlayAttitude` |
  | `overlay_text` | `overlayText` |
  | `overlay_text_list` | `overlayTextList` |
  | `overlay_rectangle` | `overlayRectangle` |
  | `get_optimal_text_size` (if copied) | `getOptimalTextSize` |

  Do not rename existing `Cb` methods. They are registered with ROS and with timers.
- **No rospy.** Use `nepi_sdk.create_subscriber`, `nepi_sdk.create_namespace` and `nepi_sdk.convert_msg2dict`, and unregister handles with `handle.unregister()` the way `unsubscribeImgTopic` does.
- **Keep the render path cheap.**
  - Convert messages in their callbacks, not in the render.
  - Draw only inside `processOverlayImage`, which is skipped when nobody is watching ([node:852–855](scripts/auto_move_app_img_pub_node.py#L852-L855)).
  - Make no full-frame copies per layer; there is already one copy per frame (node:872).
  - Cache per-size geometry the way `getOverlayFontDims` does.
  - Call img_if setters only when a value changes.
- **Guard shared state the way the node already does.** Anything a subscriber thread writes and the render reads is a dict, replaced whole under a lock and read as one reference ([node:136–142](scripts/auto_move_app_img_pub_node.py#L136-L142), [:393–410](scripts/auto_move_app_img_pub_node.py#L393-L410)). Single values from the parent status are plain attributes, as `show_*_enabled` are today.
- **Default every new attribute.** Give it a class default before first use, and run the Phase 0 `ast` check after every phase.
- **Name color order.** Give every new color constant an `_RGB` or `_BGR` suffix.
- **Never hand-edit the generated RUI app registration files**, `Nepi_IF_Apps.js` and `NepiApps.js`.

---

## 11. Engine follow-ups (not this developer's job)

1. **Fix the goto error math in `device_if_rbx.py`.**
   - Write goto_position errors in natural order (`:1936`, `:1941`).
   - Fix the yaw sign in `nepi_nav.convert_point_enu2body` (`nepi_nav.py:1517–1518`). That also fixes the goal rewrite on a stop command (`device_if_rbx.py:1885`).
   - Use one sign convention for every command, and document it in `GotoErrors.msg`.
   - Report signed, wrapped yaw errors instead of `abs()`.
   - Add cos(latitude) to the goto_location goals and errors.
   - Honor −999 in the goto_position error check (`:1901`).
2. **Move goals and errors together.** Use the same frame and axis order, and write both in the same step, so a status message never pairs new goals with reset errors. Today the goals are written at `:1869` and the first errors at `:1936`.
3. **Add `get_goals_current` to `ConnectRBXDeviceIF`**, beside `get_errors_current` (`connect_device_if_rbx.py:459`). Callers can already read `get_status_msg().goals_current` (`:263`).
4. **Write `heading_deg`** in goals and errors, or remove it.

When items 1 and 2 land, change `getHudValues` to place the marker from `−errors` and to show signed per-axis errors.

---

## 12. Open questions for the team

1. **Camera mounting** on the MATE robot: does the camera look along body x? If not, what are the angles, and should the parent's click conversion use them too?
2. **FOV:** should the node use the depth map's FOV, like the parent's `getSourceFovDeg`, so that the marker lands exactly on the click?
3. **Which RBX driver** will the HUD run against for MATE, and does it publish a `navpose_topic` with heading, orientation and depth?
4. **Vertical tape:** depth, altitude, or both?
5. **Multi-step plans:** should the marker show the current step's goal (RBX) or the whole click (parent)?
6. **Signs:** which roll and pitch sign conventions does that robot's NavPose use? Confirm on the robot before trusting the ladder.
7. **Operator switch:** is an RUI on/off switch for the HUD wanted? It needs msg, RUI and a build.
8. **Engine fix timing:** when is the error-math fix planned? Until it lands, the marker is placed from goals, not errors.
