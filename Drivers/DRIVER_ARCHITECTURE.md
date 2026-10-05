# NEPI Driver Architecture - How Drivers Work & How to Write One

This folder holds a **complete, copy-paste driver template for each NEPI device
type**. Every template is a working skeleton in the real house style, with the
correct interface class wired up and `TODO:` markers where you plug in your
device. This document explains the architecture they all share so you know
*why* each file looks the way it does. For the hands-on workflow (copy a
template, rename it, deploy it), see **`GETTING_STARTED.md`**.

```
nepi_driver_templates/
├── DRIVER_ARCHITECTURE.md   ← you are here (architecture reference)
├── GETTING_STARTED.md       ← create + deploy workflow
├── TEMPLATE_DRIFT_REPORT.md ← what changed when these were last realigned, and why
├── deploy_nepi_drivers.sh   ← deploys template drivers to the target src tree (all types, or pass categories)
├── idx_drivers/             ← Imaging / camera      (LAUNCH discovery + separate driver file)
├── lsx_drivers/             ← Lighting              (CALL discovery, serial in-node)
├── ptx_drivers/             ← Pan-Tilt (2 axes)     (CALL discovery, serial in-node)
├── npx_drivers/             ← Navigation / NavPose  (CALL discovery, TCP in-node)
├── rbx_drivers/             ← Robot / vehicle       (CALL discovery + optional companion node)
└── svx_drivers/             ← Servo (1 axis)        (CALL discovery, several channels per path)
```

> Each folder holds one template file-set, named `<cat>_template_*`. The FOLDER
> is named `<cat>_drivers` -- that is what `deploy_nepi_drivers.sh` globs and
> what the folder is called in the real `nepi_drivers` tree. Only the files get
> renamed when you make the template your own.

> These templates were reverse-engineered from the shipped drivers in
> `nepi_engine_ws/src/nepi_drivers/` and the interface classes in
> `nepi_engine_ws/src/nepi_engine/nepi_api/`, and were last realigned against
> them on 2026-09-29 — see `TEMPLATE_DRIFT_REPORT.md` for exactly what moved.
> Where the older `nepi_drivers/CLAUDE.md` describes a free
> `discoveryFunction(...)` and `nepi_sdk.launch_node()`, that is stale — the
> current code is **class-based** and launches via
> **`nepi_drvs.launchDriverNode()`**, which is what these templates use.

---

## 1. The big picture

A NEPI driver is **not** a monolith. It is a small set of files that plug into
two pieces of framework you do **not** write:

- **`drivers_mgr`** (in `nepi_managers`) — the manager. It scans the install
  folder, reads each driver's `params.yaml`, and drives discovery on a ~1–3 s
  poll. It launches/kills nodes and owns the shared `active_paths_list`.
- **A device interface class** (`IDXDeviceIF`, `LSXDeviceIF`, `NPXDeviceIF`,
  `PTXActuatorIF`, `RBXRobotIF`, `SVXActuatorIF`, in `nepi_api`) — the ROS
  surface. It creates
  **every** publisher, subscriber, and service and exposes the standard NEPI
  API for that device type. Your node hands it callbacks; it does the ROS.

Your driver only provides three things: **how to find the hardware**
(discovery), **how to talk to it** (node + optional driver), and **what it is**
(params manifest).

```
                         reads params.yaml, polls discovery
   ┌────────────┐  ───────────────────────────────────────────►  ┌──────────────────┐
   │ drivers_mgr│                                                  │ <cat>_discovery.py│
   └────────────┘  ◄─── returns active_paths_list ───────────────  └──────────────────┘
                                                                        │ launchDriverNode()
                                                                        │ + set ~drv_dict param
                                                                        ▼
                                             ┌───────────────────────────────────────┐
                                             │ <cat>_node.py                           │
                                             │   reads ~drv_dict                        │
                                             │   (optional) imports <cat>_driver.py     │
                                             │   builds settings                        │
                                             │   ┌───────────────────────────────────┐ │
                                             │   │  <Type>DeviceIF(callbacks...)      │ │  ← creates all ROS topics/services
                                             │   └───────────────────────────────────┘ │
                                             │   spin()                                 │
                                             └───────────────────────────────────────┘
```

---

## 2. The files in a driver

Every driver is a flat set of files named `{cat}_{device}_{role}` where `{cat}`
is the 3-letter type prefix (`idx`, `lsx`, `npx`, `ptx`, `rbx`, `svx`).

| File | Required? | Role |
|---|---|---|
| `{cat}_{device}_params.yaml` | **yes** | Manifest: names the files/classes, declares user options |
| `{cat}_{device}_discovery.py` | **yes** | Finds hardware, launches a node per device |
| `{cat}_{device}_node.py` | **yes** | The ROS node; registers with the interface class |
| `{cat}_{device}_driver.py` | optional | Raw hardware I/O, factored out (IDX cameras use this; serial devices usually don't) |

**Rule of thumb for the driver file:** if hardware I/O is a couple of serial
writes (LSX/PTX), do it inline in the node. If it involves an SDK, streaming
threads, or frame buffers (IDX cameras), factor it into a `_driver.py` the node
imports via `nepi_drvs.importDriverClass()`.

---

## 3. Two discovery models — `CALL` vs `LAUNCH`

This is the single most important architectural fork, set by
`DISCOVERY_DICT.process` in `params.yaml`. `drivers_mgr` dispatches on it.

### `CALL` — used by LSX, PTX, NPX, RBX, SVX
`drivers_mgr` imports your discovery class, instantiates it **once with no
arguments**, and then **calls `discoveryFunction(...)` every poll cycle** inside
its own process.

```python
class MyDiscovery:
    def __init__(self):                      # no init_node(), no spin()!
        self.logger = nepi_sdk.logger(log_name=...)

    def discoveryFunction(self, available_paths_list, active_paths_list,
                          base_namespace, drv_dict, retry_enabled=True):
        ...
        return active_paths_list             # MUST return the updated list

    def killAllDevices(self, active_paths_list):   # called on teardown
        ...
```

Use `CALL` when enumeration is cheap (a quick serial probe, checking a config
value). It is the simplest model.

### `LAUNCH` — used by IDX
Camera enumeration can be slow or blocking, so discovery runs as **its own
node**. `drivers_mgr` writes the driver's `drv_dict` to
`<discovery_node>/drv_dict` and `launchDriverNode()`s the discovery script.
Your discovery then `init_node()`s, reads `~drv_dict`, schedules its own
self-rescheduling `detectAndManageDevices(self, timer)` timer, and `spin()`s.

```python
class MyDiscovery:
    def __init__(self):
        nepi_sdk.init_node(name=self.DEFAULT_NODE_NAME)   # yes, it's a node
        self.updateDiscoveryOptions()                     # reads ~drv_dict
        nepi_sdk.start_timer_process(1.0, self.detectAndManageDevices, oneshot=True)
        nepi_sdk.on_shutdown(self.cleanup_actions)
        nepi_sdk.spin()
```

`drivers_mgr` afterward only checks whether the discovery node is still alive.

Because a `LAUNCH` discovery is long-lived, it must **re-read `~drv_dict` on a
timer**, not only once in `__init__`. `drivers_mgr` rewrites that param when an
operator edits the driver's `OPTIONS` in the RUI, and a discovery that reads it
once never sees the edit. `idx_v4l2_discovery.py` does this with a 1 s
self-rescheduling `updateDriverDictCb`, and the IDX template follows it.

> Both models end up launching device nodes the **same** way (Section 4). Only
> *how discovery itself runs* differs.

### One path, several devices (SVX)
The other categories claim a device path once: one port, one device, one node.
A servo controller drives several channels over one transport, and **each
channel is its own SVX device with its own node**. That discovery therefore
does not claim a path — it reconciles a *desired set* of `(path, channel)`
pairs against what is running, every pass:

```
desired = every configured channel on every detected board
launch  = desired - running        kill = running - desired
```

The board's path joins `active_paths_list` once **any** of its channels is
live and leaves only when the **last** one goes. Two rules make this safe:
`deepcopy` the `drv_dict` per launch (one `DEVICE_DICT` must never leak into a
sibling's), and distinguish *enumeration failed* from *no boards found* —
reconciling against an empty list after a failed scan tears down every node and
respawns it on the next pass.

---

## 4. The launch handshake (how config reaches the node)

When discovery decides a device is present, it does exactly two things, in this
order:

1. **Write the config to the ROS param server** at `<base>/<node_name>/drv_dict`.
   This is the full `drv_dict` (the parsed `params.yaml` `driver:` block) with a
   `DEVICE_DICT` sub-dict added — the per-instance facts discovery just learned
   (device name, path, serial addr, baud, ip:port, model, …).

   ```python
   self.drv_dict['DEVICE_DICT'] = {'device_name': ..., 'device_path': ..., ...}
   dict_param_name = nepi_sdk.create_namespace(base_namespace, node_name + "/drv_dict")
   nepi_sdk.set_param(dict_param_name, self.drv_dict)
   ```

2. **Launch the node:**
   ```python
   nepi_drvs.launchDriverNode(file_name, node_name, device_path=path_str)
   #   -> shells: rosrun nepi_drivers <file_name> __name:=<node_name> [_device_path:=...]
   ```

The node then reads its config straight back:
```python
self.drv_dict = nepi_sdk.get_param('~drv_dict', dict())
self.device_name = self.drv_dict['DEVICE_DICT']['device_name']
```

`drivers_mgr` injects one more key before the node sees it: **`drv_dict['path']`**
= the install directory (used by IDX to locate its `_driver.py`).

### Retry & backoff (both models)
- `NODE_LOAD_TIME_SEC` (~10 s) + a `launch_time_dict[path]` timestamp stop rapid
  relaunch loops.
- `dont_retry_list` permanently blacklists a device that failed when
  `retry_enabled` is False.
- Discovery also *purges*: each cycle it checks its launched nodes
  (`sub_process.poll()` and "is the path still present?") and kills + forgets
  any that died or disconnected, so the next cycle can rediscover them.

---

## 5. The node → interface-class contract (per type)

The node's real job: connect to hardware, build settings, then construct the
type's interface class, passing a callback for every capability the hardware
has and `None` for the rest. The interface class advertises only what you wire
up. `device_info` is required by **all** of them and needs these exact keys:

```python
device_info = dict(device_name="", path="", serial_number="", hw_version="", sw_version="")
```

| Type | Interface class | The callbacks that define it |
|---|---|---|
| **IDX** | `IDXDeviceIF` | `getColorImage`/`stopColorImageAcquisition` (and `getDepthMap`, `getPointcloud` for 3D), `getFramerate`, `setMaxFramerate`, `data_products=[...]`. `getColorImg()` returns a **5-tuple** `(ret, msg, cv2_img, timestamp, encoding)`. Optional: `getFOV`/`perspective`, `get_rtsp_url`, `setResolutionRatio`, `setContrast`/`Brightness`/`Thresholding`/`RangeRatio`, `setAutoAdjustRatio`+`autoAdjustControls`, `getNavPoseCb`+`navpose_update_rate`. |
| **LSX** | `LSXDeviceIF` | `getStatusFunction`→`DeviceLSXStatus` msg, `turnOnOffFunction`, `setIntensityRatioFunction` (0.0–1.0); optional color/kelvin/strobe/blink. `reports_temp`/`reports_power` flags. |
| **NPX** | `NPXDeviceIF` | Just `getNavPoseCb` — return a **deep copy** of `nepi_nav.BLANK_NAVPOSE_DICT` with the `has_*` flags set for whatever the sensor provides: location, heading, altitude, orientation, position, depth, **pan_tilt**. The IF polls it at `max_navpose_update_rate`. |
| **PTX** | `PTXActuatorIF` | Jog (`movePanCb`/`moveTiltCb`/`stopMovingCb`, and `movePan`/`moveTiltSpeedRatioCb` for jog-at-a-speed), absolute (`gotoPositionCb`, `getPositionCb`→`[pan_deg,tilt_deg]`), soft limits, whole-unit speed (`get/setSpeedRatioCb`, `get/setSpeedMaxCb` in **dps**), per-axis speed (`get/setPan`/`TiltSpeedRatioCb`), homing, optional `getNavPoseCb`. Requires `factoryControls` + a `factoryLimits` dict with all 8 `*_pan/tilt_hard/softstop_deg` keys. |
| **RBX** | `RBXRobotIF` | Ordered lists `states`/`modes`/`setup_actions`/`go_actions` + `get/setStateInd`, `get/setModeInd`, `setSetupActionInd`, `setGoActionInd`, `checkStopFunction`, `getBatteryPercentFunction`, an `AxisControls` DOF mask; optional `get/setMotorControlRatio(s)`, autonomous `goto*`/`gotoVelocityFunction`/home/`getNavPoseCb`. The IF returns the **index** into each list. |
| **SVX** | `SVXActuatorIF` | **One single-axis actuator.** `stopMovingCb`, `gotoPositionCb(position_deg)`, `getPositionCb`, `get/setSoftLimitsCb`, `get/setSpeedRatioCb`, `get/setSpeedMaxCb`, `goHomeCb`, `setHomePosition`/`HereCb`, `deviceResetCb`; optional `get/setSpinDirection`. Requires `factoryControls` + a `factoryLimits` dict with the 4 `min/max_hard/softstop_deg` keys — the softstops **mirror** the hardstops, which are the single clamp range. |

A pan/tilt turret built from servos is **two SVX nodes**, coordinated above by a
PTX driver or an app — not one SVX device with two axes. If your hardware has
two coupled axes you want PTX, not SVX.

### Capability flags come from which callbacks are non-`None`
This is why "pass a callback for every capability the hardware has and `None`
for the rest" is a real statement about the hardware, not a style. `PTXActuatorIF`
derives, for example:

| flag | condition |
|---|---|
| `has_absolute_positioning` | `gotoPositionCb` **or** `gotoPanPositionCb` |
| `has_seperate_pan_tilt_control` | `gotoPanPositionCb` **and** `gotoTiltPositionCb` |
| `has_timed_positioning` | `movePanCb` **or** `moveTiltCb` |
| `has_timed_speed_positioning` | `movePanSpeedRatioCb` **or** `moveTiltSpeedRatioCb` |
| `has_adjustable_speed` | `getSpeedRatioCb` **and** `setSpeedRatioCb` |
| `has_seperate_pan_tilt_speed` | **all four** of `get`/`setPan`/`TiltSpeedRatioCb` |

Wiring three of the four per-axis speed callbacks gets you nothing — the flag
stays `False` and the RUI never shows per-axis speed.

### Settings — the shared plumbing every node repeats
Runtime-tunable parameters (shown in the RUI) use one pattern across all six
types. Every device IF takes **exactly two** settings arguments and builds a
`SettingsIF` only when both are non-`None`:

```python
self.<cat>_if = <Type>DeviceIF(device_info = ...,
                               getSettingsFunction = self.getSettingsFunction,
                               setSettingFunction  = self.setSettingFunction,
                               ...)
```

A device with no settings passes **neither** — that is the correct state, not an
omission. The node implements four methods:

```python
def initSettingsDict(self):      # build the controls dict once, at startup
def refreshSettingsDict(self):   # read live values (and bounds/options) back
def getSettingsFunction(self):   # -> the current controls dict, no arguments
def setSettingFunction(self, setting_name, setting_value):
    return success, msg, self.settings_dict      # ALL THREE
```

`setSettingFunction` returns all three because `SettingsIF` replaces its own
dict with the third element. It is also called as a bare statement during
`SettingsIF.init()`, so its return being discarded must be harmless.

**The settings dict IS a `nepi_controls` controls dict.** Build a plain init
dict — each entry keyed by setting name, carrying a `type` from
`nepi_controls.CONTROL_TYPES`, a **typed** `default`, plus `bounds` for
`Int`/`Float` or `options` for `Menu`/`Selection` — then hand the whole thing to
`nepi_controls.create_controls_dict()`. Read and write through
`nepi_controls.get_value()` / `set_value()` / `set_bounds()` / `set_options()`,
never by indexing the dict directly.

The templates keep `CAP_SETTINGS` / `FACTORY_SETTINGS` /
`FACTORY_SETTINGS_OVERRIDES` / `settingFunctions` as the node's own **source
tables** and convert them in `initSettingsDict()`, which is what the shipped
drivers do. The shipped drivers resolve per-setting getters/setters through
module-level `global` functions + `globals()[name]`; the templates use
`getattr(self, name)` on bound methods instead — equivalent and easier to read.
Either is fine; pick one and be consistent.

`FACTORY_CONTROLS` is separate: it's the initial device state/config the IF
should assume (frame ids, initial on/off, FOV, etc.), not user-facing settings.
Note the IF merges a `factoryControls` key **only if that key already exists in
its own `FACTORY_CONTROLS_DICT`** — a key spelt any other way is silently
ignored.

> **Retired — these no longer exist on any device IF.** Passing one raises
> `TypeError` at construction:
> `capSettings=` · `factorySettings=` · `settingUpdateFunction=` ·
> `getCapSettings()` · `getFactorySettings()` · `getSettings()` · `setSetting()` ·
> `settingUpdateFunction()` · `nepi_settings.get_data_from_setting()` ·
> `nepi_settings.check_valid_setting()`.
> `device_if_ptx.py` still *accepts* a `getCapSettingsFunction` argument, but it
> is inert — never forwarded to `SettingsIF`, which has no such parameter.
> Capability data rides in the controls dict itself. Do not pass it.

---

## 6. The params.yaml manifest

Single top-level `driver:` mapping. `drivers_mgr` reads it to wire everything
up; the values must match the actual files/classes on disk.

```yaml
driver:
  pkg_name: LSX_TEMPLATE            # unique id; also the PKG_NAME in the .py files
  display_name: Template LED Light  # shown in the RUI
  description: ...
  type: LSX                         # IDX | LSX | NPX | PTX | RBX | SVX
  group_id: None                    # optional grouping (e.g. ONVIF)
  NODE_DICT:      { file_name: ..._node.py,      class_name: ...Node }
  DRIVER_DICT:    { file_name: None,             class_name: None }     # or a real driver file
  DISCOVERY_DICT:
    file_name: ..._discovery.py
    class_name: ...Discovery
    process: CALL                   # CALL | LAUNCH  (see Section 3)
    OPTIONS:                        # user controls; discovery reads ['value']
      baud_rate: { type: Selection, options: [All, '9600', ...], default: All, value: All }
      start_addr: { type: Int, options: ['1','255'], default: '1', value: '1' }
```

`drivers_mgr` builds a `nepi_controls` controls dict out of `OPTIONS` before the
RUI renders them, so **`type` must be a `CONTROL_TYPE`** — `Selection`, `String`,
`Int`, `Float`, `Toggle`, `Menu`, … `Discrete` is *not* one of them, and an
option declared that way is silently dropped and never appears in the RUI. (No
shipped driver uses `Discrete` any more.) An `OPTIONS` entry may also carry a
`description`, which the RUI shows alongside the control.

One thing that differs between the two surfaces: on an `Int`/`Float` **OPTION**,
the two-entry `options` list is still read as the min/max **bounds** pair. That
form is correct here, and *not* in a node's `CAP_SETTINGS`, where a numeric
control's min/max moved to `bounds`.

At runtime the dict the node receives = this `driver:` block **plus** the
injected `path` and `DEVICE_DICT` keys. `OPTIONS` value tokens like
`NEPI_NAV_IP_NMEA` or `SERIAL_DEVICES` are placeholders the framework resolves
at load time.

---

## 7. Build & install

Drivers are **not** registered individually. `nepi_drivers/CMakeLists.txt`
installs each category directory wholesale:

```cmake
install(DIRECTORY lsx_drivers/ DESTINATION /opt/nepi/nepi_engine/lib/nepi_drivers
        FILES_MATCHING PATTERN "*.py")
install(DIRECTORY lsx_drivers/ DESTINATION /opt/nepi/nepi_engine/lib/nepi_drivers
        FILES_MATCHING PATTERN "*.yaml")
```

So **all `*.py` and `*.yaml` files land flat** in
`/opt/nepi/nepi_engine/lib/nepi_drivers/`, and `drivers_mgr` scans that flat
folder. Adding a new driver needs **no CMake edit** — just drop the file-set
into the right `<cat>_drivers/` source folder and rebuild.

> Note: in this templates repo each category's template file-set is kept in its
> own `<cat>_drivers/` folder, matching the real `nepi_drivers` tree, where the
> files live **flat** inside `<cat>_drivers/` — not in per-driver subfolders.

> **Creating and deploying a driver from these templates** — the step-by-step
> workflow (copy a template into your own repo, rename, fill TODOs, run
> `deploy_nepi_drivers.sh`) lives in `GETTING_STARTED.md`.

---

## 8. Gotchas (learned from the shipped drivers)

- **`class_name` mismatches.** e.g. `ptx_sidus_ss109_serial_params.yaml` declares
  `SidusSS109SerialNode` but the class is actually `SidusSS109SerialPTXNode`.
  The yaml wins for launching, but a mismatch means the node never starts. Keep
  them in sync.
- **Don't mutate `BLANK_NAVPOSE_DICT` in place.** It's a module-level dict.
  `copy.deepcopy()` it first (a shipped PTX driver mutates the shared one —
  the templates fix this). There's also a real `time_oreantation` typo in that
  same node; the correct key is `time_orientation`.
- **Never enumerate the `has_*` navpose flags by hand.** The dict has gained
  sub-reports over time (`has_pan_tilt` is the newest), and a hardcoded list in
  a disconnect handler leaves any newer flag stuck `True`. Iterate the keys
  starting with `has_`.
- **A long-lived `LAUNCH` discovery must re-read `~drv_dict` on a timer**, or an
  operator's `OPTIONS` edit in the RUI never reaches it (Section 3).
- **`'Discrete'` is not a control type** — in a node's `CAP_SETTINGS` or in a
  params `OPTIONS`. `create_controls_dict()` wraps each entry in a bare
  `except`, so the setting is silently dropped and simply never appears in the
  RUI. A named option list is a `'Selection'`.
- **A `factoryControls` key the IF does not already know is silently ignored.**
  The IF copies a key only if it exists in its own `FACTORY_CONTROLS_DICT`, so
  `reverse_pan_control` where the IF wants `reverse_pan_enabled` does nothing at
  all — no error, no effect.
- **`getSpeedMaxCb` is called during `PTXActuatorIF` construction**, so it must
  return a real number immediately. `getPanSpeedRatioCb` must never return
  `None` either — the IF applies `math.floor()` to it directly.
- **Serial paths are not stable across reboots** (`/dev/ttyUSB0` ↔ `ttyUSB1`).
  Discovery re-probes every port each cycle; never hard-code an index.
- **CALL discovery must not call `init_node()`/`spin()`** — it runs inside
  `drivers_mgr`. Only `LAUNCH` discovery is its own node.
- **Surface hardware failures with `pub_warn`, not `pub_debug`**, so they show
  without Debug Mode enabled.
- **No hardware-in-the-loop CI.** These drivers aren't covered by automated
  tests requiring physical hardware — verify on the bench.
```
