# NEPI App Add-On Patterns — Real Code for Each Capability

Companion to `APP_ARCHITECTURE.md` Sections 3 and 3a. Every app shares the same
`NodeClassIF` + `ControlsIF` skeleton (the template); what makes your app *do*
something is which of these patterns you compose inside it. Each section below
shows the actual code from the shipped app that does it best, trimmed to the
essentials, so you can copy the shape rather than reinvent it.

All quoted code is from `nepi_engine_ws/src/nepi_apps/`.

> **On `nepi_app_fake_gps`.** It was the first controls migration and is still
> the clearest single-set worked example, so
> `src/nepi_apps/CONTROLS_MIGRATION_PATTERN.md` quotes it throughout. The
> package itself has been **deleted**: its simulator and its MAVLink
> `GPS_INPUT` injection were folded into `nepi_app_nav_sim` as a third
> simulator kind, and the running code is now `GpsSimInstance` in
> `nav_sim_app_node.py`. Where a pattern below is pure shape, the fake-GPS
> version is quoted as history and the current app to open is named beside it.

---

## 1. Background worker thread
*Shape from the retired `nepi_app_fake_gps` (`_simThreadLoop`); open
`nepi_app_nav_sim` for the running version*

For continuous work (simulation, data pumping, polling hardware) that must not
block ROS callbacks. Three rules: a daemon `threading.Thread`, a
`threading.Lock` around every read/write of shared state, and an `_alive` flag
checked together with `nepi_sdk.is_shutdown()`.

```python
import threading, copy

# In __init__, BEFORE NodeClassIF (state must exist when callbacks fire):
self._lock = threading.Lock()
self._alive = True
self.current_state = ...          # everything the thread and callbacks share

# In __init__, AFTER node_if is ready:
threading.Thread(target=self._simThreadLoop, daemon=True).start()

def _simThreadLoop(self):
    while self._alive and not nepi_sdk.is_shutdown():
        with self._lock:
            self._advanceWork()                       # mutate shared state
            snapshot = copy.deepcopy(self.current_state)  # copy OUT under lock
        self._publishOutputs(snapshot)                # publish OUTSIDE the lock
        nepi_sdk.sleep(1.0 / self.update_rate_hz)

def cleanup_actions(self):
    self._alive = False
```

Key details:
- Copy state out under the lock, publish outside it — publishing inside the
  lock stalls your subscribers' callbacks.
- ROS subscriber callbacks that mutate the same state also take `self._lock`.
- The thread is `daemon=True` so a hung loop can't block node shutdown.

---

## 2. Publishing NavPose data
*Crib from: `nepi_app_nav_sim`, `nepi_app_file_pub_depthmap`
(`_setupNavpose`, `publishNavpose`)*

Use `NavPoseIF` from `nepi_api.data_if` when your app produces position /
heading / orientation. Enable only the `pub_*` flags for what you actually
provide, and set the matching `has_*` flags in the dict you publish.

```python
from nepi_sdk import nepi_nav
from nepi_api.data_if import NavPoseIF

self._navpose_if = NavPoseIF(
    namespace=nepi_sdk.create_namespace(self.node_namespace, 'navpose'),
    data_source_description='fake_gps',
    data_ref_description='WGS84',
    pub_navpose=True, pub_location=True, pub_altitude=True,
    pub_heading=True, pub_orientation=True,
    msg_if=self.msg_if,
)

def publishNavpose(self, geo, heading_deg, yaw_enu_deg):
    t = nepi_sdk.get_time()
    d = copy.deepcopy(nepi_nav.BLANK_NAVPOSE_DICT)   # ALWAYS deepcopy the blank
    d['has_location'] = True
    d['latitude'] = geo.latitude
    d['longitude'] = geo.longitude
    d['time_location'] = t
    d['has_altitude'] = True
    d['altitude_m'] = geo.altitude
    d['time_altitude'] = t
    d['has_heading'] = True
    d['heading_deg'] = heading_deg      # degrees true north
    d['time_heading'] = t
    d['has_orientation'] = True
    d['roll_deg'] = 0.0
    d['pitch_deg'] = 0.0
    d['yaw_deg'] = yaw_enu_deg          # ENU frame
    d['time_orientation'] = t
    self._navpose_if.publish_navpose(d)

def cleanup_actions(self):
    if self._navpose_if is not None:
        self._navpose_if.unregister_pubs()
```

Gotchas:
- `copy.deepcopy(BLANK_NAVPOSE_DICT)` every time — mutating the module-level
  blank dict corrupts every other user of it.
- **`NavPoseIF` handed a NODE namespace silently occupies `<node>/navpose`**
  (it appends its own `data_product` when the basename is not already it). A
  control set named `navpose` would land on exactly that namespace and
  advertise a second `<node>/navpose/status` of a different message type.
  `nepi_app_file_pub_depthmap` names its set `navpose_source` for that reason.
  Nothing warns about the overlap at either end.
- Stamp each sub-report's `time_*` field; consumers use them for staleness.
- Wrap construction in try/except and tolerate `self._navpose_if = None` so
  the app still runs if navpose output is unavailable.

---

## 3. Periodic resource discovery
*Crib from: `nepi_app_image_viewer` (`updaterCb`)*

When your app consumes topics that appear/disappear at runtime (cameras, AI
detectors, PT units), rescan on a self-rescheduling **oneshot** timer. Oneshot
+ reschedule (rather than a periodic timer) guarantees scans never overlap if
one runs long.

```python
# In __init__:
nepi_sdk.start_timer_process(1.0, self.updaterCb, oneshot=True)

def updaterCb(self, timer):
    topics = nepi_sdk.find_topics_by_msg('Image')     # search by msg type name
    self.available_image_topics = list(topics)
    # ... react: validate current selections, auto-connect, update status ...
    nepi_sdk.start_timer_process(1.0, self.updaterCb, oneshot=True)   # re-arm
```

The same idiom takes a cached topic table when the scan is expensive — pass the
lists you already hold so the search does not re-enumerate the graph:

```python
topics = nepi_sdk.find_topics_by_msg('DevicePTXStatus',
                                     topics_list=self.active_topics,
                                     types_list=self.active_topic_types)
```

Publish whatever you discovered (e.g. `available_image_topics`) in your status
message so the RUI can offer it as a dropdown.

---

## 4. Save-data (snapshots / logging)
*Crib from: `nepi_app_image_viewer`*

Interesting shape: image_viewer wires SaveData **without touching its node** —
the save pipeline belongs to the data-producing nodes; the app only adds UI
and client control.

RUI page — drop in the standard SaveData panel:

```jsx
import NepiIFSaveData from "./Nepi_IF_SaveData"

// in render(), pointing at the namespace whose data products you want saved:
<NepiIFSaveData
  saveNamespace={allSaveNamespace}
  title={"Nepi_IF_SaveData"}
/>
```

Connector — add SaveData passthroughs for programmatic control:

```python
from nepi_api.connect_system_if import ConnectSaveDataIF

self.con_save_data_if = ConnectSaveDataIF(namespace=self.namespace)
# exposes get_data_products, save_data_pub, save_data_rate_pub, snapshot_pub, ...
```

If your app *produces* data it wants saved, publish through the `nepi_api`
data classes (Section 6) — they integrate with the save-data system on the
publishing side.

---

## 5. Driving devices / consuming other nodes
*Crib from: `../nepi_connect_templates/` — eight complete worked examples,
one per device family*

To control a NEPI device (or another app), use the `Connect*` classes from
`nepi_api` — they wrap the target's topic/service layout behind methods and
callbacks. `nepi_api` ships one per family: `connect_device_if_idx`, `_lsx`,
`_npx`, `_ptx`, `_rbx`, `_motor`, `_svx`, plus `connect_data_if` for data
sources. Each owns a `<node>/<family>_connect` namespace carrying a
`ConnectIFStatus` selector and a `select_topic` subscriber, and each inherits
`get_available_topics` / `get_selected_topic` / `check_connection` /
`unregister` from the shared `ConnectNodeIF` base.

The simplest use is to construct one and let it auto-discover and auto-select:

```python
from nepi_api.connect_device_if_ptx import ConnectPTXDeviceIF

# Every Connect*IF calls wait_for_connect_ready() at the end of its own
# __init__, so there is no second call needed here.
self.pt_connect_if = ConnectPTXDeviceIF(msg_if=self.msg_if)
```

To attach to a topic you discovered yourself instead (pattern 3):

```python
from nepi_api.connect_device_if_ptx import ConnectPTXDeviceIF

def subscribe_pt_topic(self, topic):
    if self.pt_connect_if is not None:
        self.unsubscribe_pt_topic()               # always detach the old one first

    pt_connect_if = ConnectPTXDeviceIF(namespace=topic,
                                       panTiltCb=self.panTiltCb,    # status callbacks
                                       stopPanCb=self.stopPanCb,
                                       stopTiltCb=self.stopTiltCb,
                                       msg_if=self.msg_if)
    ready = pt_connect_if.wait_for_ready()        # gate on ready before using
    if ready:
        self.pt_connect_if = pt_connect_if
        self.pt_connect_if.set_speed_ratio(self.speed_ratio)   # then command it
```

The same shape applies to other targets — check `nepi_api` for the available
`connect_device_if_*` / `connect_app_*` / `connect_system_if` /
`connect_process_if_targets` classes. For AI detection consumers, subscribe to
the detector's `Targets` / `TargetingStatus` topics from `nepi_interfaces.msg`
directly, or use `connect_process_if_targets`.

Rules:
- `wait_for_ready()` before first use; treat False as "target not there".
- Keep exactly one live connection per target; unsubscribe/unregister before
  reconnecting to a new namespace.
- Namespaces for `Connect*` classes are the **target's** namespace (a topic
  you discovered), never your own node's.
- **Running more than one interface of the same family? Pass
  `auto_select_enabled=False`.** With it on, every interface independently
  grabs the first discovered device, so a pan/tilt pair built from two SVX
  servos both land on the same channel. `ConnectIDXDeviceIF` is the one class
  that does **not** accept the argument — it always auto-selects, and passing
  it raises `TypeError`.
- The selector lives on the connect namespace (`<node>/<family>_connect`), one
  level below the node namespace. Publishing `select_topic` on the node
  namespace instead is silent and does nothing.

---

## 6. Publishing images / video (data products)
*Crib from: `nepi_app_file_pub_img` / `nepi_app_file_pub_vid`*

Publish image data through `ColorImageIF` from `nepi_api.data_if` rather than
raw `sensor_msgs/Image` publishers — you get the standard NEPI namespace
layout, RUI viewability, and save-data integration for free.

```python
from nepi_api.data_if import ColorImageIF

data_product = 'color_image'
self.image_if = ColorImageIF(namespace=self.node_namespace,
                             data_product=data_product,
                             data_source_description='file',
                             data_ref_description='source',
                             perspective='pov',
                             log_name=data_product,
                             msg_if=self.msg_if)
ready = self.image_if.wait_for_ready()

# file_pub_img registers publishers lazily: unregister until there are
# subscribers/need, and provide a needs-update callback:
self.image_if.unregister_pubs()
self.image_if.set_image_callback('needs_update_callback', self.publish_img)

def publish_img(self):
    if self.cv2_img is not None and self.image_if is not None:
        with self.cv2_lock:
            self.image_if.publish_cv2_img(self.cv2_img, encoding=self.encoding,
                                          width_deg=self.width_deg,
                                          height_deg=self.height_deg,
                                          pub_twice=self.paused)
```

Notes:
- `publish_cv2_img` takes a cv2 (numpy BGR) image; `encoding` is typically
  `'bgr8'`.
- Guard the cv2 buffer with a lock if a worker thread (pattern 1) fills it.
- `register_pubs()` / `unregister_pubs()` let you gate publishing on demand
  (file_pub_img toggles them as its stream starts/stops).
- Check `nepi_api/data_if.py` for the other data classes (depth maps,
  pointclouds, etc.) — same lifecycle: construct → `wait_for_ready()` →
  `publish_*` → `unregister_pubs()` in cleanup.

---

---

## 7. Operator-adjustable settings (`ControlsIF`)
*Crib from: `nepi_app_file_pub_img` (single set, row grouping, routed
callbacks); `nepi_app_nav_sim` (several sets in one node)*

This is the one pattern nearly every app needs, and it has its own section in
`APP_ARCHITECTURE.md` (3a) plus a full document at
`src/nepi_apps/CONTROLS_MIGRATION_PATTERN.md`. The essentials:

```python
from nepi_sdk import nepi_controls
from nepi_api.system_if import ControlsIF

CONTROLS_INIT_DICT = {                      # key order IS display order
    'paused':  {'type': 'Toggle', 'default': False,
                'display_name': 'Paused', 'display_group': 'playback',
                'description': 'Hold on the current image'},
    'rate_hz': {'type': 'Float', 'default': 5.0, 'bounds': [MIN_RATE, MAX_RATE],
                'round': 2, 'display_round': 2, 'display_width': 70,
                'display_name': 'Rate (Hz)', 'display_group': 'playback',
                'description': 'Publish rate while not paused'},
    'start':   {'type': 'Button', 'display_name': 'Start',
                'description': 'Start publishing'},
}

def controlsUpdatedCb(self, control_name, control_value):   # TWO arguments
    if control_name in BUTTON_CONTROLS:
        self.dispatchButton(control_name)
    else:
        route = self.controls_routes.get(control_name, None)   # reuse the
        if route is not None:                                  # existing setter
            value = self.getControlValue(control_name)
            if value is not None:
                route(ControlValue(value))                     # .data stand-in
    self.applyControls()          # AFTER the route, so a clamp is what lands
    if control_name not in BUTTON_CONTROLS and self.node_if is not None:
        self.node_if.save_config()
    self.publish_status()
```

Routing rather than reimplementing is the point: a setter that clamps, parses
or carries a side effect keeps that logic and just gains a second caller. Drop
the `node_if.set_param` it used to do — the control persists the value now, and
leaving both is the "two things writing one piece of state" failure the whole
migration exists to remove.

**Row grouping.** Controls sharing a non-empty `display_group` render on ONE
line, in declaration order, over *consecutive* runs only. Groupable types are
`String, Toggle, Toggles, Int, Ints, Float, Floats, Button, Buttons` — those
the renderer draws as a bare widget. `Menu`, `Selection`, `Selections`, both
sliders and `ColorRGB` come back already wrapped in their own `Label` and will
not line up. A row needs fixed `display_width`s (`"100%"` of a flex child
collapses), and bounds are not drawn inside a row, so a control that must show
its min/max stays ungrouped.

RUI side — one mount, on the **controls** namespace:

```jsx
<NepiIFControls key={controlsNamespace} namespace={controlsNamespace}
                title={null} make_section={false} allways_show_controls={true} />
```

---

## Combining patterns

A typical "real" app stacks several: a control set (7) holds the settings, a
discovery timer (3) finds sources, a Connect class (5) attaches to one, a
worker thread (1) processes, data publishers (2/6) output results, and SaveData
(4) makes them recordable — `nepi_app_nav_sim` and `nepi_app_file_pub_depthmap`
are the full-stack examples. Keep each pattern in its own methods (as the
shipped apps do) so the skeleton stays recognizable.
