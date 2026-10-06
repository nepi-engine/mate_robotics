# NEPI App Architecture - How Apps Work & How to Write One

This folder holds a **complete, copy-paste NEPI app template**
(`nepi_app_template/`) plus the deploy script. This document explains the
architecture an app plugs into so you know *why* each file looks the way it
does. For the hands-on workflow (copy, rename, deploy), see
**`GETTING_STARTED.md`**.

```
nepi_app_template/           ← the template app (a complete catkin package)
├── APP_ARCHITECTURE.md      ← you are here (architecture reference)
├── APP_PATTERNS.md          ← add-on patterns with real code excerpts
├── GETTING_STARTED.md       ← create + deploy workflow
├── setup_new_app.sh         ← renames the whole package to your app's name
├── deploy_app.sh            ← deploys THIS package to a target
├── scripts/template_app_node.py    ← the ROS node
├── params/template_app_params.yaml ← manifest read by apps_mgr
├── msg/NepiAppTemplateStatus.msg   ← the app's status message
├── api/connect_app_template.py     ← client class other nodes can import
├── rui/NepiAppTemplate.js          ← React page shown in the RUI
├── etc/  srv/                      ← placeholders (house convention)
├── CMakeLists.txt  package.xml     ← catkin package + install rules
└── LICENSE
```

`../nepi_connect_templates/` holds eight minimal "connect example" apps and a
`deploy_nepi_apps.sh` that deploys several `nepi_app_*` folders at once.

> Verified against the shipped apps in `nepi_engine_ws/src/nepi_apps/`
> (`nepi_app_image_viewer` is the smallest complete one, `nepi_app_file_pub_img`
> the fullest worked example), `src/nepi_apps/CONTROLS_MIGRATION_PATTERN.md`,
> `apps_mgr.py` and `scripts_mgr.py` in `nepi_managers`, `node_if.py` and
> `system_if.py` in `nepi_api`, and `build_nepi_rui.sh`.
>
> `nepi_app_fake_gps`, which earlier versions of this document used as the
> reference, has been RETIRED: its simulator was folded into
> `nepi_app_nav_sim` as a third simulator kind and the package deleted. The
> current collection is `nav_sim`, `file_pub_img`, `file_pub_vid`,
> `file_pub_depthmap`, `image_viewer` and `onvif_mgr`.

---

## 1. The big picture

A NEPI app is a self-contained capability (simulation, viewing, automation...)
packaged as its own catkin package. Two pieces of framework you do **not**
write manage it:

- **`apps_mgr`** (in `nepi_managers`) — scans for installed apps every 5 s,
  reads each app's params yaml, and launches/kills the app node based on its
  enabled ("active") state. Users enable/disable apps from the RUI.
- **`NodeClassIF`** (in `nepi_api.node_if`) — the node-side framework class.
  Your node describes its configs, params, publishers, subscribers and
  services as plain dicts; `NodeClassIF` creates all the ROS plumbing,
  persists params, and auto-creates the standard
  `save_config`/`reset_config`/`factory_reset_config` topics.
- **`ControlsIF`** (in `nepi_api.system_if`) — holds everything an operator
  adjusts from the RUI, and is rendered by one shared React component. Since
  the 2026-09 controls migration this is where an app's settings live; see
  Section 3a.

Unlike drivers (which register hardware-type interface classes with fixed
callback contracts), **every app uses the same skeleton**. Apps differ only in
what they compose inside it — a worker thread, extra data publishers,
subscriptions to other nodes.

```
  ┌──────────┐  scans share/nepi_apps/params/*.yaml every 5 s
  │ apps_mgr │ ────────────────────────────────────────────►  APP_DICT / RUI_DICT
  └──────────┘
       │  for each ACTIVE app:
       │    1. set <base>/<node_name>/app_dict param
       │    2. load saved config yaml (if any) into the node's namespace
       │    3. nepi_sdk.launch_node(pkg_name, app_file, node_name)   →  rosrun
       ▼
  ┌──────────────────────────────────────────────┐
  │ <name>_app_node.py                            │
  │   NodeClassIF(configs, params, pubs, subs)    │ ← creates all ROS interfaces
  │   1 Hz latched status publisher               │
  │   spin()                                      │
  └──────────────────────────────────────────────┘
```

---

## 2. Discovery & launch (apps_mgr)

- `apps_mgr` scans the installed params folder
  (`/opt/nepi/nepi_engine/share/nepi_apps`) for yaml files whose name contains
  `params`.
- From each yaml it requires the top-level **`APP_DICT`** block; **`RUI_DICT`**
  is optional. Inside `APP_DICT`, `pkg_name` (the dict key) and `app_file` are
  hard requirements — if the node file doesn't exist at
  `/opt/nepi/nepi_engine/lib/<pkg_name>/<app_file>`, the app is purged from the
  list. `group_name`, `node_name`, `config_file`, `description` are used later
  and should always be set.
- Launch is a plain **`rosrun`** via `nepi_sdk.launch_node(pkg_name, app_file,
  node_name)` — there are no `.launch` files. Before launching, apps_mgr writes
  the app's dict to the `<node_namespace>/app_dict` param and loads the app's
  saved config yaml if one exists.
- `active` (user-enabled) is persisted by apps_mgr; `running` is a live
  `check_node_by_name` check. A node that dies on its own gets marked inactive.

---

## 3. The node skeleton

Every app node follows this exact shape (see the template or
`image_viewer_app_node.py`):

```python
nepi_sdk.init_node(name=self.DEFAULT_NODE_NAME)
self.msg_if = MsgIF(log_name=self.class_name)

self.CFGS_DICT   = {'init_callback': self.initCb, 'reset_callback': self.resetCb,
                    'factory_reset_callback': self.factoryResetCb,
                    'init_configs': True, 'namespace': self.node_namespace}
self.PARAMS_DICT = {'last_run_count': {'namespace': ..., 'factory_val': 0}, ...}   # or None
self.PUBS_DICT   = {'status_pub': {'namespace': ..., 'topic': 'status',
                    'msg': <YourStatusMsg>, 'qsize': 1, 'latch': True}, ...}
self.SUBS_DICT   = {'trigger_action': {'namespace': ..., 'topic': 'trigger_action',
                    'msg': Empty, 'qsize': 10, 'callback': self.triggerActionCb,
                    'callback_args': ()}, ...}
self.SRVS_DICT   = {'device_list_query': {'namespace': ..., 'topic': 'device_list_query',
                    'srv': <YourSrv>, 'req': <YourSrv>Request(),
                    'resp': <YourSrv>Response(), 'callback': self.handler}, ...}  # optional

self.node_if = NodeClassIF(configs_dict=self.CFGS_DICT, params_dict=self.PARAMS_DICT,
                           services_dict=self.SRVS_DICT,
                           pubs_dict=self.PUBS_DICT, subs_dict=self.SUBS_DICT,
                           msg_if=self.msg_if)
self.node_if.wait_for_ready()
self.setupControls()                                        # see Section 3a
self.initCb(do_updates=True)
nepi_sdk.start_timer_process(1.0, self.statusPublishCb)     # 1 Hz latched status
nepi_sdk.on_shutdown(self.cleanup_actions)
nepi_sdk.spin()
```

Conventions the framework relies on:
- **Status**: one latched `status` topic publishing your app's Status msg at
  ~1 Hz *and* immediately after any state change. The RUI page subscribes to it.
  Status is what the node **reports**; what the operator **adjusts** is
  published separately by `ControlsIF`.
- **Params**: what remains in `PARAMS_DICT` after the controls migration is
  node-**written** state — a value the node itself sets that has to survive a
  restart. Read with `self.node_if.get_param(name)`, write with
  `set_param(name, val)`. The `initCb/resetCb/factoryResetCb` trio restores
  state from the param layer and hands the reset down to `ControlsIF`.
- **Note that a param key IS part of its wire name** (namespace + key), unlike
  pub/sub/service registry keys, whose ROS names come from namespace + topic.
  Renaming a param key renames the ROS param.
- **Services** go through `services_dict`; `nepi_app_onvif_mgr` ships four.

### 3a. The controls pipeline

`src/nepi_apps/CONTROLS_MIGRATION_PATTERN.md` is the authority and every app in
`nepi_apps` has been through it. The dividing line:

| Kind of state | Where it lives |
|---|---|
| A value the operator types, drags or toggles | a control in `CONTROLS_INIT_DICT` |
| A command carrying a compound value or a traversal verb (`goto_location`, `select_folder`, `home_folder`) | stays a topic; a Button control calls the **same** private method |
| Node-written state that merely persists (`current_folder`, `running`) | stays in `PARAMS_DICT` |
| State owned by a sub-interface (`NavPoseIF`, image IFs) | stays with that interface |

`PARAMS_DICT` does not have to reach `None`. The test is not "does this value
persist" but "does the operator type it". What must never survive a migration
is a control **and** a param both writing one piece of state.

```python
self.controls_if = ControlsIF(controls_name='controls', controls_display_name=...,
                              controls_description=..., controls_init_dict=CONTROLS_INIT_DICT,
                              controls_updated_callback=self.controlsUpdatedCb,
                              pub_status=True, save_params=True, msg_if=self.msg_if)
self.controls_if.wait_for_controls_ready(timeout=10)
```

Ordering is load-bearing:

1. `self.controls_if = None` **before** `NodeClassIF` is built. With
   `init_configs` set, `NodeClassIF` calls `initCb` during construction, before
   `ControlsIF` exists, so `initCb` runs twice at startup and every control
   read must go through a guarded `getControlValue(name, fallback)`.
2. `NodeClassIF(...)` then `wait_for_ready()`.
3. `setupControls()` then `wait_for_controls_ready(timeout=10)`.
4. `initCb(do_updates=True)` — now the restored control values are readable.
5. Other sub-interfaces, then the worker and status timers.

Things that cost real debugging time:

- **`controls_updated_callback` is called with TWO arguments**,
  `(control_name, update_value)`. A one-argument callback raises `TypeError` on
  every update.
- **A `ControlsIF` is always a direct child of the NODE namespace.** There is no
  `namespace` argument. The RUI must derive the same name independently, and a
  set name can collide with a sibling sub-interface — a set named `navpose`
  lands on the `<node>/navpose` a `NavPoseIF` already occupies.
- **`'Discrete'` is not a control type.** `nepi_controls.CONTROL_TYPES` is
  `Menu, Button, Buttons, Toggle, Toggles, String, Selection, Selections, Int,
  Ints, IntSlider, Float, Floats, FloatSlider, RangeSlider, ColorRGB`.
  `create_controls_dict()` drops a malformed entry with only a log warning, so
  validate the init dict at startup (`checkControlsInitDict` in the template)
  and name what went missing.
- **Prefer `Selection` over `Menu`** whenever the stored value is meaningful
  text: a `Menu` value is the *index* into its option list, and that index
  silently re-points when the list is rebuilt.
- **Discovered option lists are the node's job.** `ControlsIF` carries whatever
  list it is given. Call `set_control_options()` *before* setting a value into
  it — a `Selection` value not in the current list is rejected.
- **Give `ControlsIF` no `node_if`.** It builds its own. Its registry keys are
  already prefixed from its namespace, so sharing no longer collides, but a
  shared `ControlsIF` skips its own param reload: `reset()` and
  `factory_reset()` only call `reset_params()` / `factory_reset_params()` when
  the `node_if` is not shared (`system_if.py`), so the hand-down below would
  not restore the controls' saved values. For other sub-interfaces the
  original hazard still applies wherever keys are not domain-prefixed:
  `register_pubs`/`register_subs`/`add_params` do a keyed `dict.update()`, and
  a generic key silently orphans a sibling's publisher. Leave `node_if` unset
  on every sub-interface. (2026-07 DECISION LOG.)
- **Hand resets down.** `ControlsIF` owns its own config tier, so `resetCb` and
  `factoryResetCb` must call `controls_if.reset()` / `.factory_reset()` or the
  controls keep their values while the rest of the app resets.
- **Keep one implementation per command.** A Button control and a topic
  callback both delegate to the same private method; neither reimplements the
  other.

### Optional add-ons (compose inside the skeleton)

> Each row is expanded with real, copy-paste code excerpts in
> **`APP_PATTERNS.md`**.

| Your app needs… | Pattern | Crib from |
|---|---|---|
| Operator-adjustable settings | `ControlsIF` + `Nepi_IF_Controls` (Section 3a) | `nepi_app_file_pub_img` |
| Several control sets in one node, or row grouping | one `ControlsIF` per section, name derived from the instance path | `nepi_app_nav_sim` |
| A background worker | daemon `threading.Thread` + `threading.Lock` around shared state | `nepi_app_nav_sim` |
| To publish navpose data | `NavPoseIF` from `nepi_api.data_if` | `nepi_app_nav_sim`, `nepi_app_file_pub_depthmap` |
| Periodic resource discovery | self-rescheduling oneshot timer scanning `nepi_sdk.find_topics_by_msg(...)` | `nepi_app_image_viewer` (`updaterCb`) |
| Save-data (snapshots, logging) | `Nepi_IF_SaveData` in the RUI + `ConnectSaveDataIF` in the connector | `nepi_app_image_viewer` |
| To drive another NEPI device | `Connect*` classes from `nepi_api` | `../nepi_connect_templates/` (eight worked examples) |
| To publish images / video from files | data publishers from `nepi_api.data_if` | `nepi_app_file_pub_img` / `_vid` / `_depthmap` |
| Services rather than topics | `services_dict` on `NodeClassIF` + `add_service_files` in CMake | `nepi_app_onvif_mgr` |

---

## 4. The params.yaml manifest

```yaml
APP_DICT:
  display_name: Template App          # shown in the RUI
  description: ...
  pkg_name: nepi_app_template         # MUST equal the catkin package name
  group_name: DATA                    # RUI menu bucket: DEVICE | DATA | PROCESS | AUTOMATION | SYSTEM
  config_file: app_template.yaml      # saved-config filename apps_mgr loads at launch
  app_file: template_app_node.py      # the node script (existence checked!)
  node_name: app_template             # ROS node name apps_mgr launches under
  license_type: 3-clause BSD
  license_link: https://opensource.org/licenses/BSD-3-Clause
RUI_DICT:                             # optional -- omit for headless apps
  rui_files:                          # every .js file this app installs
  - NepiAppTemplate.js
  rui_main_file: NepiAppTemplate.js
  rui_main_class: NepiAppTemplate     # MUST equal the exported React class name
```

`group_name` must be one of the RUI selector buckets or the app won't appear
in any menu. `rui_main_class` must exactly match the class the js file
`export default`s (it may differ from the filename — image_viewer exports
`ImageViewerApp` from `NepiAppImageViewer.js` — but yaml and export must agree).

---

## 5. The connect API (`api/connect_app_*.py`)

A thin client class other nodes can import to command your app without knowing
its topic layout. It mirrors your node's interface through
`ConnectNodeClassIF`: your node's *subscribers* become the connector's
*publishers*, and your status topic becomes a subscription cached into
`status_msg`. It installs into the shared `nepi_api` package, so consumers
write `from nepi_api.connect_app_template import ConnectAppTemplate`.

**The connector is part of the controls migration, not an afterthought.** A
state setter that lost its topic drops its typed publisher and publishes one
`UpdateControl` on `<app>/controls/update_control` instead, while **keeping its
public method signature unchanged** so existing callers do not break. Add a
generic `set_control_value(name, value, index=None)` alongside them and a
`ControlsStatus` subscription so a consumer can read the values back. Command
publishers stay as they are.

**Two install consequences** (see also Section 7): `api/` is not live-synced,
so a change here needs a catkin build; and the install has no `--delete`, so a
file you remove from `api/` survives on the device and keeps shadowing whatever
`nepi_api` ships under that name.

**Gotcha:** every pub/sub entry in the connector must use the **target app's**
namespace (`self.namespace`, resolved from the constructor arg or
`<base>/<app_node_name>`), never the calling node's `self.node_namespace`.
Getting this wrong publishes into the caller's own namespace and silently does
nothing.

---

## 6. RUI integration

How your React page ends up in the web UI:

1. **Install**: CMake copies `rui/*.js` flat into
   `/opt/nepi/nepi_rui/src/rui_webserver/rui-app/src/` and your params yaml
   into `.../rui-app/src/apps/`.
2. **Injection**: `build_nepi_rui.sh` walks `src/apps/*.yaml`, reads
   `rui_main_file` + `rui_main_class`, and sed-injects an `import` line and a
   `["<class>", <class>]` entry into the `appsClassMap` in `Nepi_IF_Apps.js`.
3. **Runtime**: the RUI subscribes to `apps_mgr/status`; the app selector lists
   apps by `group_name` **only while they are running**, and mounts your
   component via `appsClassMap.get(rui_main_class)`.

`rui_main_class` is used as **both** the generated import binding and the
classMap key, so it must be unique across every installed app. `rui_files` is
informational — the codegen reads only `rui_main_file` and `rui_main_class`.

So RUI changes require **rebuilding the RUI** (`build_nepi_rui.sh` → npm
build), not just restarting the node.

House rules for the page itself (see the template):
- Subscribe with `setupStatusListener(ns + '/status', '<pkg>/<StatusMsg>', cb)`
  and unsubscribe in `componentWillUnmount`.
- Send commands with the Store helpers (`sendBoolMsg`, `sendStringMsg`,
  `sendFloatMsg`, `sendTriggerMsg` for Empty). Check Store.js before assuming a
  helper exists.
- **Render the adjustable half through `Nepi_IF_Controls`**, not hand-written
  widgets. Pass it the **controls** namespace (`<app>/controls`), not the app
  namespace — the component appends `/status` itself and its children append
  `/update_control`. Passing the app namespace is the single most common way to
  get an empty control box that renders its heading and never fills in. Always
  pass `key={controlsNamespace}`: without it React reuses the mounted component
  across a namespace change, keeping the old subscription and the previous
  values on screen. `title={null}` suppresses the component's default
  "CONTROLS" heading; `make_section={false}` inlines it;
  `allways_show_controls={true}` forces it open.
- **`NepiIFConfig` stays on the APP namespace**, not the controls namespace. A
  save there dumps the whole node parameter subtree, which already includes the
  controls namespace.
- Render only what the node **reports** yourself.
- Editable `Input` boxes, where you still need one: buffer edits in state, mark
  the element modified (`setElementStyleModified`) while dirty, and commit on
  Enter with `clearElementStyleModified` — never send on every keystroke.

---

## 7. Build & install

Each app is its own catkin package; `CMakeLists.txt` installs to fixed NEPI
locations:

| Source | Installed to |
|---|---|
| `scripts/*_node.py` | `lib/<pkg_name>/` (catkin bin, executable) |
| `params/` | `${NEPI_ENGINE}/share/nepi_apps/params` **and** `${NEPI_RUI}/.../src/apps` |
| `api/*.py` | `${NEPI_ENGINE}/lib/python3/dist-packages/nepi_api/` |
| `rui/` | `${NEPI_RUI}/src/rui_webserver/rui-app/src/` |
| `msg/*.msg` | built by `generate_messages` into the `<pkg_name>.msg` Python module |
| `etc/` | catkin global etc |

Adding an app = dropping the package folder into
`nepi_engine_ws/src/nepi_apps/` and rebuilding. No registry edits.

---

## 8. Gotchas (learned from the shipped apps)

- **`pkg_name` is load-bearing three ways**: it's the catkin package name, the
  rosrun package for launch, and the Python module for your msg import
  (`from <pkg_name>.msg import ...`). Rename all together.
- **`rui_main_class` vs export mismatch** silently gives a blank page — the
  classMap lookup returns undefined.
- **The RUI label above an app page is blank by design-accident**: the RUI
  reads `rui_menu_name`, but nothing ever populates it (not in `AppStatus.msg`,
  not in the yaml). Don't be surprised by the missing label.
- **Apps only appear in the selector while running** — an enabled-but-crashed
  app vanishes from the menu instead of showing an error.
- **Connector namespace bug** (Section 5) — the most common copy-paste mistake.
- **`api/` orphans.** The `api/` install writes over the shared `nepi_api`
  package with no `--delete`, so a file you remove or rename here stays on the
  device forever and keeps shadowing the engine's own copy of that name,
  through every later build, with no error. Delete it on the target by hand,
  and never name a file in `api/` after something `nepi_api` already ships.
- **`rui/` is flattened** into one folder shared by every app package, so a
  `.js` basename is globally unique or it silently overwrites another app's
  component. Keep non-source files (editor workspaces, notes) out of `rui/`.
- **Only `scripts/` live-syncs.** `deploy_app.sh` rsyncs the whole package into
  the target *source tree* (dead until a catkin build) and `scripts/` alone
  into the running container. If a change to `api/` seems to have no effect,
  check what is actually installed before hunting the bug elsewhere.
- **`rui_main_class` is used twice** by `build_nepi_rui.sh` — as the import
  binding and as the classMap key — so two apps declaring the same class name
  break the build. `rui_files` in `RUI_DICT` is informational; the codegen
  reads only `rui_main_file` and `rui_main_class`.
- **The yaml's `node_name` wins over `DEFAULT_NODE_NAME`.** `apps_mgr` launches
  with the yaml value, so a stale constant in the script is dead code and the
  RUI must derive its namespace from the yaml. `nepi_app_onvif_mgr` is exactly
  that case: its script says `onvif_app`, its yaml says `app_onvif_mgr`.
- **`ConnectNodeClassIF` takes no `namespace`/`log_class_name` kwargs.** Every
  shipped `connect_app_*` file passes them anyway — they'd `TypeError` if ever
  instantiated (they're currently unused in-tree, which is how the bug
  survives). The target namespace rides in the per-entry `namespace` fields of
  the pubs/subs dicts. The template's connector shows the correct call.
- **Services are supported and used.** `nepi_app_onvif_mgr` ships four `.srv`
  files, lists them in `add_service_files`, and registers them through
  `NodeClassIF`'s `services_dict`. The template's `srv/` is an empty
  placeholder only because the template has no services of its own.
- **`apps_mgr` package install/remove functions are currently broken** in
  `nepi_apps.py` (corrupted commented-out region) — deploy apps via the source
  tree + rebuild, not via .zip package install.

---

## 9. Connections (nepi_app_robot_stab)

`robot_stab_app_node.py` has exactly three connects, all set up in
`setupConnects()` after `setupControls()` and before the explicit `initCb`.

| Connect | Class (`nepi_api` module) | Namespace | RUI row |
|---|---|---|---|
| Robot | `ConnectRBXDeviceIF` (`connect_device_if_rbx`) | `<app>/rbx_connect` | `Nepi_IF_ConnectRBX` |
| Targets | `ConnectProcessIFTargets` (`connect_process_if_targets`) | `<app>/targets_connect` | `Nepi_IF_ConnectTargets` |
| Obstacles | `ConnectObstaclesIF` (`connect_obstacles_if`, installed by `nepi_app_obstacles`) | the selected obstacles app's namespace | inline Label + Select |

**RBX and Targets** are `ConnectNodeIF`s. Each owns its RUI selector row end to
end: discovery, its own persisted `selected_topic` param, a `std_msgs/String`
subscriber on `<app>/<connect_name>/select_topic`, and a
`nepi_interfaces/ConnectIFStatus` on `<app>/<connect_name>/status`. Both are
built once, unconditionally, with their default connect names,
`show_selector=True`, `show_controls=False`, `show_data=False`,
`msg_if=self.msg_if`, and no `namespace` or `node_if` (2026-07 DECISION LOG),
then waited on with `wait_for_ready(timeout = 10)` and unregistered on
shutdown. The RBX row is meant to select the RBX device
`nepi_app_custom_robot` hosts at `<custom_robot_app>/rbx`. The targets connect
is constructed the same way the WPILib IF app constructs its own.

**Obstacles** follows the WPILib IF app. `ConnectObstaclesIF` is not a
`ConnectNodeIF`: it has no selector, no auto-discovery and publishes no
`ConnectIFStatus`. The page lists every topic of type
`nepi_app_obstacles/ObstaclesStatus` ending in `/obstacles/status`, strips that
suffix to get app namespaces, and publishes the operator's pick (or `None`) to
`<app>/set_obstacles_namespace`. That subscriber is registered
unconditionally. `setObstaclesNamespaceCb` is change-gated and calls
`connectObstacles()`, which unregisters any previous instance, treats
`None`/empty as a disconnect, and builds `ConnectObstaclesIF(namespace=...,
dataCB=None)` inside try/except, leaving `obstacles_if = None` on failure. The
selection starts at `None` on every node start (not persisted, as in WPILib).

**Runtime dependency.** `from nepi_api.connect_obstacles_if import
ConnectObstaclesIF` resolves only on a device where `nepi_app_obstacles` (from
`first_robotics`) is installed, because that app's CMakeLists is what drops
`connect_obstacles_if.py` into `nepi_api`. Without it this node fails at import.

**Status.** `rbx_connected`, `targets_connected` and `obstacles_connected`
come from each connect's `check_connection()`; `selected_obstacles_namespace`
reports the obstacles selection, because no `ConnectIFStatus` carries it.

**Image viewer.** The RUI page keeps the 75 / 2 / 23 split. The targets
connect's `ConnectIFStatus` names no image topic, so the viewer does not follow
it. Instead the page keeps its earlier selection pattern with the camera picked
locally: a Camera Select over the IDX device namespaces in `ros.idxDevices`,
then the Data Product Select (`<selected_topic>/<data_product>`), and
`findImageTopic()` resolving the image topic from `imageTopics`.
`Nepi_IF_ImageViewer` renders only once a camera is selected. The selection is
page-local and not persisted. (The connected targets process's own
`TargetsStatus.process_status.image_pub_topics` does name its annotated image
topics; following that is a possible later change.)
