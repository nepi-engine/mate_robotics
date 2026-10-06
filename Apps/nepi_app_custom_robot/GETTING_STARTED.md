# Getting Started — Custom Robot App (`nepi_app_custom_robot`)

This app builds one NEPI robot (RBX) device out of devices that are already
on the system: a motor device for the drive channels and a NavPose (NPX)
device for position and orientation. A camera (IDX) connect drives the image
viewer on the app page. The RBX device it publishes is what the Robot
Stabilization app (`../nepi_app_robot_stab`) selects through its RBX connect.

For *how apps work* (apps_mgr, the NodeClassIF skeleton, RUI injection), read
`APP_ARCHITECTURE.md`; Section 9 there covers this app's connects and RBX.

## 0. Using this app

1. Deploy and build it like any other app (Section 3 below), then enable
   "Custom Robot" in the RUI app list. Its page is in the AUTOMATION group.
2. On the app page, pick the **Motor** device and the **NavPose (NPX)** device
   from their selector rows. Pick a **Camera (IDX)** if you want the image
   viewer.
3. The RBX device is built the first time the selected motor device reports
   connected. The status area shows **RBX Ready** and the RBX namespace
   (`<app>/rbx`). Selecting a motor device with a different name or motor
   count tears the RBX down and rebuilds it.
4. In the Robot Stabilization app, select this RBX device in its RBX row.

The rest of this file is the general create-and-deploy workflow this package
inherits from the NEPI app template. `setup_new_app.sh` lived in that template
and is not shipped in this package; rename by hand using the table in step 2
if you copy this app to start another one.

---

## 1. Create your app from the template

1. **Copy the `nepi_app_custom_robot/` folder** into your own repo/location.
   `setup_new_app.sh` and `deploy_app.sh` both ride along inside it, so there
   is nothing else to copy. (`../nepi_connect_templates/deploy_nepi_apps.sh`
   deploys several `nepi_app_*` folders at once; use that one only if you are
   managing a folder of apps.)
2. **Rename everything with `setup_new_app.sh`** (recommended). Edit the
   EDIT-THESE block at the top of `nepi_app_custom_robot/setup_new_app.sh`
   (`APP_SUFFIX`, `DISPLAY_NAME`, `DESCRIPTION`, `GROUP_NAME`, and optional
   `SHORT_NAME`), then run it:

   ```bash
   cd nepi_app_custom_robot
   ./setup_new_app.sh --dry-run   # preview the full rename plan
   ./setup_new_app.sh             # do it (renames the folder + every file/token)
   ```

   `APP_SUFFIX` (snake_case, e.g. `nav_sim`) drives every derived name so they
   stay consistent: package `nepi_app_nav_sim`, node `app_nav_sim`, node class
   `NepiNavSimApp`, connector `ConnectAppNavSim`, msg `NepiAppNavSimStatus`, RUI
   class `NepiAppNavSim`, plus the `scripts/`, `params/`, `msg/`, `rui/`, `api/`
   filenames. Set `SHORT_NAME` (e.g. `PTAuto`) when the full name is too long
   for the RUI class; the msg type and package name stay full so the node and
   RUI still agree. Delete the script from your app once you have run it.

   <details><summary>Or rename by hand — the full token checklist</summary>

   The rename must stay consistent everywhere: `pkg_name` is used as the catkin
   package name, the rosrun package, AND the msg import module.

   | File | What to rename |
   |---|---|
   | folder | `nepi_app_custom_robot` → `nepi_app_<name>` |
   | `package.xml` | `<name>nepi_app_custom_robot</name>` |
   | `CMakeLists.txt` | `project(nepi_app_custom_robot)`, the msg filename, the script path |
   | `params/*.yaml` | filename (must keep `params` in it!), `APP_DICT` fields (`pkg_name`, `app_file`, `node_name`, `config_file`, `display_name`, `description`, `group_name`), `RUI_DICT` fields |
   | `msg/NepiAppCustomRobotStatus.msg` | filename + fields you need |
   | `scripts/custom_robot_app_node.py` | filename, `from nepi_app_custom_robot.msg import ...`, class name, `DEFAULT_NODE_NAME` (the yaml's `node_name` is what actually launches — keep them equal) |
   | `api/connect_app_custom_robot.py` | filename, msg import, class name, `APP_NODE_NAME` |
   | `rui/NepiAppCustomRobot.js` | filename, class name + `export default`, `state.appName`, the status msg type string `"nepi_app_custom_robot/NepiAppCustomRobotStatus"`, `CONTROLS_NAME` if you renamed the control set |

   Three fields must agree or the app silently fails:
   - `APP_DICT.pkg_name` == catkin package name (package.xml / CMake project)
   - `APP_DICT.app_file` == the node script filename
   - `RUI_DICT.rui_main_class` == the React class your js `export default`s,
     **and unique across every installed app** — the RUI codegen uses it as
     both the import binding and the classMap key

   </details>

3. **Pick the right `group_name`** (`DEVICE`, `DATA`, `PROCESS`, `AUTOMATION`,
   or `SYSTEM`) — it decides which RUI selector menu the app appears in.

4. **Fill in your logic:**
   - *controls* — replace the example `CONTROLS_INIT_DICT` with your own. Sort
     each piece of state first: a value the operator **types or drags** is a
     control; a command carrying a compound value or a traversal verb stays a
     topic (with a Button control calling the *same* private method); a value
     the **node** writes that must survive a restart stays in `PARAMS_DICT`.
     Never let a control and a param both write one piece of state.
     `APP_ARCHITECTURE.md` Section 3a has the whole contract.
   - *node* — put your work in `updaterCb` or a worker thread, extend
     `applyControls()` and `controlRoutes()` for your controls, and keep the
     1 Hz latched status pattern.
   - *msg* — put everything the node **reports** into the Status msg. What the
     operator adjusts is published separately by `ControlsIF`, so it does not
     need a field.
   - *rui* — keep the `NepiIFControls` mount for the adjustable half (pass it
     the **controls** namespace, `<app>/controls`, and always a
     `key={controlsNamespace}`); render only what the node reports yourself.
   - *api* — keep the connector's pubs mirroring your node's subs, add a
     `set_control_value` passthrough per control, and keep each public method
     signature stable. All entries must use the app's namespace
     (`self.namespace` / `self.controls_namespace`), never
     `self.node_namespace`.
   - For worker threads, NavPose publishing, resource discovery, SaveData,
     device control, image publishing and the full controls pattern, see
     **`APP_PATTERNS.md`** — each comes with real code excerpts from the
     shipped app that does it best.

5. **Syntax-check:**
   ```bash
   python3 -m py_compile scripts/*_node.py api/connect_app_*.py
   python3 -c "import yaml; d=yaml.safe_load(open('params/<name>_app_params.yaml')); assert 'APP_DICT' in d"
   ```

---

## 2. Deploy to the NEPI src tree

`deploy_app.sh`, inside your app folder, does **two different things**, and the
difference is what to understand before you use it:

1. **Build updates** — rsyncs the *whole* package folder into the target's
   source tree at
   `/mnt/nepi_storage/nepi_src/nepi_engine_ws/src/nepi_apps/<app_folder>/`.
   Nothing there is live; it takes effect on the next catkin build.
2. **Live updates** — rsyncs **`scripts/` only** into the running container at
   `/opt/nepi/nepi_engine/lib/<app_folder>/`. That is the fast edit loop, and
   it covers the node script and nothing else.

So a change under `api/`, `params/`, `msg/` or `rui/` that you have only
"deployed" is **not running on the device**. If you are debugging a change to
`api/` and seeing old behaviour, check what is actually installed first.

```bash
export NEPI_REMOTE_SETUP=0             # 0 = running on the target, 1 = from a dev host
./deploy_app.sh
```

Remote mode (`NEPI_REMOTE_SETUP=1`) additionally needs `NEPI_TARGET_IP`, set at
the top of the script, and an SSH key (`NEPI_SSH_KEY`, defaulting to
`~/.ssh/nepi_default_ssh_key`).

To deploy several app folders at once, use
`../nepi_connect_templates/deploy_nepi_apps.sh`, which globs every
`nepi_app_*` folder beside it and syncs each into the source tree (build
updates only — no live sync).

### The `api/` orphan trap

Your app's `api/*.py` installs **over** the shared `nepi_api` package, and that
install does not use `--delete`. A file you remove or rename under `api/`
therefore survives on the device forever and keeps shadowing whatever
`nepi_api` ships under that name — through every later build, with no error.
Delete the stale file on the target by hand, and never name a file in `api/`
after something `nepi_api` already has. The same applies to `rui/`, which is
flattened into one folder shared by every app package.

---

## 3. Build & watch it come up

1. **Rebuild the workspace** so the package installs (node → `lib/<pkg>`,
   params → `share/nepi_apps/params`, api → `nepi_api`, rui → the RUI src).
2. **Rebuild the RUI** (`build_nepi_rui.sh`) — this is the step that injects
   your `rui_main_class` into the RUI's `appsClassMap`. Skipping it means the
   app page renders blank.
3. Restart NEPI. In the RUI **Apps manager**, your app should be listed —
   enable it. `apps_mgr` then launches the node (`rosrun <pkg> <app_file>`);
   watch its log for your node's "Initialization Complete".
4. The app page appears in its `group_name` menu **only while the node is
   running**.

---

## Reference apps to crib from

| Your app is like… | Read the shipped app |
|---|---|
| Smallest complete app (thin node, RUI-heavy) | `nepi_apps/nepi_app_image_viewer` |
| Fullest single-control-set worked example | `nepi_apps/nepi_app_file_pub_img` |
| File-based data publisher | `nepi_apps/nepi_app_file_pub_img`, `_vid`, `_depthmap` |
| Simulation, worker threads, several control sets, row grouping | `nepi_apps/nepi_app_nav_sim` |
| Higher-level device manager, services rather than topics | `nepi_apps/nepi_app_onvif_mgr` |
| Consuming another NEPI device's API | `../nepi_connect_templates/` (eight examples) |

The authority on the controls pipeline is
`nepi_apps/CONTROLS_MIGRATION_PATTERN.md`.

Also read the **Gotchas** section at the end of `APP_ARCHITECTURE.md` before your
first deploy.
