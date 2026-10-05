# Getting Started — Creating & Deploying a NEPI Driver from a Template

This is the practical workflow for turning one of the templates in this folder
into a real driver and getting it onto a NEPI device. For *how drivers work*
(discovery models, the launch handshake, the interface-class contract), read
`DRIVER_ARCHITECTURE.md`.

---

## 1. Create your driver from a template

Pick the folder matching your device type:

| Your device is… | Template folder |
|---|---|
| a camera / imager | `idx_drivers/` |
| a light | `lsx_drivers/` |
| a GPS / IMU / navpose source | `npx_drivers/` |
| a two-axis pan-tilt | `ptx_drivers/` |
| a whole vehicle | `rbx_drivers/` |
| a single servo or one-axis actuator | `svx_drivers/` |

(A pan/tilt turret built from two servos is **two SVX drivers**, not one PTX
driver — see `DRIVER_ARCHITECTURE.md` Section 5.)

1. **Copy that folder** into your own repo/location, **along with
   `deploy_nepi_drivers.sh`**. Keep the folder named `<cat>_drivers/` and
   sitting next to the script — that is where the deploy script looks for it.
   `setup_new_driver.sh` rides along inside the folder.
2. **Rename everything with `setup_new_driver.sh`** (recommended). Edit the
   EDIT-THESE block at the top of `<cat>_drivers/setup_new_driver.sh`
   (`DRIVER_NAME`, `DISPLAY_NAME`, `DESCRIPTION`, and optional `CLASS_BASE`),
   then run it from inside the folder:

   ```bash
   cd idx_drivers
   ./setup_new_driver.sh --dry-run   # preview the rename plan
   ./setup_new_driver.sh             # do it (renames the files + tokens)
   ```

   It auto-detects the category from the folder name and renames every
   `<cat>_template_*` file plus the `pkg_name`/`PKG_NAME`, class names, and the
   matching `NODE_DICT`/`DRIVER_DICT`/`DISCOVERY_DICT` yaml entries together, so
   they stay in lock-step. `DRIVER_NAME` (snake_case, e.g. `deepsea_sealite`)
   becomes the file prefix + `pkg_name`; set `CLASS_BASE` (e.g. `Sealite`) for
   shorter class names. The **folder is left named `<cat>_drivers/`** on
   purpose (that is what the deploy script globs), and comments that reference
   the sibling templates are left intact. Delete the script once you have run
   it.

   <details><summary>Or rename by hand — the token checklist</summary>

   Rename every `template` / `Template` token to your device across all files:
   file names, `pkg_name`, `PKG_NAME`, and the class names. Keep the
   `NODE_DICT`/`DRIVER_DICT`/`DISCOVERY_DICT` `file_name`/`class_name` entries in
   the yaml matching the real files/classes **exactly** (mismatches are a common
   silent failure).

   </details>
3. **Fill the `TODO:` markers:**
   - *discovery* — your enumeration + a real `checkForDevice` handshake, and the
     `DEVICE_DICT` fields your node needs. (SVX: your board match and the
     channel list, not a per-servo probe.)
   - *node* — the capability callbacks for your interface class, your settings
     tables, and (if serial) your `send_msg` protocol; (if IDX) your driver ctor
     call.
   - *driver* (IDX only) — the real hardware I/O against your SDK.
   - *params* — `display_name`, `description`, and the `OPTIONS` your discovery
     reads.
4. **Syntax-check:** `python3 -m py_compile <your files>.py` and
   `python3 -c "import yaml; yaml.safe_load(open('...params.yaml'))"`.

### Two things that fail silently — check them before you bench-test

- **Settings.** Every device IF takes exactly two settings arguments,
  `getSettingsFunction` and `setSettingFunction`, and your node implements
  `initSettingsDict` / `refreshSettingsDict` / `getSettingsFunction` /
  `setSettingFunction(name, value) -> [success, msg, settings_dict]`. The older
  `capSettings=` / `factorySettings=` / `settingUpdateFunction=` arguments were
  removed from every IF and now raise `TypeError` at construction. A device with
  no settings passes neither function. See `DRIVER_ARCHITECTURE.md` Section 5.
- **Control types.** A setting or a params `OPTION` declared `type: Discrete` is
  silently dropped and simply never appears in the RUI — `Discrete` is not a
  `nepi_controls` control type. A named option list is a `Selection`. And a
  numeric setting's min/max live in `bounds` in a node's controls dict, though
  they are still the two-entry `options` list in a params `OPTION`.

---

## 2. Deploy to the NEPI src tree

`deploy_nepi_drivers.sh` handles the "flatten into the right place" step for
you. It rsyncs **only the `.py` and `.yaml` files** (no docs, scripts, or
`__pycache__`) from each `<cat>_drivers/` folder flat into the target's
source tree at:

```
${NEPI_TARGET_SRC_DIR}/nepi_engine_ws/src/nepi_drivers/<cat>_drivers/
```

(Drivers must live **flat** in `<cat>_drivers/` — not in per-driver subfolders.
See `DRIVER_ARCHITECTURE.md` Section 7 for why.)

Usage:

```bash
export NEPI_REMOTE_SETUP=0          # 0 = running on the target, 1 = from a dev host
./deploy_nepi_drivers.sh            # deploy ALL template types (idx lsx ptx npx rbx svx)
./deploy_nepi_drivers.sh idx        # deploy just the idx template
./deploy_nepi_drivers.sh idx ptx    # or any subset by category
```

Remote mode (`NEPI_REMOTE_SETUP=1`) additionally needs `NEPI_TARGET_IP` (taken
from `NEPI_IP`) and the default SSH key at
`~/ssh_keys/nepi_engine_default_private_ssh_key`; it then rsyncs over SSH the
same way.

> Shell-scripting note: the rsync filters are passed as a **quoted bash array**
> (`RSYNC_FILTERS=(--include '*.py' ... --exclude '*')`). Keeping them in an
> unquoted string variable breaks badly — the shell glob-expands `*` into the
> current directory's filenames, which become extra rsync *source* arguments
> and junk gets copied to the target.

---

## 3. Build & watch it come up

After deploying, **rebuild the workspace** so the files install to
`/opt/nepi/nepi_engine/lib/nepi_drivers/` where `drivers_mgr` scans them. Then
watch the `drivers_mgr` log — it prints when it imports/launches discovery and
when a device node comes up.

---

## Reference drivers to crib from

| Your device is like… | Read the shipped driver |
|---|---|
| USB / V4L2 camera | `idx_drivers/idx_v4l2_*` |
| GigE / SDK camera | `idx_drivers/idx_genicam_*` |
| Serial light | `lsx_drivers/lsx_deepsea_sealite_*` |
| Serial pan-tilt | `ptx_drivers/ptx_sidus_ss109_serial_*` |
| Network navpose (NMEA/TCP) | `npx_drivers/npx_nmea_udp_*` |
| Binary-protocol navpose over TCP | `npx_drivers/npx_hnav_tcu_*` |
| Serial IMU (vendor ROS driver) | `npx_drivers/npx_microstrain_*` |
| Autopilot / vehicle | `rbx_drivers/rbx_ardupilot_*` |
| Simulated vehicle (network bridge) | `rbx_drivers/rbx_gazebo_*` |
| Single servo on a multi-channel board | `svx_drivers/svx_servo_maestro_*` |

`idx_drivers/idx_v4l2_node.py` is the **reference implementation for the
settings contract** — read it before writing your own `initSettingsDict`.
`src/nepi_drivers/CLAUDE.md` carries the same contract in prose.

Also read the **Gotchas** section at the end of `DRIVER_ARCHITECTURE.md` before
your first bench test.
