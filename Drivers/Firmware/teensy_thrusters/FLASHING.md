# Flashing teensy_thrusters

This page explains how to put this firmware onto a Teensy 4.1 for the NEPI `svx_teensy_thrusters` driver. The flash runs over SSH on the NEPI device, with the Teensy plugged in where the driver uses it.

The commands come from the flashes done on device1 on 2026-10-26 (firmware 1.1.0) and 2026-10-27 (firmware 1.2.0), both onto Teensy 21616540.

A Teensy holds one firmware at a time. This firmware and the team's `new_ctl` (used through `rbx_mate_rov_link`) replace each other, so switching drivers means reflashing. See the last section.

## Where things are

| What | Where |
|---|---|
| Flash host | The NEPI device's host OS, account **`nepihost`**, at 192.168.179.103 (the same box also answers at 192.168.68.23). The `nepi` account is NEPI's own shell and has none of these tools. |
| SSH key (on the laptop) | `~/.ssh/nepi_default_ssh_key` |
| Sketch on the device | `~/bench_fw/teensy_thrusters/teensy_thrusters.ino`. arduino-cli needs the folder name to match the sketch name. |
| Sketch source | `Drivers/Firmware/teensy_thrusters/teensy_thrusters.ino` in this repo. Copy it over every time it changes. |
| Tools on the device | arduino-cli 1.5.1 and `teensy_loader_cli`, both in `~/.local/bin`; Teensy core `teensy:avr@1.59.0`; PJRC udev rules in `/etc/udev/rules.d/00-teensy.rules` |

The laptop has no flashing tools installed.

## Flash

1. **On the laptop, copy the sketch over:**
   ```
   scp -i ~/.ssh/nepi_default_ssh_key ~/mate_robotics/Drivers/Firmware/teensy_thrusters/teensy_thrusters.ino nepihost@192.168.179.103:bench_fw/teensy_thrusters/
   ```
2. **Free the serial port.** Either disable SVX_TEENSY_THRUSTERS on the RUI Drivers page, or run `nepistop` in the `nepi` shell. Every thruster and servo output stops while the Teensy is in its bootloader.
3. **Log in as `nepihost` and check that the copy landed:**
   ```
   ssh -i ~/.ssh/nepi_default_ssh_key nepihost@192.168.179.103
   cd ~/bench_fw/teensy_thrusters
   grep 'PROTOCOL_VERSION\[\]' teensy_thrusters.ino     # shows the protocol you are about to flash, e.g. "1.2"
   ```
4. **Compile, then flash:**
   ```
   arduino-cli compile -b teensy:avr:teensy41 --output-dir build .
   teensy_loader_cli --mcu=TEENSY41 -w -s -v build/teensy_thrusters.ino.hex
   ```
   - The plain board name uses the core's default menu options, USB Type "Serial" and 600 MHz. Those are what the sketch's build header asks for.
   - A good flash prints `Soft reboot performed`, then `Found HalfKay Bootloader`, then `Programming....`, and finally `Booting`.
   - `-s` reboots the running sketch into the bootloader over USB. That works for any Teensyduino USB Serial sketch, including `new_ctl`, so you don't need to press the program button.
5. **Give the port back.** Re-enable the driver, or run `nepistart` in the `nepi` shell. Within about 10 s the discovery log should show:
   ```
   Teensy 21616540 on /dev/ttyACM0 identified: firmware 1.200.0, protocol 1.200, 8 thruster channels, 4 servo channels
   ```
   The NEPI logger prints `1.2` as `1.200`, so `1.200` means protocol 1.2. Next, the nodes picked in the driver's `devices` option launch. The thruster node holds every thruster at stop for its 7 s arming time.

Don't test by typing `$ID*0D` at the port while the driver is enabled. Discovery and the nodes read the same port, and they would take the reply (PROTOCOL.md section 2.1).

## Why not `arduino-cli compile -u`

With this Teensy core, the built-in upload hands off to PJRC's Teensy Loader. That loader is a desktop window, so over SSH it fails with `Unable find Teensy Loader`. `teensy_loader_cli` is PJRC's command-line loader, and it needs no window. The `arduino-cli board list` command doesn't show the Teensy's `teensy` port on this box either (`Error starting discovery: ... HELLO: EOF`). The loader doesn't need a port name, so this doesn't matter.

## One-time setup on a new or re-imaged device

Run everything as `nepihost` on the host, not inside the NEPI container.

```
# arduino-cli. The installer won't create ~/.local/bin itself.
mkdir -p ~/.local/bin
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | BINDIR=$HOME/.local/bin sh -s 1.5.1
grep -q 'local/bin' ~/.bashrc || echo 'export PATH=$HOME/.local/bin:$PATH' >> ~/.bashrc
export PATH=$HOME/.local/bin:$PATH
arduino-cli version                                   # expect 1.5.1

# Teensy core, pinned to 1.59.0 (see below)
arduino-cli config init
arduino-cli config add board_manager.additional_urls https://www.pjrc.com/teensy/package_teensy_index.json
arduino-cli core update-index && arduino-cli core install teensy:avr@1.59.0
f=$(ls ~/.arduino15/packages/teensy/tools/teensy-tools/1.59.0/teensy_post_compile) && { ldd "$f" | grep "not found" || echo "teensy-tools OK"; }

# teensy_loader_cli, built from PJRC's source
sudo apt install -y libusb-dev
git clone https://github.com/PaulStoffregen/teensy_loader_cli ~/src/teensy_loader_cli
make -C ~/src/teensy_loader_cli
cp ~/src/teensy_loader_cli/teensy_loader_cli ~/.local/bin/

# udev rules, so the loader can open the Teensy without root
sudo curl -fsSL -o /etc/udev/rules.d/00-teensy.rules https://www.pjrc.com/teensy/00-teensy.rules
sudo udevadm control --reload-rules
sudo udevadm trigger --action=add --subsystem-match=usb --attr-match=idVendor=16c0

mkdir -p ~/bench_fw/teensy_thrusters
```

- **Why core 1.59.0.** From 1.60.0 on, PJRC's Teensy tools need glibc 2.34. The NEPI device runs Ubuntu 20.04, which has glibc 2.31. Core 1.59.0 is the newest whose tools run there. The laptop is also Ubuntu 20.04, so it needs the same pin.
- **Why the targeted `udevadm trigger`.** New udev rules apply only to devices that appear after they load, so an already-plugged Teensy stays root-only. This trigger re-applies the rules only to Teensy devices (vendor 16c0). A plain `udevadm trigger` would replay every USB device on a running NEPI box, cameras included.
- `teensy_thrusters` needs no libraries beyond the core (`Wire` is bundled). The `new_ctl` sketch needs four: `"Adafruit BusIO@1.17.4"`, `"Adafruit Unified Sensor@1.1.15"`, `"Adafruit BNO08x@1.2.7"` (install it with `--no-deps`) and `"BlueRobotics MS5837 Library@1.1.1"`.

## Problems seen, and their fixes

| Symptom | Cause | Fix |
|---|---|---|
| `arduino-cli: command not found` and no `~/bench_fw` | You're logged in as `nepi`. | SSH in as `nepihost`. |
| `($BINDIR) folder not found` during install | `~/.local/bin` didn't exist. | `mkdir -p ~/.local/bin` first. |
| `GLIBC_2.34 not found` from a Teensy tool | The core is 1.60 or newer. | `arduino-cli core uninstall teensy:avr && arduino-cli core install teensy:avr@1.59.0` |
| `Unable find Teensy Loader` | `compile -u` over SSH. | Use `teensy_loader_cli`, as in the Flash steps. |
| `Unable to claim interface ... Operation not permitted`, then `Waiting for Teensy device...` | The udev rules aren't applied to the plugged-in Teensy. | Press the program button once (the loader is still waiting), or Ctrl+C, run the targeted `udevadm trigger`, and flash again. |
| Discovery: `gave no identify reply` | The Teensy isn't running teensy_thrusters, e.g. it still has `new_ctl`. | Flash it. |
| Discovery: `runs protocol 1.100, which has no depth sensor commands` | Firmware 1.1 is on the board; the depth sensor needs 1.2. | Copy the current sketch and flash it. Thrusters and servos still work on 1.1. |

## Going back to the team firmware

This is for the `rbx_mate_rov_link` setup: NEPI → UDP → the team's control Pi → Teensy. NEPI never talks to the Teensy there, so the Teensy needs the team's `new_ctl` and plugs into the control Pi.

- **From the control Pi** (`nepidev`, which has its own tools and libraries), using the port from the `arduino-cli board list` row whose protocol is `teensy`:
  ```
  cd ~/bench_fw/new_ctl && arduino-cli compile -u -b teensy:avr:teensy41 -p usb1/1-1 .
  ```
  See `Nepi_Connect/BENCH_SETUP_PLAN.md` for that bench's setup.
- **From the NEPI device**, the same two commands as above work in `~/bench_fw/new_ctl`, with `new_ctl.ino.hex` in place of `teensy_thrusters.ino.hex`. `new_ctl` was only ever built with core 1.62.0 on the control Pi. A compile error on 1.59.0 that the bench never saw probably comes from that difference. The device's copy came from `~/rov-26` at `1af4984`. It isn't recorded whether the bench edits were applied to it, so check whether `power_thrusters();` is still commented out before you flash it.
- **While a Teensy running `new_ctl` is plugged into the NEPI device,** disable SVX_TEENSY_THRUSTERS. Otherwise discovery sends it an `$ID` probe every 10 s and logs `gave no identify reply`.

To come back to this driver, follow the Flash steps again.
