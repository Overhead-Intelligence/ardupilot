# CubeOrangePlus-ODID Flasher

A single-file Windows application that converts a **Cube Orange+** to the
**CubeOrangePlus-ODID** board configuration and loads this fork's custom
**ArduPlane** firmware (RemoteID + LightWare I2C rangefinder return-value
support, and the `quadplane-terrainavoid.lua` applet).

This cannot be done from Mission Planner because the ODID board config requires
a **different bootloader**. The normal Cube Orange+ bootloader (board id `1063`)
will not accept ODID firmware (board id `11063`), so the bootloader itself must
be replaced over DFU first. Installing the ODID bootloader is a **one-time,
irreversible lockdown**: afterwards the board only accepts ODID-built firmware.

The app is meant to run on a **different machine** with no build toolchain. It
**clones this repo** (`Overhead-Intelligence/ardupilot @ OI-4.7-dev`) over HTTPS
and flashes the prebuilt Plane firmware that is committed in the repo — nothing
is compiled on the target.

> Background docs:
> [OpenDroneID dev guide](https://ardupilot.org/dev/docs/opendroneid.html),
> [Loading a bootloader with DFU](https://ardupilot.org/dev/docs/using-DFU-to-load-bootloader.html),
> [Bootloader](https://ardupilot.org/dev/docs/bootloader.html).

---

## What the app does

**Stage 0 — Clone/update the repo.** Clones the fork (shallow, single branch,
no submodules) into a local folder, or fetches+resets it if already present.
The firmware, the ODID bootloader (`Tools/bootloaders/CubeOrangePlus-ODID_bl.bin`)
and `Tools/scripts/uploader.py` all come from this clone.

**Stage 1 — Enter DFU.** Connects to the running firmware over MAVLink and sends
`MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN` with `param4 = 99`, which the stock Cube
Orange+ firmware (built with `HAL_ENABLE_DFU_BOOT`) interprets as "reboot into
the STM32 ROM DFU bootloader". **No need to open the Cube.** If that fails, the
app falls back to the manual method and prompts you to jumper **BOOT0** high and
replug.

**Stage 2 — Flash the ODID bootloader.** Ensures the WinUSB driver is bound to
the DFU device, then `dfu-util` writes the ODID bootloader to `0x08000000`.
*(This is the irreversible step.)*

**Stage 3 — Flash the firmware.** The board reboots into the ODID bootloader; the
app uploads the selected Plane `.apj` using ArduPilot's `uploader.py` protocol.
The board-id check guarantees only ODID firmware (board id `11063`) is loaded.

### Firmware images (committed in the repo, picked at runtime)

Under [`OI-ODID-Flasher/firmware/`](firmware):

| File | Contents |
|------|----------|
| `arduplane_CubeOrangePlus-ODID_DID-RNGFND-TERR.apj` | Plane: RemoteID + rangefinder + terrain-avoid *(default)* |
| `arduplane_CubeOrangePlus-ODID_ODID-CMTC-RNGFND.apj` | Plane: RemoteID + rangefinder (CMTC variant) |

The app pre-selects the `TERR` image; the operator can pick the other or Browse
to any `.apj`.

---

## Building the .exe (on Windows, once)

Requires Python 3.10+ (`py -3`) on Windows. The exe bundles **only** the native
helpers (`dfu-util.exe`, optional `wdi-simple.exe`); everything else is fetched
by the runtime clone.

```powershell
cd OI-ODID-Flasher
py -3 -m pip install -r requirements.txt
.\fetch_binaries.ps1     # downloads dfu-util.exe (+ libusb-1.0.dll)
.\build.ps1              # runs PyInstaller
```

Output: `dist\CubeOrangePlus-ODID-Flasher.exe`. Distribute just this file; it
clones the repo on first run.

> **git is required on the target machine.** Install
> [Git for Windows](https://git-scm.com/download/win). The repo is public, so
> the HTTPS clone needs no credentials.

### About the DFU driver (`wdi-simple.exe`)

`dfu-util` needs the **WinUSB/libusb** driver bound to the `STM32 BOOTLOADER`
device. The app installs it automatically if `wdi-simple.exe` (from
[libwdi](https://github.com/pbatard/libwdi)) is present in `bin\` at build time
(set `$env:WDI_SIMPLE_URL` before `fetch_binaries.ps1`, or drop the file in
`bin\` yourself). This needs a one-time admin elevation.

If you don't bundle `wdi-simple.exe`, install the driver once per PC with
**[Zadig](https://zadig.akeo.ie)** (List All Devices → *STM32 BOOTLOADER* →
WinUSB) or **STM32CubeProgrammer**. The app detects an existing driver and skips
the step.

---

## Using the app (operator)

1. Plug the Cube Orange+ into USB. Close Mission Planner / QGC.
2. Run `CubeOrangePlus-ODID-Flasher.exe`.
3. (Optional) adjust the **Repo folder** and click **Clone / Update**. You can
   skip this — clicking **Flash** clones automatically if needed.
4. Pick the firmware (defaults to the terrain-avoid Plane image) and optionally
   the COM port (blank = auto-detect).
5. Tick **"I understand this is a one-time, irreversible bootloader change."**
6. Click **Flash** and confirm the warning.
7. Follow any on-screen **unplug / replug** prompts. When it finishes you'll see
   *"Bootloader + firmware flashed successfully."*

### Manual DFU fallback (if software DFU entry fails)

Power off, pull the processor's **BOOT0** pin to **3.3 V** (the Cube has no boot
button — bridge the BOOT0 pad), plug USB back in so it powers up in DFU, then
click **Continue**. See the ArduPilot DFU guide linked above.

---

## Validate a build without hardware

```powershell
.\dist\CubeOrangePlus-ODID-Flasher.exe --check
```

Reports the bundled helpers, whether git is on PATH, and whether a repo has been
cloned yet. (Run `py -3 -m odidflasher --check` from source for full console
output, since the windowed exe has no console.)

---

## Layout

```
OI-ODID-Flasher/
  flasher.py                 PyInstaller entry point
  firmware/*.apj             committed Plane firmware (source of truth)
  odidflasher/
    config.py                addresses, board ids, VID/PID, repo URL/branch
    assets.py                resolve files from the clone, then the bundle
    repo.py                  clone / update the fork over HTTPS
    mavlink_dfu.py           stage 1: port detect + reboot-to-DFU
    dfu.py                   stage 2: driver install + dfu-util
    fw_upload.py             stage 3: drive uploader.py
    wizard.py                tkinter UI + orchestration
    __main__.py              `python -m odidflasher` / `--check`
  build.ps1                  bundle native helpers + PyInstaller
  fetch_binaries.ps1         download dfu-util (+ optional wdi-simple)
  requirements.txt
```

Run from source (dev): `py -3 -m odidflasher` from this folder.

---

## Notes / limitations

- Scope is **bootloader + firmware only**. Parameters (`DID_*`, rangefinder) and
  the `quadplane-terrainavoid.lua` script are not pushed by this app — load them
  via Mission Planner / the SD card after flashing.
- The firmware images live past the repo-wide `*.apj` ignore (re-included in
  `firmware/.gitignore`) and are marked `binary` in `firmware/.gitattributes`.
  To add a new image, drop it in `firmware/` and `git add` it.
- The ODID bootloader change is irreversible through normal tooling; recovery to
  a stock bootloader also requires DFU.
- `--force` is intentionally **not** exposed in the UI so a non-ODID image cannot
  be flashed by accident.
