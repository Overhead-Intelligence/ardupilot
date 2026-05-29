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

Designed to be **painless on a fresh Windows machine**: it's a single `.exe`,
needs **no Git and no build toolchain**, downloads the firmware itself over
HTTPS, and walks the operator through the one-time USB-driver install.

> Background docs:
> [OpenDroneID dev guide](https://ardupilot.org/dev/docs/opendroneid.html),
> [Loading a bootloader with DFU](https://ardupilot.org/dev/docs/using-DFU-to-load-bootloader.html),
> [Bootloader](https://ardupilot.org/dev/docs/bootloader.html).

---

## What the app does

**Stage 0 — Download firmware (HTTPS, no Git).** Downloads just what it needs
(~3 MB: the firmware images, the ODID bootloader, and `uploader.py`) from
`Overhead-Intelligence/ardupilot @ OI-4.7-dev` into a local cache folder. Re-runs
reuse the cache; **Download firmware** re-pulls the latest.

**Stage 1 — Identify the board.** Reboots into the ArduPilot bootloader and reads
its board id. **If the board is already the ODID bootloader (`11063`) — which it
is once converted — the app skips DFU entirely and just uploads firmware (no
driver, no Zadig, no admin).** Only a stock board (`1063`) needs the one-time
conversion below.

**Stage 2 (stock boards only) — Flash the ODID bootloader.** Enters DFU
(`MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN` `param4=99`, or the manual BOOT0 fallback),
makes sure the WinUSB driver is bound (see *DFU driver*), then `dfu-util` writes
the ODID bootloader to `0x08000000`. *(This is the one-time, irreversible step.)*

**Stage 3 — Flash the firmware.** Uploads the selected Plane `.apj` through the
ODID bootloader via ArduPilot's `uploader.py` protocol. The board-id check
guarantees only ODID firmware (board id `11063`) is loaded.

### Firmware images (picked at runtime)

Pulled from [`OI-ODID-Flasher/firmware/`](firmware) in the repo:

| File | Contents |
|------|----------|
| `arduplane_CubeOrangePlus-ODID_DID-RNGFND-TERR.apj` | Plane: RemoteID + rangefinder + terrain-avoid *(default)* |
| `arduplane_CubeOrangePlus-ODID_ODID-CMTC-RNGFND.apj` | Plane: RemoteID + rangefinder (CMTC variant) |

The app pre-selects the `TERR` image; the operator can pick the other or Browse
to any `.apj`.

### DFU driver (one time per PC)

`dfu-util` needs the **WinUSB** driver bound to the `STM32 BOOTLOADER` device.
libwdi does not publish a silent installer, so the app **bundles Zadig** and
auto-launches it with on-screen steps (List All Devices → *STM32 BOOTLOADER* →
WinUSB → Replace Driver), then continues automatically once done. If you supply
a `wdi-simple.exe` in `bin\` at build time, the install is fully silent instead.
The app skips this entirely if the driver is already present.

---

## Building the .exe (on Windows, once)

Requires Python 3.10+ (`py -3`). The exe bundles only the native helpers
(`dfu-util.exe`, `zadig.exe`, optional `wdi-simple.exe`); firmware is downloaded
at runtime.

```powershell
cd OI-ODID-Flasher
py -3 -m pip install -r requirements.txt
.\fetch_binaries.ps1     # downloads dfu-util.exe (+ libusb dll) and zadig.exe
.\build.ps1              # runs PyInstaller -> dist\CubeOrangePlus-ODID-Flasher.exe
```

Distribute just that one `.exe`. **Run it from a local drive** (copying onto the
machine), not from a network share — a one-file exe self-extracts on launch and
is slow to start from `\\…` UNC paths.

---

## Using the app (operator)

1. Plug the Cube Orange+ into USB. Close Mission Planner / QGC.
2. Run `CubeOrangePlus-ODID-Flasher.exe`.
3. (Optional) adjust the **Firmware folder** and click **Download firmware** —
   or just click **Flash** and it downloads automatically.
4. Pick the firmware (defaults to the terrain-avoid Plane image) and optionally
   the COM port (blank = auto-detect).
5. Tick **"I understand this is a one-time, irreversible bootloader change."**
6. Click **Flash** and confirm the warning.
7. If prompted, complete the one-time **Zadig** driver step, or the **unplug /
   replug** / BOOT0 fallback. When it finishes you'll see
   *"Bootloader + firmware flashed successfully."*

### Manual DFU fallback (if software DFU entry fails)

Power off, pull the processor's **BOOT0** pin to **3.3 V** (the Cube has no boot
button — bridge the BOOT0 pad), plug USB back in so it powers up in DFU, then
click **Continue**.

---

## Validate a build without hardware

Copy the exe to a local drive and run:

```powershell
.\CubeOrangePlus-ODID-Flasher.exe --check   # exit code 0 = required assets OK
```

(The windowed exe has no console; run `py -3 -m odidflasher --check` from source
for full output.)

---

## Layout

```
OI-ODID-Flasher/
  flasher.py                 PyInstaller entry point
  firmware/*.apj             committed Plane firmware (downloaded by the app)
  odidflasher/
    config.py                addresses, board ids, VID/PID, download URLs
    assets.py                resolve files from the cache, then the bundle
    fetch.py                 download firmware over HTTPS (no Git)
    mavlink_dfu.py           stage 1: port detect + reboot-to-DFU
    dfu.py                   stage 2: driver install (Zadig) + dfu-util
    fw_upload.py             stage 3: drive uploader.py
    wizard.py                tkinter UI + orchestration
    __main__.py              `python -m odidflasher` / `--check`
  build.ps1                  bundle native helpers + PyInstaller
  fetch_binaries.ps1         download dfu-util + zadig
  requirements.txt
```

Run from source (dev): `py -3 -m odidflasher` from this folder.

---

## Notes / limitations

- Scope is **bootloader + firmware only**. Parameters (`DID_*`, rangefinder) and
  the `quadplane-terrainavoid.lua` script are not pushed by this app — load them
  via Mission Planner / the SD card after flashing.
- Firmware images live past the repo-wide `*.apj` ignore (re-included in
  `firmware/.gitignore`, marked `binary` in `firmware/.gitattributes`). To add a
  new image, drop it in `firmware/` and `git add` it; the app picks it up after
  the next download.
- The ODID bootloader change is irreversible through normal tooling; recovery to
  a stock bootloader also requires DFU.
- `--force` is intentionally **not** exposed in the UI so a non-ODID image cannot
  be flashed by accident.
