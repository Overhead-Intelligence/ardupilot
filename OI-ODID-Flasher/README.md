# CubeOrangePlus-ODID Flasher

A single-file Windows application that converts a **Cube Orange+** to the
**CubeOrangePlus-ODID** board configuration and loads this fork's custom
firmware (RemoteID + LightWare I2C rangefinder return-value support, and the
`quadplane-terrainavoid.lua` applet).

This cannot be done from Mission Planner because the ODID board config requires
a **different bootloader**. The normal Cube Orange+ bootloader (board id `1063`)
will not accept ODID firmware (board id `11063`), so the bootloader itself must
be replaced over DFU first. Installing the ODID bootloader is a **one-time,
irreversible lockdown**: afterwards the board only accepts ODID-built firmware.

> Background docs:
> [OpenDroneID dev guide](https://ardupilot.org/dev/docs/opendroneid.html),
> [Loading a bootloader with DFU](https://ardupilot.org/dev/docs/using-DFU-to-load-bootloader.html),
> [Bootloader](https://ardupilot.org/dev/docs/bootloader.html).

---

## What the app does (three stages)

1. **Enter DFU.** Connects to the running firmware over MAVLink and sends
   `MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN` with `param4 = 99`, which the stock
   Cube Orange+ firmware (built with `HAL_ENABLE_DFU_BOOT`) interprets as
   "reboot into the STM32 ROM DFU bootloader". **No need to open the Cube.**
   If that fails, the app falls back to the manual method and prompts you to
   jumper **BOOT0** high and replug.
2. **Flash the ODID bootloader.** Ensures the WinUSB driver is bound to the
   DFU device, then `dfu-util` writes `CubeOrangePlus-ODID_bl.bin` to
   `0x08000000`. *(This is the irreversible step.)*
3. **Flash the firmware.** The board reboots into the ODID bootloader; the app
   uploads the `.apj` you selected using ArduPilot's `uploader.py` protocol.
   The board-id check guarantees only ODID firmware is loaded.

The two firmware images bundled from this fork are:

| File | Contents |
|------|----------|
| `4.7-dev(DID-RNGFND-TERR).apj` | RemoteID + rangefinder + terrain-avoid |
| `4.7 (ODID-CMTC-RNGFND).apj`   | RemoteID + rangefinder (CMTC variant) |

Both are board id `11063`. The operator picks one at runtime.

---

## Building the .exe (on Windows, once)

Requires Python 3.10+ (`py -3`) on Windows. Do **not** build from WSL — the exe
must be native Windows.

```powershell
cd OI-ODID-Flasher
py -3 -m pip install -r requirements.txt
.\fetch_binaries.ps1     # downloads dfu-util.exe (+ libusb-1.0.dll)
.\build.ps1              # stages assets + runs PyInstaller
```

Output: `dist\CubeOrangePlus-ODID-Flasher.exe` — a self-contained file with the
bootloader, both firmware images, `uploader.py`, and `dfu-util.exe` embedded.

### About the DFU driver (`wdi-simple.exe`)

`dfu-util` needs the **WinUSB/libusb** driver bound to the `STM32 BOOTLOADER`
device. The app can install it automatically if `wdi-simple.exe` (from
[libwdi](https://github.com/pbatard/libwdi)) is present in `bin\` at build time
(set `$env:WDI_SIMPLE_URL` before `fetch_binaries.ps1`, or drop the file in
`bin\` yourself). This auto-install needs a one-time admin elevation.

If you don't bundle `wdi-simple.exe`, the app still works — just install the
driver once on each PC with **[Zadig](https://zadig.akeo.ie)** (List All
Devices → *STM32 BOOTLOADER* → WinUSB → Replace Driver) or by installing
**STM32CubeProgrammer**. The app detects an existing driver and skips the step.

---

## Using the app (operator)

1. Plug the Cube Orange+ into USB. Close Mission Planner / QGC.
2. Run `CubeOrangePlus-ODID-Flasher.exe`.
3. Pick the firmware image and (optionally) the COM port — leave the port blank
   to auto-detect.
4. Tick **"I understand this is a one-time, irreversible bootloader change."**
5. Click **Flash** and confirm the warning.
6. Follow any on-screen **unplug / replug** prompts. When it finishes you'll see
   *"Bootloader + firmware flashed successfully."*

### Manual DFU fallback (if step 1 can't enter DFU in software)

Power off, pull the processor's **BOOT0** pin to **3.3 V** (the Cube has no boot
button — bridge the BOOT0 pad), plug USB back in so it powers up in DFU, then
click **Continue**. See the ArduPilot DFU guide linked above.

---

## Validate a build without hardware

```powershell
.\dist\CubeOrangePlus-ODID-Flasher.exe --check
```

Lists the embedded assets and confirms `uploader.py` loads. Exit code `0` = OK.

---

## Layout

```
OI-ODID-Flasher/
  flasher.py                 PyInstaller entry point
  odidflasher/
    config.py                addresses, board ids, VID/PID
    assets.py                locate bundled files (dev + frozen)
    mavlink_dfu.py           stage 1: port detect + reboot-to-DFU
    dfu.py                   stage 2: driver install + dfu-util
    fw_upload.py             stage 3: drive uploader.py
    wizard.py                tkinter UI + orchestration
    __main__.py              `python -m odidflasher` / `--check`
  build.ps1                  stage payload + PyInstaller
  fetch_binaries.ps1         download dfu-util (+ optional wdi-simple)
  requirements.txt
```

Run from source (dev): `py -3 -m odidflasher` from this folder (uses the repo's
`Tools/scripts/uploader.py` and the firmware in `..\build\CubeOrangePlus-ODID\bin`).

---

## Notes / limitations

- Scope is **bootloader + firmware only**. Parameters (`DID_*`, rangefinder) and
  the `quadplane-terrainavoid.lua` script are not pushed by this app — load them
  via Mission Planner / the SD card after flashing.
- The ODID bootloader change is irreversible through normal tooling. Recovery
  back to a stock bootloader also requires DFU.
- `--force` behaviour is intentionally **not** exposed in the UI so a mismatched
  (non-ODID) image cannot be flashed by accident.
