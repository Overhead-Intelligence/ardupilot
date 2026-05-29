"""Stage 2: ensure the WinUSB driver, then flash the ODID bootloader via dfu-util.

On Windows the STM32 ROM DFU device enumerates with ST's default driver, which
dfu-util/libusb cannot open. We bind WinUSB to it with libwdi's ``wdi-simple``
(the same engine Zadig uses). That step needs administrator rights, so it is
launched elevated via ShellExecute "runas" when the app itself is not elevated.
"""
from __future__ import annotations

import ctypes
import subprocess
import time

from . import assets, config


# --------------------------------------------------------------------------- #
# DFU device presence
# --------------------------------------------------------------------------- #
def _run(cmd: list[str], timeout: float = 120.0) -> subprocess.CompletedProcess:
    return subprocess.run(
        cmd,
        capture_output=True,
        text=True,
        timeout=timeout,
        creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0),
    )


def dfu_device_present() -> bool:
    """True if dfu-util can see (i.e. open) our DFU device -> driver is OK."""
    try:
        res = _run([str(assets.dfu_util()), "-l"], timeout=15)
    except Exception:  # noqa: BLE001
        return False
    needle = f"{config.DFU_VID:04x}:{config.DFU_PID:04x}"
    return needle.lower() in (res.stdout + res.stderr).lower()


def usb_device_attached() -> bool:
    """True if the DFU VID/PID is on the USB bus at all (driver may be wrong).

    Uses pyusb if available; otherwise falls back to the dfu-util check.
    """
    try:
        import usb.core  # type: ignore

        return usb.core.find(idVendor=config.DFU_VID, idProduct=config.DFU_PID) is not None
    except Exception:  # noqa: BLE001 - pyusb optional / no backend
        return dfu_device_present()


def wait_for_dfu(timeout: float = 30.0, log=print) -> bool:
    """Poll until a DFU device appears on the bus (any driver)."""
    log("Waiting for the board to re-enumerate in DFU mode ...")
    deadline = time.time() + timeout
    while time.time() < deadline:
        if usb_device_attached():
            log("DFU device detected.")
            return True
        time.sleep(0.5)
    return False


# --------------------------------------------------------------------------- #
# Driver install (libwdi / wdi-simple), elevated if necessary
# --------------------------------------------------------------------------- #
def is_admin() -> bool:
    try:
        return bool(ctypes.windll.shell32.IsUserAnAdmin())
    except Exception:  # noqa: BLE001
        return False


def ensure_driver(log=print) -> bool:
    """Make sure WinUSB is bound to the DFU device. Returns True on success."""
    if dfu_device_present():
        log("WinUSB driver already present for the DFU device.")
        return True

    if not assets.wdi_simple().is_file():
        log("dfu-util cannot see the DFU device and wdi-simple.exe is not "
            "bundled to auto-install the driver.")
        log("Install the WinUSB driver once with Zadig (select 'STM32 "
            "BOOTLOADER' -> WinUSB) or STM32CubeProgrammer, then retry.")
        return False

    log("Installing WinUSB driver for the DFU device (libwdi) ...")
    exe = str(assets.wdi_simple())
    args = [
        "--vid", f"0x{config.DFU_VID:04X}",
        "--pid", f"0x{config.DFU_PID:04X}",
        "--type", "0",          # 0 = WinUSB
        "--name", "STM32 BOOTLOADER (ODID Flasher)",
    ]

    if is_admin():
        try:
            res = _run([exe, *args], timeout=180)
            log(res.stdout.strip())
            if res.returncode != 0:
                log(f"wdi-simple failed (rc={res.returncode}): {res.stderr.strip()}")
        except Exception as e:  # noqa: BLE001
            log(f"wdi-simple error: {e}")
    else:
        log("Requesting administrator elevation for driver install ...")
        # ShellExecute returns >32 on success of the *launch* (not the program).
        rc = ctypes.windll.shell32.ShellExecuteW(
            None, "runas", exe, subprocess.list2cmdline(args), None, 1
        )
        if rc <= 32:
            log(f"Elevation was declined or failed (code {rc}).")
            return False
        log("Waiting for the elevated driver install to finish ...")

    # Poll for the driver to take effect.
    deadline = time.time() + 60
    while time.time() < deadline:
        if dfu_device_present():
            log("WinUSB driver installed.")
            return True
        time.sleep(1.0)
    log("Driver install did not complete in time.")
    return False


# --------------------------------------------------------------------------- #
# Flash the bootloader
# --------------------------------------------------------------------------- #
def flash_bootloader(log=print, leave: bool = True) -> None:
    """dfu-util the ODID bootloader to internal flash. Raises on failure."""
    bl = assets.bootloader_bin()
    addr = f"0x{config.DFU_FLASH_ADDR:08x}"
    spec = f"{config.DFU_VID:04x}:{config.DFU_PID:04x}"
    dfuse = f"{addr}:leave" if leave else addr
    cmd = [
        str(assets.dfu_util()),
        "-d", spec,
        "-a", "0",
        "-s", dfuse,
        "-D", str(bl),
    ]
    log("Flashing ODID bootloader:")
    log("  " + subprocess.list2cmdline(cmd))
    res = _run(cmd, timeout=180)
    out = (res.stdout or "") + (res.stderr or "")
    for line in out.splitlines():
        log("  " + line)
    if res.returncode != 0:
        raise RuntimeError(f"dfu-util exited with code {res.returncode}")
    log("Bootloader flashed; board is rebooting into the ODID bootloader.")
