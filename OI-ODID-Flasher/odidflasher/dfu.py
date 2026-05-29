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


ZADIG_STEPS = (
    "Zadig is opening to install the USB driver (one time per PC).\n\n"
    "In the Zadig window:\n"
    "  1. Options menu -> tick 'List All Devices'.\n"
    "  2. In the dropdown, select 'STM32 BOOTLOADER'.\n"
    "  3. Make sure the target driver (right box) is 'WinUSB'.\n"
    "  4. Click 'Replace Driver' (or 'Install Driver') and wait for success.\n"
    "  5. Close Zadig and click Continue here.\n\n"
    "(If Windows asks for permission, click Yes.)"
)


def _wdi_simple_install(log) -> None:
    exe = str(assets.wdi_simple())
    args = [
        "--vid", f"0x{config.DFU_VID:04X}",
        "--pid", f"0x{config.DFU_PID:04X}",
        "--type", "0",          # 0 = WinUSB
        "--name", "STM32 BOOTLOADER (ODID Flasher)",
    ]
    if is_admin():
        res = _run([exe, *args], timeout=180)
        if res.stdout.strip():
            log(res.stdout.strip())
        if res.returncode != 0:
            log(f"wdi-simple failed (rc={res.returncode}): {res.stderr.strip()}")
    else:
        log("Requesting administrator elevation for driver install ...")
        rc = ctypes.windll.shell32.ShellExecuteW(
            None, "runas", exe, subprocess.list2cmdline(args), None, 1)
        if rc <= 32:
            log(f"Elevation was declined or failed (code {rc}).")


def ensure_driver(log=print, guide=None) -> bool:
    """Make sure WinUSB is bound to the DFU device. Returns True on success.

    `guide` (optional) is a callback that displays instructions and BLOCKS until
    the user confirms -- used for the Zadig flow. Strategy:
      1. driver already present -> done.
      2. wdi-simple.exe bundled -> silent install (with elevation).
      3. Zadig bundled -> launch it, walk the user through, wait, re-check.
      4. otherwise -> instructions, fail.
    """
    if dfu_device_present():
        log("WinUSB driver already present for the DFU device.")
        return True

    # 2. Silent install if a wdi-simple binary was provided.
    if assets.wdi_simple().is_file():
        log("Installing WinUSB driver (wdi-simple) ...")
        try:
            _wdi_simple_install(log)
        except Exception as e:  # noqa: BLE001
            log(f"wdi-simple error: {e}")
        if _wait_driver(log):
            return True
        log("Silent install did not take; falling back to Zadig.")

    # 3. Guided Zadig install.
    if assets.zadig().is_file():
        log("Launching Zadig to install the WinUSB driver ...")
        try:
            subprocess.Popen([str(assets.zadig())])   # Zadig self-elevates (UAC)
        except Exception as e:  # noqa: BLE001
            log(f"Could not launch Zadig: {e}")
            return False
        if guide is not None:
            guide(ZADIG_STEPS)        # blocks until the user clicks Continue
        return _wait_driver(log, timeout=20)

    # 4. No installer available.
    log("dfu-util cannot see the DFU device and no driver installer is bundled.")
    log("Install the WinUSB driver once with Zadig (select 'STM32 BOOTLOADER' "
        "-> WinUSB) or STM32CubeProgrammer, then retry.")
    return False


def _wait_driver(log, timeout: float = 60.0) -> bool:
    deadline = time.time() + timeout
    while time.time() < deadline:
        if dfu_device_present():
            log("WinUSB driver is active for the DFU device.")
            return True
        time.sleep(1.0)
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
