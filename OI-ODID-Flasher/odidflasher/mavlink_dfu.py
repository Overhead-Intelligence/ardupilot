"""Stage 1: find the running flight controller and reboot it into DFU mode.

The stock CubeOrangePlus ArduPilot firmware (board_id 1063) is built with
HAL_ENABLE_DFU_BOOT, so we can drop it into the STM32 ROM DFU bootloader purely
over USB-serial MAVLink -- no need to open the Cube and jumper BOOT0. The BOOT0
method remains available as a manual fallback (see the wizard).
"""
from __future__ import annotations

import time
from dataclasses import dataclass

import serial.tools.list_ports
from pymavlink import mavutil

from . import config


@dataclass
class PortInfo:
    device: str        # e.g. "COM7"
    description: str
    vid: int | None
    pid: int | None

    @property
    def label(self) -> str:
        v = f" [{self.vid:04x}:{self.pid:04x}]" if self.vid and self.pid else ""
        return f"{self.device} - {self.description}{v}"


def _port_rank(p: PortInfo) -> tuple:
    """Sort key: real MAVLink ports first, SLCAN/CAN/DFU/debug last."""
    d = (p.description or "").lower()
    is_mav = "mavlink" in d or "fmu" in d
    is_aux = any(x in d for x in ("slcan", "can", "dfu", "debug", "bootloader"))
    is_cube = p.vid in config.CUBE_USB_VIDS
    bucket = 0 if is_mav else (2 if is_aux else 1)
    return (bucket, not is_cube, p.device)


def list_serial_ports() -> list[PortInfo]:
    ports = []
    for p in serial.tools.list_ports.comports():
        ports.append(PortInfo(p.device, p.description or "", p.vid, p.pid))
    ports.sort(key=_port_rank)   # MAVLink-looking ports first, SLCAN/CAN last
    return ports


def _has_heartbeat(port: str, timeout: float, log) -> bool:
    """Open `port` briefly; True iff a MAVLink heartbeat arrives. Never raises
    (access-denied / SLCAN / wrong ports just return False)."""
    try:
        m = mavutil.mavlink_connection(port, baud=config.MAVLINK_BAUD)
    except Exception as e:  # noqa: BLE001 - busy/denied/ghost port
        log(f"    {port}: cannot open ({e})")
        return False
    try:
        if m.wait_heartbeat(timeout=timeout) is not None:
            log(f"    {port}: MAVLink heartbeat (system {m.target_system})")
            return True
        log(f"    {port}: no heartbeat")
        return False
    except Exception as e:  # noqa: BLE001
        log(f"    {port}: no heartbeat ({e})")
        return False
    finally:
        try:
            m.close()
        except Exception:  # noqa: BLE001
            pass


def find_mavlink_port(preferred: str | None = None, log=print,
                      per_port_timeout: float = 4.0) -> str | None:
    """Return the COM port that actually speaks MAVLink.

    Tries `preferred` first (if given), then every other port in rank order, so
    the autopilot's MAVLink port wins over its SLCAN port and any busy/ghost
    port is skipped automatically.
    """
    order: list[str] = []
    if preferred:
        order.append(preferred)
    for p in list_serial_ports():
        if p.device not in order:
            order.append(p.device)
    log("Looking for the autopilot's MAVLink port ...")
    for dev in order:
        if _has_heartbeat(dev, per_port_timeout, log):
            return dev
    return None


def autodetect_mavlink_port(log=print, per_port_timeout: float = 4.0) -> str | None:
    return find_mavlink_port(preferred=None, log=log, per_port_timeout=per_port_timeout)


class PortBusyError(RuntimeError):
    """The serial port is held by another program (Windows 'Access is denied')."""


def _is_access_denied(exc: Exception) -> bool:
    s = f"{exc}".lower()
    return ("access is denied" in s or "permissionerror" in s
            or "permission denied" in s or getattr(exc, "errno", None) == 13)


def _open(port: str, log=print, attempts: int = 3, delay: float = 1.0):
    """Open a MAVLink connection, retrying briefly. Raises PortBusyError if the
    port stays locked by another application."""
    last = None
    for i in range(1, attempts + 1):
        try:
            return mavutil.mavlink_connection(port, baud=config.MAVLINK_BAUD)
        except Exception as e:  # noqa: BLE001
            last = e
            if _is_access_denied(e):
                log(f"  {port} is busy (attempt {i}/{attempts}); "
                    f"another program may have it open.")
                time.sleep(delay)
                continue
            raise
    raise PortBusyError(
        f"{port} is in use by another program. Close Mission Planner / "
        f"QGroundControl / any serial monitor, unplug and replug the board, "
        f"then retry. (last error: {last})")


def reboot_to_bootloader(port: str, log=print) -> bool:
    """Command the running firmware to reboot into the ArduPilot bootloader.

    Used to read the board id and to upload firmware. Returns True if the
    command was sent. Non-destructive (no flash write).
    """
    log(f"Rebooting {port} into the bootloader ...")
    try:
        m = _open(port, log=log, attempts=2)
    except Exception as e:  # noqa: BLE001
        log(f"  could not open {port}: {e}")
        return False
    try:
        if m.wait_heartbeat(timeout=10) is None:
            log(f"  no heartbeat on {port}")
            return False
        m.mav.command_long_send(
            m.target_system, m.target_component,
            mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
            0, 3, 0, 0, 0, 0, 0, 0,   # param1=3 -> reboot to bootloader
        )
        time.sleep(1.0)
        log("Reboot-to-bootloader command sent.")
        return True
    finally:
        m.close()


def reboot_to_dfu(port: str, log=print) -> None:
    """Connect on `port` and command the firmware into DFU mode.

    Raises PortBusyError if the port is locked, or RuntimeError on heartbeat
    failure. The board disappears from the serial bus on success; the caller
    then waits for the DFU USB device.
    """
    log(f"Connecting to {port} ...")
    m = _open(port, log=log)
    try:
        hb = m.wait_heartbeat(timeout=10)
        if hb is None:
            raise RuntimeError(f"No MAVLink heartbeat on {port}")
        log(f"Connected (system {m.target_system}). Commanding reboot-to-DFU ...")
        # Requires the full magic: param1=42, param2=24, param3=71, param4=99.
        m.mav.command_long_send(
            m.target_system,
            m.target_component,
            mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
            0,                              # confirmation
            config.REBOOT_MAGIC_P1,         # param1 = 42
            config.REBOOT_MAGIC_P2,         # param2 = 24
            config.REBOOT_MAGIC_P3,         # param3 = 71
            config.REBOOT_TO_DFU_MAGIC,     # param4 = 99 -> boot_to_dfu()
            0, 0, 0,                        # param5-7
        )
        # The board reboots immediately and the USB port drops; that's expected.
        time.sleep(1.0)
        log("Reboot-to-DFU command sent.")
    finally:
        try:
            m.close()
        except Exception:  # noqa: BLE001 - port already gone after reboot
            pass
