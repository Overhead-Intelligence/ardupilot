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


def list_serial_ports() -> list[PortInfo]:
    ports = []
    for p in serial.tools.list_ports.comports():
        ports.append(PortInfo(p.device, p.description or "", p.vid, p.pid))
    # Surface likely Cube/ArduPilot ports first.
    ports.sort(key=lambda p: (p.vid not in config.CUBE_USB_VIDS, p.device))
    return ports


def autodetect_mavlink_port(log=print, per_port_timeout: float = 4.0) -> str | None:
    """Try each serial port for a MAVLink heartbeat; return the first that answers."""
    for p in list_serial_ports():
        log(f"  probing {p.label} ...")
        try:
            m = mavutil.mavlink_connection(p.device, baud=config.MAVLINK_BAUD)
        except Exception as e:  # noqa: BLE001 - report and move on
            log(f"    open failed: {e}")
            continue
        try:
            hb = m.wait_heartbeat(timeout=per_port_timeout)
            if hb is not None:
                log(f"    heartbeat from system {m.target_system} on {p.device}")
                return p.device
        except Exception as e:  # noqa: BLE001
            log(f"    no heartbeat: {e}")
        finally:
            m.close()
    return None


def reboot_to_dfu(port: str, log=print) -> None:
    """Connect on `port` and command the firmware into DFU mode.

    Raises on connection / heartbeat failure. The board disappears from the
    serial bus on success; the caller then waits for the DFU USB device.
    """
    log(f"Connecting to {port} ...")
    m = mavutil.mavlink_connection(port, baud=config.MAVLINK_BAUD)
    try:
        hb = m.wait_heartbeat(timeout=10)
        if hb is None:
            raise RuntimeError(f"No MAVLink heartbeat on {port}")
        log(f"Connected (system {m.target_system}). Commanding reboot-to-DFU ...")
        m.mav.command_long_send(
            m.target_system,
            m.target_component,
            mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
            0,                              # confirmation
            0, 0, 0,                        # param1-3
            config.REBOOT_TO_DFU_MAGIC,     # param4 == 99 -> boot_to_dfu()
            0, 0, 0,                        # param5-7
        )
        # The board reboots immediately; we don't expect an ACK back.
        time.sleep(1.0)
        log("Reboot-to-DFU command sent.")
    finally:
        m.close()
