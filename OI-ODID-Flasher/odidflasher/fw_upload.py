"""Stage 3: upload the chosen .apj firmware through the ODID serial bootloader.

We reuse ArduPilot's own ``uploader.py`` (the canonical bootloader-protocol
implementation) as a library, loaded by file path so it works identically from
source and inside the PyInstaller bundle. ``uploader.upload()`` already refuses
a firmware whose board_id does not match the bootloader, which is exactly the
ODID safety check we want.
"""
from __future__ import annotations

import importlib.util
import sys
import time
from pathlib import Path

import serial.tools.list_ports

from . import assets, config

_uploader_mod = None


def _uploader_py_path() -> Path:
    """Return the path to uploader.py (cloned repo, bundle, or dev tree)."""
    p = assets.uploader_py()
    if p is not None:
        return p
    # dev fallback: <ardupilot>/Tools/scripts/uploader.py
    repo_root = Path(__file__).resolve().parents[2]
    return repo_root / "Tools" / "scripts" / "uploader.py"


def _load_uploader():
    global _uploader_mod
    if _uploader_mod is not None:
        return _uploader_mod
    path = _uploader_py_path()
    if not path.is_file():
        raise FileNotFoundError(f"uploader.py not found at {path}")
    spec = importlib.util.spec_from_file_location("ap_uploader", str(path))
    mod = importlib.util.module_from_spec(spec)
    sys.modules["ap_uploader"] = mod
    spec.loader.exec_module(mod)  # type: ignore[union-attr]
    _uploader_mod = mod
    return mod


def candidate_ports() -> list[str]:
    """Serial ports that might be the freshly-booted ODID bootloader."""
    ports = []
    for p in serial.tools.list_ports.comports():
        ports.append(p.device)
    return ports


def open_bootloader(timeout: float = 30.0, log=print):
    """Scan for an ArduPilot bootloader; return an OPEN, synced uploader.

    Returns (uploader, port) with the connection still open (read up.board_type),
    or (None, None) if none found. The caller must close the uploader.
    """
    up_mod = _load_uploader()
    log("Looking for the ArduPilot bootloader ...")
    deadline = time.time() + timeout
    while time.time() < deadline:
        for port in candidate_ports():
            try:
                up = up_mod.uploader(port, 115200, 57600)
            except Exception:  # noqa: BLE001 - port busy / not openable yet
                continue
            try:
                if up_mod.find_bootloader(up, port):
                    log(f"Found bootloader on {port}: board id {up.board_type}")
                    return up, port           # keep open for the caller
            except Exception:  # noqa: BLE001
                pass
            try:
                up.close()
            except Exception:  # noqa: BLE001
                pass
        time.sleep(0.5)
    return None, None


def wait_for_bootloader_port(timeout: float = 30.0, log=print) -> str | None:
    """Compatibility helper: find the bootloader and return its port (closed)."""
    up, port = open_bootloader(timeout=timeout, log=log)
    if up is not None:
        try:
            up.close()
        except Exception:  # noqa: BLE001
            pass
    return port


def upload_on(up, apj_path: str, force: bool = False, log=print) -> None:
    """Upload `apj_path` using an already-open, synced uploader. Raises on error."""
    up_mod = _load_uploader()
    fw = up_mod.firmware(apj_path)
    board_id = fw.property("board_id")
    log(f"Firmware: {Path(apj_path).name} (board_id {board_id})")
    if board_id != config.BOARD_ID_ODID and not force:
        raise RuntimeError(
            f"Selected firmware board_id {board_id} is not the ODID id "
            f"{config.BOARD_ID_ODID}; refusing to upload.")
    if up.board_type != board_id and not force:
        raise RuntimeError(
            f"Board id mismatch: bootloader reports {up.board_type}, firmware is "
            f"{board_id}. The board may not have the ODID bootloader installed.")
    log("Uploading firmware (this can take a minute) ...")
    up.upload(fw, force=force)
    log("Firmware upload complete. The board will boot the new firmware.")


def upload_firmware(port: str, apj_path: str, force: bool = False, log=print) -> None:
    """Identify the bootloader on `port` and upload `apj_path`. Raises on error."""
    up_mod = _load_uploader()
    up = up_mod.uploader(port, 115200, 57600)
    try:
        if not up_mod.find_bootloader(up, port):
            raise RuntimeError(f"Could not sync with bootloader on {port}")
        upload_on(up, apj_path, force=force, log=log)
    finally:
        try:
            up.close()
        except Exception:  # noqa: BLE001
            pass
