"""Locate bundled assets and vendored modules.

Works both when running from source (dev) and when frozen by PyInstaller
(``sys._MEIPASS`` holds the unpacked data dir). Asset layout inside the bundle
mirrors the ``payload/`` staging directory produced by ``build.ps1``:

    <root>/
        bin/dfu-util.exe
        bin/wdi-simple.exe
        bootloader/CubeOrangePlus-ODID_bl.bin
        firmware/<something>.apj ...
"""
from __future__ import annotations

import sys
from pathlib import Path

from . import config


def _bundle_root() -> Path:
    """Directory that holds bin/, bootloader/, firmware/."""
    meipass = getattr(sys, "_MEIPASS", None)
    if meipass:
        return Path(meipass)
    # dev: payload sits next to the package
    return Path(__file__).resolve().parent.parent / "payload"


# --- cloned-repo source (set once the repo is cloned/updated) ----------------
_repo_root: Path | None = None


def set_repo_root(path) -> None:
    """Point asset resolution at a cloned repo. Pass None to use the bundle."""
    global _repo_root
    _repo_root = Path(path) if path else None


def repo_root() -> Path | None:
    return _repo_root


def bin_path(name: str) -> Path:
    # Native helpers always ship inside the exe (never come from the repo).
    return _bundle_root() / "bin" / name


def dfu_util() -> Path:
    return bin_path(config.DFU_UTIL_EXE)


def wdi_simple() -> Path:
    return bin_path(config.WDI_SIMPLE_EXE)


def bootloader_bin() -> Path:
    """Prefer the cloned repo's bootloader; fall back to a bundled copy."""
    if _repo_root:
        c = _repo_root / config.REPO_BOOTLOADER_PATH
        if c.is_file():
            return c
    return _bundle_root() / "bootloader" / config.BOOTLOADER_BIN


def uploader_py() -> Path | None:
    """Path to uploader.py: cloned repo first, then bundle. None if neither."""
    if _repo_root:
        c = _repo_root / config.REPO_UPLOADER_PATH
        if c.is_file():
            return c
    b = _bundle_root() / "uploader.py"
    return b if b.is_file() else None


def firmware_dir() -> Path:
    """Prefer the cloned repo's firmware folder; fall back to the bundle."""
    if _repo_root:
        c = _repo_root / config.REPO_FIRMWARE_SUBDIR
        if c.is_dir():
            return c
    return _bundle_root() / config.FIRMWARE_DIR


def list_firmware() -> list[Path]:
    """Every *.apj image shipped in the bundle, sorted by name."""
    d = firmware_dir()
    if not d.is_dir():
        return []
    return sorted(d.glob("*.apj"))


def missing_assets() -> list[str]:
    """Bundle-only requirements that must be present at startup (dfu-util)."""
    problems: list[str] = []
    if not dfu_util().is_file():
        problems.append(f"dfu-util not found (rebuild the exe): {dfu_util()}")
    return problems


def flash_readiness() -> list[str]:
    """What's still missing to flash *now*, given the current source (repo/bundle)."""
    problems = missing_assets()
    if not bootloader_bin().is_file():
        problems.append(f"bootloader not found: {bootloader_bin()}")
    if uploader_py() is None:
        problems.append("uploader.py not found (clone the repo first)")
    if not list_firmware():
        problems.append(f"no .apj firmware available in: {firmware_dir()}")
    return problems


def optional_warnings() -> list[str]:
    """Non-fatal gaps. wdi-simple is only needed if the WinUSB driver is absent."""
    warnings: list[str] = []
    if not wdi_simple().is_file():
        warnings.append(
            "wdi-simple.exe not bundled: automatic DFU-driver install is "
            "disabled. Install the WinUSB driver once via Zadig or "
            "STM32CubeProgrammer if dfu-util cannot see the board."
        )
    return warnings
