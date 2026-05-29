"""Fetch firmware over HTTPS (no Git required).

The app downloads just what it needs from the fork -- the firmware images, the
ODID bootloader, and uploader.py -- into a local cache folder, laid out with the
same relative paths the repo uses so assets.py resolves them transparently
(via assets.set_repo_root(cache_dir)).
"""
from __future__ import annotations

import json
import os
import ssl
import urllib.request
from pathlib import Path

from . import config

_UA = {"User-Agent": "OI-ODID-Flasher"}


def default_cache_dir() -> Path:
    base = os.environ.get("LOCALAPPDATA") or os.environ.get("APPDATA") or str(Path.home())
    return Path(base) / "OI-ODID-Flasher" / config.DEFAULT_CACHE_DIRNAME


def _ctx() -> ssl.SSLContext:
    # Default context uses the OS / Python CA bundle; GitHub certs validate fine.
    return ssl.create_default_context()


def _get(url: str, timeout: float = 60.0) -> bytes:
    req = urllib.request.Request(url, headers=_UA)
    with urllib.request.urlopen(req, timeout=timeout, context=_ctx()) as r:
        return r.read()


def _download(url: str, dest: Path, log=print) -> None:
    dest.parent.mkdir(parents=True, exist_ok=True)
    log(f"  GET {url}")
    req = urllib.request.Request(url, headers=_UA)
    with urllib.request.urlopen(req, timeout=120, context=_ctx()) as r:
        total = int(r.headers.get("Content-Length") or 0)
        got = 0
        tmp = dest.with_suffix(dest.suffix + ".part")
        with open(tmp, "wb") as f:
            while True:
                chunk = r.read(65536)
                if not chunk:
                    break
                f.write(chunk)
                got += len(chunk)
        os.replace(tmp, dest)
    size = f"{got/1024:.0f} KB" + (f" / {total/1024:.0f} KB" if total else "")
    log(f"      saved {dest.name} ({size})")


def list_firmware_names(log=print) -> list[str]:
    """Enumerate *.apj in the repo's firmware folder; fall back to a known list."""
    try:
        data = json.loads(_get(config.CONTENTS_API).decode("utf-8"))
        names = [e["name"] for e in data
                 if isinstance(e, dict) and e.get("name", "").endswith(".apj")]
        if names:
            return sorted(names)
        log("  contents API returned no .apj; using built-in list.")
    except Exception as e:  # noqa: BLE001 - offline / rate limited
        log(f"  contents API unavailable ({e}); using built-in list.")
    return list(config.KNOWN_FIRMWARE)


def fetch_all(dest, log=print) -> Path:
    """Download firmware + bootloader + uploader.py into `dest`. Returns `dest`.

    Layout mirrors the repo so assets.set_repo_root(dest) just works:
        dest/OI-ODID-Flasher/firmware/*.apj
        dest/Tools/bootloaders/CubeOrangePlus-ODID_bl.bin
        dest/Tools/scripts/uploader.py
    """
    dest = Path(dest)
    log(f"Downloading firmware from {config.REPO_OWNER}/{config.REPO_NAME} "
        f"@ {config.REPO_BRANCH} into {dest} ...")

    # 1. firmware images
    names = list_firmware_names(log=log)
    for name in names:
        url = f"{config.RAW_BASE}/{config.REPO_FIRMWARE_SUBDIR}/{name}"
        _download(url, dest / config.REPO_FIRMWARE_SUBDIR / name, log=log)

    # 2. ODID bootloader
    _download(f"{config.RAW_BASE}/{config.REPO_BOOTLOADER_PATH}",
              dest / config.REPO_BOOTLOADER_PATH, log=log)

    # 3. uploader.py
    _download(f"{config.RAW_BASE}/{config.REPO_UPLOADER_PATH}",
              dest / config.REPO_UPLOADER_PATH, log=log)

    log("Firmware download complete.")
    return dest
