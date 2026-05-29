"""Entry point: ``python -m odidflasher`` or the frozen exe.

``--check`` runs a no-hardware self-test (asset presence + uploader import) so
the bundle can be validated in CI / on a machine without a board attached.
"""
from __future__ import annotations

import sys

from . import assets


class _NullWriter:
    """Swallow writes; in --windowed builds sys.stdout/err can be None."""

    def write(self, *_a):  # noqa: D401
        return 0

    def flush(self):
        pass


def _guard_streams() -> None:
    if sys.stdout is None:
        sys.stdout = _NullWriter()
    if sys.stderr is None:
        sys.stderr = sys.stdout


def _self_check() -> int:
    from . import config, fetch

    print("Asset check:")
    # If firmware was downloaded before, resolve against that cache.
    d = fetch.default_cache_dir()
    if (d / config.REPO_FIRMWARE_SUBDIR).is_dir():
        assets.set_repo_root(d)
        print(f"  firmware cache: {d}")
    else:
        print(f"  firmware cache: empty (would download from {config.RAW_BASE})")
    print(f"  dfu-util:    {assets.dfu_util()}  exists={assets.dfu_util().is_file()}")
    print(f"  zadig:       {assets.zadig()}  exists={assets.zadig().is_file()}")
    print(f"  bootloader:  {assets.bootloader_bin()}  exists={assets.bootloader_bin().is_file()}")
    for p in assets.list_firmware():
        print(f"  firmware:    {p}")

    problems = assets.missing_assets()   # bundle-only requirement: dfu-util
    up = assets.uploader_py()
    if up is None:
        print("  uploader.py: not downloaded yet (fetched with firmware)")
    else:
        try:
            from . import fw_upload
            fw_upload._load_uploader()
            print(f"  uploader.py: loaded OK ({up})")
        except Exception as e:  # noqa: BLE001
            problems.append(f"uploader.py load failed: {e}")
            print(f"  uploader.py: FAILED ({e})")
    if problems:
        print("MISSING (blocks flashing):")
        for p in problems:
            print("  - " + p)
    return 1 if problems else 0


def main() -> int:
    _guard_streams()
    if "--check" in sys.argv:
        return _self_check()
    from .wizard import run

    run()
    return 0


if __name__ == "__main__":
    sys.exit(main())
