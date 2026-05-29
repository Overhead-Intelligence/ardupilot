"""Clone / update the Overhead-Intelligence ardupilot fork on the target machine.

The flasher pulls its firmware (and the bootloader + uploader.py) from this
clone, so the repo is the single source of truth. Clone is shallow, single
branch, and skips submodules -- none of which are needed for flashing (they
matter only when building from source).
"""
from __future__ import annotations

import os
import shutil
import subprocess
from pathlib import Path

from . import config

_NO_WINDOW = getattr(subprocess, "CREATE_NO_WINDOW", 0)


def git_available() -> bool:
    return shutil.which("git") is not None


def default_clone_dir() -> Path:
    base = os.environ.get("LOCALAPPDATA") or os.environ.get("APPDATA") or str(Path.home())
    return Path(base) / "OI-ODID-Flasher" / config.DEFAULT_CLONE_DIRNAME


def _run_git(args: list[str], cwd: str | None = None, log=print) -> None:
    cmd = ["git", *args]
    log("  $ " + " ".join(cmd))
    proc = subprocess.Popen(
        cmd,
        cwd=cwd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        creationflags=_NO_WINDOW,
    )
    assert proc.stdout is not None
    for line in proc.stdout:
        line = line.rstrip()
        if line:
            log("    " + line)
    rc = proc.wait()
    if rc != 0:
        raise RuntimeError(f"git {args[0]} failed (exit {rc})")


def _is_git_repo(path: Path) -> bool:
    return (path / ".git").exists()


def clone_or_update(dest: str | os.PathLike, log=print) -> Path:
    """Ensure `dest` holds an up-to-date checkout of the fork. Returns its path.

    If `dest` already contains the repo, it is fetched and hard-reset to the
    remote branch; otherwise a fresh shallow clone is made.
    """
    if not git_available():
        raise RuntimeError(
            "git is not installed or not on PATH. Install Git for Windows "
            "(https://git-scm.com/download/win) and retry."
        )
    dest = Path(dest)
    branch = config.REPO_BRANCH

    if _is_git_repo(dest):
        log(f"Updating existing repo at {dest} ...")
        _run_git(["remote", "set-url", "origin", config.REPO_URL], cwd=str(dest), log=log)
        _run_git(
            ["fetch", "--depth", str(config.REPO_CLONE_DEPTH), "origin", branch],
            cwd=str(dest), log=log,
        )
        _run_git(["checkout", "-B", branch, "FETCH_HEAD"], cwd=str(dest), log=log)
        _run_git(["reset", "--hard", "FETCH_HEAD"], cwd=str(dest), log=log)
    else:
        log(f"Cloning {config.REPO_URL} ({branch}) into {dest} ...")
        dest.parent.mkdir(parents=True, exist_ok=True)
        args = [
            "clone",
            "--branch", branch,
            "--single-branch",
            "--depth", str(config.REPO_CLONE_DEPTH),
        ]
        if not config.REPO_SUBMODULES:
            pass  # submodules simply not requested
        args += [config.REPO_URL, str(dest)]
        _run_git(args, log=log)

    log("Repository ready.")
    return dest
