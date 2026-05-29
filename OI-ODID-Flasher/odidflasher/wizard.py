"""Tkinter wizard that drives the three flashing stages.

Layout is a single window: firmware + port pickers, a one-time-acknowledgement
checkbox, a Flash button, a Continue button used for the manual unplug/replug
fallback, and a scrolling log. All flashing happens on a worker thread; the UI
is updated by draining a queue from the Tk main loop.
"""
from __future__ import annotations

import contextlib
import os
import queue
import threading
import tkinter as tk
from tkinter import filedialog, messagebox, scrolledtext, ttk

from . import assets, config, dfu, fetch, fw_upload, mavlink_dfu

WARNING_TEXT = (
    "This will install the CubeOrangePlus-ODID bootloader and is a ONE-TIME, "
    "IRREVERSIBLE change.\n\n"
    "After this, the flight controller will ONLY accept ODID-built firmware "
    "(board id 11063). Standard CubeOrangePlus firmware can no longer be loaded "
    "through the normal bootloader.\n\n"
    "Make sure the board is connected by USB and nothing else is talking to it "
    "(close Mission Planner / QGC). Continue?"
)


class _StreamToLog:
    """File-like object that forwards writes (e.g. uploader.py output) to the log."""

    def __init__(self, log):
        self._log = log
        self._buf = ""
        self._last = None

    def _emit(self, line):
        line = line.rstrip()
        if line and line != self._last:   # collapse progress-bar redraws
            self._last = line
            self._log(line)

    def write(self, s):
        self._buf += s
        while "\n" in self._buf or "\r" in self._buf:
            idx = min(
                (self._buf.index(c) for c in "\n\r" if c in self._buf),
                default=-1,
            )
            line, self._buf = self._buf[:idx], self._buf[idx + 1:]
            self._emit(line)

    def flush(self):
        if self._buf.strip():
            self._emit(self._buf)
            self._buf = ""


class FlasherWizard(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title(config.APP_TITLE)
        self.geometry("720x560")
        self.minsize(640, 480)

        self._q: queue.Queue = queue.Queue()
        self._continue_evt = threading.Event()
        self._worker: threading.Thread | None = None
        self._busy_flag = False

        self._build_ui()
        self._check_assets()
        self.after(100, self._drain_queue)

    # ------------------------------------------------------------------ UI
    def _build_ui(self):
        pad = {"padx": 10, "pady": 6}

        top = ttk.Frame(self)
        top.pack(fill="x", **pad)

        ttk.Label(top, text="Firmware folder:").grid(row=0, column=0, sticky="w")
        self.repo_var = tk.StringVar(value=str(fetch.default_cache_dir()))
        self.repo_entry = ttk.Entry(top, textvariable=self.repo_var, width=60)
        self.repo_entry.grid(row=0, column=1, sticky="we", padx=4)
        self.clone_btn = ttk.Button(top, text="Download firmware", command=self._start_download)
        self.clone_btn.grid(row=0, column=2)

        ttk.Label(top, text="Firmware:").grid(row=1, column=0, sticky="w")
        self.fw_var = tk.StringVar()
        self.fw_combo = ttk.Combobox(top, textvariable=self.fw_var, width=60, state="readonly")
        self.fw_combo.grid(row=1, column=1, sticky="we", padx=4)
        ttk.Button(top, text="Browse...", command=self._browse_fw).grid(row=1, column=2)

        ttk.Label(top, text="Board port:").grid(row=2, column=0, sticky="w")
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(top, textvariable=self.port_var, width=60)
        self.port_combo.grid(row=2, column=1, sticky="we", padx=4)
        ttk.Button(top, text="Detect", command=self._refresh_ports).grid(row=2, column=2)

        top.columnconfigure(1, weight=1)

        self.ack_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(
            self,
            text="I understand this is a one-time, irreversible bootloader change.",
            variable=self.ack_var,
            command=self._update_buttons,
        ).pack(anchor="w", padx=12)

        btns = ttk.Frame(self)
        btns.pack(fill="x", **pad)
        self.flash_btn = ttk.Button(btns, text="Flash", command=self._start, state="disabled")
        self.flash_btn.pack(side="left")
        self.continue_btn = ttk.Button(btns, text="Continue", command=self._on_continue, state="disabled")
        self.continue_btn.pack(side="left", padx=8)

        self.status_var = tk.StringVar(value="Ready.")
        ttk.Label(self, textvariable=self.status_var, anchor="w").pack(fill="x", padx=12)

        self.log = scrolledtext.ScrolledText(self, height=18, state="disabled", wrap="word")
        self.log.pack(fill="both", expand=True, padx=10, pady=(0, 10))

    def _check_assets(self):
        problems = assets.missing_assets()   # dfu-util only
        # Reuse a previous download if the cache folder already has firmware.
        d = fetch.default_cache_dir()
        if (d / config.REPO_FIRMWARE_SUBDIR).is_dir():
            assets.set_repo_root(d)
            self.repo_var.set(str(d))
        self._refresh_firmware()
        self._refresh_ports()
        if problems:
            self._log("Asset check FAILED:")
            for p in problems:
                self._log("  - " + p)
            self._log("Rebuild the exe with build.ps1 (see README).")
        else:
            self._log("dfu-util present.")
        if not assets.list_firmware():
            self._log("Firmware downloads from GitHub over HTTPS (no Git needed). "
                      "Click 'Download firmware', or just click Flash.")
        for w in assets.optional_warnings():
            self._log("NOTE: " + w)
        self._update_buttons()

    # --------------------------------------------------------------- helpers
    def _log(self, text: str):
        self._q.put(("log", text))

    def _set_status(self, text: str):
        self._q.put(("status", text))

    def _drain_queue(self):
        try:
            while True:
                kind, payload = self._q.get_nowait()
                if kind == "log":
                    self.log.configure(state="normal")
                    self.log.insert("end", payload + "\n")
                    self.log.see("end")
                    self.log.configure(state="disabled")
                elif kind == "status":
                    self.status_var.set(payload)
                elif kind == "prompt":
                    self.status_var.set(payload)
                    self.continue_btn.configure(state="normal")
                elif kind == "busy":
                    self._busy_flag = payload
                    self._update_buttons()
                elif kind == "refresh_fw":
                    self._refresh_firmware()
                elif kind == "done":
                    ok, msg = payload
                    self._busy_flag = False
                    self.continue_btn.configure(state="disabled")
                    self._update_buttons()
                    if ok:
                        messagebox.showinfo(config.APP_TITLE, msg)
                    elif ok is False:
                        messagebox.showerror(config.APP_TITLE, msg)
        except queue.Empty:
            pass
        self.after(100, self._drain_queue)

    def _browse_fw(self):
        path = filedialog.askopenfilename(
            title="Select ODID firmware (.apj)",
            filetypes=[("ArduPilot firmware", "*.apj"), ("All files", "*.*")],
        )
        if path:
            vals = list(self.fw_combo["values"])
            if path not in vals:
                vals.append(path)
                self.fw_combo["values"] = vals
            self.fw_var.set(path)

    def _refresh_ports(self):
        ports = mavlink_dfu.list_serial_ports()
        self.port_combo["values"] = [p.label for p in ports]
        if ports and not self.port_var.get():
            self.port_var.set(ports[0].label)

    def _refresh_firmware(self):
        """Populate the firmware dropdown from the current source (repo/bundle)."""
        items = [str(p) for p in assets.list_firmware()]
        self.fw_combo["values"] = items
        cur = self.fw_var.get()
        if items and (not cur or cur not in items):
            pref = [i for i in items
                    if config.PREFERRED_FIRMWARE_TAG in os.path.basename(i)]
            self.fw_var.set(pref[0] if pref else items[0])
        if items:
            self._log(f"{len(items)} firmware image(s) available; "
                      f"selected: {os.path.basename(self.fw_var.get())}")

    def _selected_port_device(self) -> str | None:
        val = self.port_var.get().strip()
        if not val:
            return None
        return val.split(" ", 1)[0]  # "COM7 - ..." -> "COM7"

    def _update_buttons(self):
        busy = self._busy_flag
        self.flash_btn.configure(
            state="normal" if (self.ack_var.get() and not busy) else "disabled")
        self.clone_btn.configure(state="disabled" if busy else "normal")

    # -------------------------------------------------------------- download
    def _start_download(self):
        if self._busy_flag:
            return
        self._busy_flag = True
        self._update_buttons()
        self._worker = threading.Thread(target=self._download_worker, daemon=True)
        self._worker.start()

    def _download_worker(self):
        log = self._log
        try:
            dest = self.repo_var.get().strip() or str(fetch.default_cache_dir())
            self._set_status("Downloading firmware ...")
            fetch.fetch_all(dest, log=log)
            assets.set_repo_root(dest)
            self._q.put(("refresh_fw", None))
            self._set_status("Firmware ready.")
            self._q.put(("done", (None, None)))   # re-enable buttons, no dialog
        except Exception as e:  # noqa: BLE001
            log(f"ERROR: {e}")
            self._set_status("Download failed.")
            self._q.put(("done", (False, f"Firmware download failed:\n{e}")))

    def _on_continue(self):
        self.continue_btn.configure(state="disabled")
        self._continue_evt.set()

    def _prompt_continue(self, text: str):
        """Block the worker until the user clicks Continue."""
        self._continue_evt.clear()
        self._q.put(("prompt", text))
        self._continue_evt.wait()

    # ----------------------------------------------------------- orchestration
    def _start(self):
        if not self.ack_var.get():
            return
        if not messagebox.askyesno(config.APP_TITLE, WARNING_TEXT):
            return
        self._busy_flag = True
        self._update_buttons()
        self._worker = threading.Thread(target=self._run_flow, daemon=True)
        self._worker.start()

    def _resolve_firmware(self) -> str:
        """Pick the firmware path: explicit selection, else preferred, else first."""
        apj = self.fw_var.get().strip()
        if apj and os.path.isfile(apj):
            return apj
        fws = assets.list_firmware()
        pref = [str(p) for p in fws if config.PREFERRED_FIRMWARE_TAG in p.name]
        if pref:
            return pref[0]
        return str(fws[0]) if fws else ""

    def _run_flow(self):
        log = self._log
        try:
            # -- Stage 0: ensure firmware is downloaded -----------------------
            if assets.repo_root() is None or not assets.list_firmware():
                self._set_status("Preparing firmware (downloading) ...")
                dest = self.repo_var.get().strip() or str(fetch.default_cache_dir())
                fetch.fetch_all(dest, log=log)
                assets.set_repo_root(dest)
                self._q.put(("refresh_fw", None))

            apj = self._resolve_firmware()
            if not apj:
                raise RuntimeError("No firmware image available after cloning.")
            log(f"Using firmware: {os.path.basename(apj)}")

            if assets.missing_assets():
                raise RuntimeError("; ".join(assets.missing_assets()))
            if not assets.bootloader_bin().is_file():
                raise RuntimeError(f"Bootloader not found: {assets.bootloader_bin()}")
            if assets.uploader_py() is None:
                raise RuntimeError("uploader.py not found (clone may have failed).")

            port = self._selected_port_device()

            # -- Stage 1: enter DFU -------------------------------------------
            self._set_status("Stage 1/3: entering DFU mode")
            # Resolve the *actual* MAVLink port by heartbeat: this picks the
            # autopilot's Mavlink port over its SLCAN port and skips any
            # busy/ghost COM, using the dropdown selection only as a hint.
            resolved = mavlink_dfu.find_mavlink_port(preferred=port, log=log)
            if resolved and resolved != port:
                log(f"Using {resolved} (responds to MAVLink) instead of {port or 'auto'}.")
            port = resolved
            dfu_ready = False
            if port:
                try:
                    mavlink_dfu.reboot_to_dfu(port, log=log)
                    dfu_ready = dfu.wait_for_dfu(timeout=30, log=log)
                except mavlink_dfu.PortBusyError as e:
                    # Port is held by another app -> let the user free it and retry.
                    log(f"{e}")
                    self._prompt_continue(
                        f"{port} is in use by another program (Mission Planner, "
                        "QGroundControl, a serial monitor, ...). Close it and "
                        "DISCONNECT, then click Continue to retry."
                    )
                    try:
                        retry_port = self._selected_port_device() or port
                        mavlink_dfu.reboot_to_dfu(retry_port, log=log)
                        dfu_ready = dfu.wait_for_dfu(timeout=30, log=log)
                    except Exception as e2:  # noqa: BLE001
                        log(f"Retry failed: {e2}")
                except Exception as e:  # noqa: BLE001
                    log(f"Software reboot-to-DFU failed: {e}")
            if not dfu_ready:
                log("Falling back to the manual DFU method.")
                self._prompt_continue(
                    "MANUAL DFU: unplug the board, hold BOOT0 high (jumper the "
                    "BOOT0 pad to 3.3V), plug USB back in, then click Continue."
                )
                dfu_ready = dfu.wait_for_dfu(timeout=30, log=log)
            if not dfu_ready:
                raise RuntimeError("DFU device never appeared. See README for the BOOT0 method.")

            # -- Stage 2: driver + bootloader ---------------------------------
            self._set_status("Stage 2/3: flashing ODID bootloader")
            if not dfu.ensure_driver(log=log, guide=self._prompt_continue):
                raise RuntimeError("Could not install the WinUSB driver for DFU.")
            dfu.flash_bootloader(log=log, leave=True)

            # -- Stage 3: firmware --------------------------------------------
            self._set_status("Stage 3/3: uploading firmware")
            self._prompt_continue(
                "Bootloader installed. If the board does not auto-detect, unplug "
                "and replug it (normally, no BOOT0), then click Continue."
            )
            with contextlib.redirect_stdout(_StreamToLog(log)), \
                    contextlib.redirect_stderr(_StreamToLog(log)):
                bl_port = fw_upload.wait_for_bootloader_port(timeout=40, log=log)
                if not bl_port:
                    raise RuntimeError("ODID bootloader serial port not found.")
                fw_upload.upload_firmware(bl_port, apj, log=log)

            self._set_status("Done.")
            self._q.put(("done", (True, "Bootloader + firmware flashed successfully.")))
        except Exception as e:  # noqa: BLE001
            log(f"ERROR: {e}")
            self._set_status("Failed.")
            self._q.put(("done", (False, f"Flashing failed:\n{e}")))


def run():
    FlasherWizard().mainloop()
