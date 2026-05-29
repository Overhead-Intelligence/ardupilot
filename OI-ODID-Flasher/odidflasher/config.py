"""Static configuration for the CubeOrangePlus-ODID flasher.

All addresses / IDs here are verified against this ArduPilot fork:
  * Bootloader load address (internal flash base):            0x08000000
  * CubeOrangePlus normal board id:                           1063
  * CubeOrangePlus-ODID board id (set in the .apj firmware):  11063
  * STM32 ROM system-bootloader DFU device:                   VID 0x0483 / PID 0xDF11

The ODID bootloader refuses any firmware whose board_id != 11063, which is the
one-time "lockdown" the operator is warned about.
"""

# --- DFU (STM32 system bootloader, exposed after reboot-to-DFU) --------------
DFU_VID = 0x0483
DFU_PID = 0xDF11
DFU_FLASH_ADDR = 0x08000000  # internal flash base; bootloader lives here

# --- Board identity ----------------------------------------------------------
BOARD_ID_NORMAL = 1063        # AP_HW_CUBEORANGEPLUS
BOARD_ID_ODID = 11063         # AP_HW_CUBEORANGEPLUS_ODID

# USB VID for Hex/CubePilot boards running ArduPilot (used only as a hint when
# auto-detecting the MAVLink COM port; detection ultimately relies on a
# MAVLink heartbeat, so an unknown VID is not fatal).
CUBE_USB_VIDS = (0x2DAE, 0x1209, 0x0483)

# --- MAVLink reboot-to-DFU ---------------------------------------------------
# ArduPilot's MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN "enter DFU" action is gated by a
# magic sequence in param1..param3, then the action code 99 in param4
# (GCS_Common.cpp handle_preflight_reboot: param1==42 && param2==24 && param3==71,
# then param4==99 -> boot_to_dfu()). All four are required.
REBOOT_MAGIC_P1 = 42.0
REBOOT_MAGIC_P2 = 24.0
REBOOT_MAGIC_P3 = 71.0
REBOOT_TO_DFU_MAGIC = 99.0   # param4 action code
MAVLINK_BAUD = 115200

# --- Bundled asset filenames (resolved by assets.py) -------------------------
BOOTLOADER_BIN = "CubeOrangePlus-ODID_bl.bin"
DFU_UTIL_EXE = "dfu-util.exe"
WDI_SIMPLE_EXE = "wdi-simple.exe"   # optional silent driver install (not shipped by libwdi)
ZADIG_EXE = "zadig.exe"             # bundled GUI driver installer (auto-launched)
FIRMWARE_DIR = "firmware"           # folder of selectable *.apj images

# --- Firmware source: direct HTTPS download (no Git needed) ------------------
# The app downloads just the files it needs from the fork over HTTPS into a
# local cache folder, mirroring the repo's paths so assets.py resolves them.
REPO_OWNER = "Overhead-Intelligence"
REPO_NAME = "ardupilot"
REPO_BRANCH = "OI-4.7-dev"
RAW_BASE = f"https://raw.githubusercontent.com/{REPO_OWNER}/{REPO_NAME}/{REPO_BRANCH}"

# Paths within the repo (also used as raw URL suffixes and local cache layout).
REPO_FIRMWARE_SUBDIR = "OI-ODID-Flasher/firmware"
REPO_BOOTLOADER_PATH = "Tools/bootloaders/CubeOrangePlus-ODID_bl.bin"
REPO_UPLOADER_PATH = "Tools/scripts/uploader.py"

# GitHub contents API to enumerate firmware images (falls back to KNOWN_FIRMWARE).
CONTENTS_API = (f"https://api.github.com/repos/{REPO_OWNER}/{REPO_NAME}/contents/"
                f"{REPO_FIRMWARE_SUBDIR}?ref={REPO_BRANCH}")
KNOWN_FIRMWARE = [
    "arduplane_CubeOrangePlus-ODID_DID-RNGFND-TERR.apj",
    "arduplane_CubeOrangePlus-ODID_ODID-CMTC-RNGFND.apj",
]

# Local cache folder (overridable in the UI), under %LOCALAPPDATA%\OI-ODID-Flasher.
DEFAULT_CACHE_DIRNAME = "firmware-cache"

# Prefer the firmware image whose name contains this tag when auto-selecting.
PREFERRED_FIRMWARE_TAG = "TERR"

APP_TITLE = "CubeOrangePlus-ODID Flasher"
