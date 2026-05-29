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
# ArduPilot interprets MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN with param4 == 99 as
# "reboot into DFU" (GCS_Common.cpp), gated on HAL_ENABLE_DFU_BOOT which the
# stock CubeOrange/CubeOrangePlus firmware enables (CubeOrange/hwdef.inc:320).
REBOOT_TO_DFU_MAGIC = 99.0
MAVLINK_BAUD = 115200

# --- Bundled asset filenames (resolved by assets.py) -------------------------
BOOTLOADER_BIN = "CubeOrangePlus-ODID_bl.bin"
DFU_UTIL_EXE = "dfu-util.exe"
WDI_SIMPLE_EXE = "wdi-simple.exe"
FIRMWARE_DIR = "firmware"     # folder of selectable *.apj images

# --- Source repository (the app clones this to fetch firmware) ---------------
# HTTPS (not the SSH "git@" remote) so it works on a fresh machine. If the repo
# is private, git will prompt for credentials via the Windows credential helper.
REPO_URL = "https://github.com/Overhead-Intelligence/ardupilot.git"
REPO_BRANCH = "OI-4.7-dev"
REPO_CLONE_DEPTH = 1          # shallow; we only need committed files, not history
REPO_SUBMODULES = False       # not needed for flashing (only for building)

# Paths *inside* the cloned repo where the flasher finds what it needs.
REPO_FIRMWARE_SUBDIR = "OI-ODID-Flasher/firmware"
REPO_BOOTLOADER_PATH = "Tools/bootloaders/CubeOrangePlus-ODID_bl.bin"
REPO_UPLOADER_PATH = "Tools/scripts/uploader.py"

# Default folder to clone into (overridable in the UI).
DEFAULT_CLONE_DIRNAME = "OI-ardupilot"   # under %LOCALAPPDATA%\OI-ODID-Flasher

# Prefer the firmware image whose name contains this tag when auto-selecting.
PREFERRED_FIRMWARE_TAG = "TERR"

APP_TITLE = "CubeOrangePlus-ODID Flasher"
