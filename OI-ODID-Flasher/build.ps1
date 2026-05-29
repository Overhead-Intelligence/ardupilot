<#
build.ps1 - produce the single-file flasher .exe with PyInstaller.

The exe bundles ONLY the native Windows helpers (dfu-util.exe, wdi-simple.exe).
The firmware, the ODID bootloader, and uploader.py are pulled at runtime from
the cloned Overhead-Intelligence/ardupilot repo (see odidflasher/repo.py), so
they are NOT baked into the exe.

Run from Windows (NOT WSL) in this folder:
    py -3 -m pip install -r requirements.txt
    .\fetch_binaries.ps1        # one time: get dfu-util.exe (+ optional wdi-simple.exe)
    .\build.ps1

Output: dist\CubeOrangePlus-ODID-Flasher.exe
#>
$ErrorActionPreference = "Stop"
$here = Split-Path -Parent $MyInvocation.MyCommand.Definition
$payload = Join-Path $here "payload"

Write-Host "==> Staging native helpers"
if (Test-Path $payload) { Remove-Item -Recurse -Force $payload }
New-Item -ItemType Directory -Force -Path (Join-Path $payload "bin") | Out-Null

# dfu-util.exe (required) + wdi-simple.exe (optional) + libusb dll if present.
$dfu = Join-Path $here "bin\dfu-util.exe"
if (-not (Test-Path $dfu)) { throw "Missing dfu-util.exe in bin\ - run fetch_binaries.ps1 first" }
Copy-Item $dfu (Join-Path $payload "bin")

$wdi = Join-Path $here "bin\wdi-simple.exe"
if (Test-Path $wdi) {
    Copy-Item $wdi (Join-Path $payload "bin")
} else {
    Write-Warning "wdi-simple.exe not present - auto driver install disabled (install driver via Zadig/CubeProgrammer)."
}

$libusb = Join-Path $here "bin\libusb-1.0.dll"
if (Test-Path $libusb) { Copy-Item $libusb (Join-Path $payload "bin") }

Write-Host "==> Running PyInstaller"
$sep = ";"   # Windows add-data separator: "SRC;DEST"
$addBin = "$payload\bin" + $sep + "bin"
py -3 -m PyInstaller `
    --noconfirm --clean --onefile --windowed `
    --name "CubeOrangePlus-ODID-Flasher" `
    --add-data $addBin `
    --hidden-import "serial.tools.list_ports" `
    --collect-submodules pymavlink `
    flasher.py

Write-Host "==> Done. Output: $(Join-Path $here 'dist\CubeOrangePlus-ODID-Flasher.exe')"
Write-Host "    The exe clones the repo on first run to fetch firmware."
