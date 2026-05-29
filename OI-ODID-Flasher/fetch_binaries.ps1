<#
fetch_binaries.ps1 - download the native helper executables into .\bin

  * dfu-util.exe + libusb-1.0.dll   (DFU flashing)        - REQUIRED
  * wdi-simple.exe                  (auto WinUSB install)  - OPTIONAL

If wdi-simple.exe cannot be fetched automatically, the flasher still works as
long as the WinUSB/libusb driver is bound to the "STM32 BOOTLOADER" device by
some other means (Zadig, or installing STM32CubeProgrammer). The app detects an
existing driver and skips the auto-install step.

Edit the URLs below if a version moves. Run from Windows in this folder.
#>
$ErrorActionPreference = "Stop"
$here = Split-Path -Parent $MyInvocation.MyCommand.Definition
$bin = Join-Path $here "bin"
New-Item -ItemType Directory -Force -Path $bin | Out-Null

# --- dfu-util (REQUIRED) ----------------------------------------------------
# Official Windows binaries (contains win64\dfu-util.exe + libusb-1.0.dll).
$dfuUrl = "https://dfu-util.sourceforge.net/releases/dfu-util-0.11-binaries.tar.xz"
$tmp = Join-Path $env:TEMP "dfu-util.tar.xz"
$ext = Join-Path $env:TEMP "dfu-util-extract"

Write-Host "==> Downloading dfu-util from $dfuUrl"
Invoke-WebRequest -Uri $dfuUrl -OutFile $tmp
if (Test-Path $ext) { Remove-Item -Recurse -Force $ext }
New-Item -ItemType Directory -Force -Path $ext | Out-Null
# Windows 10/11 ship bsdtar, which handles .tar.xz.
tar -xf $tmp -C $ext
$dfuExe = Get-ChildItem -Path $ext -Recurse -Filter dfu-util.exe |
    Where-Object { $_.FullName -match "win64" } | Select-Object -First 1
if (-not $dfuExe) {
    $dfuExe = Get-ChildItem -Path $ext -Recurse -Filter dfu-util.exe | Select-Object -First 1
}
if (-not $dfuExe) { throw "dfu-util.exe not found in archive" }
Copy-Item $dfuExe.FullName (Join-Path $bin "dfu-util.exe") -Force
$libusb = Get-ChildItem -Path $dfuExe.Directory -Filter libusb-1.0.dll | Select-Object -First 1
if ($libusb) { Copy-Item $libusb.FullName (Join-Path $bin "libusb-1.0.dll") -Force }
Write-Host "    dfu-util.exe ready."

# --- wdi-simple (OPTIONAL) --------------------------------------------------
# libwdi ships wdi-simple as an example binary. Releases live at:
#   https://github.com/pbatard/libwdi/releases
# Asset names vary by version, so this is best-effort. If it fails, install the
# driver once with Zadig (https://zadig.akeo.ie) or STM32CubeProgrammer instead.
$wdiUrl = $env:WDI_SIMPLE_URL   # set this env var to a direct wdi-simple.exe URL
if ($wdiUrl) {
    try {
        Write-Host "==> Downloading wdi-simple from $wdiUrl"
        Invoke-WebRequest -Uri $wdiUrl -OutFile (Join-Path $bin "wdi-simple.exe")
        Write-Host "    wdi-simple.exe ready."
    } catch {
        Write-Warning "wdi-simple download failed: $_"
    }
} else {
    Write-Warning @"
wdi-simple.exe was NOT fetched (no WDI_SIMPLE_URL set).
Either:
  * drop a wdi-simple.exe into .\bin yourself (build from github.com/pbatard/libwdi), or
  * pre-install the WinUSB driver once with Zadig / STM32CubeProgrammer.
The flasher will still build; it just won't auto-install the DFU driver.
"@
}

Write-Host "==> bin\ contents:"
Get-ChildItem $bin | Format-Table Name, Length
