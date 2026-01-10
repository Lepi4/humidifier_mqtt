# Прошивка по USB (COM-порт) через PlatformIO
# Использование:
#   powershell -NoProfile -ExecutionPolicy Bypass -File .\scripts\flash_usb.ps1
#   powershell -NoProfile -ExecutionPolicy Bypass -File .\scripts\flash_usb.ps1 -Port COM5
#
# По умолчанию прошивает окружение esp32dev.

param(
  [string]$Port = "COM3",
  [string]$Env = "esp32dev"
)

$ErrorActionPreference = 'Stop'

function Resolve-Pio {
  $cmd = Get-Command pio -ErrorAction SilentlyContinue
  if ($cmd) { return $cmd.Path }

  $default = Join-Path $env:USERPROFILE ".platformio\penv\Scripts\pio.exe"
  if (Test-Path $default) { return $default }

  throw "PlatformIO 'pio' не найден. Установите PlatformIO или добавьте pio в PATH. Ожидался: $default"
}

$pio = Resolve-Pio
$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot ".."))
Set-Location $projectRoot

Write-Host "Project: $projectRoot"
Write-Host "PIO:     $pio"
Write-Host "Env:     $Env"
Write-Host "Port:    $Port"

# Сборка (чтобы гарантированно получить актуальный firmware.bin)
& $pio run -e $Env -t build

$fwRel = ".pio\build\$Env\firmware.bin"
$fw = Resolve-Path $fwRel
Write-Host "Firmware: $fw"

# Заливка
& $pio run -e $Env -t upload --upload-port $Port
