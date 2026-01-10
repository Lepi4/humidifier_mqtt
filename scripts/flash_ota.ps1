# Прошивка по OTA (IP) через PlatformIO
# Использование:
#   powershell -NoProfile -ExecutionPolicy Bypass -File .\scripts\flash_ota.ps1
#   powershell -NoProfile -ExecutionPolicy Bypass -File .\scripts\flash_ota.ps1 -Ip 192.168.1.95
#
# По умолчанию прошивает окружение esp32ota.

param(
  [string]$Ip = "192.168.1.95",
  [string]$Env = "esp32ota"
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
Write-Host "IP:      $Ip"

# Сборка (чтобы гарантированно получить актуальный firmware.bin)
& $pio run -e $Env -t build

$fwRel = ".pio\build\$Env\firmware.bin"
$fw = Resolve-Path $fwRel
Write-Host "Firmware: $fw"

# OTA заливка
& $pio run -e $Env -t upload --upload-port $Ip
