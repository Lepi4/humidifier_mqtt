param(
  [string]$Port = 'COM4',
  [int]$Baud = 115200,
  [int]$Seconds = 12,
  [switch]$Reset
)

$sp = New-Object System.IO.Ports.SerialPort $Port,$Baud,'None',8,'One'
$sp.ReadTimeout = 200
$sp.NewLine = "`n"
$sp.DtrEnable = $false
$sp.RtsEnable = $false

$sp.Open()

if ($Reset) {
  # Best-effort ESP32 auto-reset sequence using DTR/RTS.
  # Different USB-UART adapters wire these differently, so this may not work everywhere.
  try {
    $sp.DtrEnable = $true
    $sp.RtsEnable = $false
    Start-Sleep -Milliseconds 120
    $sp.DtrEnable = $false
    $sp.RtsEnable = $true
    Start-Sleep -Milliseconds 120
    $sp.RtsEnable = $false
    Start-Sleep -Milliseconds 200
  } catch {
    # ignore
  }
}

Write-Host ("--- Serial {0} @{1} for {2}s ---" -f $Port, $Baud, $Seconds)

$sw = [System.Diagnostics.Stopwatch]::StartNew()
while ($sw.Elapsed.TotalSeconds -lt $Seconds) {
  try {
    $line = $sp.ReadLine()
    if ($line) {
      $line = $line.TrimEnd("`r", "`n")
      if ($line.Length -gt 0) {
        Write-Host $line
      }
    }
  } catch [System.TimeoutException] {
    # ignore
  }
}

$sp.Close()
Write-Host '--- done ---'
