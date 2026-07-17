#Requires -Version 5.1
<#
.SYNOPSIS
  One-command bring-up for the VR hand-teleop suite (Windows PowerShell 5.1).

.DESCRIPTION
  Starts only the servers that are not already listening, then deploys the /hand
  page to every connected Quest headset. Idempotent — safe to re-run.

    cockpit  :8013  .venv-so101 python  scripts\so101_cockpit_server.py   (SO-101 leader/follower)
    hand     :8001  python  scripts\server.py --offline --real-hand        (COM9 5-finger hand)
    home     :8010  python  scripts\home_server.py                         (launcher)

  SO-101 follow (teleop) is NOT auto-enabled — turn it on from the cockpit toggle
  (safety: no unattended motion at boot).

.PARAMETER NoDeploy
  Start the servers but skip the Quest deploy step.

.PARAMETER DryRunDeploy
  Pass --dry-run to deploy_hand.py (print the adb/CDP plan, touch no device).

.EXAMPLE
  powershell -ExecutionPolicy Bypass -File scripts\teleop_all.ps1
#>
param(
  [switch]$NoDeploy,
  [switch]$DryRunDeploy
)

$ErrorActionPreference = 'Stop'
# scripts\ -> repo root
$root = Split-Path -Parent $PSScriptRoot
Set-Location $root

function Test-PortListening {
  param([int]$Port)
  $client = New-Object System.Net.Sockets.TcpClient
  try {
    $iar = $client.BeginConnect('127.0.0.1', $Port, $null, $null)
    if ($iar.AsyncWaitHandle.WaitOne(300) -and $client.Connected) {
      $client.EndConnect($iar); return $true
    }
    return $false
  } catch {
    return $false
  } finally {
    $client.Close()
  }
}

function Start-Server {
  param(
    [string]$Name,
    [int]$Port,
    [string]$Exe,
    [string[]]$ExeArgs
  )
  if (Test-PortListening -Port $Port) {
    Write-Host ("  [skip ] {0,-8} already up on :{1}" -f $Name, $Port) -ForegroundColor DarkGray
    return
  }
  if (-not (Get-Command $Exe -ErrorAction SilentlyContinue) -and -not (Test-Path $Exe)) {
    Write-Host ("  [WARN ] {0,-8} skipped: '{1}' not found" -f $Name, $Exe) -ForegroundColor Yellow
    return
  }
  Write-Host ("  [start] {0,-8} -> :{1}" -f $Name, $Port) -ForegroundColor Cyan
  Start-Process -FilePath $Exe -ArgumentList $ExeArgs -WorkingDirectory $root -WindowStyle Minimized | Out-Null
}

Write-Host "== VR teleop suite bring-up ==" -ForegroundColor White

# 1. SO-101 cockpit (needs the lerobot venv). Skips itself if the venv is absent.
$venvPy = Join-Path $root '.venv-so101\Scripts\python.exe'
if (Test-Path $venvPy) {
  Start-Server -Name 'cockpit' -Port 8013 -Exe $venvPy -ExeArgs @('scripts\so101_cockpit_server.py')
} else {
  if (Test-PortListening -Port 8013) {
    Write-Host "  [skip ] cockpit  already up on :8013" -ForegroundColor DarkGray
  } else {
    Write-Host "  [WARN ] cockpit  skipped: .venv-so101 not found (SO-101 teleop unavailable)" -ForegroundColor Yellow
  }
}

# 2. Hand server (virtual arm + REAL hand on COM9). Serves /hand for the Quests.
Start-Server -Name 'hand' -Port 8001 -Exe 'python' -ExeArgs @('scripts\server.py', '--offline', '--real-hand', '--port', '8001')

# 3. Home launcher.
Start-Server -Name 'home' -Port 8010 -Exe 'python' -ExeArgs @('scripts\home_server.py')

# Give freshly-started servers a moment to bind before deploy / status.
Write-Host "  waiting for servers to bind..." -ForegroundColor DarkGray
Start-Sleep -Seconds 3

Write-Host ""
Write-Host "port status:" -ForegroundColor White
foreach ($p in @(@{n='cockpit';port=8013}, @{n='hand';port=8001}, @{n='home';port=8010})) {
  $up = Test-PortListening -Port $p.port
  $tag = if ($up) { 'UP  ' } else { 'down' }
  $col = if ($up) { 'Green' } else { 'Red' }
  Write-Host ("  {0,-8} :{1}  {2}" -f $p.n, $p.port, $tag) -ForegroundColor $col
}

# 4. Deploy /hand to the Quest headsets.
if ($NoDeploy) {
  Write-Host ""
  Write-Host "Quest deploy skipped (-NoDeploy)." -ForegroundColor DarkGray
} else {
  Write-Host ""
  Write-Host "== deploying /hand to Quest headsets ==" -ForegroundColor White
  $deployArgs = @('scripts\quest\deploy_hand.py', '--port', '8001')
  if ($DryRunDeploy) { $deployArgs += '--dry-run' }
  try {
    & python @deployArgs
  } catch {
    Write-Host ("deploy_hand.py failed: {0}" -f $_.Exception.Message) -ForegroundColor Yellow
    Write-Host "(servers are still up; you can retry: python scripts\quest\deploy_hand.py)" -ForegroundColor DarkGray
  }
}

# 5. Summary.
Write-Host ""
Write-Host "== ready ==" -ForegroundColor Green
Write-Host "  home (launcher)     http://localhost:8010/"
Write-Host "  hand teleop (/hand) http://localhost:8001/hand"
Write-Host "  SO-101 cockpit      http://localhost:8013/"
Write-Host ""
Write-Host "  On each Quest: open the tab, tap 「VR 開始」, show your right hand." -ForegroundColor White
Write-Host "  SO-101 follow (teleop) ON  =  toggle it in the cockpit (:8013) — not auto-started." -ForegroundColor White
Write-Host "  Two headsets share /hand/fingers latest-wins (no exclusion). See docs/VR_TELEOP.md."
