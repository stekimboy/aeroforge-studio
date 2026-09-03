# AeroForge launcher (PowerShell). If script execution is blocked, either run:
#   Set-ExecutionPolicy -Scope Process Bypass
# or simply double-click run.bat instead.
$ErrorActionPreference = "Stop"
Set-Location $PSScriptRoot

# ---- Locate Python (prefer 3.11, then 3.12, 3.13, generic) ----
$pycmd = $null
foreach ($cand in @(@("py", "-3.11"), @("py", "-3.12"), @("py", "-3.13"), @("python"))) {
    try {
        & $cand[0] $cand[1..($cand.Count - 1)] --version *> $null
        if ($LASTEXITCODE -eq 0) { $pycmd = $cand; break }
    } catch {}
}
if ($null -eq $pycmd) {
    Write-Host "[AeroForge] Python not found. Install Python 3.11/3.12 from python.org (check 'Add python.exe to PATH')."
    exit 1
}
Write-Host "[AeroForge] Using interpreter: $($pycmd -join ' ')"

# ---- Create venv if missing ----
if (-not (Test-Path ".venv\Scripts\python.exe")) {
    Write-Host "[AeroForge] Creating virtual environment..."
    & $pycmd[0] $pycmd[1..($pycmd.Count - 1)] -m venv .venv
    if ($LASTEXITCODE -ne 0) { Write-Host "[AeroForge] venv creation failed."; exit 1 }
}
$vpy = ".venv\Scripts\python.exe"

# ---- Install dependencies once ----
if (-not (Test-Path ".venv\.deps_installed")) {
    Write-Host "[AeroForge] Installing dependencies (first run only)..."
    & $vpy -m pip install --upgrade pip
    & $vpy -m pip install -r requirements.txt
    if ($LASTEXITCODE -ne 0) { Write-Host "[AeroForge] Dependency install failed."; exit 1 }
    "ok" | Out-File ".venv\.deps_installed" -Encoding utf8
}

# ---- Start server ----
Write-Host "[AeroForge] Starting server at http://127.0.0.1:8000 ..."
Start-Process -WindowStyle Minimized $vpy -ArgumentList "-m", "uvicorn", "backend.main:app", "--host", "127.0.0.1", "--port", "8000"

# ---- Wait for health, then open browser ----
$healthy = $false
for ($i = 0; $i -lt 120; $i++) {
    try {
        Invoke-WebRequest -UseBasicParsing -Uri "http://127.0.0.1:8000/health" -TimeoutSec 2 *> $null
        $healthy = $true; break
    } catch { Start-Sleep -Seconds 1 }
}
if (-not $healthy) { Write-Host "[AeroForge] Server did not become healthy."; exit 1 }

Start-Process "http://127.0.0.1:8000"
Write-Host "[AeroForge] Running. Close the server window to stop the app."
