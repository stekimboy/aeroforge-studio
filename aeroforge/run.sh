#!/usr/bin/env bash
# ============================================================
#  AeroForge launcher for macOS / Linux - the run.bat equivalent.
#    chmod +x run.sh && ./run.sh
#  Creates a venv, installs dependencies (first run only),
#  starts the server in the background, and opens your browser.
# ============================================================
set -euo pipefail
cd "$(dirname "$0")"

# ---- Locate Python (prefer 3.12, then 3.11, 3.13, then generic) ----
PYCMD=""
for c in python3.12 python3.11 python3.13 python3; do
  if command -v "$c" >/dev/null 2>&1; then PYCMD="$c"; break; fi
done
if [ -z "$PYCMD" ]; then
  echo "[AeroForge] Python 3 was not found. Install it with:  brew install python@3.12"
  exit 1
fi
echo "[AeroForge] Using interpreter: $PYCMD ($($PYCMD --version))"

# ---- Create venv if missing ----
if [ ! -x ".venv/bin/python" ]; then
  echo "[AeroForge] Creating virtual environment..."
  "$PYCMD" -m venv .venv
fi
VPY=".venv/bin/python"

# ---- Install dependencies once (marker file skips reinstall) ----
if [ ! -f ".venv/.deps_installed" ]; then
  echo "[AeroForge] Installing dependencies - a few minutes on first run (cadquery/OCP wheels are large)..."
  "$VPY" -m pip install --upgrade pip
  "$VPY" -m pip install -r requirements.txt
  echo ok > ".venv/.deps_installed"
fi

# ---- Start the server in the background, log beside it ----
echo "[AeroForge] Starting server at http://127.0.0.1:8000 (log: server.log) ..."
nohup "$VPY" -m uvicorn backend.main:app --host 127.0.0.1 --port 8000 > server.log 2> server.err &
echo $! > .server.pid

# ---- Wait for the health endpoint (up to ~120 s), then open the browser ----
for i in $(seq 1 120); do
  if curl -s -o /dev/null http://127.0.0.1:8000/health; then
    if command -v open >/dev/null 2>&1; then open http://127.0.0.1:8000
    elif command -v xdg-open >/dev/null 2>&1; then xdg-open http://127.0.0.1:8000
    fi
    echo
    echo "[AeroForge] Running (pid $(cat .server.pid)). Stop with:  kill \$(cat .server.pid)"
    echo
    exit 0
  fi
  sleep 1
done
echo "[AeroForge] Server did not become healthy in time - see server.err"
exit 1
