#!/bin/bash
# Double-click in Finder to start the Airframe Simulator (PX4 SITL, or the Pixhawk if one is plugged in).
cd "$(dirname "$0")/.." || exit 1
if [ ! -x .venv/bin/python ]; then
  echo "Creating Python environment…"
  python3 -m venv .venv && .venv/bin/pip install -q pymavlink numpy fastapi "uvicorn[standard]" pyserial websockets
fi
exec .venv/bin/python -m airframe_sim --mode auto --launch-px4
