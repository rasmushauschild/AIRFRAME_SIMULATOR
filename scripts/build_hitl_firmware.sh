#!/bin/bash
# Build PX4 firmware for a flight controller WITH the HIL output driver (pwm_out_sim) that HITL needs.
# Standard PX4 release firmware for most boards leaves it out (CONFIG_MODULES_SIMULATION_PWM_OUT_SIM=n).
#
#   scripts/build_hitl_firmware.sh [board]            e.g. px4_fmu-v6x (default), px4_fmu-v5, px4_fmu-v6c
#   scripts/build_hitl_firmware.sh px4_fmu-v6x upload  # build, then flash over USB (unplug/replug when asked)
#
# Needs the ARM toolchain PX4 uses:  brew tap osx-cross/arm && brew install arm-gcc-bin@13 && brew link --overwrite --force arm-gcc-bin@13
set -euo pipefail
BOARD="${1:-px4_fmu-v6x}"
ACTION="${2:-build}"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
HERE="$(cd "$(dirname "$0")/.." && pwd)"
export PATH="$HERE/.venv/bin:$PATH"

if ! command -v arm-none-eabi-gcc >/dev/null; then
  echo "arm-none-eabi-gcc not found. Install the toolchain first:"
  echo "  brew tap osx-cross/arm && brew install arm-gcc-bin@13 && brew link --overwrite --force arm-gcc-bin@13"
  exit 1
fi
cd "$PX4_DIR"
CFG="boards/${BOARD/_//}/default.px4board"       # px4_fmu-v6x -> boards/px4/fmu-v6x/default.px4board
[ -f "$CFG" ] || { echo "no board config at $CFG"; exit 1; }

if grep -q "CONFIG_MODULES_SIMULATION_PWM_OUT_SIM=y" "$CFG"; then
  echo "pwm_out_sim already enabled in $CFG"
else
  echo "enabling pwm_out_sim in $CFG"
  echo "CONFIG_MODULES_SIMULATION_PWM_OUT_SIM=y" >> "$CFG"
fi

make "${BOARD}_default"
OUT="build/${BOARD}_default/${BOARD}_default.px4"
echo
echo "Firmware: $PX4_DIR/$OUT"
echo "Flash it with QGroundControl (Vehicle Setup > Firmware > Advanced > Custom firmware file),"
echo "or run:  scripts/build_hitl_firmware.sh $BOARD upload"

if [ "$ACTION" = "upload" ]; then
  echo "Uploading over USB — if it waits, unplug and replug the board."
  make "${BOARD}_default" upload
fi
