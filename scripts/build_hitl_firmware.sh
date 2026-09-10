#!/bin/bash
# Build PX4 firmware for a flight controller WITH the HIL output driver (pwm_out_sim) that HITL needs.
# Standard PX4 release firmware for most boards leaves it out (CONFIG_MODULES_SIMULATION_PWM_OUT_SIM=n).
#
#   scripts/build_hitl_firmware.sh [board] [build|upload] [variant]
#     board    px4_fmu-v6x (default), px4_fmu-v5, px4_fmu-v6c, ...
#     upload   build, then flash over USB (unplug/replug when asked)
#     variant  board config variant; default: "multicopter" if the board has one, else "default".
#              (fmu-v6x "default" is 100% full and does not link with pwm_out_sim added; "multicopter"
#               drops fixed-wing/VTOL/airspeed code and fits.)
#
# Needs the ARM toolchain PX4 uses:  brew tap osx-cross/arm; brew trust osx-cross/arm && brew install osx-cross/arm/arm-gcc-bin@13 && brew link --overwrite --force arm-gcc-bin@13
set -euo pipefail
BOARD="${1:-px4_fmu-v6x}"
ACTION="${2:-build}"
VARIANT="${3:-${VARIANT:-}}"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
HERE="$(cd "$(dirname "$0")/.." && pwd)"
export PATH="$HERE/.venv/bin:$PATH"
# Homebrew's arm-gcc-bin@13 is keg-only unless linked; use it directly when present.
for d in /opt/homebrew/opt/arm-gcc-bin@13/bin /usr/local/opt/arm-gcc-bin@13/bin; do [ -d "$d" ] && export PATH="$d:$PATH"; done

if ! command -v arm-none-eabi-gcc >/dev/null; then
  echo "arm-none-eabi-gcc not found. Install the toolchain first:"
  echo "  brew tap osx-cross/arm; brew trust osx-cross/arm && brew install osx-cross/arm/arm-gcc-bin@13 && brew link --overwrite --force arm-gcc-bin@13"
  exit 1
fi
cd "$PX4_DIR"
BOARD_DIR="boards/${BOARD/_//}"                    # px4_fmu-v6x -> boards/px4/fmu-v6x
if [ -z "$VARIANT" ]; then
  if [ -f "$BOARD_DIR/multicopter.px4board" ]; then VARIANT=multicopter; else VARIANT=default; fi
fi
CFG="$BOARD_DIR/$VARIANT.px4board"
TARGET="${BOARD}_${VARIANT}"
[ -f "$CFG" ] || { echo "no board config at $CFG"; exit 1; }
echo "building $TARGET from $(git describe --tags --always) with pwm_out_sim"

if grep -q "CONFIG_MODULES_SIMULATION_PWM_OUT_SIM=y" "$CFG"; then
  echo "pwm_out_sim already enabled in $CFG"
else
  echo "enabling pwm_out_sim in $CFG"
  echo "CONFIG_MODULES_SIMULATION_PWM_OUT_SIM=y" >> "$CFG"
fi

restore_cfg() { git checkout -q -- "$CFG" 2>/dev/null || true; }
trap restore_cfg EXIT      # leave the checkout clean; the built .px4 keeps the module regardless

make "$TARGET"
OUT="build/$TARGET/$TARGET.px4"
echo
echo "Firmware: $PX4_DIR/$OUT"
echo "Flash it with QGroundControl (Vehicle Setup > Firmware > Advanced > Custom firmware file),"
echo "or run:  scripts/build_hitl_firmware.sh $BOARD upload $VARIANT"

if [ "$ACTION" = "upload" ]; then
  echo "Uploading over USB — if it waits, unplug and replug the board."
  make "$TARGET" upload
fi
