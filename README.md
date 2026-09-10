# AIRFRAME_SIMULATOR

A small, hackable multirotor physics simulator that plugs into PX4 in two ways:

* **SITL** – runs the PX4 SITL binary on this Mac and drives it in lockstep.
* **HITL** – talks to a real Pixhawk over USB, so the flight controller runs its normal firmware against a virtual vehicle.

The point of it: **rotor positions and thrust axes are first-class, editable in a 3D UI, and exported 1:1 to PX4's
control-allocation parameters** (`CA_ROTORn_PX/PY/PZ`, `CA_ROTORn_AX/AY/AZ`, `CA_ROTORn_KM`, output function mapping).
Drag a rotor, push, fly. The same message layer (PX4's Simulator MAVLink API: `HIL_SENSOR`, `HIL_GPS`,
`HIL_STATE_QUATERNION` out, `HIL_ACTUATOR_CONTROLS` in) serves both SITL and HITL.

```
airframe_sim/
  airframe.py    rotor geometry + mass properties, JSON load/save, PX4 parameter export
  physics.py     6-DOF rigid body, per-rotor motor lag, drag, leg/ground contact
  sensors.py     IMU / mag / baro / GPS models -> HIL_SENSOR, HIL_GPS
  link.py        MAVLink: HIL channel (TCP for SITL, serial for HITL), params, commands, QGC proxy
  simulator.py   the loop: actuators -> physics -> sensors -> PX4, lockstep + real-time pacing
  server.py      FastAPI + websocket for the UI
  __main__.py    CLI
ui/              three.js editor (vendored, no build step)
airframes/       saved airframe JSON files (quad_x, hex_x presets)
```

## Setup

```bash
python3 -m venv .venv
.venv/bin/pip install pymavlink numpy fastapi "uvicorn[standard]" pyserial websockets
```

For SITL you need a PX4 build. The default location is `~/PX4-Autopilot`:

```bash
git clone --recursive https://github.com/PX4/PX4-Autopilot.git ~/PX4-Autopilot
.venv/bin/pip install -r ~/PX4-Autopilot/Tools/setup/requirements.txt
cd ~/PX4-Autopilot && PATH="$OLDPWD/.venv/bin:$PATH" make px4_sitl_default
```

(`cmake`, `ninja` and `ccache` from Homebrew are enough on macOS; no Gazebo or ROS required.)

## Run: SITL

```bash
.venv/bin/python -m airframe_sim --mode sitl --launch-px4
```

This starts PX4 SITL (`PX4_SIM_MODEL=none_iris`, i.e. the plain `simulator_mavlink` airframe), listens for it on
TCP 4560, and opens the UI at http://127.0.0.1:8080. PX4's own MAVLink goes to UDP 14550, so QGroundControl connects
by itself. If another PX4 SITL is already running on this machine the launcher picks the next free instance
(ports shift by the instance number; the log tells you which).

Useful flags: `--speed 0` (as fast as PX4 allows), `--rate 500`, `--home lat,lon,alt`, `--airframe airframes/hex_x.json`,
`--px4-dir`, `--px4-instance`, `--no-browser`.

To run PX4 yourself instead of `--launch-px4`:

```bash
cd ~/PX4-Autopilot && PX4_SIM_MODEL=none_iris ./build/px4_sitl_default/bin/px4 -d -w /tmp/px4wd ./build/px4_sitl_default/etc
```

## Run: HITL (real Pixhawk)

The easy way: plug the Pixhawk in over USB and open the **Connect** tab (or click the blue "Pixhawk detected" pill).
It lists the board, connects with one click, and walks a checklist:

1. **Pixhawk detected on USB** – found by USB vendor/product.
2. **Serial link up / Parameters downloaded** – automatic after Connect.
3. **Firmware supports HITL** – PX4 only includes the HIL output driver (`pwm_out_sim`) when the firmware is built
   with `CONFIG_MODULES_SIMULATION_PWM_OUT_SIM=y`, and the standard release firmware for most boards (fmu-v6x
   included) leaves it out. If the board lacks it, the checklist offers **Build firmware** (runs
   `scripts/build_hitl_firmware.sh` for the detected board) and then **Flash firmware**. Building needs the ARM
   toolchain once: `brew tap osx-cross/arm && brew install arm-gcc-bin@13 && brew link --overwrite --force arm-gcc-bin@13`.
   Parameters survive the flash.
4. **HITL enabled on the board** – **Enable HITL** sets `SYS_HITL = 1`, saves, reboots; the link reconnects itself.
5. **Board is in HIL mode and streaming** – the heartbeat carries the HIL flag and actuator outputs arrive.
6. **Airframe geometry pushed** – **Push geometry** writes the CA_ROTOR parameters and the HIL_ACT_FUNC mapping.

From the command line: `--mode auto` picks the Pixhawk if one is plugged in, otherwise SITL;
`--mode hitl --serial /dev/cu.usbmodem01` forces a port. `scripts/start.command` is a double-clickable launcher.

QGroundControl keeps working during HITL: the simulator forwards the vehicle's MAVLink to UDP 14550. Disable QGC's
**serial** auto-connect (Application Settings → General → AutoConnect) or QGC grabs the USB port before the simulator.

HITL runs in real time (no lockstep). Keep `--rate` at 250 Hz or below on USB.

## Editing an airframe

* Click a rotor in the 3D view or its row in the table. `W` = move gizmo, `E` = rotate the thrust axis, `Esc` = deselect.
* The table edits the same things numerically: position (FRD metres), tilt from vertical and tilt direction
  (0° = forward, 90° = right), spin direction (click to flip), max thrust.
* `mirror L/R`, `mirror F/B`, `+ add rotor`, `motor → all` speed up building symmetric layouts.
* Mass, inertia (`estimate` computes it from the layout), drag, body size and leg geometry live in the same tab.
* Save/load airframes as JSON in `airframes/`.

### Conventions (identical to PX4)

* Body frame FRD: X forward, Y right, Z down. A rotor pointing "up" has axis `(0, 0, -1)`.
* Rotor *i* in the UI is PX4 **Motor i+1** and is exported as `CA_ROTOR{i}_*` with output function `10{i+1}`.
* `KM` > 0 means the rotor spins **CCW seen from above**, `KM` < 0 CW. Torque on the body is `-KM * thrust * axis`,
  exactly the expression in PX4's `ActuatorEffectivenessRotors`, so physics and allocator can never disagree about
  yaw direction.

## PX4 Export tab

* **Push geometry to PX4** writes `CA_AIRFRAME`, `CA_ROTOR_COUNT`, all `CA_ROTORn_*`, and the output-function mapping
  (`PWM_MAIN_FUNCn` for SITL, `HIL_ACT_FUNCn` for HITL), verifies each echoed `PARAM_VALUE`, then issues
  `MAV_CMD_PREFLIGHT_STORAGE` so the values survive a reboot. The table shows export vs. on-vehicle values.
* **download .params** produces a QGroundControl parameter file you can load through QGC instead.

PX4 rebuilds the effectiveness matrix on parameter change, no reboot needed for `CA_*`.

## Parameters tab

Full parameter list from the vehicle with descriptions, units, ranges, enums and bitmasks from PX4's metadata.
Search, filter by group, click a row to edit, `save to flash`, `reboot PX4`.

## Flight / Sim tab

Arm, disarm, kill, takeoff, land, hold, position, RTL; sim speed, sensor noise on/off, wind, home location;
per-motor override sliders that bypass PX4 (useful for checking a geometry's static thrust/torque);
live per-rotor command, speed and thrust.

## Notes and limits

* The physics is a clean rigid body with per-rotor thrust/torque, quadratic drag and spring-damper legs. There is no
  rotor aerodynamics beyond that (no inflow, no blade flapping, no ground effect). That is intentional: it is the model
  PX4's allocator assumes, so it isolates the geometry question.
* PX4 SITL is built with the lockstep scheduler; the simulator waits for each `HIL_ACTUATOR_CONTROLS` before stepping.
  If PX4 stalls you will see the `waits` counter in the status bar grow.
* The magnetic field is a dipole approximation (inclination from latitude, 0.5 gauss). PX4's EKF handles declination
  from its own tables.
* Airframes with more than 12 rotors cannot be exported (PX4 limit).
