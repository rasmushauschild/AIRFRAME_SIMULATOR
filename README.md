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
   toolchain once: `brew tap osx-cross/arm; brew trust osx-cross/arm && brew install osx-cross/arm/arm-gcc-bin@13 && brew link --overwrite --force arm-gcc-bin@13`.
   Parameters survive the flash.
4. **HITL enabled on the board** – **Enable HITL** sets `SYS_HITL = 1`, saves, reboots; the link reconnects itself.
5. **Board is in HIL mode and streaming** – the heartbeat carries the HIL flag and actuator outputs arrive.
6. **Airframe geometry pushed** – **Push geometry** writes the CA_ROTOR parameters and the HIL_ACT_FUNC mapping.

From the command line: `--mode auto` picks the Pixhawk if one is plugged in, otherwise SITL;
`--mode hitl --serial /dev/cu.usbmodem01` forces a port. `scripts/start.command` is a double-clickable launcher.

QGroundControl keeps working during HITL: the simulator forwards the vehicle's MAVLink to UDP 14550. Disable QGC's
**serial** auto-connect (Application Settings → General → AutoConnect) or QGC grabs the USB port before the simulator.

HITL runs in real time (no lockstep). Keep `--rate` at 250 Hz or below on USB.

### HITL and SD logging

The HITL export sets `SDLOG_MODE = -1` (logging off). Starting the SD-card logger at arming starves the USB MAVLink
link on the board: the HIL sensor stream gaps, EKF2 loses its attitude and PX4 terminates the flight a second after
"Armed". Verified on an FMU v6X, PX4 v1.17: identical takeoff, logging on → termination, logging off → hover.
Record HITL flights from the simulator side instead. When the board's shell stops answering (a saturated USB link
does that; the checklist then shows the estimator restart failing), Reset reboots the board, which is the reliable
way to get a freshly aligned estimator.

## Editing an airframe

* Click a rotor in the 3D view or its row in the table. `W` = move gizmo, `E` = rotate the thrust axis, `Esc` = deselect.
* The table edits the same things numerically: position (FRD metres), tilt (forward lean of the thrust axis from
  vertical, negative = backward), cant (sideways lean, positive = outward from the centreline), spin direction
  (click to flip), max thrust. The Optimize tab uses the same two angles.
* `mirror L/R`, `mirror F/B`, `+ add rotor`, `motor → all` speed up building symmetric layouts.
* Mass, inertia (`estimate` computes it from the layout), drag, body size and leg geometry live in the same tab.
* Save/load airframes as JSON in `airframes/`.

### Conventions (identical to PX4)

* Body frame FRD: X forward, Y right, Z down. A rotor pointing "up" has axis `(0, 0, -1)`.
* Rotor *i* in the UI is PX4 **Motor i+1** and is exported as `CA_ROTOR{i}_*` with output function `10{i+1}`.
* `KM` > 0 means the rotor spins **CCW seen from above**, `KM` < 0 CW. Torque on the body is `-KM * thrust * axis`,
  exactly the expression in PX4's `ActuatorEffectivenessRotors`, so physics and allocator can never disagree about
  yaw direction.

## PX4 Parameters tab

One tab holds everything that ends up on the flight controller:

* **Geometry**: rotors, motors, mass, hover pitch, legs. Exported as the `CA_ROTORn_*` set, the output mapping and
  `SENS_BOARD_Y_OFF`; the derived values are listed under *Parameters derived from the geometry*.
* **Edited parameters**: any parameter you change in the list below is written to the vehicle immediately and
  remembered with the airframe (`px4_overrides` in the JSON). **Save** stores them with the preset, **Load** brings
  them back, and **Update PX4** re-applies them together with the geometry. The ✕ forgets an edit (the vehicle keeps
  its current value until you change it again).
* **All parameters**: the full list from the vehicle with descriptions, units, ranges, enums and bitmasks. Edited
  ones are marked with a dot.

## PX4 Export (part of the PX4 Parameters tab)

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

## Rotor types: propellers and ducted fans

Each rotor has a `kind`. The **Motors** card on the Geometry tab switches all rotors and fills in sensible defaults:

| | propeller | ducted fan |
|---|---|---|
| reaction torque per thrust (`km`, exported as `CA_ROTORn_KM`) | 0.05 | 0.01 (stator cancels the swirl) |
| spool-up time constant | 0.04 s | 0.12 s |
| fan / prop diameter | 0.25 m | 0.12 m |
| ram (momentum) drag | off | on |

Ram drag models the duct swallowing a mass flow `mdot = sqrt(rho A T)` that arrives with the vehicle's airspeed and
loses that momentum: `-mdot * v_air` at the duct location. In crossflow it is a side force that also pitches/rolls a
layout with ducts above or below the CG; in axial flow it is the thrust loss every jet has with forward speed. PX4 does
not know about this; it just sees the resulting motion through the simulated sensors.

### Jetfoils (horizontal fans, deflected jets)

A ducted fan can be mounted horizontally with a jetfoil that bends the jet to the thrust direction. Set the rotor's
**Duct** column to `foil`: the fan axis becomes the body X axis (`duct_axis`), the rotor's tilt/cant still define where
the thrust points, and the readout shows the bend angle. Bending costs thrust: **Jetfoil loss %** on the Motors card is
the loss at 90°, scaled linearly with the bend (10% default). The effective maximum thrust is used by the physics, the
hover check and the optimiser; the PX4 export is unchanged (PX4 only needs the thrust direction).

### Wing

The **Wing** card adds a lifting surface (a delta by default) that acts at its position: Polhamus lift (potential +
vortex lift from the aspect ratio `span² / area`), drag `CD0 + CL·tan α`, and a flat-plate blend past the stall angle.
Incidence is the chord angle above the structural X axis. It does nothing in hover and carries part of the weight in
cruise, which is what the Optimize tab trades against the jet angles. A 60° delta has aspect ratio 2.31, so
`span = sqrt(2.31 · area)`.

Low `km` also means little yaw authority from torque differences. PX4's allocator then needs large thrust differences
to yaw, which the hover check on the Geometry tab reflects.

## Hover pitch (aircraft that hover nose-up)

If the ducts/rotors are tilted forward for cruise, the aircraft has to hover pitched nose-up so the thrust axes point
at the sky. PX4's multirotor mode holds its body frame level, so the trick is to tell PX4 that *its* level is your
airframe at that pitch. Set **Hover pitch°** on the Geometry tab:

* rotor positions and axes are exported rotated into that frame (`CA_ROTORn_*`);
* `SENS_BOARD_Y_OFF` is exported with the same angle, so the IMU mounted in the structural frame is read in the hover
  frame (on the real vehicle this is the same parameter you would set; keep `SENS_BOARD_ROT` for the board's mounting);
* the simulator rests and reports the vehicle in the structural frame, so you see it sit and hover nose-up while PX4
  reports pitch ≈ 0.

Verified in SITL: all ten ducts at 45°, hover pitch 45° → takes off, rotates to 46° nose-up and holds altitude with
PX4 reporting +1° pitch. The hover check runs in the hover frame; it will tell you when the axes are not vertical in
hover (residual force → PX4 leans) or when yaw cannot be cancelled (all rotors spinning the same way with parallel
axes: alternate spins or cant rotors in opposing pairs).

## Optimize tab: jet angles for hover and cruise

The tab evaluates the current geometry in two conditions and searches the angles that serve both:

* **Hover** at the hover pitch: PX4's pseudo-inverse allocation, the busiest motor's share of its (effective) maximum
  thrust, the thrust wasted by motors fighting each other, and the roll/pitch/yaw torque available before a motor
  saturates.
* **Cruise** at the target speed (Cruise → Speed, saved with the airframe): a trim solve with PX4's allocation in the
  loop finds the body pitch, collective and pitch torque for zero net force and moment, with wing lift, ram drag,
  body drag and gravity. It reports the PX4 pitch (structural pitch minus hover pitch; must stay inside
  `MPC_TILTMAX_AIR`), power relative to hover (momentum theory, per duct), the wing's share of the weight and its
  angle of attack. It also flags `MPC_XY_VEL_MAX` / `MPC_XY_CRUISE` when they are below the cruise speed.

**Variables**: rotors with identical angles form a group (letters under the table reassign motors). Per group the tilt
(forward/back, from vertical) and the cant (symmetric left/right lean) can be optimised within a range, plus the hover
pitch. **Objective**: the slider weights hover margin (busiest motor + wasted thrust) against cruise power (relative
to hover power); designs with negative or saturated motors, no trim solution, a stalled wing or a PX4 pitch beyond
the tilt limit are penalised. The search samples the ranges, refines the best starts with Nelder–Mead, and lists the
best designs by score plus a spread of the Pareto front (● rows: no other design is better in both hover margin and
cruise power). **Apply** loads a row into the geometry; Update PX4 then exports it as usual.

Verified on ATLAS_04 (13 kg, 50 km/h, 0.5 m² delta at 10°): the optimiser moves the hover pitch to ≈47° with rear
jets at 50–55° and the front ducts at ≈32–41°, taking the busiest motor from 47% to 38% in hover and cruise power from
104% to 81% of hover, with the wing carrying ≈40% of the weight at 15° angle of attack.

## Flying with a USB remote (RadioMaster / EdgeTX)

**RadioMaster T8L** (RadioMaster's own firmware): power it on with **M + Power** (config/VCP mode), plug in USB-C,
click **Connect radio** on the Flight tab and pick "RadioMaster T8L" in the browser's serial list (Web Serial, Chrome
or Edge). The page speaks the same protocol as RadioMaster's web configurator: it polls the radio at 460800 baud and
reads the ten output channels 50 times a second. Once granted, the port reconnects without the picker.

**EdgeTX radios** (TX16S, Boxer, Pocket, …): choose *USB Joystick (HID)* on the radio and click **HID joystick**; the
page reads the radio's own HID report layout. Safari has neither picker; there the gamepad fallback finds a joystick
when you move a stick.
Map roll, pitch, throttle and yaw with *Learn* (press, then move that stick), tick **Send to PX4**, and the page streams
MAVLink `MANUAL_CONTROL` at 50 Hz, exactly what QGroundControl sends for a joystick. SITL only: in HITL the real receiver on the board is the manual input, and the card is hidden.

PX4 picks the input source with `COM_RC_IN_MODE`: 3 (default) uses whichever of RC receiver or joystick appears first,
1 is joystick only. The card's *Input priority* writes it and it is saved with the airframe. Stick arming (throttle low,
yaw right) works in Stabilized/Position as on a normal radio; use the Arm button otherwise.

## Notes and limits

* The physics is a clean rigid body with per-rotor thrust/torque, quadratic drag and spring-damper legs. There is no
  rotor aerodynamics beyond that (no inflow, no blade flapping, no ground effect). That is intentional: it is the model
  PX4's allocator assumes, so it isolates the geometry question.
* PX4 SITL is built with the lockstep scheduler; the simulator waits for each `HIL_ACTUATOR_CONTROLS` before stepping.
  If PX4 stalls you will see the `waits` counter in the status bar grow.
* The magnetic field is a dipole approximation (inclination from latitude, 0.5 gauss). PX4's EKF handles declination
  from its own tables.
* Airframes with more than 12 rotors cannot be exported (PX4 limit).
