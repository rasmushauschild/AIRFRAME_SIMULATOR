"""Simulation loop: physics + sensors -> PX4 over the HIL link, actuators back.

SITL (lockstep): each HIL_SENSOR we send advances PX4's clock; we wait for the
HIL_ACTUATOR_CONTROLS reply before stepping again, then pace to wall clock.
HITL (real time): the Pixhawk runs on its own clock; we stream sensors at the
configured rate and use whatever the latest actuator values are.
"""
from __future__ import annotations

import threading
import time
from typing import Callable

import numpy as np

from .airframe import Airframe
from .link import PX4Link
from .physics import RigidBodySim
from .sensors import SensorSuite, Home, SensorNoise


class Simulator:
    def __init__(self, airframe: Airframe, link: PX4Link | None = None, sensor_rate: float = 250.0, physics_substeps: int = 4,
                 gps_rate: float = 10.0, speed: float = 1.0, lockstep: bool | None = None,
                 home: Home | None = None, log: Callable[[str], None] | None = None):
        self.link = link
        self.log = log or (lambda s: print(s, flush=True))
        self.lock = threading.RLock()
        self.airframe = airframe
        self.sim = RigidBodySim(airframe)
        self.sensors = SensorSuite(home=home)
        self.sensor_rate = sensor_rate
        self.substeps = physics_substeps
        self.gps_every = max(1, int(round(sensor_rate / gps_rate)))
        self.state_every = max(1, int(round(sensor_rate / 50.0)))
        self.speed = speed                     # 1.0 = real time, 0 = as fast as PX4 allows (SITL only)
        self.lockstep = (link is not None and link.mode == "sitl") if lockstep is None else bool(lockstep)
        self.time_usec = 0
        self.paused = False
        self.running = False
        self.step_count = 0
        self.lockstep_timeouts = 0
        self.real_time_factor = 0.0
        self.motor_override: list[float] | None = None   # manual motor test from the UI
        self.link_seq = 0
        self._last_send_err = 0.0
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()

    # ------------------------------------------------------------ control
    def start(self) -> None:
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="sim-loop", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()

    def reset(self, yaw: float = 0.0) -> None:
        with self.lock:
            self.sim.reset(yaw=yaw)
            self.motor_override = None
            if self.link is not None:
                self.link.clear_actuators()

    def set_airframe(self, airframe: Airframe, keep_state: bool = True) -> None:
        with self.lock:
            self.airframe = airframe
            self.sim.set_airframe(airframe)
            if not keep_state:
                self.sim.reset()

    def set_link(self, link, lockstep: bool) -> None:
        """Swap the PX4 link at runtime (SITL <-> HITL). None pauses the sensor stream."""
        with self.lock:
            self.link = link
            self.lockstep = lockstep
            self.link_seq += 1
            self.lockstep_timeouts = 0
            if link is not None:
                self.sim.reset()

    def set_wind(self, north: float, east: float, down: float = 0.0) -> None:
        with self.lock:
            self.sim.wind_ned = np.array([north, east, down], dtype=float)

    # --------------------------------------------------------------- loop
    def _run(self) -> None:
        self.running = True
        dt = 1.0 / self.sensor_rate
        sub_dt = dt / self.substeps
        wall_start = time.perf_counter()
        sim_start = self.time_usec
        last_hb = 0.0
        rtf_t0, rtf_sim0 = time.perf_counter(), self.time_usec
        self.log(f"[sim] loop started: {self.sensor_rate:.0f} Hz sensors, physics {self.sensor_rate * self.substeps:.0f} Hz")
        while not self._stop.is_set():
            now = time.perf_counter()
            link = self.link
            if link is not None and now - last_hb > 1.0:
                try:
                    link.send_heartbeat()
                except Exception:
                    pass
                last_hb = now

            # SITL: nothing to do until PX4 has connected and sent its first heartbeat.
            if link is None or self.paused or (link.mode == "sitl" and not link.connected):
                time.sleep(0.02)
                wall_start = time.perf_counter()
                sim_start = self.time_usec
                continue

            # 1. actuators -> physics
            with self.lock:
                cmd = self.motor_override if self.motor_override is not None else link.actuators
                self.sim.set_motor_commands(cmd)
                for _ in range(self.substeps):
                    self.sim.step(sub_dt)
                self.time_usec += int(round(dt * 1e6))
                t_us = self.time_usec
                sensor = self.sensors.hil_sensor(self.sim, t_us)
                gps = self.sensors.hil_gps(self.sim, t_us) if self.step_count % self.gps_every == 0 else None
                # HIL_STATE_QUATERNION is ground truth for SITL logging only. On real hardware PX4's mavlink receiver
                # feeds its accel/gyro fields into the *same* simulated IMU as HIL_SENSOR (with a different unit
                # convention), which corrupts the estimator, so it is never sent in HITL.
                send_state = link.mode == "sitl" and self.step_count % self.state_every == 0
                state = self.sensors.hil_state_quaternion(self.sim, t_us) if send_state else None
            self.step_count += 1

            # 2. sensors -> PX4
            seq_before = link.actuator_seq
            try:
                if gps is not None:
                    link.send_hil_gps(gps)
                if state is not None:
                    link.send_hil_state_quaternion(state)
                link.send_hil_sensor(sensor)
            except Exception as e:
                if time.time() - self._last_send_err > 3.0:
                    self.log(f"[sim] send failed: {e}")
                    self._last_send_err = time.time()
                time.sleep(0.1)
                continue

            # 3. lockstep: wait for PX4 to consume it
            if self.lockstep:
                if not link.wait_for_actuators(seq_before, timeout=0.1):
                    self.lockstep_timeouts += 1

            # 4. pacing
            if self.speed > 0:
                target = wall_start + (self.time_usec - sim_start) / 1e6 / self.speed
                remaining = target - time.perf_counter()
                if remaining > 0:
                    time.sleep(remaining)
                elif remaining < -0.5:
                    wall_start = time.perf_counter()   # fell far behind, resync instead of racing
                    sim_start = self.time_usec

            if time.perf_counter() - rtf_t0 > 1.0:
                self.real_time_factor = ((self.time_usec - rtf_sim0) / 1e6) / (time.perf_counter() - rtf_t0)
                rtf_t0, rtf_sim0 = time.perf_counter(), self.time_usec
        self.running = False
        self.log("[sim] loop stopped")

    # --------------------------------------------------------- UI snapshot
    def snapshot(self) -> dict:
        with self.lock:
            s = self.sim
            r, p, y = s.euler
            return {
                "t": self.time_usec / 1e6,
                "pos": s.pos.tolist(),
                "vel": s.vel.tolist(),
                "q": s.q.tolist(),
                "euler": [r, p, y],
                "rates": s.rates.tolist(),
                "accel_body": s.accel_body.tolist(),
                "on_ground": bool(s.on_ground),
                "rotors": [{"cmd": float(c), "omega": float(o), "thrust": float(t)}
                           for c, o, t in zip(s.cmd, s.omega, s.thrust)],
                "rtf": self.real_time_factor,
                "paused": self.paused,
                "lockstep_timeouts": self.lockstep_timeouts,
                "motor_override": self.motor_override,
                "wind": s.wind_ned.tolist(),
            }
