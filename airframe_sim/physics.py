"""6-DOF rigid-body multirotor dynamics.

World frame: NED (North, East, Down), origin at the home point on the ground.
Body frame:  FRD (Forward, Right, Down).
Attitude:    unit quaternion q = [w, x, y, z] rotating body vectors into NED.

Per rotor: first-order spin-up of the normalized speed toward the commanded value,
thrust = max_thrust * omega_norm ** exponent along the rotor axis, applied at the rotor
position, plus PX4-consistent reaction torque (-km * thrust * axis).

Ground contact: spring-damper at each leg point with friction, so odd geometries tip
over realistically instead of being glued to the ground.
"""
from __future__ import annotations

import numpy as np

from .airframe import Airframe

G = 9.80665


# ------------------------------------------------------------ quaternion utils
def q_normalize(q: np.ndarray) -> np.ndarray:
    return q / np.linalg.norm(q)


def q_to_rotmat(q: np.ndarray) -> np.ndarray:
    """Rotation matrix R such that v_ned = R @ v_body."""
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def q_from_euler(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = np.cos(roll / 2), np.sin(roll / 2)
    cp, sp = np.cos(pitch / 2), np.sin(pitch / 2)
    cy, sy = np.cos(yaw / 2), np.sin(yaw / 2)
    return np.array([
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ])


def q_to_euler(q: np.ndarray) -> tuple[float, float, float]:
    w, x, y, z = q
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    sinp = np.clip(2 * (w * y - z * x), -1.0, 1.0)
    pitch = np.arcsin(sinp)
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return float(roll), float(pitch), float(yaw)


def q_deriv(q: np.ndarray, omega_body: np.ndarray) -> np.ndarray:
    w, x, y, z = q
    p, qq, r = omega_body
    return 0.5 * np.array([
        -x * p - y * qq - z * r,
        w * p + y * r - z * qq,
        w * qq - x * r + z * p,
        w * r + x * qq - y * p,
    ])


# ------------------------------------------------------------------ dynamics
class RigidBodySim:
    def __init__(self, airframe: Airframe):
        self.set_airframe(airframe)
        self.ground_k = 3000.0       # N/m per leg
        self.ground_c = 150.0        # N/(m/s) per leg
        self.ground_mu = 0.8         # friction coefficient
        self.wind_ned = np.zeros(3)  # m/s
        self.reset()

    # -- configuration
    def set_airframe(self, airframe: Airframe) -> None:
        self.af = airframe
        n = len(airframe.rotors)
        self.mass = float(airframe.mass)
        self.I = np.diag(airframe.inertia).astype(float)
        self.I_inv = np.linalg.inv(self.I)
        self.rotor_pos = np.array([r.pos for r in airframe.rotors], dtype=float).reshape(n, 3)
        axes = np.array([r.axis for r in airframe.rotors], dtype=float).reshape(n, 3)
        norms = np.linalg.norm(axes, axis=1, keepdims=True)
        norms[norms < 1e-9] = 1.0
        self.rotor_axis = axes / norms
        self.rotor_km = np.array([r.km for r in airframe.rotors], dtype=float)
        self.rotor_tmax = np.array([r.max_thrust for r in airframe.rotors], dtype=float)
        self.rotor_tau = np.array([max(r.tau, 1e-3) for r in airframe.rotors], dtype=float)
        self.rotor_exp = np.array([r.thrust_exponent for r in airframe.rotors], dtype=float)
        self.drag_q = np.array(airframe.drag_quadratic, dtype=float)
        self.drag_ang = np.array(airframe.drag_angular, dtype=float)
        self.legs = np.array(airframe.leg_points, dtype=float).reshape(-1, 3)
        # keep state arrays consistent with rotor count
        if not hasattr(self, "omega") or len(self.omega) != n:
            self.omega = np.zeros(n)
            self.cmd = np.zeros(n)
            self.thrust = np.zeros(n)

    def reset(self, yaw: float = 0.0) -> None:
        self.t = 0.0
        self.pos = np.zeros(3)
        self.vel = np.zeros(3)
        self.q = q_from_euler(0.0, 0.0, yaw)
        self.rates = np.zeros(3)
        self.omega = np.zeros(len(self.af.rotors))
        self.cmd = np.zeros(len(self.af.rotors))
        self.thrust = np.zeros(len(self.af.rotors))
        self.accel_body = np.array([0.0, 0.0, -G])  # specific force, what an accelerometer reads
        self.on_ground = True
        # rest the legs on the ground: lowest leg point at z = 0
        if len(self.legs):
            self.pos[2] = -float(np.max(self.legs[:, 2]))

    def set_motor_commands(self, cmd) -> None:
        c = np.asarray(cmd, dtype=float)[: len(self.cmd)]
        self.cmd[: len(c)] = np.clip(c, 0.0, 1.0)

    # -- forces
    def _forces_moments(self, pos, vel, q, rates, omega):
        R = q_to_rotmat(q)
        thrust = self.rotor_tmax * np.power(np.clip(omega, 0.0, 1.0), self.rotor_exp)
        thrust_vec = thrust[:, None] * self.rotor_axis                  # body, N
        F_body = thrust_vec.sum(axis=0)
        M_body = (np.cross(self.rotor_pos, thrust_vec) - (self.rotor_km * thrust)[:, None] * self.rotor_axis).sum(axis=0)

        # aerodynamic drag (body frame, relative to air)
        v_air_body = R.T @ (vel - self.wind_ned)
        F_body += -self.drag_q * v_air_body * np.abs(v_air_body)
        M_body += -self.drag_ang * rates * np.abs(rates)

        F_ned = R @ F_body + np.array([0.0, 0.0, self.mass * G])

        # ground contact at leg points
        on_ground = False
        for leg in self.legs:
            p_ned = pos + R @ leg
            pen = p_ned[2]  # > 0 means below ground (down positive)
            if pen > 0.0:
                on_ground = True
                v_pt = vel + R @ np.cross(rates, leg)
                fn = self.ground_k * pen + self.ground_c * max(v_pt[2], 0.0)
                fn = max(fn, 0.0)
                f_ned = np.array([0.0, 0.0, -fn])
                # friction: oppose horizontal velocity of the contact point
                vh = v_pt[:2]
                sp = np.linalg.norm(vh)
                if sp > 1e-4:
                    f_fric = -self.ground_mu * fn * vh / max(sp, 0.2)  # smooth near zero speed
                    f_ned[:2] += f_fric
                F_ned += f_ned
                M_body += np.cross(leg, R.T @ f_ned)
        return F_ned, M_body, thrust, on_ground, R

    def _deriv(self, pos, vel, q, rates, omega):
        F_ned, M_body, thrust, on_ground, R = self._forces_moments(pos, vel, q, rates, omega)
        acc = F_ned / self.mass
        ang_acc = self.I_inv @ (M_body - np.cross(rates, self.I @ rates))
        d_omega = (self.cmd - omega) / self.rotor_tau
        return vel, acc, q_deriv(q, rates), ang_acc, d_omega, F_ned, thrust, on_ground, R

    # -- integrate (semi-implicit Euler with small dt is plenty for this, RK4 for rotation)
    def step(self, dt: float) -> None:
        vel, acc, dq, ang_acc, d_omega, F_ned, thrust, on_ground, R = self._deriv(
            self.pos, self.vel, self.q, self.rates, self.omega)

        self.omega = np.clip(self.omega + d_omega * dt, 0.0, 1.0)
        self.vel = self.vel + acc * dt
        self.pos = self.pos + self.vel * dt
        self.rates = self.rates + ang_acc * dt
        self.q = q_normalize(self.q + q_deriv(self.q, self.rates) * dt)

        # ground damping of residual jitter when resting
        if on_ground and np.linalg.norm(self.vel) < 0.05 and np.linalg.norm(self.rates) < 0.05 \
                and thrust.sum() < 0.5 * self.mass * G:
            self.vel *= 0.5
            self.rates *= 0.5

        # accelerometer reads specific force: (a - g) in body frame
        gravity_ned = np.array([0.0, 0.0, G])
        self.accel_body = R.T @ (acc - gravity_ned)
        self.thrust = thrust
        self.on_ground = on_ground
        self.t += dt

    # -- convenience
    @property
    def euler(self) -> tuple[float, float, float]:
        return q_to_euler(self.q)

    @property
    def rotmat(self) -> np.ndarray:
        return q_to_rotmat(self.q)
