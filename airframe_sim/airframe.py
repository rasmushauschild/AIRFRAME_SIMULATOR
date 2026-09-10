"""Airframe description: rotor geometry, mass properties, and PX4 control-allocation export.

Frames
------
Everything here is in the PX4 body frame, FRD: X forward, Y right, Z down (metres).
A rotor whose thrust points "up" has axis (0, 0, -1).

Rotor spin direction and torque
-------------------------------
``km`` is the moment coefficient as PX4 defines it (CA_ROTORn_KM):
    reaction torque on the body = -km * thrust * axis
Positive km  ->  rotor spins counter-clockwise when viewed from above.
Negative km  ->  rotor spins clockwise when viewed from above.
This is exactly what PX4's ActuatorEffectivenessRotors assumes, so the physics
and the flight controller agree by construction.
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass, field, asdict
from pathlib import Path
from typing import Any

MAX_PX4_ROTORS = 12  # CA_ROTOR0 .. CA_ROTOR11


def _unit(v: list[float]) -> list[float]:
    n = math.sqrt(sum(c * c for c in v))
    if n < 1e-9:
        return [0.0, 0.0, -1.0]
    return [c / n for c in v]


ROTOR_KINDS = {
    # defaults applied when a rotor is switched to this kind (km keeps its sign = spin direction)
    "prop": {"km": 0.05, "tau": 0.04, "prop_diameter": 0.25, "thrust_exponent": 2.0, "ram_drag": False},
    # Electric ducted fan: stator vanes cancel most of the swirl (tiny reaction torque), small heavy rotor at
    # high rpm spools slower, and the duct swallows a mass flow that produces momentum ("ram") drag in crossflow.
    "ducted": {"km": 0.01, "tau": 0.12, "prop_diameter": 0.12, "thrust_exponent": 2.0, "ram_drag": True},
}


@dataclass
class Rotor:
    pos: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])      # m, FRD
    axis: list[float] = field(default_factory=lambda: [0.0, 0.0, -1.0])    # unit thrust direction, FRD
    km: float = 0.05            # moment coefficient, signed (see module docstring)
    max_thrust: float = 8.0     # N at full command
    tau: float = 0.04           # motor/prop spin-up time constant, s
    prop_diameter: float = 0.25  # m, visual + disc area (fan diameter for a duct)
    thrust_exponent: float = 2.0  # thrust = max_thrust * omega_norm ** exponent
    kind: str = "prop"          # "prop" | "ducted"
    ram_drag: bool = False      # momentum drag of the inlet mass flow in crossflow (ducted fans)

    def set_kind(self, kind: str) -> "Rotor":
        d = ROTOR_KINDS.get(kind, ROTOR_KINDS["prop"])
        self.kind = kind
        self.km = (1.0 if self.km >= 0 else -1.0) * d["km"]
        self.tau, self.prop_diameter, self.thrust_exponent, self.ram_drag = d["tau"], d["prop_diameter"], d["thrust_exponent"], d["ram_drag"]
        return self

    def normalized(self) -> "Rotor":
        self.axis = _unit(self.axis)
        return self

    @property
    def ccw(self) -> bool:
        return self.km >= 0.0


@dataclass
class Airframe:
    name: str = "Quad X"
    mass: float = 1.5                                            # kg
    inertia: list[float] = field(default_factory=lambda: [0.02, 0.02, 0.035])  # kg m^2, diagonal FRD
    drag_quadratic: list[float] = field(default_factory=lambda: [0.10, 0.10, 0.20])  # N/(m/s)^2 per body axis
    drag_angular: list[float] = field(default_factory=lambda: [0.005, 0.005, 0.005])  # Nm/(rad/s)^2
    body_size: list[float] = field(default_factory=lambda: [0.16, 0.16, 0.06])  # visual box, m
    leg_points: list[list[float]] = field(default_factory=lambda: [
        [0.10, 0.10, 0.12], [0.10, -0.10, 0.12], [-0.10, 0.10, 0.12], [-0.10, -0.10, 0.12],
    ])  # ground contact points, FRD
    rotors: list[Rotor] = field(default_factory=list)
    hover_pitch_deg: float = 0.0   # nose-up pitch of the structural frame in hover; PX4's "level" is this attitude
    px4_overrides: dict = field(default_factory=dict)   # PX4 parameters edited by hand, saved with the airframe
    landed_pitch_deg: float = 0.0  # nose-up pitch of the structural frame when standing on its feet
    leg_height: float = 0.2        # feet: distance below the CG (measured along the landed "down")
    leg_spread: float = 0.2        # feet: half-width / half-length of the foot rectangle

    def generate_legs(self) -> list[list[float]]:
        """Four feet on a plane perpendicular to gravity when the airframe stands at landed_pitch_deg,
        expressed in the structural frame."""
        phi = math.radians(self.landed_pitch_deg)
        c, s_ = math.cos(phi), math.sin(phi)
        h, sp = self.leg_height, self.leg_spread
        pts = []
        for x, y in ((sp, sp), (sp, -sp), (-sp, sp), (-sp, -sp)):
            # inverse of the pitch rotation: landed-frame (x, y, h) -> structural
            pts.append([round(c * x - s_ * h, 4), round(y, 4), round(s_ * x + c * h, 4)])
        return pts

    # ------------------------------------------------------- hover frame
    def hover_rotation(self):
        """Rotation matrix taking structural-frame vectors into PX4's body frame (the aircraft pitched nose-up
        by hover_pitch_deg is 'level' for PX4). x_structural -> (cos, 0, -sin): the nose points up."""
        import numpy as np
        phi = math.radians(self.hover_pitch_deg)
        c, s_ = math.cos(phi), math.sin(phi)
        return np.array([[c, 0.0, s_], [0.0, 1.0, 0.0], [-s_, 0.0, c]])

    def rotors_in_px4_frame(self) -> list[tuple[list[float], list[float]]]:
        import numpy as np
        R = self.hover_rotation()
        out = []
        for r in self.rotors:
            out.append(((R @ np.array(r.pos, float)).tolist(), (R @ np.array(_unit(r.axis), float)).tolist()))
        return out

    # ------------------------------------------------------------------ io
    def to_dict(self) -> dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_dict(cls, d: dict[str, Any]) -> "Airframe":
        d = dict(d)
        rotors = [Rotor(**r).normalized() for r in d.pop("rotors", [])]
        af = cls(**{k: v for k, v in d.items() if k in cls.__dataclass_fields__})
        af.rotors = rotors
        return af

    @classmethod
    def load(cls, path: str | Path) -> "Airframe":
        with open(path) as f:
            return cls.from_dict(json.load(f))

    def save(self, path: str | Path) -> None:
        with open(path, "w") as f:
            json.dump(self.to_dict(), f, indent=2)

    # --------------------------------------------------------- validation
    def validate(self) -> list[str]:
        problems = []
        if not (1 <= len(self.rotors) <= MAX_PX4_ROTORS):
            problems.append(f"PX4 supports 1..{MAX_PX4_ROTORS} rotors, airframe has {len(self.rotors)}")
        if self.mass <= 0:
            problems.append("mass must be positive")
        if any(i <= 0 for i in self.inertia):
            problems.append("inertia diagonal must be positive")
        total = sum(r.max_thrust for r in self.rotors)
        if total < self.mass * 9.81 * 1.2:
            problems.append(f"thrust/weight is {total / (self.mass * 9.81):.2f}, hover will be marginal")
        return problems

    # -------------------------------------------------------- PX4 export
    def px4_params(self, hitl: bool = True) -> dict[str, float | int]:
        """Parameters that make PX4 fly this exact geometry.

        Rotor i of this airframe maps to PX4 "Motor i+1", which is output function 101+i.
        For HITL/SITL the outputs are the HIL actuator functions (HIL_ACT_FUNCn).
        """
        p: dict[str, float | int] = {}
        # hand-edited parameters first; the geometry-derived ones below always win
        for k, v in (self.px4_overrides or {}).items():
            p[k] = int(v) if isinstance(v, bool) else v
        p["CA_AIRFRAME"] = 0            # multirotor
        p["CA_ROTOR_COUNT"] = len(self.rotors)
        for i, (r, (pos, ax)) in enumerate(zip(self.rotors, self.rotors_in_px4_frame())):
            p[f"CA_ROTOR{i}_PX"] = round(pos[0], 4)
            p[f"CA_ROTOR{i}_PY"] = round(pos[1], 4)
            p[f"CA_ROTOR{i}_PZ"] = round(pos[2], 4)
            p[f"CA_ROTOR{i}_AX"] = round(ax[0], 4)
            p[f"CA_ROTOR{i}_AY"] = round(ax[1], 4)
            p[f"CA_ROTOR{i}_AZ"] = round(ax[2], 4)
            p[f"CA_ROTOR{i}_KM"] = round(r.km, 4)
        # The flight controller is mounted in the structural frame; tell PX4 its board is pitched nose-up by
        # hover_pitch so sensor data lands in the hover frame the CA_ROTOR geometry above is expressed in.
        p["SENS_BOARD_Y_OFF"] = round(float(self.hover_pitch_deg), 2)
        # Output function mapping: HIL_ACT_FUNCn (SITL simulator_mavlink and HITL both use pwm_out_sim)
        for n in range(1, 17):
            p[f"HIL_ACT_FUNC{n}"] = 101 + (n - 1) if n <= len(self.rotors) else 0
        if hitl:
            p["SYS_HITL"] = 1
        return p

    def px4_params_file(self, hitl: bool = True) -> str:
        """QGroundControl .params file text (loadable in QGC > Parameters > Tools > Load)."""
        lines = ["# Onboard parameters exported by AIRFRAME_SIMULATOR",
                 f"# Airframe: {self.name}",
                 "# Vehicle-Id Component-Id Name Value Type"]
        for k, v in self.px4_params(hitl).items():
            if isinstance(v, int):
                lines.append(f"1\t1\t{k}\t{v}\t6")   # 6 = MAV_PARAM_TYPE_INT32
            else:
                lines.append(f"1\t1\t{k}\t{v:.6f}\t9")  # 9 = MAV_PARAM_TYPE_REAL32
        return "\n".join(lines) + "\n"

    def geometry_param_names(self) -> set[str]:
        """Parameters px4_params() derives from the geometry (an override of these is ignored)."""
        return {k for k in self.px4_params(hitl=True) if k not in (self.px4_overrides or {})} | {"SYS_HITL"}

    # ------------------------------------------------- PX4 hover feasibility
    def hover_check(self) -> dict:
        """Reproduce PX4's control allocation for a pure-thrust hover and flag rotors it would drive negative.

        PX4 (CA_METHOD auto / pseudo-inverse) solves the minimum-norm least-squares allocation without a
        positivity constraint, then clips to [0, 1]. If the unconstrained solution wants negative thrust on
        some rotor (typical for layouts where all rotors of one spin direction sit on one side, or where the
        CG is far from the rotor cloud), the clipped result no longer produces the demanded thrust and the
        vehicle cannot lift off even though the geometry looks fine.
        """
        import numpy as np
        n = len(self.rotors)
        if n == 0:
            return {"ok": False, "problems": ["no rotors"], "shares": []}
        rows = []
        for r, (pos, axis) in zip(self.rotors, self.rotors_in_px4_frame()):   # allocation happens in PX4's (hover) frame
            p = np.array(pos, float)
            ax = np.array(axis, float)
            rows.append(np.concatenate([np.cross(p, ax) - r.km * ax, ax]))
        E = np.array(rows).T                              # 6 x n, unit thrust coefficient (CT cancels)
        sp = np.array([0, 0, 0, 0, 0, -1.0])
        u = np.linalg.pinv(E) @ sp
        resid = E @ u - sp                                # least-squares residual: what no motor mix can cancel
        umax = float(u.max()) if u.max() > 1e-9 else 1.0
        shares = (u / umax).tolist()
        neg = [i + 1 for i, v in enumerate(shares) if v < -1e-6]
        # thrust-weighted check: shares scaled so the vehicle hovers, against each rotor's max thrust
        weight = self.mass * 9.80665
        thrust_up = float(-(E[5] @ u))                     # unit-CT thrust per unit u
        scale = weight / thrust_up if thrust_up > 1e-9 else float("inf")
        hover_thrust = [float(v * scale) for v in u]
        util = [t / r.max_thrust if r.max_thrust > 0 else float("inf") for t, r in zip(hover_thrust, self.rotors)]
        over = [i + 1 for i, x in enumerate(util) if x > 0.85]
        problems = []
        force_resid = float(np.abs(resid[3:5]).max())
        yaw_resid = float(abs(resid[2]))
        rp_resid = float(np.abs(resid[:2]).max())
        unbalanced = force_resid > 1e-3 or rp_resid > 1e-3
        if unbalanced:
            problems.append(f"no motor mix gives zero net force and roll/pitch torque at {self.hover_pitch_deg:g}° nose-up "
                            f"(residual force {force_resid:.2f} per unit of lift): PX4 will have to lean away from its level "
                            f"attitude to hover. Set the hover pitch so all thrust axes are vertical in hover (usually equal "
                            f"to the rotor tilt), or tilt rotors in opposing pairs.")
        if yaw_resid > 2e-3:
            problems.append(f"yaw torque cannot be cancelled: every rotor spins the same way and the axes are parallel, so "
                            f"a residual yaw torque of {yaw_resid:.3f} per unit of lift remains and there is no yaw authority. "
                            f"Alternate spin directions, or cant rotors left/right in opposing pairs so thrust vectors can yaw.")
        if neg:
            hint = ("Rebalance spin directions (mix CW/CCW on each side) or move the CG relative to the rotors."
                    if all(abs(_unit(r.axis)[2]) > 0.98 for r in self.rotors) else
                    "With tilted rotors, set the hover pitch so the thrust axes are vertical in hover, and tilt all rotors "
                    "the same way; any rotor pointing a different way must be cancelled by another.")
            problems.append(f"PX4's allocator wants negative thrust on motor(s) {neg} to hover with zero torque "
                            f"(at {self.hover_pitch_deg:g}° nose-up); it will clip them to zero and the vehicle will not lift off. {hint}")
        if over:
            problems.append(f"motor(s) {over} above 85% of max thrust just to hover; no control margin")
        return {"ok": not neg and not over and not unbalanced and yaw_resid <= 2e-3, "problems": problems, "shares": shares,
                "hover_thrust": hover_thrust, "hover_utilisation": util, "negative": neg,
                "residual_force": resid[3:].tolist(), "residual_torque": resid[:3].tolist()}

    # ---------------------------------------------------------- helpers
    def estimate_inertia(self, motor_mass: float = 0.06, body_fraction: float = 0.6) -> list[float]:
        """Rough inertia estimate: a central body block plus point-mass motors at the rotor positions."""
        n = max(1, len(self.rotors))
        m_body = self.mass * body_fraction
        m_rot = (self.mass - m_body) / n
        bx, by, bz = self.body_size
        ixx = m_body * (by ** 2 + bz ** 2) / 12
        iyy = m_body * (bx ** 2 + bz ** 2) / 12
        izz = m_body * (bx ** 2 + by ** 2) / 12
        for r in self.rotors:
            x, y, z = r.pos
            ixx += m_rot * (y * y + z * z)
            iyy += m_rot * (x * x + z * z)
            izz += m_rot * (x * x + y * y)
        return [ixx, iyy, izz]


# ----------------------------------------------------------------- presets
def quad_x(arm: float = 0.25, max_thrust: float = 8.0) -> Airframe:
    """PX4 Quad X numbering: 1 front-right CCW, 2 rear-left CCW, 3 front-left CW, 4 rear-right CW."""
    a = arm / math.sqrt(2)
    af = Airframe(name="Quad X")
    af.rotors = [
        Rotor(pos=[a, a, 0.0], km=0.05, max_thrust=max_thrust),
        Rotor(pos=[-a, -a, 0.0], km=0.05, max_thrust=max_thrust),
        Rotor(pos=[a, -a, 0.0], km=-0.05, max_thrust=max_thrust),
        Rotor(pos=[-a, a, 0.0], km=-0.05, max_thrust=max_thrust),
    ]
    af.inertia = af.estimate_inertia()
    return af


def hex_x(arm: float = 0.30, max_thrust: float = 7.0) -> Airframe:
    """PX4 Hexa X numbering (motors at 30deg offsets), alternating CW/CCW."""
    af = Airframe(name="Hex X", mass=2.2)
    # PX4 hexa X order: 1 right-front? Use the documented geometry: angles measured from +X (forward), positive to the right
    layout = [(30, True), (210, True), (-30, False), (150, False), (90, False), (-90, True)]
    for ang, ccw in layout:
        rad = math.radians(ang)
        af.rotors.append(Rotor(pos=[arm * math.cos(rad), arm * math.sin(rad), 0.0],
                               km=0.05 if ccw else -0.05, max_thrust=max_thrust))
    af.leg_points = [[0.15, 0.15, 0.14], [0.15, -0.15, 0.14], [-0.15, 0.15, 0.14], [-0.15, -0.15, 0.14]]
    af.inertia = af.estimate_inertia()
    return af
