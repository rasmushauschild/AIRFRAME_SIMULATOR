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


@dataclass
class Rotor:
    pos: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])      # m, FRD
    axis: list[float] = field(default_factory=lambda: [0.0, 0.0, -1.0])    # unit thrust direction, FRD
    km: float = 0.05            # moment coefficient, signed (see module docstring)
    max_thrust: float = 8.0     # N at full command
    tau: float = 0.04           # motor/prop spin-up time constant, s
    prop_diameter: float = 0.25  # m, visual + disc area
    thrust_exponent: float = 2.0  # thrust = max_thrust * omega_norm ** exponent

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
        p: dict[str, float | int] = {
            "CA_AIRFRAME": 0,            # multirotor
            "CA_ROTOR_COUNT": len(self.rotors),
        }
        for i, r in enumerate(self.rotors):
            ax = _unit(r.axis)
            p[f"CA_ROTOR{i}_PX"] = round(r.pos[0], 4)
            p[f"CA_ROTOR{i}_PY"] = round(r.pos[1], 4)
            p[f"CA_ROTOR{i}_PZ"] = round(r.pos[2], 4)
            p[f"CA_ROTOR{i}_AX"] = round(ax[0], 4)
            p[f"CA_ROTOR{i}_AY"] = round(ax[1], 4)
            p[f"CA_ROTOR{i}_AZ"] = round(ax[2], 4)
            p[f"CA_ROTOR{i}_KM"] = round(r.km, 4)
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
