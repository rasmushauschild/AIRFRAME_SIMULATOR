"""Aerodynamics beyond the rotor discs, steady cruise trim, control margins, and the jet-angle optimiser.

Everything is in the structural body frame (FRD) unless stated otherwise.

Wing model
----------
A delta wing with Polhamus' leading-edge-suction analogy: CL = Kp sin(a) cos^2(a) + Kv cos(a) sin^2(a), where Kp is
the potential-flow lift slope (Helmbold, from the aspect ratio) and Kv the vortex-lift constant (about pi for sharp
leading edges). The resultant of a sharp-edged delta is normal to the chord, so the lift-dependent drag is CL tan(a)
on top of a parasite CD0. Past the stall angle the coefficients blend into a flat plate over 15 degrees.

Ducted fans with jetfoils
-------------------------
The fan itself may be horizontal (``duct_axis``) while a jetfoil turns the jet to the rotor's ``axis``. Turning the
jet costs a fraction ``turn_loss`` of the thrust at 90 degrees, scaled linearly with the deflection angle. The inlet
still swallows mdot = sqrt(rho A T) of air that arrives with the vehicle's airspeed; that momentum is lost, which is
the ram (momentum) drag: -mdot * v_air applied at the duct.

Cruise trim
-----------
PX4 flies the vehicle as a multirotor: the allocator turns a collective thrust along the hover-frame -z and a pitch
torque into per-motor thrusts (pseudo-inverse, clipped), and the attitude loop holds whatever pitch makes the forces
balance. The trim solve finds that pitch, the collective, and the pitch torque for which the net force and pitching
moment are zero at the target airspeed, with wing lift, ram drag, body drag and gravity included.
"""
from __future__ import annotations

import copy
import math
import time
from dataclasses import dataclass
from typing import Callable

import numpy as np

RHO = 1.225
G = 9.80665


# ------------------------------------------------------------------ wing
@dataclass
class WingAero:
    area: float
    span: float
    incidence: float   # rad, chord above the body x axis
    cd0: float
    stall: float       # rad
    kp: float
    kv: float

    @classmethod
    def of(cls, w) -> "WingAero":
        ar = (w.span * w.span / w.area) if w.area > 0 else 1.0
        kp = 2 * math.pi * ar / (2 + math.sqrt(4 + ar * ar))      # Helmbold lift slope, 1/rad
        return cls(w.area, w.span, math.radians(w.incidence_deg), w.cd0, math.radians(w.stall_deg), kp,
                   math.pi * float(w.vortex_lift))

    def coeffs(self, alpha: float) -> tuple[float, float]:
        s = 1.0 if alpha >= 0 else -1.0
        a = abs(alpha)

        def pre(a):
            cl = self.kp * math.sin(a) * math.cos(a) ** 2 + self.kv * math.cos(a) * math.sin(a) ** 2
            return cl, self.cd0 + cl * math.tan(a)

        if a <= self.stall:
            cl, cd = pre(a)
        else:
            cls_, cds = pre(self.stall)
            t = min(1.0, (a - self.stall) / math.radians(15))
            clf, cdf = 1.2 * math.sin(a) * math.cos(a), self.cd0 + 1.2 * math.sin(a) ** 2
            cl, cd = (1 - t) * cls_ + t * clf, (1 - t) * cds + t * cdf
        return s * cl, cd

    def force(self, v_body: np.ndarray) -> tuple[np.ndarray, float, float, float]:
        """Force on the body (N) from the relative airflow ``v_body`` (air velocity of the wing through still air,
        body frame). Returns (F, alpha_wing, lift, drag). Sideslip is ignored: only the x-z components act."""
        u, w = float(v_body[0]), float(v_body[2])
        v2 = u * u + w * w
        if v2 < 1e-4:
            return np.zeros(3), self.incidence, 0.0, 0.0
        v = math.sqrt(v2)
        alpha = math.atan2(w, u) + self.incidence
        cl, cd = self.coeffs(alpha)
        q = 0.5 * RHO * v2 * self.area
        lift_dir = np.array([w, 0.0, -u]) / v
        drag_dir = -np.array([u, 0.0, w]) / v
        return q * (cl * lift_dir + cd * drag_dir), alpha, q * cl, q * cd


def pitch_rotation(theta: float) -> np.ndarray:
    """World <- body for a nose-up pitch theta (level flight along world x, z down)."""
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


# ------------------------------------------------------------- vehicle model
class VehicleModel:
    """Static force/moment model of an airframe, shared by the trim solver and the optimiser."""

    def __init__(self, af):
        self.af = af
        n = len(af.rotors)
        self.n = n
        self.pos = np.array([r.pos for r in af.rotors], float).reshape(n, 3)
        ax = np.array([r.axis for r in af.rotors], float).reshape(n, 3)
        ax /= np.maximum(np.linalg.norm(ax, axis=1, keepdims=True), 1e-9)
        self.axis = ax
        self.km = np.array([r.km for r in af.rotors], float)
        self.tmax = np.array([r.effective_max_thrust() for r in af.rotors], float)
        self.area = np.array([math.pi * (r.prop_diameter / 2) ** 2 for r in af.rotors], float)
        self.ram = np.array([bool(r.ram_drag) for r in af.rotors], bool)
        self.mdot_coef = np.sqrt(RHO * self.area) * self.ram         # mdot = coef * sqrt(T)
        self.ducted = np.array([r.kind == "ducted" for r in af.rotors], bool)
        self.drag_q = np.array(af.drag_quadratic, float)
        self.wings = [(np.array(w.pos, float), WingAero.of(w)) for w in (af.wings or []) if w.enabled]
        self.mass = float(af.mass)
        # PX4 effectiveness (hover frame), unit thrust per motor
        R = af.hover_rotation()
        rows = []
        for i in range(n):
            p, a = R @ self.pos[i], R @ self.axis[i]
            rows.append(np.concatenate([np.cross(p, a) - self.km[i] * a, a]))
        self.E = np.array(rows).T if n else np.zeros((6, 0))
        self.E_pinv = np.linalg.pinv(self.E) if n else np.zeros((0, 6))

    # -- PX4's allocation for a setpoint [torque xyz, force xyz] in the hover frame, thrusts in N (clipped)
    def allocate(self, sp: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        u = self.E_pinv @ sp
        return u, np.clip(u, 0.0, self.tmax)

    def ideal_power(self, thrust: np.ndarray) -> float:
        """Momentum-theory power: T^1.5 / (2 sqrt(rho A)) for a duct, T^1.5 / sqrt(2 rho A) for an open prop."""
        t = np.maximum(thrust, 0.0)
        p = np.where(self.ducted, t ** 1.5 / (2 * np.sqrt(RHO * self.area)), t ** 1.5 / np.sqrt(2 * RHO * self.area))
        return float(p.sum())

    def forces(self, theta: float, thrust: np.ndarray, airspeed: float) -> dict:
        """Net force and moment (body frame) in steady flight at nose-up pitch ``theta`` and airspeed along world x."""
        Rwb = pitch_rotation(theta)
        v = Rwb.T @ np.array([airspeed, 0.0, 0.0])          # airflow the vehicle moves through, body frame
        tv = thrust[:, None] * self.axis
        F = tv.sum(axis=0)
        M = (np.cross(self.pos, tv) - (self.km * thrust)[:, None] * self.axis).sum(axis=0)
        mdot = self.mdot_coef * np.sqrt(np.maximum(thrust, 0.0))
        f_ram = -mdot[:, None] * v[None, :]
        F_ram = f_ram.sum(axis=0)
        F += F_ram
        M += np.cross(self.pos, f_ram).sum(axis=0)
        F_body = -self.drag_q * v * np.abs(v)
        F += F_body
        lift = drag = 0.0
        alphas = []
        for p, wa in self.wings:
            fw, alpha, l, d = wa.force(v)
            F += fw
            M += np.cross(p, fw)
            lift += l
            drag += d
            alphas.append(alpha)
        F += Rwb.T @ np.array([0.0, 0.0, self.mass * G])
        return {"F": F, "M": M, "ram": float(np.linalg.norm(F_ram)), "body_drag": float(np.linalg.norm(F_body)),
                "lift": lift, "wing_drag": drag, "alpha": (max(alphas, key=abs) if alphas else None)}

    # ------------------------------------------------------------ hover
    def hover(self) -> dict:
        weight = self.mass * G
        u_raw, u = self.allocate(np.array([0, 0, 0, 0, 0, -1.0]))
        thrust_up = float(-(self.E[5] @ u_raw))
        if thrust_up <= 1e-9 or self.n == 0:
            return {"ok": False, "max_util": float("inf"), "problems": ["no lift"], "thrust": [], "util": [],
                    "negative": [], "power": 0.0, "authority": {}}
        scale = weight / thrust_up
        u_raw = u_raw * scale
        resid = self.E @ u_raw - np.array([0, 0, 0, 0, 0, -weight])
        negative = [i + 1 for i, v in enumerate(u_raw) if v < -1e-6]
        thrust = np.clip(u_raw, 0.0, None)
        util = thrust / np.maximum(self.tmax, 1e-9)
        # control authority: torque (Nm) about each axis before the first motor leaves [0, Tmax] from the hover mix
        auth = {}
        for k, name in enumerate(("roll", "pitch", "yaw")):
            sp = np.zeros(6); sp[k] = 1.0
            du = self.E_pinv @ sp
            lim = []
            for sign in (1.0, -1.0):
                d = du * sign
                with np.errstate(divide="ignore", invalid="ignore"):
                    room = np.where(d > 1e-9, (self.tmax - thrust) / d, np.where(d < -1e-9, -thrust / d, np.inf))
                lim.append(float(room.min()) if len(room) else 0.0)
            auth[name] = min(lim)
        force_resid = float(np.abs(resid[3:5]).max())
        rp_resid = float(np.abs(resid[:2]).max())
        yaw_resid = float(abs(resid[2]))
        problems = []
        if force_resid > 1e-3 * weight or rp_resid > 1e-3 * weight:
            problems.append("hover needs a net force or torque no motor mix gives")
        if negative:
            problems.append(f"allocator wants negative thrust on motor(s) {negative}")
        if yaw_resid > 2e-3 * weight:
            problems.append("no yaw authority")
        return {"ok": not problems, "problems": problems, "thrust": thrust.tolist(), "util": util.tolist(),
                "max_util": float(util.max()), "negative": negative, "power": self.ideal_power(thrust),
                "total_thrust": float(thrust.sum()), "waste": float(thrust.sum() / weight - 1.0),
                "authority": auth, "residual_force": float(force_resid), "residual_yaw": yaw_resid}

    # ----------------------------------------------------------- cruise
    def cruise(self, airspeed: float, tilt_limit_deg: float = 45.0) -> dict:
        """Steady level flight at ``airspeed`` (m/s): solve pitch, collective and pitch torque for zero net force
        and pitching moment with PX4's allocation in the loop."""
        if self.n == 0:
            return {"ok": False, "problems": ["no rotors"]}
        weight = self.mass * G
        hover_phi = math.radians(self.af.hover_pitch_deg)

        def residual(x):
            theta, tz, tau = x
            u_raw, u = self.allocate(np.array([0.0, tau, 0.0, 0.0, 0.0, -tz]))
            f = self.forces(theta, u, airspeed)
            return np.array([f["F"][0], f["F"][2], f["M"][1]]), u_raw, u, f

        best = None
        for theta0 in (hover_phi - 0.4, hover_phi, hover_phi - 0.8, hover_phi + 0.3):
            x = np.array([theta0, weight * 0.8, 0.0])
            lam = 1e-2
            r, *_ = residual(x)
            stall_count = 0
            for _ in range(40):
                J = np.zeros((3, 3))
                h = np.array([1e-4, 1e-2, 1e-3])
                for j in range(3):
                    xp = x.copy(); xp[j] += h[j]
                    J[:, j] = (residual(xp)[0] - r) / h[j]
                try:
                    dx = -np.linalg.solve(J.T @ J + lam * np.diag(np.diag(J.T @ J) + 1e-9), J.T @ r)
                except np.linalg.LinAlgError:
                    break
                xn = x + dx
                xn[0] = float(np.clip(xn[0], -math.pi / 2, math.pi / 2))
                xn[1] = max(0.0, xn[1])
                rn, *_ = residual(xn)
                if np.linalg.norm(rn) < np.linalg.norm(r):
                    x, r, lam = xn, rn, max(lam / 3, 1e-6)
                    stall_count = 0
                else:
                    lam = min(lam * 5, 1e3)
                    stall_count += 1
                if np.linalg.norm(r) < 1e-3 * weight or stall_count >= 6:
                    break
            if best is None or np.linalg.norm(r) < np.linalg.norm(best[1]):
                best = (x.copy(), r.copy())
            if np.linalg.norm(best[1]) < 1e-3 * weight:
                break
        x, r = best
        r, u_raw, u, f = residual(x)
        theta, tz, tau = (float(v) for v in x)
        converged = bool(np.linalg.norm(r) < 5e-3 * weight)
        px4_pitch = theta - hover_phi
        util = u / np.maximum(self.tmax, 1e-9)
        saturated = [i + 1 for i in range(self.n) if u_raw[i] > self.tmax[i] + 1e-6]
        negative = [i + 1 for i in range(self.n) if u_raw[i] < -1e-6]
        problems = []
        if not converged:
            problems.append("no steady state found at this speed")
        if saturated:
            problems.append(f"motor(s) {saturated} saturated in cruise")
        if negative:
            problems.append(f"allocator wants negative thrust on motor(s) {negative} in cruise")
        if abs(math.degrees(px4_pitch)) > tilt_limit_deg:
            problems.append(f"PX4 pitch {math.degrees(px4_pitch):.0f}° exceeds MPC_TILTMAX_AIR ({tilt_limit_deg:g}°)")
        alpha = f["alpha"]
        wing_stalled = bool(self.wings and alpha is not None and abs(alpha) > min(w.stall for _, w in self.wings))
        if wing_stalled:
            problems.append(f"wing stalled ({math.degrees(alpha):.0f}° angle of attack)")
        return {"ok": not problems, "converged": converged, "problems": problems, "airspeed": airspeed,
                "pitch_deg": math.degrees(theta), "px4_pitch_deg": math.degrees(px4_pitch),
                "collective": tz, "pitch_torque": tau, "thrust": u.tolist(), "util": util.tolist(),
                "max_util": float(util.max()), "total_thrust": float(u.sum()), "power": self.ideal_power(u),
                "lift": f["lift"], "lift_share": f["lift"] / weight, "wing_drag": f["wing_drag"], "ram_drag": f["ram"],
                "body_drag": f["body_drag"], "alpha_deg": (math.degrees(alpha) if alpha is not None else None),
                "saturated": saturated, "negative": negative, "residual": float(np.linalg.norm(r))}


def analyse(af, airspeed: float | None = None) -> dict:
    m = VehicleModel(af)
    speed = airspeed if airspeed is not None else af.design.get("cruise_speed_kmh", 50.0) / 3.6
    h = m.hover()
    c = m.cruise(speed)
    if h.get("power"):
        c["power_ratio"] = c["power"] / h["power"] if c.get("power") is not None else None
    return {"hover": h, "cruise": c, "airspeed": speed}


# ---------------------------------------------------------------- optimiser
def _tilt_axis(tilt_deg: float, forward: bool, cant_deg: float, right: bool) -> list[float]:
    t = math.radians(tilt_deg)
    ax = np.array([math.sin(t) * (1.0 if forward else -1.0), 0.0, -math.cos(t)])
    c = math.radians(cant_deg) * (1.0 if right else -1.0)
    ax = np.array([ax[0], -math.sin(c) * ax[2], math.cos(c) * ax[2]])
    return [round(float(v), 5) for v in ax]


def rotor_tilt_cant(axis) -> tuple[float, bool, float, bool]:
    """Inverse of _tilt_axis: (tilt_deg, forward, cant_deg, right) of a thrust axis."""
    a = np.array(axis, float)
    a /= max(np.linalg.norm(a), 1e-9)
    cant = math.degrees(math.atan2(abs(a[1]), -a[2])) if a[2] < 0 else 90.0
    az = -math.hypot(a[1], a[2])
    tilt = math.degrees(math.atan2(abs(a[0]), -az))
    return tilt, a[0] >= 0, cant, a[1] >= 0


@dataclass
class Variable:
    kind: str            # "tilt" | "cant" | "hover_pitch"
    group: str
    lo: float
    hi: float


def _apply(af, spec: dict, variables: list[Variable], x: np.ndarray):
    """Airframe copy with the design variables ``x`` applied."""
    new = copy.deepcopy(af)
    groups = spec["groups"]     # {name: {"rotors": [idx...]}}
    per_group: dict[str, dict[str, float]] = {}
    for var, val in zip(variables, x):
        if var.kind == "hover_pitch":
            new.hover_pitch_deg = float(val)
        else:
            per_group.setdefault(var.group, {})[var.kind] = float(val)
    for gname, vals in per_group.items():
        for i in groups[gname]["rotors"]:
            r = new.rotors[i]
            tilt, fwd, cant, right = rotor_tilt_cant(r.axis)
            if r.pos[1] != 0:
                right = r.pos[1] > 0
            r.axis = _tilt_axis(vals.get("tilt", tilt), fwd, vals.get("cant", cant), right)
    return new


def _score(m: dict, weight: float, tilt_limit: float) -> float:
    h, c = m["hover"], m["cruise"]
    # hover cost: the busiest motor's share plus the thrust wasted on motors fighting each other
    s = weight * (h["max_util"] + h.get("waste", 0.0)) + (1 - weight) * (c.get("power_ratio") or 5.0)
    if not h["ok"]:
        s += 5.0 + 2.0 * len(h.get("negative", []))
    if not c.get("converged", False):
        s += 5.0
    s += 2.0 * len(c.get("saturated", [])) + 2.0 * len(c.get("negative", []))
    over = abs(c.get("px4_pitch_deg", 0.0)) - tilt_limit
    if over > 0:
        s += over / 10.0
    if any("stalled" in p for p in c.get("problems", [])):
        s += 1.0
    return float(s)


def _nelder_mead(f: Callable[[np.ndarray], float], x0: np.ndarray, lo: np.ndarray, hi: np.ndarray,
                 step: np.ndarray, max_eval: int) -> tuple[np.ndarray, float]:
    n = len(x0)
    pts = [np.clip(x0, lo, hi)]
    for i in range(n):
        p = x0.copy(); p[i] = np.clip(p[i] + step[i], lo[i], hi[i])
        if abs(p[i] - x0[i]) < 1e-9:
            p[i] = np.clip(x0[i] - step[i], lo[i], hi[i])
        pts.append(p)
    vals = [f(p) for p in pts]
    evals = n + 1
    while evals < max_eval:
        order = np.argsort(vals)
        pts = [pts[i] for i in order]; vals = [vals[i] for i in order]
        if vals[-1] - vals[0] < 1e-5 and max(np.abs(pts[-1] - pts[0])) < 0.05:
            break
        centroid = np.mean(pts[:-1], axis=0)
        xr = np.clip(centroid + (centroid - pts[-1]), lo, hi); fr = f(xr); evals += 1
        if fr < vals[0]:
            xe = np.clip(centroid + 2 * (centroid - pts[-1]), lo, hi); fe = f(xe); evals += 1
            if fe < fr:
                pts[-1], vals[-1] = xe, fe
            else:
                pts[-1], vals[-1] = xr, fr
        elif fr < vals[-2]:
            pts[-1], vals[-1] = xr, fr
        else:
            xc = np.clip(centroid + 0.5 * (pts[-1] - centroid), lo, hi); fc = f(xc); evals += 1
            if fc < vals[-1]:
                pts[-1], vals[-1] = xc, fc
            else:
                for i in range(1, len(pts)):
                    pts[i] = np.clip(pts[0] + 0.5 * (pts[i] - pts[0]), lo, hi); vals[i] = f(pts[i]); evals += 1
    i = int(np.argmin(vals))
    return pts[i], vals[i]


def optimise(af, spec: dict, progress: Callable[[float, str], None] | None = None) -> dict:
    """Search jet angles (per group tilt / cant) and the hover pitch for the best hover margin and cruise power.

    spec = {"groups": {"A": {"rotors": [0, 1], "tilt": [lo, hi] | None, "cant": [lo, hi] | None}, ...},
            "hover_pitch": [lo, hi] | None, "weight": 0..1 (1 = hover only), "speed_kmh": 50,
            "samples": 300, "refine": 8, "tilt_limit_deg": 45}
    """
    speed = float(spec.get("speed_kmh", 50.0)) / 3.6
    weight = float(np.clip(spec.get("weight", 0.5), 0.0, 1.0))
    tilt_limit = float(spec.get("tilt_limit_deg", 45.0))
    variables: list[Variable] = []
    for gname, g in spec["groups"].items():
        if g.get("tilt"):
            variables.append(Variable("tilt", gname, *g["tilt"]))
        if g.get("cant"):
            variables.append(Variable("cant", gname, *g["cant"]))
    if spec.get("hover_pitch"):
        variables.append(Variable("hover_pitch", "", *spec["hover_pitch"]))
    if not variables:
        return {"ok": False, "error": "nothing to optimise: enable at least one variable"}
    lo = np.array([v.lo for v in variables]); hi = np.array([v.hi for v in variables])

    cache: dict[tuple, dict] = {}

    def evaluate(x: np.ndarray) -> dict:
        key = tuple(np.round(x, 2))
        if key in cache:
            return cache[key]
        cand = _apply(af, spec, variables, np.array(key))
        time.sleep(0.001)          # let the simulation thread have the interpreter between evaluations
        m = analyse(cand, speed)
        m["score"] = _score(m, weight, tilt_limit)
        m["x"] = [float(v) for v in key]
        cache[key] = m
        return m

    # current design as the first sample
    x_now = []
    for v in variables:
        if v.kind == "hover_pitch":
            x_now.append(af.hover_pitch_deg)
        else:
            i0 = spec["groups"][v.group]["rotors"][0]
            tilt, _, cant, _ = rotor_tilt_cant(af.rotors[i0].axis)
            x_now.append(tilt if v.kind == "tilt" else cant)
    x_now = np.clip(np.array(x_now, float), lo, hi)
    if int(spec.get("samples", 300)) > 1000:
        spec["samples"] = 1000

    n_samples = int(spec.get("samples", 300))
    rng = np.random.default_rng(int(spec.get("seed", 1)))
    samples = [x_now]
    # Latin-hypercube-ish coverage plus a coarse grid on the first two variables
    for k in range(n_samples):
        samples.append(lo + (hi - lo) * rng.random(len(variables)))
    if len(variables) <= 2:
        grid = np.linspace(0, 1, 13)
        for a in grid:
            if len(variables) == 1:
                samples.append(lo + (hi - lo) * a)
            else:
                for b in grid:
                    samples.append(lo + (hi - lo) * np.array([a, b]))
    results = []
    for k, x in enumerate(samples):
        results.append(evaluate(x))
        if progress and k % 20 == 0:
            progress(0.7 * k / len(samples), f"sampling {k}/{len(samples)}")
    results.sort(key=lambda m: m["score"])
    n_refine = int(spec.get("refine", 6))
    step = (hi - lo) * 0.08
    for k, start in enumerate(results[:n_refine]):
        if progress:
            progress(0.7 + 0.3 * k / n_refine, f"refining {k + 1}/{n_refine}")
        _nelder_mead(lambda x: evaluate(x)["score"], np.array(start["x"]), lo, hi, step, max_eval=90)
    allres = sorted(cache.values(), key=lambda m: m["score"])
    feasible = [m for m in allres if m["hover"]["ok"] and m["hover"]["max_util"] < 1.0
                and m["cruise"].get("converged") and not m["cruise"].get("saturated")]
    # Pareto front on (hover max utilisation, cruise power ratio) among feasible designs
    pareto = []
    for m in sorted(feasible, key=lambda m: (m["hover"]["max_util"], m["cruise"]["power_ratio"] or 9)):
        pr = m["cruise"]["power_ratio"] or 9
        if all(pr < (p["cruise"]["power_ratio"] or 9) for p in pareto):
            pareto.append(m)
    # thin the front to a handful of evenly spread designs so the table stays readable
    if len(pareto) > 8:
        idx = sorted({int(round(i)) for i in np.linspace(0, len(pareto) - 1, 8)})
        pareto = [pareto[i] for i in idx]
    pareto_keys = {tuple(m["x"]) for m in pareto}

    def pack(m):
        cand = _apply(af, spec, variables, np.array(m["x"]))
        return {"x": m["x"], "score": m["score"], "pareto": tuple(m["x"]) in pareto_keys,
                "hover": {k: m["hover"][k] for k in ("ok", "max_util", "power", "authority", "problems", "waste", "total_thrust")},
                "cruise": {k: m["cruise"].get(k) for k in ("ok", "converged", "pitch_deg", "px4_pitch_deg", "max_util",
                                                            "total_thrust", "power", "power_ratio", "lift_share",
                                                            "alpha_deg", "problems")},
                "hover_pitch_deg": cand.hover_pitch_deg,
                "axes": [r.axis for r in cand.rotors]}

    seen: set[tuple] = set()
    top = []
    for m in allres:
        key = tuple(int(round(v / 2.0)) for v in m["x"])
        if key in seen:
            continue
        seen.add(key)
        top.append(pack(m))
        if len(top) >= 6:
            break
    for m in pareto:
        key = tuple(int(round(v / 2.0)) for v in m["x"])
        if key not in seen:
            seen.add(key)
            top.append(pack(m))
    if progress:
        progress(1.0, "done")
    return {"ok": True, "variables": [{"kind": v.kind, "group": v.group, "lo": v.lo, "hi": v.hi} for v in variables],
            "evaluated": len(cache), "feasible": len(feasible), "results": top,
            "current": pack(evaluate(x_now))}


def default_groups(af) -> dict:
    """Rotors that share a tilt and cant (to the degree) form one group, named A, B, C... front to back."""
    keys: dict[tuple, list[int]] = {}
    for i, r in enumerate(af.rotors):
        tilt, fwd, cant, _ = rotor_tilt_cant(r.axis)
        keys.setdefault((round(tilt), fwd, round(cant)), []).append(i)
    ordered = sorted(keys.values(), key=lambda idx: -max(af.rotors[i].pos[0] for i in idx))
    return {chr(65 + k): {"rotors": idx} for k, idx in enumerate(ordered)}
