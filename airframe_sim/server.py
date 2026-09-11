"""FastAPI web server: serves the 3D UI, streams sim state over a websocket, exposes control REST endpoints."""
from __future__ import annotations

import asyncio
import json
import time
from collections import deque
from pathlib import Path
from typing import Any

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, JSONResponse, PlainTextResponse
from fastapi.staticfiles import StaticFiles
from starlette.concurrency import run_in_threadpool

from . import airframe as af_mod
from .airframe import Airframe
from .link import mavlink
from . import param_meta

UI_DIR = Path(__file__).resolve().parent.parent / "ui"
AIRFRAME_DIR = Path(__file__).resolve().parent.parent / "airframes"

# PX4 custom mode encoding: main_mode << 16 | sub_mode << 24
PX4_MODES = {
    "manual": (1, 0), "altitude": (2, 0), "position": (3, 0), "acro": (5, 0), "stabilized": (7, 0),
    "takeoff": (4, 2), "hold": (4, 3), "mission": (4, 4), "rtl": (4, 5), "land": (4, 6),
}
PX4_MAIN_MODE_NAMES = {1: "Manual", 2: "Altitude", 3: "Position", 4: "Auto", 5: "Acro", 6: "Offboard", 7: "Stabilized",
                       8: "Rattitude", 9: "Simple", 10: "Termination"}
PX4_SUB_MODE_NAMES = {1: "Ready", 2: "Takeoff", 3: "Hold", 4: "Mission", 5: "RTL", 6: "Land", 8: "Follow", 9: "Precland"}


def mode_name(custom_mode: int) -> str:
    main = (custom_mode >> 16) & 0xFF
    sub = (custom_mode >> 24) & 0xFF
    name = PX4_MAIN_MODE_NAMES.get(main, f"mode{main}")
    if main == 4:
        name = PX4_SUB_MODE_NAMES.get(sub, f"Auto{sub}")
    return name


class _NoLink:
    """Stand-in while no PX4 link exists so the endpoints degrade gracefully."""
    mode = "none"
    connected = False
    ctl_connected = False
    params: dict = {}
    param_count = 0

    def status(self):
        return {"mode": "none", "address": "", "connected": False, "ctl_connected": False, "armed": False,
                "hil_enabled": False, "custom_mode": 0, "rx_count": 0, "param_count": 0, "params_loaded": 0,
                "actuator_seq": 0, "qgc_proxy": None, "target_system": 0, "ctl_address": ""}

    def __getattr__(self, name):
        def noop(*a, **k):
            return {"ok": False, "error": "PX4 not connected"}
        return noop


class AppState:
    def __init__(self, simulator, conn, args, log_buffer: deque, log):
        self.simulator = simulator
        self.conn = conn                     # ConnectionManager
        self.args = args
        self.log_buffer = log_buffer
        self.log = log
        self.meta: dict[str, dict] = {}
        self.meta_source = ""
        self.export_log: deque = deque(maxlen=500)

    @property
    def link(self):
        return self.conn.link if self.conn.link is not None else _NoLink()


def build_app(state: AppState) -> FastAPI:
    app = FastAPI(title="AIRFRAME_SIMULATOR")
    app.mount("/static", StaticFiles(directory=str(UI_DIR)), name="static")
    sim = state.simulator

    class _LinkProxy:
        def __getattr__(self, name):
            return getattr(state.link, name)

    link = _LinkProxy()   # always resolves to the current link

    # -------------------------------------------------------------- pages
    @app.get("/")
    async def index():
        return FileResponse(str(UI_DIR / "index.html"))

    # ------------------------------------------------------------- status
    def status_dict() -> dict:
        s = link.status()
        s["mode_name"] = mode_name(s["custom_mode"])
        s["px4_running"] = state.conn.px4_running()
        s["conn_mode"] = state.conn.mode
        s["conn_error"] = state.conn.error
        s["flashing"] = state.conn.firmware_job.running() and state.conn.firmware_job.action == "upload"
        # arm gating: PX4's last arming-check summary must report no system errors and a usable position
        ready, why = False, "waiting for PX4's arming check report"
        for x in reversed(list(getattr(state.link, "recent_events", []) or [])):
            if x.get("name") == "commander_arming_check_summary":
                d = dict(zip(x.get("arg_names", []), x.get("args", [])))
                # PX4 lists the modes it would arm in; the error mask also carries the always-failing
                # offboard/mission checks ("system"), so it is not usable as a gate on its own.
                can = str(d.get("can_arm", ""))
                mode_now = s.get("mode_name", "").lower()
                aliases = {"hold": "loiter", "stabilized": "stab", "position": "posctl", "altitude": "altctl"}
                ready = ("takeoff" in can) or ("loiter" in can) or (aliases.get(mode_now, mode_now) in can.split("|"))
                why = "" if ready else "PX4 will not arm yet (estimator or health checks); see the Flight tab"
                break
        s["resetting"] = max(0.0, state.conn._reset_busy_until - time.time())
        s["arm_ready"] = bool(s["ctl_connected"]) and (ready or bool(s["armed"])) and s["resetting"] <= 0
        s["arm_block_reason"] = why
        s["px4_ports"] = [p["device"] for p in state.conn.list_ports_cached() if p["likely_px4"]]
        # a RadioMaster radio that shows up as a *serial* port was powered on in VCP/config mode (M + Power);
        # in that mode it is not a joystick
        s["radio_vcp_ports"] = [p["device"] for p in state.conn.list_ports_cached() if "radiomaster" in (p["device"] + p["description"]).lower()]
        s["meta_loaded"] = len(state.meta)
        s["meta_source"] = state.meta_source
        s["home"] = {"lat": sim.sensors.home.lat, "lon": sim.sensors.home.lon, "alt": sim.sensors.home.alt}
        s["speed"] = sim.speed
        s["sensor_rate"] = sim.sensor_rate
        s["lockstep"] = sim.lockstep
        s["noise"] = sim.sensors.noise.enabled
        return s

    @app.get("/api/status")
    async def get_status():
        return status_dict()

    @app.get("/api/log")
    async def get_log(since: float = 0.0):
        return [e for e in state.log_buffer if e[0] > since]

    # ----------------------------------------------------------- airframe
    @app.get("/api/airframe")
    async def get_airframe():
        return sim.airframe.to_dict()

    @app.post("/api/airframe")
    async def set_airframe(body: dict):
        try:
            af = Airframe.from_dict(body.get("airframe", body))
        except Exception as e:
            return JSONResponse({"ok": False, "error": f"invalid airframe: {e}"}, status_code=400)
        keep = bool(body.get("keep_state", True))
        af.leg_points = af.generate_legs()
        sim.set_airframe(af, keep_state=keep)
        hc = af.hover_check()
        return {"ok": True, "airframe": af.to_dict(), "problems": af.validate() + hc["problems"], "hover": hc}

    @app.get("/api/airframe/hover_check")
    async def hover_check():
        return sim.airframe.hover_check()

    @app.get("/api/airframes")
    async def list_airframes():
        files = sorted(p.name for p in AIRFRAME_DIR.glob("*.json"))
        files.sort(key=lambda n: (n != "multirotor_10.json", n))
        return {"presets": ["quad_x", "hex_x"], "files": files}

    @app.post("/api/airframe/load")
    async def load_airframe(body: dict):
        name = body.get("name", "")
        if name == "quad_x":
            af = af_mod.quad_x()
        elif name == "hex_x":
            af = af_mod.hex_x()
        else:
            p = AIRFRAME_DIR / name
            if not p.is_file():
                return JSONResponse({"ok": False, "error": f"not found: {name}"}, status_code=404)
            af = Airframe.load(p)
        sim.set_airframe(af, keep_state=False)
        hc = af.hover_check()
        return {"ok": True, "airframe": af.to_dict(), "problems": af.validate() + hc["problems"], "hover": hc}

    @app.post("/api/airframe/save")
    async def save_airframe(body: dict):
        name = body.get("name", "").strip()
        if not name:
            return JSONResponse({"ok": False, "error": "name required"}, status_code=400)
        if not name.endswith(".json"):
            name += ".json"
        name = Path(name).name
        AIRFRAME_DIR.mkdir(exist_ok=True)
        sim.airframe.save(AIRFRAME_DIR / name)
        return {"ok": True, "path": str(AIRFRAME_DIR / name)}

    @app.post("/api/airframe/estimate_inertia")
    async def estimate_inertia():
        af = sim.airframe
        af.inertia = af.estimate_inertia()
        sim.set_airframe(af)
        return {"ok": True, "inertia": af.inertia}

    # ------------------------------------------------------------ connection
    @app.get("/api/connection")
    async def get_connection():
        st = state.conn.status()
        st["checklist"] = state.conn.checklist(export_params() if state.conn.mode == "hitl" else None)
        return st

    @app.post("/api/connection/connect")
    async def connect(body: dict):
        mode = body.get("mode", "sitl")
        if mode == "hitl":
            r = await run_in_threadpool(state.conn.connect_hitl, body.get("serial"), body.get("baud"))
        else:
            r = await run_in_threadpool(state.conn.connect_sitl, body.get("launch"))
        return r

    @app.post("/api/connection/disconnect")
    async def disconnect():
        return await run_in_threadpool(state.conn.disconnect)

    @app.post("/api/px4/reset_all")
    async def px4_reset_all():
        r = await run_in_threadpool(state.conn.reset_all)
        state.log("[px4] reset: " + ", ".join(r.get("steps", [])))
        return r

    @app.post("/api/px4/recover")
    async def px4_recover():
        r = await run_in_threadpool(state.conn.recover)
        state.log("[px4] recover: " + ", ".join(r.get("steps", [])))
        return r

    @app.post("/api/connection/restart_estimator")
    async def restart_estimator():
        return await run_in_threadpool(state.conn.restart_estimator)

    @app.post("/api/connection/enable_hitl")
    async def enable_hitl():
        return await run_in_threadpool(state.conn.enable_hitl)

    @app.post("/api/firmware/build")
    async def firmware_build(body: dict | None = None):
        return await run_in_threadpool(state.conn.build_firmware, (body or {}).get("target"))

    @app.post("/api/firmware/upload")
    async def firmware_upload(body: dict | None = None):
        return await run_in_threadpool(state.conn.upload_firmware, (body or {}).get("target"))

    @app.post("/api/px4/shell")
    async def px4_shell(body: dict):
        cmd = str(body.get("command", "")).strip()
        if not cmd:
            return JSONResponse({"ok": False, "error": "command required"}, status_code=400)
        if not link.ctl_connected:
            return JSONResponse({"ok": False, "error": "PX4 not connected"}, status_code=409)
        out = await run_in_threadpool(link.shell, cmd, float(body.get("timeout", 3.0)))
        return {"ok": True, "output": out}

    @app.get("/api/rc")
    async def get_rc():
        rc = dict(getattr(state.link, "rc", {}) or {})
        rc = rc if rc and time.time() - rc.get("t", 0) < 3.0 else {}
        # PX4 channel mapping (1-based channel numbers, 0 = unassigned)
        names = {"RC_MAP_ROLL": "Roll", "RC_MAP_PITCH": "Pitch", "RC_MAP_THROTTLE": "Throttle", "RC_MAP_YAW": "Yaw",
                 "RC_MAP_FLTMODE": "Flight mode", "RC_MAP_ARM_SW": "Arm", "RC_MAP_KILL_SW": "Kill", "RC_MAP_RETURN_SW": "Return",
                 "RC_MAP_LOITER_SW": "Loiter", "RC_MAP_OFFB_SW": "Offboard", "RC_MAP_GEAR_SW": "Gear", "RC_MAP_FLAPS": "Flaps",
                 "RC_MAP_AUX1": "Aux 1", "RC_MAP_AUX2": "Aux 2", "RC_MAP_AUX3": "Aux 3", "RC_MAP_AUX4": "Aux 4",
                 "RC_MAP_AUX5": "Aux 5", "RC_MAP_AUX6": "Aux 6", "RC_MAP_PARAM1": "Param 1", "RC_MAP_PARAM2": "Param 2",
                 "RC_MAP_PARAM3": "Param 3", "RC_MAP_TRANS_SW": "Transition", "RC_MAP_ENG_MOT": "Engine/motor",
                 "RC_MAP_PAY_SW": "Payload", "RC_MAP_FAILSAFE": "Failsafe"}
        mapping: dict[int, list[str]] = {}
        params = getattr(state.link, "params", {}) or {}
        for k, label in names.items():
            v = params.get(k, {}).get("value")
            if isinstance(v, (int, float)) and int(v) > 0:
                mapping.setdefault(int(v), []).append(label)
        rc["mapping"] = {str(k): v for k, v in mapping.items()}
        rc["rc_in_mode"] = params.get("COM_RC_IN_MODE", {}).get("value")
        return rc

    @app.get("/api/events")
    async def get_events():
        return {"source": state.conn.event_decoder.source if state.conn.event_decoder else "",
                "events": list(link.recent_events) if hasattr(state.link, "recent_events") else []}

    @app.post("/api/events/meta/fetch")
    async def fetch_events_meta():
        if not link.ctl_connected:
            return JSONResponse({"ok": False, "error": "PX4 control link not connected"}, status_code=409)
        local, msg = await run_in_threadpool(param_meta.fetch_extra, link, "all_events.json.xz", state.log)
        if local is None:
            return JSONResponse({"ok": False, "error": msg}, status_code=500)
        from .events import _read_json
        n = state.conn.event_decoder.load(_read_json(local), str(local))
        return {"ok": True, "count": n}

    @app.get("/api/firmware")
    async def firmware_status():
        b = state.conn.detected_board()
        return {"job": state.conn.firmware_job.status(), "board": b, "toolchain": state.conn.toolchain_present(),
                "built": state.conn.firmware_file(b["target"])}

    # ------------------------------------------------------------ PX4 export
    def export_params() -> dict[str, float | int]:
        hitl = link.mode == "hitl"
        p = sim.airframe.px4_params(hitl=hitl)
        if not hitl:
            # SITL uses the PWM_MAIN output driver, HITL the HIL_ACT one
            for n in range(1, 17):
                p[f"PWM_MAIN_FUNC{n}"] = p.pop(f"HIL_ACT_FUNC{n}")
        return p

    @app.get("/api/px4/export")
    async def get_export():
        params = export_params()
        current = {k: link.params.get(k, {}).get("value") for k in params}
        ov = sim.airframe.px4_overrides or {}
        return {"params": params, "current": current, "problems": sim.airframe.validate() + sim.airframe.hover_check()["problems"],
                "file": sim.airframe.px4_params_file(hitl=link.mode == "hitl"),
                "overrides": ov, "geometry_keys": [k for k in params if k not in ov]}

    @app.get("/api/px4/export.params")
    async def get_export_file():
        return PlainTextResponse(sim.airframe.px4_params_file(hitl=link.mode == "hitl"),
                                 headers={"Content-Disposition": f'attachment; filename="{sim.airframe.name}.params"'})

    @app.post("/api/px4/push")
    async def push_params(body: dict | None = None):
        body = body or {}
        if not link.ctl_connected:
            return JSONResponse({"ok": False, "error": "PX4 control link not connected"}, status_code=409)
        if link.armed:
            return JSONResponse({"ok": False, "error": "vehicle is armed; disarm before updating PX4"}, status_code=409)
        params = export_params()
        only = body.get("only")
        if only:
            params = {k: v for k, v in params.items() if k in only}
        # skip output-function params the firmware does not have (e.g. HIL_ACT on SITL)
        if link.params:
            missing = [k for k in params if k not in link.params]
            params = {k: v for k, v in params.items() if k in link.params}
        else:
            missing = []
        state.export_log.clear()
        rot_before = link.params.get("SENS_BOARD_Y_OFF", {}).get("value")

        def progress(name, res):
            state.export_log.append({"t": time.time(), **res})

        results = await run_in_threadpool(link.set_params, params, progress)
        ok = all(r["ok"] for r in results)
        if body.get("save", True) and ok:
            link.preflight_storage(True)
        rot_after = params.get("SENS_BOARD_Y_OFF")
        if ok and rot_after is not None and rot_before is not None and abs(float(rot_after) - float(rot_before)) > 1e-3:
            # the IMU frame just changed under the running estimator: rest the sim at the new hover attitude and
            # restart EKF2 so it aligns from clean data
            state.log(f"[export] board rotation changed ({rot_before} -> {rot_after} deg): resetting sim, restarting estimator")
            sim.reset()
            await run_in_threadpool(link.restart_estimator)
        failed = [r for r in results if not r["ok"]]
        state.log(f"[export] pushed {len(results) - len(failed)}/{len(results)} params to PX4"
                  + (f", failed: {[r['name'] for r in failed]}" if failed else ""))
        return {"ok": ok, "results": results, "missing": missing}

    # ---------------------------------------------------------- parameters
    @app.get("/api/params")
    async def get_params():
        return {"count": link.param_count, "params": link.params}

    @app.post("/api/params/refresh")
    async def refresh_params():
        if not link.ctl_connected:
            return JSONResponse({"ok": False, "error": "PX4 control link not connected"}, status_code=409)
        params = await run_in_threadpool(link.fetch_all_params)
        return {"ok": True, "count": link.param_count, "params": params}

    @app.post("/api/params/set")
    async def set_param(body: dict):
        name = body.get("name")
        value = body.get("value")
        if name is None or value is None:
            return JSONResponse({"ok": False, "error": "name and value required"}, status_code=400)
        res = await run_in_threadpool(link.set_param, name, value)
        if res.get("ok"):
            # remember it with the airframe so Save keeps it and Update PX4 re-applies it
            sim.airframe.px4_overrides[name] = res["value"]
        return res

    @app.post("/api/airframe/override")
    async def set_override(body: dict):
        """Record an edited parameter without touching the vehicle (used when not connected)."""
        name, value = body.get("name"), body.get("value")
        if not name or value is None:
            return JSONResponse({"ok": False, "error": "name and value required"}, status_code=400)
        sim.airframe.px4_overrides[name] = value
        return {"ok": True, "overrides": sim.airframe.px4_overrides}

    @app.post("/api/airframe/override_remove")
    async def remove_override(body: dict):
        sim.airframe.px4_overrides.pop(body.get("name", ""), None)
        return {"ok": True, "overrides": sim.airframe.px4_overrides}

    @app.post("/api/params/save")
    async def save_params():
        link.preflight_storage(True)
        return {"ok": True}

    @app.get("/api/params/meta")
    async def get_meta():
        return {"source": state.meta_source, "meta": state.meta}

    @app.post("/api/params/meta/fetch")
    async def fetch_meta():
        if not link.ctl_connected:
            return JSONResponse({"ok": False, "error": "PX4 control link not connected"}, status_code=409)
        meta, src = await run_in_threadpool(param_meta.fetch_from_vehicle, link, state.log)
        if meta:
            state.meta, state.meta_source = meta, src
            return {"ok": True, "count": len(meta), "source": src}
        return JSONResponse({"ok": False, "error": src}, status_code=500)

    # ------------------------------------------------------------ vehicle
    @app.post("/api/px4/command")
    async def px4_command(body: dict):
        cmd = body.get("command", "")
        if cmd == "arm":
            link.send_command_long(mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1.0, 21196.0 if body.get("force") else 0.0)
        elif cmd == "disarm":
            link.send_command_long(mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0.0, 21196.0 if body.get("force") else 0.0)
        elif cmd == "kill":
            link.send_command_long(mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0.0, 21196.0)
        elif cmd == "mode":
            main, sub = PX4_MODES.get(body.get("mode", ""), (None, None))
            if main is None:
                return JSONResponse({"ok": False, "error": "unknown mode"}, status_code=400)
            link.send_command_long(mavlink.MAV_CMD_DO_SET_MODE, float(mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED),
                                   float(main), float(sub))
        elif cmd == "takeoff":
            main, sub = PX4_MODES["takeoff"]
            link.send_command_long(mavlink.MAV_CMD_DO_SET_MODE, float(mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED),
                                   float(main), float(sub))
        elif cmd == "reboot":
            link.reboot()
        elif cmd == "save_params":
            link.preflight_storage(True)
        else:
            return JSONResponse({"ok": False, "error": "unknown command"}, status_code=400)
        return {"ok": True}

    # ---------------------------------------------------------------- sim
    @app.post("/api/sim/reset")
    async def sim_reset(body: dict | None = None):
        body = body or {}
        sim.reset(yaw=float(body.get("yaw", 0.0)))
        return {"ok": True}

    @app.post("/api/sim/pause")
    async def sim_pause(body: dict):
        sim.paused = bool(body.get("paused", not sim.paused))
        return {"ok": True, "paused": sim.paused}

    @app.post("/api/sim/speed")
    async def sim_speed(body: dict):
        sim.speed = max(0.0, float(body.get("speed", 1.0)))
        return {"ok": True, "speed": sim.speed}

    @app.post("/api/sim/motor_override")
    async def motor_override(body: dict):
        v = body.get("values")
        sim.motor_override = None if v is None else [float(x) for x in v]
        return {"ok": True}

    @app.post("/api/sim/wind")
    async def sim_wind(body: dict):
        sim.set_wind(float(body.get("north", 0)), float(body.get("east", 0)), float(body.get("down", 0)))
        return {"ok": True}

    @app.post("/api/sim/noise")
    async def sim_noise(body: dict):
        sim.sensors.noise.enabled = bool(body.get("enabled", True))
        return {"ok": True}

    @app.post("/api/sim/home")
    async def sim_home(body: dict):
        sim.sensors.set_home(float(body["lat"]), float(body["lon"]), float(body.get("alt", 0.0)))
        return {"ok": True}

    # ------------------------------------------------------------ websocket
    @app.websocket("/ws")
    async def ws(websocket: WebSocket):
        await websocket.accept()
        last_log = 0.0
        last_status = 0.0

        async def receive_loop():
            """Client -> server: joystick frames (and nothing else for now)."""
            while True:
                raw = await websocket.receive_text()
                try:
                    m = json.loads(raw)
                except Exception:
                    continue
                if m.get("type") == "manual" and link.ctl_connected and state.conn.mode == "sitl":   # USB remote is SITL-only
                    try:
                        await run_in_threadpool(link.send_manual_control, float(m.get("roll", 0)), float(m.get("pitch", 0)),
                                                float(m.get("throttle", 0)), float(m.get("yaw", 0)), int(m.get("buttons", 0)),
                                                [float(v) for v in (m.get("aux") or [])])
                    except Exception as e:
                        state.log(f"[joystick] send failed: {e}")

        rx_task = asyncio.create_task(receive_loop())
        try:
            await websocket.send_text(json.dumps({"type": "airframe", "airframe": sim.airframe.to_dict()}))
            while True:
                now = time.time()
                msg: dict[str, Any] = {"type": "state", "state": sim.snapshot()}
                if now - last_status > 0.5:
                    msg["status"] = status_dict()
                    last_status = now
                new_logs = [e for e in state.log_buffer if e[0] > last_log]
                if new_logs:
                    msg["log"] = new_logs
                    last_log = new_logs[-1][0]
                await websocket.send_text(json.dumps(msg))
                await asyncio.sleep(1 / 30)
        except WebSocketDisconnect:
            pass
        except Exception:
            pass
        finally:
            rx_task.cancel()

    return app
