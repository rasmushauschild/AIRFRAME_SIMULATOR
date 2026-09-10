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
                       8: "Rattitude"}
PX4_SUB_MODE_NAMES = {1: "Ready", 2: "Takeoff", 3: "Hold", 4: "Mission", 5: "RTL", 6: "Land", 8: "Follow", 9: "Precland"}


def mode_name(custom_mode: int) -> str:
    main = (custom_mode >> 16) & 0xFF
    sub = (custom_mode >> 24) & 0xFF
    name = PX4_MAIN_MODE_NAMES.get(main, f"mode{main}")
    if main == 4:
        name = PX4_SUB_MODE_NAMES.get(sub, f"Auto{sub}")
    return name


class AppState:
    def __init__(self, simulator, link, args, log_buffer: deque, log):
        self.simulator = simulator
        self.link = link
        self.args = args
        self.log_buffer = log_buffer
        self.log = log
        self.meta: dict[str, dict] = {}
        self.meta_source = ""
        self.px4_process = None
        self.export_log: deque = deque(maxlen=500)


def build_app(state: AppState) -> FastAPI:
    app = FastAPI(title="AIRFRAME_SIMULATOR")
    app.mount("/static", StaticFiles(directory=str(UI_DIR)), name="static")
    sim = state.simulator
    link = state.link

    # -------------------------------------------------------------- pages
    @app.get("/")
    async def index():
        return FileResponse(str(UI_DIR / "index.html"))

    # ------------------------------------------------------------- status
    def status_dict() -> dict:
        s = link.status()
        s["mode_name"] = mode_name(s["custom_mode"])
        s["px4_running"] = state.px4_process is not None and state.px4_process.poll() is None
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
        sim.set_airframe(af, keep_state=keep)
        return {"ok": True, "airframe": af.to_dict(), "problems": af.validate()}

    @app.get("/api/airframes")
    async def list_airframes():
        files = sorted(p.name for p in AIRFRAME_DIR.glob("*.json"))
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
        return {"ok": True, "airframe": af.to_dict(), "problems": af.validate()}

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
        return {"params": params, "current": current, "problems": sim.airframe.validate(),
                "file": sim.airframe.px4_params_file(hitl=link.mode == "hitl")}

    @app.get("/api/px4/export.params")
    async def get_export_file():
        return PlainTextResponse(sim.airframe.px4_params_file(hitl=link.mode == "hitl"),
                                 headers={"Content-Disposition": f'attachment; filename="{sim.airframe.name}.params"'})

    @app.post("/api/px4/push")
    async def push_params(body: dict | None = None):
        body = body or {}
        if not link.ctl_connected:
            return JSONResponse({"ok": False, "error": "PX4 control link not connected"}, status_code=409)
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

        def progress(name, res):
            state.export_log.append({"t": time.time(), **res})

        results = await run_in_threadpool(link.set_params, params, progress)
        ok = all(r["ok"] for r in results)
        if body.get("save", True) and ok:
            link.preflight_storage(True)
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
        return res

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

    return app
