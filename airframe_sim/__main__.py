"""AIRFRAME_SIMULATOR entry point.

  python -m airframe_sim --mode sitl --launch-px4          # run PX4 SITL + sim + UI
  python -m airframe_sim --mode hitl --serial /dev/cu.usbmodem01   # Pixhawk over USB
"""
from __future__ import annotations

import argparse
import atexit
import glob
import os
import signal
import subprocess
import sys
import threading
import time
import webbrowser
from collections import deque
from pathlib import Path

import uvicorn

from .airframe import Airframe, quad_x
from .link import PX4Link
from .sensors import Home
from .simulator import Simulator
from .server import AppState, build_app
from . import param_meta

PROJECT_DIR = Path(__file__).resolve().parent.parent


def guess_serial() -> str | None:
    cands = sorted(glob.glob("/dev/cu.usbmodem*")) + sorted(glob.glob("/dev/ttyACM*"))
    return cands[0] if cands else None


def free_px4_instance(start: int = 0) -> int:
    """PX4 SITL holds an flock on /tmp/px4_lock-<instance>; find the first instance nobody holds."""
    import fcntl
    for i in range(start, start + 16):
        path = f"/tmp/px4_lock-{i}"
        try:
            fd = os.open(path, os.O_RDWR | os.O_CREAT, 0o644)
        except OSError:
            continue
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            fcntl.flock(fd, fcntl.LOCK_UN)
            os.close(fd)
            return i
        except OSError:
            os.close(fd)
    return start


def launch_px4(px4_dir: str, model: str, log, instance: int = 0, rootfs: str | None = None) -> subprocess.Popen:
    build = Path(px4_dir) / "build" / "px4_sitl_default"
    binary = build / "bin" / "px4"
    if not binary.is_file():
        raise SystemExit(f"PX4 SITL binary not found at {binary}. Build it with: cd {px4_dir} && make px4_sitl_default")
    # PX4's rcS is a shell script that splices the working directory into paths unquoted,
    # so the working directory must not contain spaces. Default: ~/.airframe_sim/px4_rootfs
    rootfs = Path(rootfs or os.path.expanduser("~/.airframe_sim/px4_rootfs"))
    if " " in str(rootfs):
        raise SystemExit(f"PX4 working directory must not contain spaces: {rootfs} (use --px4-rootfs)")
    rootfs.mkdir(parents=True, exist_ok=True)
    env = dict(os.environ)
    env["PX4_SIM_MODEL"] = model
    env.setdefault("PX4_SIMULATOR", "mavlink")
    cmd = [str(binary), "-d", "-i", str(instance), "-w", str(rootfs), str(build / "etc")]
    log(f"[px4] launching: PX4_SIM_MODEL={model} {' '.join(cmd)}")
    proc = subprocess.Popen(cmd, cwd=str(rootfs), env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                            text=True, bufsize=1, start_new_session=True)

    def pump():
        for line in proc.stdout:
            log(f"[px4] {line.rstrip()}")
        log(f"[px4] exited with code {proc.poll()}")

    threading.Thread(target=pump, daemon=True).start()
    return proc


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="airframe_sim", description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--mode", choices=["sitl", "hitl"], default="sitl")
    ap.add_argument("--tcp", default="0.0.0.0:4560", help="SITL: address to listen on for PX4 (default 0.0.0.0:4560)")
    ap.add_argument("--ctl", default=None, help="SITL: pymavlink address of PX4's onboard MAVLink link for params/commands "
                                               "(default udpin:127.0.0.1:14540+instance)")
    ap.add_argument("--serial", default=None, help="HITL: Pixhawk serial port (default: first /dev/cu.usbmodem*)")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--qgc", default="127.0.0.1:14550", help="HITL: forward vehicle MAVLink to QGC at this UDP address ('' to disable)")
    ap.add_argument("--airframe", default=None, help="airframe JSON to load (default: airframes/quad_x.json or built-in Quad X)")
    ap.add_argument("--px4-dir", default=os.path.expanduser("~/PX4-Autopilot"))
    ap.add_argument("--launch-px4", action="store_true", help="SITL: start PX4 SITL from --px4-dir automatically")
    ap.add_argument("--px4-model", default="none_iris", help="PX4_SIM_MODEL for --launch-px4 (default none_iris)")
    ap.add_argument("--px4-rootfs", default=None, help="PX4 SITL working dir (params, logs); default ~/.airframe_sim/px4_rootfs")
    ap.add_argument("--px4-instance", type=int, default=None,
                    help="SITL instance number (default: first free); the simulator port is 4560 + instance")
    ap.add_argument("--param-meta", default=None, help="parameters.json(.xz) for parameter descriptions")
    ap.add_argument("--rate", type=float, default=250.0, help="sensor rate Hz (default 250)")
    ap.add_argument("--speed", type=float, default=1.0, help="SITL real-time factor, 0 = unthrottled")
    ap.add_argument("--no-lockstep", action="store_true")
    ap.add_argument("--home", default="55.6761,12.5683,10", help="lat,lon,alt of the home point")
    ap.add_argument("--http", default="127.0.0.1:8080", help="UI address")
    ap.add_argument("--no-browser", action="store_true")
    args = ap.parse_args(argv)

    log_buffer: deque = deque(maxlen=1000)

    def log(s: str) -> None:
        print(s, flush=True)
        log_buffer.append((time.time(), s))

    # airframe
    if args.airframe:
        airframe = Airframe.load(args.airframe)
    elif (PROJECT_DIR / "airframes" / "quad_x.json").is_file():
        airframe = Airframe.load(PROJECT_DIR / "airframes" / "quad_x.json")
    else:
        airframe = quad_x()
    log(f"[sim] airframe: {airframe.name} ({len(airframe.rotors)} rotors, {airframe.mass:.2f} kg)")

    # link
    if args.mode == "hitl":
        serial = args.serial or guess_serial()
        if not serial:
            raise SystemExit("HITL: no serial port found; plug in the Pixhawk or pass --serial")
        link = PX4Link("hitl", serial, baud=args.baud, qgc_proxy=args.qgc or None, log=log)
    else:
        instance = args.px4_instance
        if instance is None:
            instance = free_px4_instance() if args.launch_px4 else 0
            if instance:
                log(f"[px4] SITL instance 0 is busy (another PX4 is running); using instance {instance}")
        args.px4_instance = instance
        if args.tcp == "0.0.0.0:4560" and instance:
            args.tcp = f"0.0.0.0:{4560 + instance}"
        ctl = args.ctl or f"udpin:127.0.0.1:{14540 + instance}"
        link = PX4Link("sitl", args.tcp, ctl_address=ctl, log=log)
    link.open()

    lat, lon, alt = (float(x) for x in args.home.split(","))
    simulator = Simulator(airframe, link, sensor_rate=args.rate, speed=args.speed,
                          lockstep=(False if args.no_lockstep else None), home=Home(lat, lon, alt), log=log)
    simulator.start()

    state = AppState(simulator, link, args, log_buffer, log)
    state.meta, state.meta_source = param_meta.load_local(args.px4_dir, args.param_meta)
    log(f"[params] metadata: {len(state.meta)} entries from {state.meta_source or 'nowhere (use Fetch from vehicle)'}")

    if args.mode == "sitl" and args.launch_px4:
        state.px4_process = launch_px4(args.px4_dir, args.px4_model, log, instance=args.px4_instance, rootfs=args.px4_rootfs)

    # after the link comes up, download the parameter list in the background
    def auto_fetch():
        while not link.ctl_connected:
            time.sleep(0.5)
        time.sleep(1.0)
        try:
            link.fetch_all_params()
        except Exception as e:
            log(f"[params] fetch failed: {e}")

    threading.Thread(target=auto_fetch, daemon=True).start()

    app = build_app(state)
    host, port = args.http.rsplit(":", 1)
    if not args.no_browser:
        threading.Timer(1.0, lambda: webbrowser.open(f"http://{host}:{port}/")).start()

    def stop_px4():
        proc = state.px4_process
        if proc and proc.poll() is None:
            try:
                os.killpg(proc.pid, signal.SIGINT)
                proc.wait(3)
            except (subprocess.TimeoutExpired, ProcessLookupError, PermissionError):
                try:
                    os.killpg(proc.pid, signal.SIGKILL)
                except Exception:
                    pass

    def shutdown(*_):
        simulator.stop()
        link.close()
        stop_px4()

    atexit.register(stop_px4)
    try:
        uvicorn.run(app, host=host, port=int(port), log_level="warning")
    finally:
        shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
