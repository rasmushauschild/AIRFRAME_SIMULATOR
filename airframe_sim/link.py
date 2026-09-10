"""MAVLink link to PX4: the Simulator MAVLink API plus the parameter protocol.

One class serves both modes, with two logical channels:
  hil  - HIL_SENSOR / HIL_GPS out, HIL_ACTUATOR_CONTROLS in
  ctl  - parameters, commands, vehicle status (HEARTBEAT, STATUSTEXT)

  SITL  - hil: we listen on TCP 4560(+instance), PX4's simulator_mavlink connects to us and lockstep is driven
               by our HIL_SENSOR timestamps. That module speaks *only* the HIL messages, so
          ctl: a UDP client on PX4's onboard MAVLink instance (PX4 sends to 14540+instance, listens on 14580+instance).
  HITL  - hil and ctl are the same connection: the Pixhawk's USB serial port. Everything the vehicle says is
          optionally proxied to QGroundControl over UDP so QGC can be used at the same time.

One reader thread per connection drains it; the simulator loop and the parameter client only write.
"""
from __future__ import annotations

import os
import socket
import struct
import threading
import time
from collections import deque
from typing import Callable

os.environ.setdefault("MAVLINK20", "1")
os.environ.setdefault("MAVLINK_DIALECT", "common")
from pymavlink import mavutil  # noqa: E402
from pymavlink.dialects.v20 import common as mavlink  # noqa: E402

PARAM_TYPE_INT32 = mavlink.MAV_PARAM_TYPE_INT32
PARAM_TYPE_REAL32 = mavlink.MAV_PARAM_TYPE_REAL32


def _float_to_int_bits(f: float) -> int:
    return struct.unpack("<i", struct.pack("<f", f))[0]


def _int_bits_to_float(i: int) -> float:
    return struct.unpack("<f", struct.pack("<i", int(i)))[0]


class PX4Link:
    def __init__(self, mode: str, address: str = "0.0.0.0:4560", baud: int = 921600,
                 qgc_proxy: str | None = None, ctl_address: str = "udpin:127.0.0.1:14540",
                 log: Callable[[str], None] | None = None):
        assert mode in ("sitl", "hitl")
        self.mode = mode
        self.address = address
        self.ctl_address = ctl_address if mode == "sitl" else address
        self.baud = baud
        self.qgc_proxy = qgc_proxy
        self.log = log or (lambda s: print(s, flush=True))
        self.target_system = 1
        self.target_component = 1

        self.conn = None          # hil channel
        self.ctl = None           # control channel (== conn in HITL)
        self.connected = False    # hil channel alive
        self.ctl_connected = False
        self._write_lock = threading.Lock()      # hil channel
        self._ctl_lock = threading.Lock()        # ctl channel
        self._stop = threading.Event()

        # actuators
        self.actuators = [0.0] * 16
        self.actuator_seq = 0
        self.actuator_armed = False
        self.actuator_time_usec = 0
        self._act_cond = threading.Condition()

        # vehicle status from HEARTBEAT
        self.heartbeat_time = 0.0
        self.armed = False
        self.custom_mode = 0
        self.mav_type = 0
        self.hil_enabled = False

        # parameters
        self.params: dict[str, dict] = {}     # name -> {"value":, "type":, "index":}
        self.param_count = 0
        self._param_event = threading.Event()
        self._param_set_result: dict[str, dict] = {}

        self.statustext = deque(maxlen=200)
        self.firmware: dict = {}      # from AUTOPILOT_VERSION
        self.rx_count = 0
        self.last_rx_time = 0.0
        self._qgc_sock = None
        self._qgc_addr = None
        self._readers: list[threading.Thread] = []
        self.ctl_rx_time = 0.0

    # ----------------------------------------------------------------- lifecycle
    def open(self) -> None:
        if self.mode == "sitl":
            host, port = self.address.rsplit(":", 1)
            self.log(f"[link] SITL: listening for PX4 simulator link on tcp://{host}:{port}")
            self.conn = mavutil.mavlink_connection(f"tcpin:{host}:{port}", source_system=1, source_component=51,
                                                   dialect="common")
            self.log(f"[link] SITL: parameters/commands via PX4 onboard link {self.ctl_address}")
            self.ctl = mavutil.mavlink_connection(self.ctl_address, source_system=255, source_component=190,
                                                  dialect="common")
        else:
            self.log(f"[link] HITL: opening serial {self.address} @ {self.baud}")
            self.conn = mavutil.mavlink_connection(self.address, baud=self.baud, source_system=1, source_component=51,
                                                   dialect="common", autoreconnect=True)
            if self.qgc_proxy:
                host, port = self.qgc_proxy.rsplit(":", 1)
                self._qgc_target = (host, int(port))
                self._qgc_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self._qgc_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                self._qgc_sock.bind(("0.0.0.0", 0))
                self._qgc_sock.setblocking(False)
                self.log(f"[link] proxying vehicle MAVLink to QGroundControl at udp://{host}:{port}")
            self.ctl = self.conn
            self._ctl_lock = self._write_lock
        self._readers = [threading.Thread(target=self._read_loop, args=(self.conn, True, self.ctl is self.conn),
                                          name="mavlink-hil-reader", daemon=True)]
        if self.ctl is not self.conn:
            self._readers.append(threading.Thread(target=self._read_loop, args=(self.ctl, False, True),
                                                  name="mavlink-ctl-reader", daemon=True))
        for t in self._readers:
            t.start()

    def close(self) -> None:
        self._stop.set()
        for c in {id(self.conn): self.conn, id(self.ctl): self.ctl}.values():
            try:
                if c:
                    c.close()
            except Exception:
                pass

    # ------------------------------------------------------------------- write
    def send_hil_sensor(self, s: dict) -> None:
        with self._write_lock:
            self.conn.mav.hil_sensor_send(**s)

    def send_hil_gps(self, g: dict) -> None:
        with self._write_lock:
            self.conn.mav.hil_gps_send(**g)

    def send_hil_state_quaternion(self, st: dict) -> None:
        with self._write_lock:
            self.conn.mav.hil_state_quaternion_send(**st)

    def send_heartbeat(self) -> None:
        with self._write_lock:
            self.conn.mav.heartbeat_send(mavlink.MAV_TYPE_GENERIC, mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        if self.ctl is not self.conn:
            with self._ctl_lock:
                self.ctl.mav.heartbeat_send(mavlink.MAV_TYPE_GCS, mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)

    def send_command_long(self, command: int, *params: float) -> None:
        p = list(params) + [0.0] * (7 - len(params))
        with self._ctl_lock:
            self.ctl.mav.command_long_send(self.target_system, self.target_component, command, 0, *p[:7])

    # --------------------------------------------------------------- lockstep
    def wait_for_actuators(self, seq_before: int, timeout: float) -> bool:
        """Block until a HIL_ACTUATOR_CONTROLS newer than seq_before arrives (lockstep)."""
        with self._act_cond:
            if self.actuator_seq > seq_before:
                return True
            self._act_cond.wait_for(lambda: self.actuator_seq > seq_before, timeout=timeout)
            return self.actuator_seq > seq_before

    # ------------------------------------------------------------------ reader
    def _read_loop(self, conn, is_hil: bool, is_ctl: bool) -> None:
        name = "simulator link" if (is_hil and not is_ctl) else ("vehicle link" if is_ctl and is_hil else "control link")
        while not self._stop.is_set():
            try:
                msg = conn.recv_match(blocking=True, timeout=0.05)
            except Exception as e:  # serial unplugged etc.
                if self._stop.is_set():
                    break
                self.log(f"[link] read error on {name}: {e}")
                time.sleep(0.5)
                continue
            if is_hil:
                self._poll_qgc()
            if msg is None:
                if is_hil and self.connected and time.time() - self.last_rx_time > 5.0 and self.mode == "sitl":
                    self.connected = False
                    self.log(f"[link] {name} lost")
                if is_ctl and self.ctl_connected and time.time() - self.ctl_rx_time > 5.0:
                    self.ctl_connected = False
                    self.log(f"[link] {name} lost")
                continue
            self.rx_count += 1
            if is_hil:
                self.last_rx_time = time.time()
                if not self.connected:
                    self.connected = True
                    self.log(f"[link] PX4 {name} up (sysid {msg.get_srcSystem()} compid {msg.get_srcComponent()})")
            if is_ctl:
                self.ctl_rx_time = time.time()
                if not self.ctl_connected:
                    self.ctl_connected = True
                    self.target_system = msg.get_srcSystem()
                    self.log(f"[link] PX4 {name} up (sysid {msg.get_srcSystem()} compid {msg.get_srcComponent()})")
            t = msg.get_type()
            if t == "HIL_ACTUATOR_CONTROLS":
                with self._act_cond:
                    self.actuators = list(msg.controls)
                    self.actuator_armed = bool(msg.mode & mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                    self.actuator_time_usec = msg.time_usec
                    self.actuator_seq += 1
                    self._act_cond.notify_all()
            elif t == "HEARTBEAT" and is_ctl:
                if msg.get_srcComponent() == 1 and msg.autopilot == mavlink.MAV_AUTOPILOT_PX4:
                    self.target_system = msg.get_srcSystem()
                    self.heartbeat_time = time.time()
                    self.armed = bool(msg.base_mode & mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                    self.hil_enabled = bool(msg.base_mode & mavlink.MAV_MODE_FLAG_HIL_ENABLED)
                    self.custom_mode = msg.custom_mode
                    self.mav_type = msg.type
            elif t == "PARAM_VALUE":
                self._handle_param_value(msg)
            elif t == "AUTOPILOT_VERSION":
                v = msg.flight_sw_version
                ver = f"{(v >> 24) & 0xFF}.{(v >> 16) & 0xFF}.{(v >> 8) & 0xFF}"
                typ = {0: "dev", 64: "alpha", 128: "beta", 192: "rc", 255: "release"}.get(v & 0xFF, "")
                gh = bytes(msg.flight_custom_version).hex()[:8]
                self.firmware = {"version": f"{ver} {typ}".strip(), "git": gh, "board": msg.board_version,
                                 "vendor_id": msg.vendor_id, "product_id": msg.product_id}
                self.log(f"[link] firmware PX4 v{ver} {typ} ({gh})")
            elif t == "STATUSTEXT":
                text = msg.text if isinstance(msg.text, str) else msg.text.decode(errors="ignore")
                self.statustext.append((time.time(), msg.severity, text))
                self.log(f"[px4] {text}")
            elif t == "BAD_DATA":
                continue
            # HITL: forward everything the vehicle says to QGC
            if self._qgc_sock is not None and t != "BAD_DATA":
                try:
                    self._qgc_sock.sendto(msg.get_msgbuf(), self._qgc_target)
                except OSError:
                    pass

    def _poll_qgc(self) -> None:
        if self._qgc_sock is None:
            return
        for _ in range(32):
            try:
                data, addr = self._qgc_sock.recvfrom(65535)
            except (BlockingIOError, OSError):
                return
            self._qgc_addr = addr
            with self._write_lock:
                try:
                    self.conn.write(data)
                except Exception:
                    pass

    # -------------------------------------------------------------- parameters
    def _decode_param(self, msg) -> float | int:
        if msg.param_type in (mavlink.MAV_PARAM_TYPE_INT32, mavlink.MAV_PARAM_TYPE_INT16, mavlink.MAV_PARAM_TYPE_INT8,
                              mavlink.MAV_PARAM_TYPE_UINT32, mavlink.MAV_PARAM_TYPE_UINT16, mavlink.MAV_PARAM_TYPE_UINT8):
            return _float_to_int_bits(msg.param_value)
        return float(msg.param_value)

    def _handle_param_value(self, msg) -> None:
        name = msg.param_id if isinstance(msg.param_id, str) else msg.param_id.decode(errors="ignore")
        name = name.rstrip("\x00")
        value = self._decode_param(msg)
        self.params[name] = {"value": value, "type": int(msg.param_type), "index": int(msg.param_index)}
        self.param_count = int(msg.param_count)
        self._param_set_result[name] = self.params[name]
        self._param_event.set()

    def param_request_list(self) -> None:
        with self._ctl_lock:
            self.ctl.mav.param_request_list_send(self.target_system, self.target_component)

    def param_request_read(self, name: str, index: int = -1) -> None:
        with self._ctl_lock:
            self.ctl.mav.param_request_read_send(self.target_system, self.target_component,
                                                 name.encode()[:16], index)

    def fetch_all_params(self, timeout: float = 30.0) -> dict[str, dict]:
        """Blocking full parameter download with retry of missing indices."""
        self.params.clear()
        self.param_count = 0
        self.param_request_list()
        deadline = time.time() + timeout
        last_n = -1
        stall = time.time()
        while time.time() < deadline:
            self._param_event.wait(0.5)
            self._param_event.clear()
            n = len(self.params)
            if self.param_count and n >= self.param_count:
                break
            if n != last_n:
                last_n = n
                stall = time.time()
            elif time.time() - stall > 1.5:
                # retry missing indices
                have = {p["index"] for p in self.params.values()}
                missing = [i for i in range(self.param_count) if i not in have][:50]
                if not missing:
                    break
                for i in missing:
                    self.param_request_read("", i)
                stall = time.time()
        self.log(f"[link] parameters: {len(self.params)}/{self.param_count}")
        return self.params

    def get_param(self, name: str, timeout: float = 2.0) -> dict | None:
        self._param_set_result.pop(name, None)
        self.param_request_read(name)
        t0 = time.time()
        while time.time() - t0 < timeout:
            if name in self._param_set_result:
                return self._param_set_result[name]
            time.sleep(0.01)
        return None

    def set_param(self, name: str, value: float | int, timeout: float = 2.0, retries: int = 3) -> dict:
        """Set a parameter with PX4's byte-wise int encoding; verifies the echoed PARAM_VALUE."""
        info = self.params.get(name)
        if info is None:
            info = self.get_param(name)
        if info is None:
            return {"name": name, "ok": False, "error": "unknown parameter"}
        ptype = info["type"]
        if ptype == PARAM_TYPE_INT32:
            v_int = int(round(value))
            wire = _int_bits_to_float(v_int)
            want = v_int
        else:
            wire = float(value)
            want = float(value)
        for _ in range(retries):
            self._param_set_result.pop(name, None)
            with self._ctl_lock:
                self.ctl.mav.param_set_send(self.target_system, self.target_component, name.encode()[:16], wire, ptype)
            t0 = time.time()
            while time.time() - t0 < timeout:
                res = self._param_set_result.get(name)
                if res is not None:
                    got = res["value"]
                    ok = (got == want) if ptype == PARAM_TYPE_INT32 else abs(float(got) - want) <= 1e-5 * max(1.0, abs(want))
                    if ok:
                        return {"name": name, "ok": True, "value": got}
                    break
                time.sleep(0.005)
        return {"name": name, "ok": False, "error": "no matching PARAM_VALUE ack",
                "value": self._param_set_result.get(name, {}).get("value")}

    def set_params(self, values: dict[str, float | int], progress: Callable[[str, dict], None] | None = None) -> list[dict]:
        results = []
        for k, v in values.items():
            r = self.set_param(k, v)
            results.append(r)
            if progress:
                progress(k, r)
        return results

    # --------------------------------------------------------------- commands
    def request_autopilot_version(self) -> None:
        self.send_command_long(mavlink.MAV_CMD_REQUEST_MESSAGE, float(mavlink.MAVLINK_MSG_ID_AUTOPILOT_VERSION))

    def preflight_storage(self, save: bool = True) -> None:
        """MAV_CMD_PREFLIGHT_STORAGE: 1 = write params to storage."""
        self.send_command_long(mavlink.MAV_CMD_PREFLIGHT_STORAGE, 1.0 if save else 0.0)

    def reboot(self) -> None:
        self.send_command_long(mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 1.0)

    def status(self) -> dict:
        return {
            "mode": self.mode,
            "address": self.address,
            "connected": self.connected and (time.time() - self.last_rx_time < 3.0),
            "ctl_connected": self.ctl_connected and (time.time() - self.ctl_rx_time < 3.0),
            "ctl_address": self.ctl_address,
            "target_system": self.target_system,
            "armed": self.armed,
            "hil_enabled": self.hil_enabled,
            "custom_mode": self.custom_mode,
            "rx_count": self.rx_count,
            "param_count": self.param_count,
            "params_loaded": sum(1 for v in self.params.values() if v["index"] < self.param_count),
            "actuator_seq": self.actuator_seq,
            "qgc_proxy": self.qgc_proxy,
        }
