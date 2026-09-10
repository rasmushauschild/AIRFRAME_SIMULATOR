"""Decode PX4 EVENT messages (MAVLink id 410) into text using PX4's events metadata.

PX4 >= 1.13 reports arming denials, health/estimator problems and mode changes as *events*, not STATUSTEXT.
Each EVENT carries an id and packed arguments; the text template lives in all_events.json, which is generated
by the PX4 build (build/<target>/events/all_events.json) and also served by the vehicle over MAVLink FTP as
/etc/extras/all_events.json.xz.
"""
from __future__ import annotations

import json
import lzma
import re
import struct
from pathlib import Path

CACHE_DIR = Path(__file__).resolve().parent.parent / ".param_cache"
LEVELS = {0: "emergency", 1: "alert", 2: "critical", 3: "error", 4: "warning", 5: "notice", 6: "info", 7: "debug",
          8: "protocol", 9: "disabled"}
_SIZES = {"uint8_t": ("B", 1), "int8_t": ("b", 1), "uint16_t": ("H", 2), "int16_t": ("h", 2), "uint32_t": ("I", 4),
          "int32_t": ("i", 4), "uint64_t": ("Q", 8), "int64_t": ("q", 8), "float": ("f", 4)}
_FMT = re.compile(r"\{(\d+)(?::\.(\d+))?([a-zA-Z/_%]*)\}")
_PROFILE = re.compile(r"<profile name=\"[^\"]*\">.*?</profile>", re.S)
_TAGS = re.compile(r"</?(param|a|b|i)[^>]*>")


def _read_json(path: Path) -> dict:
    data = path.read_bytes()
    if path.suffix == ".xz" or data[:6] == b"\xfd7zXZ\x00":
        data = lzma.decompress(data)
    return json.loads(data)


class EventDecoder:
    def __init__(self):
        self.events: dict[int, dict] = {}
        self.enums: dict[str, dict] = {}
        self.source = ""

    # ------------------------------------------------------------ loading
    def load(self, meta: dict, source: str = "") -> int:
        """Wire event id = (component id << 24) | 24-bit name hash; the JSON stores the hash per component."""
        self.enums = {}
        self.events = {}
        for cid, comp in meta.get("components", {}).items():
            self.enums.update({k.split("::")[-1]: v for k, v in comp.get("enums", {}).items()})
            for gname, g in comp.get("event_groups", {}).items():
                for eid, e in g.get("events", {}).items():
                    e = dict(e)
                    e["group"] = gname
                    e["component"] = int(cid)
                    self.events[(int(cid) << 24) | (int(eid) & 0xFFFFFF)] = e
        self.source = source
        return len(self.events)

    def load_local(self, px4_dir: str | None) -> int:
        cands = []
        if px4_dir:
            b = Path(px4_dir) / "build"
            if b.is_dir():
                for d in sorted(b.iterdir()):
                    cands.append(d / "events" / "all_events.json")
        cands += [CACHE_DIR / "all_events.json.xz", CACHE_DIR / "all_events.json"]
        for c in cands:
            if c.is_file():
                try:
                    return self.load(_read_json(c), str(c))
                except Exception:
                    continue
        return 0

    # ----------------------------------------------------------- decoding
    @staticmethod
    def parse_message(raw: bytes) -> dict | None:
        """raw = the full MAVLink frame pymavlink hands back for unknown ids (v1 or v2)."""
        if not raw:
            return None
        if raw[0] == 0xFD:
            n, payload = raw[1], raw[10:10 + raw[1]]
        elif raw[0] == 0xFE:
            n, payload = raw[1], raw[6:6 + raw[1]]
        else:
            payload = raw
        payload = bytes(payload) + b"\x00" * (53 - len(payload))
        eid, t_ms, seq, dst_comp, dst_sys, levels = struct.unpack_from("<IIHBBB", payload, 0)
        args = payload[13:53]
        return {"id": eid, "time_ms": t_ms, "seq": seq, "level": levels & 0xF, "level_internal": levels >> 4,
                "args": args}

    def _arg_values(self, e: dict, args: bytes) -> list:
        vals = []
        off = 0
        for a in e.get("arguments", []):
            t = a.get("type", "uint8_t").split("::")[-1]
            enum = self.enums.get(t)
            base = enum.get("type", "uint32_t") if enum else t
            code, size = _SIZES.get(base, ("I", 4))
            if off + size > len(args):
                vals.append(None)
                continue
            v = struct.unpack_from("<" + code, args, off)[0]
            off += size
            if enum:
                ent = enum.get("entries", {}).get(str(v))
                if ent:
                    v = ent.get("description") or ent.get("name")
                elif enum.get("is_bitfield") or "component" in t or "mode_group" in t:
                    names = [x.get("name", k) for k, x in enum.get("entries", {}).items() if int(k) and (int(v) & int(k)) == int(k)]
                    v = "|".join(names) if names else v
            vals.append(v)
        return vals

    def format(self, ev: dict) -> dict | None:
        e = self.events.get(ev["id"])
        if e is None:
            return None
        vals = self._arg_values(e, ev["args"])

        def sub(m):
            i = int(m.group(1)) - 1
            prec, unit = m.group(2), m.group(3)
            v = vals[i] if 0 <= i < len(vals) else "?"
            if isinstance(v, float):
                v = f"{v:.{int(prec) if prec else 2}f}"
            unit = (unit or "").replace("_v", "")
            return f"{v}{(' ' + unit) if unit else ''}"

        text = _FMT.sub(sub, e.get("message", e.get("name", "")))
        desc = _TAGS.sub("", _PROFILE.sub("", e.get("description", ""))).strip()
        desc = _FMT.sub(sub, desc)
        return {"id": ev["id"], "name": e.get("name", ""), "group": e.get("group", ""), "level": ev["level"],
                "level_name": LEVELS.get(ev["level"], str(ev["level"])), "text": text, "description": desc,
                "time_ms": ev["time_ms"], "args": [v if isinstance(v, (int, float, str)) or v is None else str(v) for v in vals],
                "arg_names": [a.get("name", "") for a in e.get("arguments", [])]}
