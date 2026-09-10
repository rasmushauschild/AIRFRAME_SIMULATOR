"""PX4 parameter metadata (descriptions, units, ranges, enums) for the parameter editor.

Sources, in order of preference:
  1. an explicit --param-meta file (parameters.json or parameters.json.xz)
  2. the PX4 build tree: <PX4_DIR>/build/<target>/parameters.json
  3. downloaded from the vehicle over MAVLink FTP (/etc/extras/parameters.json.xz), cached locally
"""
from __future__ import annotations

import json
import lzma
import os
from pathlib import Path

CACHE_DIR = Path(__file__).resolve().parent.parent / ".param_cache"


def _read_json(path: Path) -> dict:
    data = path.read_bytes()
    if path.suffix == ".xz" or data[:6] == b"\xfd7zXZ\x00":
        data = lzma.decompress(data)
    return json.loads(data)


def flatten(meta: dict) -> dict[str, dict]:
    """PX4 parameters.json -> {name: {group, short, long, type, unit, min, max, increment, decimal, values, bitmask, reboot}}

    Handles both layouts PX4 has used: a flat list of parameters each carrying a "group" key,
    and a list of groups each holding a "parameters" list.
    """
    out: dict[str, dict] = {}

    def add(p: dict, gname: str) -> None:
        name = p.get("name")
        if not name:
            return
        out[name] = {
            "group": p.get("group", gname),
            "category": p.get("category", ""),
            "short": p.get("shortDesc", ""),
            "long": p.get("longDesc", ""),
            "type": p.get("type", ""),
            "unit": p.get("units", ""),
            "min": p.get("min"),
            "max": p.get("max"),
            "increment": p.get("increment"),
            "decimal": p.get("decimalPlaces"),
            "default": p.get("default"),
            "values": p.get("values"),      # [{value, description}]
            "bitmask": p.get("bitmask"),    # [{index, description}]
            "reboot": bool(p.get("rebootRequired", False)),
        }

    for entry in meta.get("parameters", []):
        if "parameters" in entry and isinstance(entry["parameters"], list):
            for p in entry["parameters"]:
                add(p, entry.get("group", ""))
        else:
            add(entry, entry.get("group", ""))
    return out


def find_local(px4_dir: str | None, explicit: str | None = None) -> Path | None:
    candidates = []
    if explicit:
        candidates.append(Path(explicit))
    if px4_dir:
        b = Path(px4_dir) / "build"
        if b.is_dir():
            for d in sorted(b.iterdir()):
                candidates.append(d / "parameters.json")
    candidates.append(CACHE_DIR / "parameters.json")
    candidates.append(CACHE_DIR / "parameters.json.xz")
    for c in candidates:
        if c.is_file():
            return c
    return None


def load_local(px4_dir: str | None, explicit: str | None = None) -> tuple[dict[str, dict], str]:
    p = find_local(px4_dir, explicit)
    if p is None:
        return {}, ""
    try:
        return flatten(_read_json(p)), str(p)
    except Exception as e:
        return {}, f"failed to read {p}: {e}"


def fetch_extra(link, name: str, log=print) -> tuple[Path | None, str]:
    """Download /etc/extras/<name> from the vehicle via MAVLink FTP into the cache dir."""
    try:
        from pymavlink import mavftp
    except Exception as e:
        return None, f"pymavlink mavftp unavailable: {e}"
    if getattr(link, "mode", "") == "hitl":
        return None, ("MAVLink FTP over the shared USB link is not supported while the simulation is running; "
                      "metadata is taken from the local PX4 build instead (build/<target>/parameters.json, events/all_events.json)")
    CACHE_DIR.mkdir(exist_ok=True)
    local = CACHE_DIR / name
    try:
        ftp = mavftp.MAVFTP(link.ctl, target_system=link.target_system, target_component=link.target_component)
        ret = ftp.cmd_get([f"/etc/extras/{name}", str(local)])
        ftp.process_ftp_reply("OpenFileRO", timeout=60)
        if ret is not None and hasattr(ret, "error_code") and ret.error_code != 0:
            return None, f"ftp error {ret.display_message()}"
        if not local.is_file() or local.stat().st_size == 0:
            return None, "ftp download produced no file"
        return local, str(local)
    except Exception as e:
        return None, f"ftp download failed: {e}"


def fetch_from_vehicle(link, log=print) -> tuple[dict[str, dict], str]:
    """Download /etc/extras/parameters.json.xz via MAVLink FTP."""
    local, msg = fetch_extra(link, "parameters.json.xz", log)
    if local is None:
        return {}, msg
    try:
        return flatten(_read_json(local)), str(local)
    except Exception as e:
        return {}, f"failed to read {local}: {e}"
