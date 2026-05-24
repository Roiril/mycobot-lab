"""Spatial short-term memory — what the arm "remembers seeing" while no longer
looking that direction.

Like human short-term memory: only the *latest* observation per direction is
kept. Observations decay (marked stale) after a TTL so old data doesn't masquerade
as current state. Persisted to data/spatial_memory.json so it survives server
restarts (but staleness clock keeps ticking).

This is NOT a permanent observation log. It's the working set the arm uses to
plan motion in directions it can't currently see.
"""
from __future__ import annotations
import json, time, pathlib, threading, math
from typing import Optional

DEFAULT_TTL_S = 600  # observations older than 10 min are "stale"
DEFAULT_HALF_LIFE_S = 180  # confidence halves every 3 min

# Direction binning: divide base XY plane into 8 sectors of 45° each, by J1.
# We bin observations by sector so that "looking at J1=85" and "looking at J1=95"
# both update the same memory cell ('right').
SECTORS = [
    ("back",       -22.5,   22.5),
    ("back_right",  22.5,   67.5),
    ("right",       67.5,  112.5),
    ("front_right",112.5,  157.5),
    ("front",     157.5,  180.0),  # split at ±180
    ("front",    -180.0, -157.5),  # other half of 'front'
    ("front_left",-157.5, -112.5),
    ("left",      -112.5,  -67.5),
    ("back_left",  -67.5,  -22.5),
]


def j1_to_sector(j1_deg: float) -> str:
    """Map a J1 angle to its sector name."""
    a = ((j1_deg + 180) % 360) - 180
    for name, lo, hi in SECTORS:
        if lo <= a < hi: return name
    return "back"


EVENTS_MAX = 50  # keep last N events (older drop off)


class SpatialMemory:
    """Thread-safe spatial memory store.

    Two layers:
    - sectors: per-direction LATEST observation (current spatial state)
    - events: chronological log of all observations + annotations (history)
    """
    def __init__(self, path: pathlib.Path, ttl_s: float = DEFAULT_TTL_S,
                 half_life_s: float = DEFAULT_HALF_LIFE_S):
        self.path = path
        self.ttl_s = ttl_s
        self.half_life_s = half_life_s
        self._lock = threading.Lock()
        self._sectors: dict = {}  # sector_name → entry dict
        self._events: list = []   # chronological history
        self._load()

    def _load(self) -> None:
        if not self.path.exists(): return
        try:
            d = json.loads(self.path.read_text(encoding="utf-8"))
            self._sectors = d.get("sectors", {})
            self._events = d.get("events", [])
        except Exception:
            self._sectors = {}; self._events = []

    def _save(self) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        out = {"version": 2, "sectors": self._sectors, "events": self._events[-EVENTS_MAX:],
               "ttl_s": self.ttl_s, "half_life_s": self.half_life_s}
        self.path.write_text(json.dumps(out, indent=2, ensure_ascii=False), encoding="utf-8")

    def events(self, limit: int = 20) -> list:
        with self._lock:
            return list(self._events[-limit:])

    def _push_event(self, kind: str, payload: dict) -> None:
        ev = {"ts": time.time(), "iso": time.strftime("%Y-%m-%d %H:%M:%S"),
              "kind": kind, **payload}
        self._events.append(ev)
        if len(self._events) > EVENTS_MAX * 2:
            self._events = self._events[-EVENTS_MAX:]

    def record(self, *, j1_deg: float, frame_path: Optional[str],
               camera_pose: dict, observer: str,
               description: str = "", objects: Optional[list] = None,
               frames_dir: Optional[pathlib.Path] = None,
               keep_per_sector: int = 3) -> dict:
        """Record an observation. Overwrites any prior entry in the same sector.

        objects: list of {label, position_mm?: [x,y,z], radius_mm?, note?}
        frames_dir: if provided, delete old frames of the same sector keeping
                    only the last `keep_per_sector` jpegs (rotation).
        """
        sector = j1_to_sector(j1_deg)
        entry = {
            "sector": sector,
            "j1_deg": j1_deg,
            "timestamp": time.time(),
            "iso_time": time.strftime("%Y-%m-%d %H:%M:%S"),
            "frame_path": frame_path,
            "camera_pose": camera_pose,  # flange_mm + camera_dir_hint
            "observer": observer,
            "description": description,
            "objects": objects or [],
        }
        with self._lock:
            self._sectors[sector] = entry
            self._push_event("observe", {"sector": sector, "j1_deg": j1_deg,
                                          "frame_path": frame_path, "observer": observer})
            self._save()
        if frames_dir is not None:
            self._rotate_frames(frames_dir, sector, keep_per_sector)
        return entry

    @staticmethod
    def _rotate_frames(frames_dir: pathlib.Path, sector: str, keep: int) -> None:
        """Delete old observe_*_<sector>.jpg files keeping only the newest `keep`."""
        try:
            cands = sorted(frames_dir.glob(f"observe_*_{sector}.jpg"),
                           key=lambda p: p.stat().st_mtime, reverse=True)
            for old in cands[keep:]:
                try: old.unlink()
                except OSError: pass
        except Exception: pass

    def annotate(self, *, j1_deg: float, description: str,
                 objects: Optional[list] = None, observer: Optional[str] = None) -> Optional[dict]:
        """Update an existing observation in-place (e.g. after Shubie views the
        frame and writes a description). Returns the updated entry or None."""
        sector = j1_to_sector(j1_deg)
        with self._lock:
            e = self._sectors.get(sector)
            if e is None: return None
            if description: e["description"] = description
            if objects is not None: e["objects"] = objects
            if observer: e["observer"] = observer
            e["annotated_at"] = time.time()
            self._push_event("annotate", {"sector": sector, "observer": observer or "",
                                           "description_preview": (description or "")[:60],
                                           "n_objects": len(objects or [])})
            self._save()
            return e

    def all(self) -> dict:
        """Return all sectors with computed freshness."""
        now = time.time()
        with self._lock:
            out = {}
            for name, e in self._sectors.items():
                age = now - e["timestamp"]
                # exponential decay confidence
                conf = math.pow(0.5, age / self.half_life_s)
                out[name] = {
                    **e,
                    "age_s": round(age, 1),
                    "stale": age > self.ttl_s,
                    "freshness": round(conf, 3),  # 1.0 = just now, 0.5 = one half-life ago
                }
            return out

    def clear(self) -> None:
        with self._lock:
            self._sectors = {}
            self._save()
