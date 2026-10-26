"""Motion authoring and transport for the SO-101 digital twin.

Programs contain commands, never sampled physics. Hardware observes the same
reference as the simulator, rather than copying simulated deflection/contact.
The host provides one reentrant lock and a small simulator adapter. This class
does not own a serial port; TwinLink is the only hardware writer.
"""
from __future__ import annotations

import copy
import hashlib
import json
import math
import os
from pathlib import Path

from . import profile, safety
from .controller import plan_joint_path

JOINTS = list(profile.JOINT_NAMES) + [profile.GRIPPER_NAME]
LIMITS = list(profile.JOINT_LIMITS) + [profile.GRIPPER_LIMIT_DEG]
MAX_STEPS = 200
MAX_DURATION_S = 600.0
REAL_SPEED_DPS = 15.0


def finite_number(value, label, minimum, maximum):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{label} は数値で指定してください")
    value = float(value)
    if not math.isfinite(value) or not minimum <= value <= maximum:
        raise ValueError(f"{label} は {minimum:g}〜{maximum:g} で指定してください")
    return value


def validate_program(value):
    """Return a canonical, independent version-1 program or reject it."""
    if not isinstance(value, dict) or type(value.get("version")) is not int or value.get("version") != 1:
        raise ValueError("対応している動作ファイルは version 1 です")
    name = value.get("name")
    if not isinstance(name, str) or not name.strip() or len(name.strip()) > 80:
        raise ValueError("動作名は1〜80文字で指定してください")
    steps = value.get("steps")
    if not isinstance(steps, list) or len(steps) > MAX_STEPS:
        raise ValueError(f"姿勢は {MAX_STEPS} 個までです")
    out = []
    total = 0.0
    for i, step in enumerate(steps):
        if not isinstance(step, dict):
            raise ValueError(f"姿勢 {i + 1} の形式が違います")
        label = step.get("label", f"姿勢 {i + 1}")
        if not isinstance(label, str) or not label.strip() or len(label) > 80:
            raise ValueError("姿勢名は1〜80文字で指定してください")
        angles = step.get("angles")
        if not isinstance(angles, list) or len(angles) != len(JOINTS):
            raise ValueError("各姿勢には6軸の角度が必要です")
        angles = [finite_number(a, n, *limit)
                  for a, n, limit in zip(angles, JOINTS, LIMITS)]
        move = finite_number(step.get("move_s"), "移動時間", 0.1, 120.0)
        hold = finite_number(step.get("hold_s"), "待機時間", 0.0, 120.0)
        total += move + hold
        if total > MAX_DURATION_S:
            raise ValueError("動作の合計時間は600秒までです")
        out.append({"label": label.strip(), "angles": angles,
                    "move_s": move, "hold_s": hold})
    return {"version": 1, "name": name.strip(), "steps": out}


def sample_program(program, start, elapsed):
    """Linear joint reference with exact boundary/hold semantics (pure)."""
    previous = list(start)
    offset = 0.0
    for index, step in enumerate(program["steps"]):
        end = offset + step["move_s"] + step["hold_s"]
        if elapsed < end:
            ratio = max(0.0, min(1.0, (elapsed - offset) / step["move_s"]))
            angles = [a + (b - a) * ratio for a, b in zip(previous, step["angles"])]
            return angles, index, False
        previous = list(step["angles"])
        offset = end
    return previous, max(0, len(program["steps"]) - 1), True


def validate_real_path(program, start):
    """Preflight the complete immutable program before any real playback."""
    previous = list(start)
    for i, step in enumerate(program["steps"]):
        speed = max(abs(b - a) / step["move_s"]
                    for a, b in zip(previous, step["angles"]))
        if speed > REAL_SPEED_DPS + 1e-6:
            raise ValueError(f"姿勢 {i + 1} は {speed:.1f}°/s。移動時間を延ばして15°/s以下にしてください")
        for angles in [previous] + plan_joint_path(previous, step["angles"], 1.0):
            ok, reason, _ = safety.check_angles(angles[:5])
            if not ok:
                raise ValueError(f"姿勢 {i + 1} への経路を実行できません: {reason}")
        previous = list(step["angles"])


class MotionLibrary:
    """Name-addressed local programs; names never become filesystem paths."""
    def __init__(self, root):
        self.root = Path(root)

    def _path(self, name):
        if not isinstance(name, str) or not name.strip() or len(name.strip()) > 80:
            raise ValueError("動作名が正しくありません")
        key = hashlib.sha256(name.strip().encode("utf-8")).hexdigest()
        return self.root / (key + ".json")

    def save(self, program):
        program = validate_program(program)
        path = self._path(program["name"])
        self.root.mkdir(parents=True, exist_ok=True)
        tmp = path.with_suffix(".json.tmp")
        try:
            tmp.write_text(json.dumps(program, ensure_ascii=False, indent=2), encoding="utf-8")
            validate_program(json.loads(tmp.read_text(encoding="utf-8")))
            os.replace(tmp, path)
        finally:
            tmp.unlink(missing_ok=True)

    def load(self, name):
        try:
            program = validate_program(json.loads(self._path(name).read_text(encoding="utf-8")))
        except FileNotFoundError:
            raise ValueError("保存した動作が見つかりません") from None
        if program["name"] != name.strip():
            raise ValueError("保存された動作名が一致しません")
        return program

    def list(self):
        if not self.root.exists():
            return []
        out = []
        for path in self.root.glob("*.json"):
            try:
                p = validate_program(json.loads(path.read_text(encoding="utf-8")))
                if path == self._path(p["name"]):
                    out.append({"name": p["name"], "steps": len(p["steps"])})
            except (ValueError, OSError, TypeError):
                continue
        return sorted(out, key=lambda item: item["name"])


class TwinStudio:
    """Uses the simulator lock; callbacks must never acquire it from hardware."""
    def __init__(self, sim, hardware, library):
        self.sim, self.hardware = sim, hardware
        self.library = MotionLibrary(library)
        self.program = {"version": 1, "name": "新しい動作", "steps": []}
        self.revision = 0
        self.saved_revision = -1
        self.saved_names = self.library.list()
        self.playback = {"playing": False, "index": 0, "elapsed": 0.0,
                         "duration": 0.0, "reason": "待機中"}
        self._run = None
        self._was_following = False
        self._last_target = None

    def goals(self):
        return self.sim.target_degrees()

    def assert_owner(self, client_id):
        state = self.hardware.snapshot()
        if state["following"] and state.get("owner") != client_id:
            raise ValueError("別の画面が実機を操作しています。停止はどの画面からでもできます")

    def assert_physics_editable(self):
        if self.hardware.snapshot()["following"]:
            raise ValueError("実機追従中は仮想の設定変更・一時停止・リセットを使えません")

    def stop_playback(self, reason="停止しました"):
        self.playback["playing"] = False
        self.playback["reason"] = reason
        self._run = None

    def tick(self):
        """Called by the physics worker under sim.lock, not by browser polling."""
        following = self.hardware.snapshot()["following"]
        if self._was_following and not following:
            self.stop_playback("実機追従が終了したため停止しました")
        self._was_following = following
        if self._run:
            run = self._run
            elapsed = max(0.0, self.sim.sim_time() - run["t0"])
            angles, index, done = sample_program(run["program"], run["start"], elapsed)
            self.sim.set_goals(dict(zip(JOINTS, angles)))
            self.playback.update(index=index, elapsed=min(elapsed, self.playback["duration"]))
            if done:
                self.stop_playback("最後の姿勢まで指令しました")
        goals = self.goals()
        if following and goals != self._last_target:
            try:
                self.hardware.set_target(goals)
                self._last_target = goals
            except ValueError as e:
                self.hardware.command("stop", {})
                self.stop_playback(f"実機への指令を停止しました: {e}")
        if not following:
            self._last_target = None

    def snapshot(self):
        with self.sim.lock:
            hw = self.hardware.snapshot()
            measured = [hw["joints"].get(n, {}).get("q") for n in JOINTS]
            if (hw["connected"] and hw["age_s"] is not None and hw["age_s"] <= 0.5
                    and all(v is not None and math.isfinite(v) for v in measured)):
                goals = self.goals()
                hw["max_error_deg"] = max(abs(goals[n] - q) for n, q in zip(JOINTS, measured))
            else:
                hw["max_error_deg"] = None
            return {"program": copy.deepcopy(self.program), "revision": self.revision,
                    "saved_revision": self.saved_revision, "library": copy.deepcopy(self.saved_names),
                    "playback": dict(self.playback), "hardware": hw}

    def command(self, cmd):
        if not isinstance(cmd, dict):
            raise ValueError("操作をオブジェクトで指定してください")
        op, client = cmd.get("op"), cmd.get("client_id")
        # Do not wait for serial I/O while holding the simulation lock. Stop
        # invalidates the link's command generation before its worker holds.
        if op == "stop":
            result = self.hardware.command("stop", {})
            with self.sim.lock:
                self.stop_playback()
            return {"ok": True, **self.snapshot(), **(result or {})}
        if op in ("hardware_connect", "hardware_disconnect", "heartbeat"):
            if op == "hardware_disconnect":
                self.assert_owner(client)
                with self.sim.lock:
                    self.stop_playback()
            name = {"hardware_connect": "connect", "hardware_disconnect": "disconnect",
                    "heartbeat": "heartbeat"}[op]
            result = self.hardware.command(name, cmd)
            return {"ok": True, **self.snapshot(), **(result or {})}
        with self.sim.lock:
            self.assert_owner(client)
            expected = cmd.get("expected_revision")
            if expected is not None and (type(expected) is not int or expected != self.revision):
                raise ValueError("別の画面で動作が更新されました。下書きをJSONへ書き出してから保存した動作を読み直してください")
            if op in ("program", "capture", "load") and self.playback["playing"]:
                raise ValueError("再生を停止してから動作を編集してください")
            if op == "program":
                program = validate_program(cmd.get("program"))
                if program != self.program:
                    self.program = program
                    self.revision += 1
            elif op == "capture":
                step = {"label": cmd.get("label") or f"姿勢 {len(self.program['steps']) + 1}",
                        "angles": [self.goals()[n] for n in JOINTS],
                        "move_s": cmd.get("move_s", 2.0), "hold_s": cmd.get("hold_s", 0.5)}
                self.program = validate_program({**self.program, "steps": self.program["steps"] + [step]})
                self.revision += 1
            elif op == "save":
                self.library.save(self.program)
                self.saved_revision = self.revision
                self.saved_names = self.library.list()
            elif op == "load":
                self.program = self.library.load(cmd.get("name"))
                self.revision += 1
                self.saved_revision = self.revision
            elif op == "play":
                if not self.program["steps"]:
                    raise ValueError("姿勢を追加してから再生してください")
                if self.playback["playing"]:
                    raise ValueError("動作は既に再生中です")
                start = [self.goals()[n] for n in JOINTS]
                if self.hardware.snapshot()["following"]:
                    if self.saved_revision != self.revision:
                        raise ValueError("実機で再生する動作を先に保存してください")
                    validate_real_path(self.program, start)
                self.sim.set_run(paused=False)
                self._run = {"program": copy.deepcopy(self.program), "start": start,
                             "t0": self.sim.sim_time()}
                self.playback = {"playing": True, "index": 0, "elapsed": 0.0,
                                 "duration": sum(s["move_s"] + s["hold_s"] for s in self.program["steps"]),
                                 "reason": "再生中"}
            elif op == "import_actual":
                self.assert_physics_editable()
                hw = self.hardware.snapshot()
                if not hw["connected"] or hw["age_s"] is None or hw["age_s"] > 0.5:
                    raise ValueError("実機の新しい計測値が必要です。接続を確認してください")
                values = [hw["joints"].get(n, {}).get("q") for n in JOINTS]
                if any(v is None or not math.isfinite(v) for v in values):
                    raise ValueError("全軸の校正済み角度を取得できません")
                self.stop_playback("実機姿勢を仮想へ取り込みました")
                self.sim.reset_to_measured(values)
            elif op == "follow":
                if cmd.get("ready") is not True:
                    raise ValueError("台座の固定・周囲・停止手段を確認してください")
                if self.playback["playing"]:
                    raise ValueError("再生を停止してから追従を開始してください")
                if self.sim.paused or self.sim.rtf != 1.0:
                    raise ValueError("実機追従は仮想の再生速度1×・再開状態で開始してください")
                # Capture goals atomically with the mode transition. Hardware
                # never calls back into the simulator, so lock ordering is fixed.
                result = self.hardware.command("follow", {**cmd, "goals": self.goals()})
                self._was_following = True
                return {"ok": True, **self.snapshot(), **(result or {})}
            else:
                raise ValueError("対応していない操作です")
            return {"ok": True, **self.snapshot()}
