"""Deterministic authoring/transport tests, without hardware or MuJoCo."""
import copy
import json
import pathlib
import sys
import tempfile
import threading
import unittest
from unittest.mock import patch

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "src"))
from robots.so101.twin_studio import (JOINTS, MotionLibrary, TwinStudio,
                                     sample_program, validate_program, validate_real_path)


def program():
    return {"version": 1, "name": "挨拶", "steps": [
        {"label": "向く", "angles": [10, 0, 0, 0, 0, 0], "move_s": 2, "hold_s": 1},
        {"label": "戻る", "angles": [0, 0, 0, 0, 0, 0], "move_s": 2, "hold_s": 0}]}


class Sim:
    def __init__(self):
        self.lock = threading.RLock()
        self.time = 0
        self.target = dict.fromkeys(JOINTS, 0.0)
        self.paused = False
        self.rtf = 1.0

    def target_degrees(self):
        return dict(self.target)

    def set_goals(self, values):
        self.target.update(values)

    def sim_time(self):
        return self.time

    def set_run(self, paused=None):
        self.paused = bool(paused)

    def reset_to_measured(self, values):
        self.target = dict(zip(JOINTS, values))


class Link:
    def __init__(self):
        self.state = {"mode": "mock", "following": False, "owner": None,
                      "connected": True, "age_s": 0.0,
                      "joints": {n: {"q": 0} for n in JOINTS}}
        self.writes = []
        self.commands = []

    def snapshot(self):
        return copy.deepcopy(self.state)

    def set_target(self, values):
        self.writes.append(dict(values))

    def command(self, op, payload):
        self.commands.append(op)
        if op == "follow":
            self.state.update(following=True, owner=payload["client_id"])
            return {"lease": "secret"}
        if op == "stop":
            self.state.update(following=False, owner=None)
        return {}


class ProgramTests(unittest.TestCase):
    def test_sample_interpolation_hold_and_end(self):
        p, start = program(), [0] * 6
        self.assertEqual(sample_program(p, start, 1), ([5, 0, 0, 0, 0, 0], 0, False))
        self.assertEqual(sample_program(p, start, 2.8)[0][0], 10)
        self.assertEqual(sample_program(p, start, 4)[0][0], 5)
        self.assertEqual(sample_program(p, start, 5), (start, 1, True))

    def test_reject_invalid_angles_times_and_structure(self):
        for value in (float("nan"), float("inf"), True, "1", 500):
            with self.subTest(value=value):
                p = program()
                p["steps"][0]["angles"][0] = value
                with self.assertRaises(ValueError):
                    validate_program(p)
        for value in (-1, 0, float("nan"), True):
            p = program()
            p["steps"][0]["move_s"] = value
            with self.assertRaises(ValueError):
                validate_program(p)
        with self.assertRaises(ValueError):
            validate_program({"version": 2, "name": "a", "steps": []})

    def test_preflight_checks_full_path_and_speed(self):
        p = program()
        with patch("robots.so101.twin_studio.safety.check_angles", return_value=(True, "ok", [])) as check:
            validate_real_path(p, [0] * 6)
            self.assertGreater(check.call_count, 20)
        with patch("robots.so101.twin_studio.safety.check_angles", return_value=(False, "床", [])):
            with self.assertRaisesRegex(ValueError, "床"):
                validate_real_path(p, [0] * 6)
        p["steps"][0]["move_s"] = 0.1
        with self.assertRaisesRegex(ValueError, "15"):
            validate_real_path(p, [0] * 6)

    def test_atomic_library_name_does_not_escape(self):
        with tempfile.TemporaryDirectory() as tmp:
            library = MotionLibrary(pathlib.Path(tmp) / "motions")
            p = program()
            p["name"] = "../../名前"
            library.save(p)
            self.assertEqual(library.load(p["name"]), p)
            self.assertEqual(library.list()[0]["name"], p["name"])
            files = list((pathlib.Path(tmp) / "motions").iterdir())
            self.assertEqual(len(files), 1)
            (pathlib.Path(tmp) / "motions" / "broken.json").write_text("{", encoding="utf-8")
            self.assertEqual(len(library.list()), 1)
            with patch("robots.so101.twin_studio.os.replace", side_effect=OSError("locked")):
                with self.assertRaises(OSError):
                    library.save({**p, "steps": []})
            self.assertEqual(library.load(p["name"]), p)


class StudioTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.sim, self.link = Sim(), Link()
        self.studio = TwinStudio(self.sim, self.link, self.tmp.name)

    def cmd(self, op, **kwargs):
        return self.studio.command({"op": op, "client_id": "alice", **kwargs})

    def test_edit_save_load_and_replay_use_command_not_physics(self):
        self.cmd("program", program=program())
        self.cmd("save")
        self.cmd("program", program={**program(), "steps": []})
        self.cmd("load", name="挨拶")
        self.cmd("play")
        self.sim.time = 1
        self.studio.tick()
        self.assertEqual(self.sim.target[JOINTS[0]], 5)
        self.sim.time = 9
        self.studio.tick()
        self.assertFalse(self.studio.playback["playing"])
        self.assertEqual(self.sim.target[JOINTS[0]], 0)
        self.assertEqual(self.link.writes, [])

    def test_empty_playback_rejected(self):
        with self.assertRaises(ValueError):
            self.cmd("play")

    def test_owner_and_settings_gates(self):
        self.cmd("follow", ready=True)
        self.studio.assert_owner("alice")
        with self.assertRaises(ValueError):
            self.studio.assert_owner("bob")
        with self.assertRaises(ValueError):
            self.studio.assert_physics_editable()
        self.studio.command({"op": "stop", "client_id": "bob"})
        self.assertFalse(self.link.state["following"])

    def test_import_is_read_only_and_requires_fresh_complete_angles(self):
        self.cmd("import_actual")
        self.assertEqual(self.link.commands, [])
        self.assertEqual(self.sim.target[JOINTS[0]], 0)
        self.link.state["age_s"] = 2
        with self.assertRaises(ValueError):
            self.cmd("import_actual")
        self.link.state["age_s"] = 0
        self.link.state["joints"][JOINTS[0]]["q"] = None
        with self.assertRaises(ValueError):
            self.cmd("import_actual")

    def test_stop_and_lease_loss_cancel_transport_no_replay(self):
        self.cmd("program", program=program())
        self.cmd("save")
        self.cmd("follow", ready=True)
        with patch("robots.so101.twin_studio.safety.check_angles", return_value=(True, "ok", [])):
            self.cmd("play")
        self.link.state["following"] = False
        self.sim.time = 1
        self.studio.tick()
        self.assertFalse(self.studio.playback["playing"])
        self.assertEqual(self.sim.target[JOINTS[0]], 0)
        self.cmd("stop")
        self.sim.time = 10
        self.studio.tick()
        self.assertEqual(self.sim.target[JOINTS[0]], 0)

    def test_follow_rejects_pause_slow_time_and_missing_ready(self):
        for attr, value in (("paused", True), ("rtf", 0.5)):
            setattr(self.sim, attr, value)
            with self.assertRaises(ValueError):
                self.cmd("follow", ready=True)
            setattr(self.sim, attr, False if attr == "paused" else 1)
        with self.assertRaises(ValueError):
            self.cmd("follow")

    def test_program_edits_do_not_mutate_active_snapshot(self):
        p = program()
        self.cmd("program", program=p)
        self.cmd("play")
        p["steps"][0]["angles"][0] = 55
        with self.assertRaises(ValueError):
            self.cmd("program", program=p)
        self.sim.time = 1
        self.studio.tick()
        self.assertEqual(self.sim.target[JOINTS[0]], 5)


if __name__ == "__main__":
    unittest.main()
