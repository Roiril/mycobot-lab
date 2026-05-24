"""Tests for arm.spatial_memory — sector binning, annotation preservation, atomic save."""
import unittest, tempfile, pathlib, time, threading
import sys
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / "src"))

from arm.spatial_memory import SpatialMemory, j1_to_sector


class TestSectorBinning(unittest.TestCase):
    def test_back_sector(self):
        # back spans -22.5..+22.5
        for j1 in (-22.0, -10, 0, 10, 22.0):
            self.assertEqual(j1_to_sector(j1), "back")

    def test_right_sector(self):
        for j1 in (90.0, 80, 100):
            self.assertEqual(j1_to_sector(j1), "right")

    def test_left_sector(self):
        for j1 in (-90.0, -80, -100):
            self.assertEqual(j1_to_sector(j1), "left")

    def test_front_wraps_both_sides(self):
        # front sector defined as 157.5..180 AND -180..-157.5
        self.assertEqual(j1_to_sector(170), "front")
        self.assertEqual(j1_to_sector(-170), "front")
        self.assertEqual(j1_to_sector(165), "front")

    def test_intermediate_sectors(self):
        self.assertEqual(j1_to_sector(45), "back_right")
        self.assertEqual(j1_to_sector(-45), "back_left")
        self.assertEqual(j1_to_sector(135), "front_right")


class TestRecordAndAnnotate(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.NamedTemporaryFile(suffix=".json", delete=False)
        self.tmp.close()
        self.path = pathlib.Path(self.tmp.name)
        self.mem = SpatialMemory(self.path)

    def tearDown(self):
        try: self.path.unlink()
        except OSError: pass
        for ext in (".tmp",):
            tmp = self.path.with_suffix(self.path.suffix + ext)
            if tmp.exists(): tmp.unlink()

    def test_record_creates_sector_entry(self):
        self.mem.record(j1_deg=0, frame_path="f.jpg",
                        camera_pose={"flange_mm": [0,0,0]}, observer="test",
                        description="initial", objects=[{"label": "X"}])
        m = self.mem.all()
        self.assertIn("back", m)
        self.assertEqual(m["back"]["description"], "initial")

    def test_reobserve_preserves_prior_annotation(self):
        # Annotate first
        self.mem.record(j1_deg=0, frame_path="f1.jpg",
                        camera_pose={"flange_mm": [0,0,0]}, observer="shubie",
                        description="ある物がある", objects=[{"label": "本"}])
        # Re-observe with empty description (simulating /observe with no annot)
        self.mem.record(j1_deg=0, frame_path="f2.jpg",
                        camera_pose={"flange_mm": [1,1,1]}, observer="pending",
                        description="", objects=None)
        m = self.mem.all()
        # description and objects should be preserved
        self.assertEqual(m["back"]["description"], "ある物がある")
        self.assertEqual(len(m["back"]["objects"]), 1)
        self.assertEqual(m["back"]["objects"][0]["label"], "本")
        # but frame_path and camera_pose update
        self.assertEqual(m["back"]["frame_path"], "f2.jpg")

    def test_explicit_new_description_overwrites(self):
        self.mem.record(j1_deg=0, frame_path="f.jpg",
                        camera_pose={"flange_mm": [0,0,0]}, observer="a",
                        description="old", objects=[{"label":"A"}])
        self.mem.record(j1_deg=0, frame_path="f.jpg",
                        camera_pose={"flange_mm": [0,0,0]}, observer="b",
                        description="new", objects=[{"label":"B"}])
        m = self.mem.all()
        self.assertEqual(m["back"]["description"], "new")
        self.assertEqual(m["back"]["objects"][0]["label"], "B")

    def test_annotate_updates_entry(self):
        self.mem.record(j1_deg=0, frame_path="f.jpg",
                        camera_pose={}, observer="auto", description="")
        self.mem.annotate(j1_deg=0, description="annotated by shubie",
                          observer="shubie",
                          objects=[{"label":"plant"}])
        m = self.mem.all()
        self.assertEqual(m["back"]["description"], "annotated by shubie")
        self.assertEqual(m["back"]["observer"], "shubie")

    def test_freshness_decays_over_time(self):
        self.mem.record(j1_deg=0, frame_path=None, camera_pose={},
                        observer="t", description="d")
        # Hack: rewind timestamp by 200s
        self.mem._sectors["back"]["timestamp"] -= 200
        m = self.mem.all()
        self.assertLess(m["back"]["freshness"], 0.7)  # past one half-life
        self.assertGreater(m["back"]["age_s"], 180)

    def test_events_appended_on_record(self):
        self.mem.record(j1_deg=0, frame_path=None, camera_pose={},
                        observer="t", description="")
        evs = self.mem.events()
        self.assertGreaterEqual(len(evs), 1)
        self.assertEqual(evs[-1]["kind"], "observe")

    def test_clear(self):
        self.mem.record(j1_deg=0, frame_path=None, camera_pose={},
                        observer="t", description="x")
        self.mem.clear()
        self.assertEqual(self.mem.all(), {})


class TestAtomicSave(unittest.TestCase):
    def test_save_is_atomic_no_partial_file(self):
        """If interrupted mid-write, the .json file is never empty/partial.
        We can't easily simulate Ctrl-C, but we can verify the file always
        contains valid JSON after save."""
        tmp = tempfile.NamedTemporaryFile(suffix=".json", delete=False); tmp.close()
        path = pathlib.Path(tmp.name)
        try:
            mem = SpatialMemory(path)
            # Many quick writes
            for i in range(20):
                mem.record(j1_deg=i*10, frame_path=f"f{i}", camera_pose={},
                           observer="t", description=f"d{i}")
                # Verify file is valid JSON at every step
                import json
                data = json.loads(path.read_text(encoding="utf-8"))
                self.assertIn("sectors", data)
        finally:
            try: path.unlink()
            except OSError: pass


class TestConcurrentAccess(unittest.TestCase):
    def test_concurrent_record_no_crash(self):
        tmp = tempfile.NamedTemporaryFile(suffix=".json", delete=False); tmp.close()
        path = pathlib.Path(tmp.name)
        try:
            mem = SpatialMemory(path)
            errors = []
            def worker(i):
                try:
                    for _ in range(10):
                        mem.record(j1_deg=i*30, frame_path=None,
                                   camera_pose={}, observer=f"t{i}",
                                   description="")
                except Exception as e:
                    errors.append(e)
            threads = [threading.Thread(target=worker, args=(i,)) for i in range(4)]
            for t in threads: t.start()
            for t in threads: t.join()
            self.assertEqual(errors, [])
        finally:
            try: path.unlink()
            except OSError: pass


if __name__ == "__main__":
    unittest.main()
