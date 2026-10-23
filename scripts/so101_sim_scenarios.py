"""Compare the old follower loop with the governed engine on the simulated arm.

    py -3.10 scripts/so101_sim_scenarios.py              # all scenarios
    py -3.10 scripts/so101_sim_scenarios.py table payload --seconds 60

The arm is src/robots/so101/sim/servo_sim.py: STS3215 physics on MuJoCo with
parameters from the 12 V datasheet, some of them ASSUMED (thermal constants, D
scaling). Read the results as "does it trip / which way does the temperature
go", not as real-world numbers.

Scenarios (leader motion is scripted):
  table     the leader pushes the gripper 4 cm into a table and stays there
  endstop   the elbow's real stop sits 8 deg inside the calibrated range; the
            leader goes to the calibrated end and stays there
  payload   250 g at the gripper, arm held out horizontally
  engage    follow is switched on with the leader 30 deg away

Loops compared:
  legacy    what the cockpit did until 2026-10: goal = leader clamped to the
            calibrated range, written whenever it moved >= 2 ticks, Goal_Velocity
            800, Acceleration 30, full torque, no governor
  governed  TeleopEngine (governor active, session settings, write-on-change)
"""
from __future__ import annotations

import argparse
import json
import math
import pathlib
import sys
from dataclasses import dataclass
from typing import Callable, Dict, List, Optional

ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "src"))

from robots.so101 import sts3215  # noqa: E402
from robots.so101.sim.servo_sim import (  # noqa: E402
    JOINTS, ScriptedLeaderBus, SimArm, SimFollowerBus, World,
)
from robots.so101.teleop_engine import OneEuroConfig, OneEuroFilter, TeleopConfig, TeleopEngine  # noqa: E402

DT = 1.0 / 60.0

# The follower calibration of this rig (2026-06-10), so ticks <-> angles match the real arm.
RANGES = {
    "shoulder_pan": (874, 3159), "shoulder_lift": (815, 3145), "elbow_flex": (859, 3065),
    "wrist_flex": (911, 3193), "wrist_roll": (0, 4095), "gripper": (1605, 3086),
}


def ramp(a: Dict[str, float], b: Dict[str, float], t0: float, t1: float) -> Callable[[float], Dict[str, float]]:
    def f(t: float) -> Dict[str, float]:
        s = 0.0 if t <= t0 else 1.0 if t >= t1 else (t - t0) / (t1 - t0)
        s = s * s * (3 - 2 * s)
        return {n: a.get(n, 0.0) + (b.get(n, 0.0) - a.get(n, 0.0)) * s for n in JOINTS}
    return f


@dataclass
class Scenario:
    name: str
    start: List[float]                     # follower start pose (5 arm joints, deg)
    path: Callable[[float], Dict[str, float]]
    world: World
    seconds: float
    engage_at: float = 0.0
    regs: Optional[Dict[str, Dict[str, int]]] = None   # registers left in EEPROM


def scenarios(seconds: Optional[float] = None) -> Dict[str, Scenario]:
    # the base stands on the table (z = 0): the TCP starts 6 cm above it and the
    # leader asks for 4 cm below it
    above = {"shoulder_pan": 0, "shoulder_lift": -20, "elbow_flex": 30, "wrist_flex": 60}
    pressed = {"shoulder_pan": 0, "shoulder_lift": 10, "elbow_flex": 40, "wrist_flex": 40}
    reach = {"shoulder_pan": 0, "shoulder_lift": 0, "elbow_flex": -60, "wrist_flex": 0}
    over_end = {"shoulder_pan": 0, "shoulder_lift": 0, "elbow_flex": -97, "wrist_flex": 0}
    held_out = {"shoulder_pan": 0, "shoulder_lift": 80, "elbow_flex": -80, "wrist_flex": 0}
    s = seconds
    return {
        "table": Scenario("table", [0, -20, 30, 60, 0], ramp(above, pressed, 1.0, 3.0),
                          World(table_z_m=0.0), s or 25.0),
        "endstop": Scenario("endstop", [0, 0, -60, 0, 0], ramp(reach, over_end, 1.0, 3.0),
                            World(stops_deg={"elbow_flex": (-89.0, 96.8)}), s or 25.0),
        # the operator drives the leader 10 cm under the table surface
        "table_deep": Scenario("table_deep", [0, -20, 30, 60, 0],
                               ramp(above, {"shoulder_pan": 0, "shoulder_lift": 50,
                                            "elbow_flex": 10, "wrist_flex": 40}, 1.0, 3.0),
                               World(table_z_m=0.0), s or 25.0),
        # the 2026-08-07 hypothesis: the elbow's EEPROM carries a far stiffer P
        # than its neighbours (the legacy loop never wrote P/I/D)
        "endstop_p64": Scenario("endstop_p64", [0, 0, -60, 0, 0], ramp(reach, over_end, 1.0, 3.0),
                                World(stops_deg={"elbow_flex": (-89.0, 96.8)}), s or 25.0,
                                regs={"elbow_flex": {"P_Coefficient": 64}}),
        "payload": Scenario("payload", [0, 80, -80, 0, 0], lambda t: dict(held_out),
                            World(payload_kg=0.25), s or 120.0),
        "engage": Scenario("engage", [0, 0, 0, 0, 0],
                           lambda t: {"shoulder_pan": 30, "shoulder_lift": -20,
                                      "elbow_flex": 30, "wrist_flex": -20},
                           World(), s or 6.0, engage_at=0.5),
    }


def make_arm(sc: Scenario) -> SimArm:
    arm = SimArm(ranges=RANGES, world=sc.world, start_deg=sc.start)
    for n, regs in (sc.regs or {}).items():
        s = arm.servo(n)
        for reg, v in regs.items():
            s.write(reg, v, arm.ticks(JOINTS.index(n)))
    return arm


def _deg_to_ticks(n: str, deg: float) -> int:
    lo, hi = RANGES[n]
    return int(round((lo + hi) / 2 + deg * 4095 / 360))


class Recorder:
    def __init__(self, arm: SimArm):
        self.arm = arm
        self.peak_temp = {n: 0.0 for n in JOINTS}
        self.peak_current = {n: 0.0 for n in JOINTS}
        self.sat_time = {n: 0.0 for n in JOINTS}      # |I| > 1.6 A
        self.peak_speed_dps = 0.0
        self.peak_contact_n = 0.0
        self.min_supply_v = 99.0
        self.states: Dict[str, int] = {}

    def sample(self, dt: float, governor_out=None) -> None:
        a = self.arm
        for s in a.servos:
            self.peak_temp[s.name] = max(self.peak_temp[s.name], s.temp_c)
            self.peak_current[s.name] = max(self.peak_current[s.name], abs(s.current))
            if abs(s.current) > 1.6:
                self.sat_time[s.name] += dt
        self.peak_speed_dps = max(self.peak_speed_dps,
                                  max(abs(math.degrees(v)) for v in a.d.qvel[:5]))
        self.peak_contact_n = max(self.peak_contact_n, a.contact_force)
        self.min_supply_v = min(self.min_supply_v, a.supply_v)
        if governor_out is not None:
            for n, js in governor_out.joints.items():
                if js.state not in ("track", "off"):
                    key = f"{n}:{js.state}"
                    self.states[key] = self.states.get(key, 0) + 1

    def summary(self) -> dict:
        trips = {}
        for s in self.arm.servos:
            for k, v in s.trips.items():
                if v:
                    trips[f"{s.name}:{k}"] = v
        hot = max(self.peak_temp, key=lambda n: self.peak_temp[n])
        amp = max(self.peak_current, key=lambda n: self.peak_current[n])
        return {
            "firmware_trips": trips,
            "peak_temp_c": {hot: round(float(self.peak_temp[hot]), 1)},
            "peak_current_a": {amp: round(float(self.peak_current[amp]), 2)},
            "time_over_1.6A_s": {n: round(float(v), 1) for n, v in self.sat_time.items() if v > 0},
            "peak_speed_dps": round(float(self.peak_speed_dps), 1),
            "peak_table_force_n": round(float(self.peak_contact_n), 1),
            "min_supply_v": round(float(self.min_supply_v), 2),
            "governor_states_cycles": self.states,
        }


def run_legacy(sc: Scenario) -> dict:
    arm = make_arm(sc)
    bus = SimFollowerBus(arm, realtime=False)
    leader = ScriptedLeaderBus(RANGES, sc.path, clock=lambda: arm.t)
    bus.sync_write("Goal_Velocity", {n: 800 for n in JOINTS})
    bus.sync_write("Acceleration", {n: 30 for n in JOINTS})
    bus.sync_write("Torque_Limit", {n: (500 if n == "gripper" else 1000) for n in JOINTS})
    start = bus.read_positions()
    bus.sync_write("Goal_Position", start)
    bus.sync_write("Torque_Enable", {n: 1 for n in JOINTS})
    filt = {n: OneEuroFilter(OneEuroConfig(), DT) for n in JOINTS}
    last = dict(start)
    rec = Recorder(arm)
    following = False
    t = 0.0
    while t < sc.seconds:
        if not following and t >= sc.engage_at:
            following = True
        if following:
            lead = leader.read_positions()
            goals = {}
            for n in JOINTS:
                f = filt[n].filter(float(lead[n]), t)
                lo, hi = RANGES[n]
                tgt = min(hi, max(lo, f))
                step = max(-170, min(170, tgt - last[n]))
                g = int(round(last[n] + step))
                if abs(g - last[n]) >= 2:
                    goals[n] = g
                    last[n] = g
            if goals:
                bus.sync_write("Goal_Position", goals)
        arm.advance(DT)
        rec.sample(DT)
        t += DT
    return rec.summary()


def run_governed(sc: Scenario) -> dict:
    arm = make_arm(sc)
    bus = SimFollowerBus(arm, realtime=False)
    leader = ScriptedLeaderBus(RANGES, sc.path, clock=lambda: arm.t)
    eng = TeleopEngine(leader, bus, RANGES, config=TeleopConfig(torque_stagger_s=0.0),
                       sleep=lambda s: None)
    eng.configure_follower()
    eng.enable_follower_torque_staged()
    rec = Recorder(arm)
    engaged = False
    t = 0.0
    while t < sc.seconds:
        if not engaged and t >= sc.engage_at:
            ok, why = eng.start_teleop(now=t)
            engaged = True
        res = eng.cycle(now=t)
        arm.advance(DT)
        rec.sample(DT, res["governor"])
        t += DT
    out = rec.summary()
    out["final_message"] = eng.last_out.message if eng.last_out else ""
    out["following_at_end"] = eng.active
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("names", nargs="*",
                    default=["table", "table_deep", "endstop", "endstop_p64", "payload", "engage"])
    ap.add_argument("--seconds", type=float, default=None)
    ap.add_argument("--json", type=str, default=None, help="write the results here")
    args = ap.parse_args(argv)
    sys.stdout.reconfigure(encoding="utf-8")
    all_sc = scenarios(args.seconds)
    results = {}
    for name in args.names:
        sc = all_sc[name]
        results[name] = {"legacy": run_legacy(sc), "governed": run_governed(sc)}
        print(f"\n=== {name} ({sc.seconds:.0f} s simulated) ===")
        for loop in ("legacy", "governed"):
            print(f"  [{loop}]")
            for k, v in results[name][loop].items():
                print(f"    {k}: {v}")
    if args.json:
        pathlib.Path(args.json).write_text(json.dumps(results, ensure_ascii=False, indent=2),
                                           encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
