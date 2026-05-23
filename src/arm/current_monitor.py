"""Servo current monitor — runs as a daemon thread during motion.

Polls `get_servo_currents()` at CURRENT_POLL_HZ. If any joint exceeds
CURRENT_THRESHOLD_MA for SUSTAINED_OVER_COUNT consecutive polls, sets the
hub's abort_flag, which the waypoint loop honors at its next check.

This is NOT a fast emergency stop — at 10 Hz the response latency is ~300 ms.
Treat it as a "stalled / pushing hard" detector, not a "touched something" detector.
"""
from __future__ import annotations
import threading, time, logging
from typing import Callable, List

from .constants import CURRENT_THRESHOLD_MA, CURRENT_POLL_HZ, SUSTAINED_OVER_COUNT
from .kinematics import JOINT_LIMITS

log = logging.getLogger("mycobot.current")


class CurrentMonitor(threading.Thread):
    def __init__(self,
                 read_currents: Callable[[], List[int] | None],
                 on_overcurrent: Callable[[List[int]], None],
                 threshold_mA: int = CURRENT_THRESHOLD_MA,
                 poll_hz: float = CURRENT_POLL_HZ,
                 sustained: int = SUSTAINED_OVER_COUNT):
        super().__init__(daemon=True)
        self._read = read_currents
        self._on_over = on_overcurrent
        self._thr = threshold_mA
        self._dt = 1.0 / poll_hz
        self._sustained = sustained
        self._stop_evt = threading.Event()  # NOT self._stop (would shadow Thread._stop method)
        self.last_currents: List[int] | None = None
        self.peak_currents: List[int] = [0] * len(JOINT_LIMITS)
        self.triggered = False
        self.peak_joint: int | None = None  # 1-based; which joint peaked
        self.peak_value: int = 0

    def stop(self):
        self._stop_evt.set()

    def run(self):
        over = 0
        while not self._stop_evt.is_set():
            try:
                cs = self._read()
            except Exception as e:
                log.warning("get_currents raised: %s", e)
                time.sleep(self._dt); continue
            if not cs or not isinstance(cs, list) or len(cs) != 6:
                time.sleep(self._dt); continue
            self.last_currents = cs
            for i, c in enumerate(cs):
                ac = abs(c)
                if ac > self.peak_currents[i]:
                    self.peak_currents[i] = ac
                if ac > self.peak_value:
                    self.peak_value = ac; self.peak_joint = i + 1
            spike = [i+1 for i, c in enumerate(cs) if abs(c) > self._thr]
            if spike:
                over += 1
                if over >= self._sustained:
                    log.warning("過電流: joints=%s currents=%s thr=%dmA", spike, cs, self._thr)
                    self.triggered = True
                    # invoke callback before raising — if callback dies, the trigger is already set
                    try: self._on_over(cs)
                    except Exception as e: log.error("on_overcurrent callback: %s", e)
                    return
            else:
                over = 0
            time.sleep(self._dt)
