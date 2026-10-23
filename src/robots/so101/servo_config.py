"""SO-101 servo configuration: audit what is in each servo, apply what we intend.

Why
---
Individual STS3215s on this arm have not had the same settings: on 2026-07-17
ids 3/4/5 carried Max_Voltage = 12.0 V while 1/2/6 had the factory 14.0 V, and
the 12.2 V supply then latched a permanent voltage error on three servos. The
teleop path also never wrote P/I/D, so the follower ran on whatever an earlier
tool had left in EEPROM. The control law in effort.py depends on P (its gain is
P/4 permille per tick), so it must know - and preferably set - the real value.

Two layers
----------
- SESSION settings (P/D/I, Acceleration, Goal_Velocity, Torque_Limit) are written
  at every connect with the EEPROM lock left closed (Lock = 1): the servo uses
  them immediately and forgets them at power-off, so nothing on the servo is
  changed permanently by simply running the cockpit.
- PERSISTENT settings are never written from here. A too-tight
  Max_Voltage_Limit (the 2026-07-17 case) is fixed with the dedicated
  scripts/so101_check_voltage_limits.py --set-max 14.0.

`audit` reads every configuration register of every servo; `findings` turns the
table into Japanese, operator-facing warnings.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Dict, List, Optional, Tuple

from . import sts3215

ARM_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
GRIPPER = "gripper"

AUDIT_REGS = [
    "Firmware_Major", "Firmware_Minor", "Model_Number", "Return_Delay_Time",
    "Min_Position_Limit", "Max_Position_Limit", "Max_Temperature_Limit",
    "Max_Voltage_Limit", "Min_Voltage_Limit", "Max_Torque_Limit", "Phase",
    "Unloading_Condition", "LED_Alarm_Condition", "P_Coefficient", "D_Coefficient",
    "I_Coefficient", "Minimum_Startup_Force", "CW_Dead_Zone", "CCW_Dead_Zone",
    "Protection_Current", "Homing_Offset", "Operating_Mode", "Protective_Torque",
    "Protection_Time", "Overload_Torque", "Over_Current_Protection_Time",
    "Torque_Limit", "Acceleration", "Goal_Velocity", "Lock",
]


@dataclass(frozen=True)
class SessionSettings:
    """What the follower runs with during a cockpit session."""
    p: int = 16              # lerobot's value: smooth, and the governor's gain assumes it is known
    d: int = 32
    i: int = 0               # integral stays OFF on the servo: it winds up against obstacles
    acceleration: int = 50   # x100 step/s^2 -> ~440 deg/s^2
    goal_velocity: int = 512  # step/s -> 45 deg/s ("ふつう")
    torque_limit_arm: int = 1000
    torque_limit_gripper: int = 500


# deg/s presets for the cockpit. Goal_Velocity is in step/s (4096 / 360 per deg).
SPEED_PRESETS = {
    "slow": {"label": "おだやか", "dps": 30.0},
    "normal": {"label": "ふつう", "dps": 45.0},
    "fast": {"label": "はやい", "dps": 70.0},
}


def dps_to_goal_velocity(dps: float) -> int:
    return max(20, min(3000, int(round(sts3215.deg_to_ticks(dps)))))


ReadFn = Callable[[str, str], Tuple[Optional[int], int]]


def audit(read_reg: ReadFn, names: List[str]) -> Dict[str, Dict[str, Optional[int]]]:
    """{servo: {register: value or None}} — reads every AUDIT_REGS entry."""
    table: Dict[str, Dict[str, Optional[int]]] = {}
    for n in names:
        row: Dict[str, Optional[int]] = {}
        for reg in AUDIT_REGS:
            try:
                v, _err = read_reg(reg, n)
            except Exception:
                v = None
            row[reg] = v
        table[n] = row
    return table


@dataclass
class Finding:
    level: str            # "error" | "warn" | "info"
    servo: str
    register: str
    value: Optional[int]
    message: str


def findings(table: Dict[str, Dict[str, Optional[int]]], *, supply_v: float = 12.2,
             role: str = "follower") -> List[Finding]:
    """Operator-facing warnings for an audit table (pure)."""
    out: List[Finding] = []

    def add(level, servo, reg, msg):
        out.append(Finding(level, servo, reg, table.get(servo, {}).get(reg), msg))

    for n, row in table.items():
        if all(v is None for v in row.values()):
            add("error", n, "ID", "応答がありません（配線・ID を確認）")
            continue
        vmax, vmin = row.get("Max_Voltage_Limit"), row.get("Min_Voltage_Limit")
        if vmax and vmax / 10.0 < supply_v + 0.5:
            add("error", n, "Max_Voltage_Limit",
                f"最高電圧 {vmax / 10:.1f}V が電源 {supply_v:.1f}V に近すぎます。電圧エラーが出続けます"
                "（scripts/so101_check_voltage_limits.py --set-max 14.0 で直せます）")
        if vmin and role == "follower" and vmin / 10.0 > 9.0:
            add("warn", n, "Min_Voltage_Limit",
                f"最低電圧 {vmin / 10:.1f}V が高めです。電源が少し下がるだけで電圧エラーになります")
        t = row.get("Max_Temperature_Limit")
        if t is not None and t > sts3215.OVERHEAT_TRIP_C:
            add("warn", n, "Max_Temperature_Limit",
                f"過熱保護が {t}℃ です（工場出荷は 70℃）。高温まで出力が切れません")
        if row.get("Operating_Mode") not in (None, 0):
            add("error", n, "Operating_Mode", f"位置制御モードではありません（{row['Operating_Mode']}）")
        unload = row.get("Unloading_Condition")
        if unload and (unload & 0x24) != 0x24:
            add("warn", n, "Unloading_Condition",
                f"過熱・過負荷の保護が無効になっています（値 {unload}）")
        if n != GRIPPER:
            for reg, expect in (("Overload_Torque", 80), ("Protective_Torque", 20),
                                ("Protection_Time", 200)):
                v = row.get(reg)
                if v and v != expect:
                    add("info", n, reg, f"{reg} が {v} です（工場出荷は {expect}）")
        p = row.get("P_Coefficient")
        if p is not None and role == "follower" and p != SessionSettings.p:
            add("info", n, "P_Coefficient",
                f"P が {p} です。追従中はセッション設定（P={SessionSettings.p}）で上書きします")
        if row.get("I_Coefficient") not in (None, 0):
            add("warn", n, "I_Coefficient",
                f"I が {row['I_Coefficient']} です。障害物に当たると積分が溜まり、押し続けます")
        if row.get("Return_Delay_Time") not in (None, 0):
            add("info", n, "Return_Delay_Time",
                f"応答遅延が {row['Return_Delay_Time']} です（0 にすると読み出しが速くなります）")
        lo, hi = row.get("Min_Position_Limit"), row.get("Max_Position_Limit")
        if lo is not None and hi is not None and hi <= lo and (lo, hi) != (0, 0):
            add("warn", n, "Max_Position_Limit", f"位置制限が逆転しています（{lo}〜{hi}）")

    fw = {(r.get("Firmware_Major"), r.get("Firmware_Minor")) for r in table.values()
          if r.get("Firmware_Major") is not None}
    if len(fw) > 1:
        out.append(Finding("warn", "*", "Firmware", None,
                           "ファームウェアの版が揃っていません: "
                           + ", ".join(f"{a}.{b}" for a, b in sorted(fw))))
    phases = {r.get("Phase") for r in table.values() if r.get("Phase") is not None}
    if len(phases) > 1:
        out.append(Finding("warn", "*", "Phase", None,
                           f"Phase（速度の単位など）が揃っていません: {sorted(phases)}"))
    return out


WriteFn = Callable[[str, str, int], Tuple[bool, int]]


def session_writes(settings: SessionSettings, names: List[str],
                   goal_velocity: Optional[int] = None) -> List[Tuple[str, str, int]]:
    """(register, servo, value) list applied at connect (pure)."""
    gv = settings.goal_velocity if goal_velocity is None else int(goal_velocity)
    out: List[Tuple[str, str, int]] = []
    for n in names:
        # close the EEPROM lock first: P/D/I live in the EEPROM area and must
        # only change for this power cycle
        out += [("Lock", n, 1),
                ("P_Coefficient", n, settings.p), ("D_Coefficient", n, settings.d),
                ("I_Coefficient", n, settings.i), ("Acceleration", n, settings.acceleration),
                ("Goal_Velocity", n, gv),
                ("Torque_Limit", n, settings.torque_limit_gripper if n == GRIPPER
                 else settings.torque_limit_arm)]
    return out


def apply_session(write_reg: WriteFn, read_reg: ReadFn, settings: SessionSettings,
                  names: List[str], goal_velocity: Optional[int] = None,
                  log: Callable[[str], None] = lambda _m: None) -> Dict[str, int]:
    """Write the session settings (lock stays closed) and read P back.

    Returns {servo: P actually in effect} so the governor uses the real gain
    even when a write was refused.
    """
    for reg, n, v in session_writes(settings, names, goal_velocity):
        ok, _err = write_reg(reg, n, v)
        if not ok:
            log(f"⚠ {n} の {reg}={v} を書けませんでした")
    p_eff: Dict[str, int] = {}
    for n in names:
        v, _err = read_reg("P_Coefficient", n)
        p_eff[n] = int(v) if v is not None else settings.p
    return p_eff
