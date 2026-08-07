"""SO-101 leader/follower serial-port discovery and role identification.

Both SO-101 arms hang off an identical Seeed XIAO driver board (CH343,
VID 1A86 / PID 55D3), so Windows hands them interchangeable COM numbers that
move when you replug or change hub socket. Hard-coding ``COM13``/``COM14`` is
therefore not just fragile, it is *dangerous*: if the two numbers swap, the
teleop loop torque-enables the LEADER and drives it to the follower's pose.

This module identifies which port is which before anything is energised.

Discriminator
-------------
Supply voltage, read from the servos themselves: the follower runs on 12 V and
the leader on the 7.4 V-class supply, so a ``Present_Voltage`` read separates
them with a wide margin and touches nothing but a read register.

Anything less than certain is refused rather than guessed — a role mix-up drives
the wrong arm. On an inconclusive probe the last confirmed mapping (remembered
by USB identity in ``data/so101_ports.json``) is offered as a *fallback*, flagged
so the caller can decline to auto-start motion from it.

Pure vs I/O
-----------
``port_keys`` / ``classify_arm_voltage`` / ``decide_roles`` / ``roles_from_cache``
are pure and import nothing from lerobot, so they unit-test on plain Python 3.10
with no hardware. Only ``list_so101_ports`` (pyserial) and ``probe_port``
(lerobot, imported lazily via teleop_engine.make_bus) touch the world.
"""
from __future__ import annotations

import json
import time
import pathlib
import statistics
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Sequence, Tuple

# src/robots/so101/ports.py -> repo root
ROOT = pathlib.Path(__file__).resolve().parents[3]
DEFAULT_CACHE = ROOT / "data" / "so101_ports.json"

# USB identity of the Seeed Bus Servo Driver Board for XIAO (CH343P onboard).
SO101_USB_IDS: Tuple[Tuple[int, int], ...] = ((0x1A86, 0x55D3),)
# Same vendor, different product: the myCobot 320's CH9102. Must never match.
MYCOBOT_USB_IDS: Tuple[Tuple[int, int], ...] = ((0x1A86, 0x55D4),)

LEADER = "leader"
FOLLOWER = "follower"

# Voltage windows (V). The gap between them is deliberately dead: a follower
# browning out into leader territory must read "unknown", never "leader".
FOLLOWER_MIN_VOLT = 9.5      # 12 V arm; still well clear when sagging
LEADER_MAX_VOLT = 9.0        # 7.4 V-class leader (measured ~5-8 V)
MIN_PLAUSIBLE_VOLT = 3.0     # below this the read is stale/garbage, not a supply


class PortError(RuntimeError):
    """Ports could not be identified (message is operator-facing Japanese)."""


# ==============================================================================
# Port enumeration
# ==============================================================================
@dataclass(frozen=True)
class PortInfo:
    device: str
    description: str = ""
    hwid: str = ""
    serial_number: Optional[str] = None
    location: Optional[str] = None
    vid: Optional[int] = None
    pid: Optional[int] = None

    def label(self) -> str:
        return f"{self.device} ({self.description})" if self.description else self.device


def list_so101_ports() -> List[PortInfo]:
    """Every attached SO-101 driver board, by USB VID/PID (I/O: pyserial)."""
    from serial.tools import list_ports as _lp
    out: List[PortInfo] = []
    for p in _lp.comports():
        ids = (p.vid, p.pid)
        if ids in MYCOBOT_USB_IDS:
            continue
        desc = p.description or ""
        if ids in SO101_USB_IDS or (p.vid is None and "CH343" in desc):
            out.append(PortInfo(
                device=p.device, description=desc, hwid=p.hwid or "",
                serial_number=p.serial_number, location=p.location,
                vid=p.vid, pid=p.pid,
            ))
    return sorted(out, key=lambda p: p.device)


def port_keys(ports: Sequence[PortInfo]) -> Dict[str, str]:
    """Stable identity per port, for remembering a confirmed mapping (pure).

    Prefers the USB serial number, but only when it actually distinguishes the
    boards — CH343s commonly ship with a shared (or absent) serial, in which
    case the USB topology location is the next best thing, and the COM number
    the last resort.
    """
    serials = [p.serial_number for p in ports if p.serial_number]
    duplicated = {s for s in serials if serials.count(s) > 1}
    keys: Dict[str, str] = {}
    for p in ports:
        if p.serial_number and p.serial_number not in duplicated:
            keys[p.device] = f"sn:{p.serial_number}"
        elif p.location:
            keys[p.device] = f"loc:{p.location}"
        else:
            keys[p.device] = f"dev:{p.device}"
    return keys


# ==============================================================================
# Role decision (pure)
# ==============================================================================
def classify_arm_voltage(volt: Optional[float]) -> str:
    """Supply voltage -> "follower" | "leader" | "unknown" (pure)."""
    if volt is None or volt < MIN_PLAUSIBLE_VOLT:
        return "unknown"
    if volt >= FOLLOWER_MIN_VOLT:
        return FOLLOWER
    if volt <= LEADER_MAX_VOLT:
        return LEADER
    return "unknown"


@dataclass
class Probe:
    """Result of looking at one port. `volt` is the median responding servo."""
    device: str
    volt: Optional[float] = None
    motors: List[str] = field(default_factory=list)
    ok: bool = False
    error: str = ""

    @property
    def role(self) -> str:
        return classify_arm_voltage(self.volt) if self.ok else "unknown"

    def describe(self) -> str:
        if not self.ok:
            return f"{self.device}: 応答なし（{self.error}）"
        v = f"{self.volt:.1f}V" if self.volt is not None else "電圧不明"
        role = {FOLLOWER: "フォロワー", LEADER: "リーダー"}.get(self.role, "判別不能")
        return f"{self.device}: {v} / {len(self.motors)}軸応答 → {role}"


@dataclass
class Roles:
    """A resolved leader/follower assignment plus how it was arrived at."""
    leader: str
    follower: str
    source: str                 # "manual" | "probe" | "cache"
    detail: str = ""

    @property
    def confirmed(self) -> bool:
        """True when the roles were measured, not remembered or asserted.

        Callers that start motion without a human in the loop (cockpit
        auto-follow) require this: a remembered mapping is keyed by USB socket
        and silently lies if the two cables were swapped.
        """
        return self.source == "probe"


def decide_roles(probes: Sequence[Probe]) -> Roles:
    """Assign leader/follower from probe results, or raise PortError (pure).

    Accepts three shapes of evidence, in descending order of directness:
      1. exactly one port reads as the follower and exactly one as the leader;
      2. exactly two ports, one confidently classified — the other is the rest;
    anything else (both alike, nothing responding, ambiguous voltages) refuses.
    """
    followers = [p for p in probes if p.role == FOLLOWER]
    leaders = [p for p in probes if p.role == LEADER]

    if len(followers) == 1 and len(leaders) == 1:
        return Roles(leader=leaders[0].device, follower=followers[0].device,
                     source="probe",
                     detail=f"{leaders[0].describe()} / {followers[0].describe()}")

    if len(probes) == 2:
        if len(followers) == 1 and not leaders:
            other = [p for p in probes if p is not followers[0]][0]
            return Roles(leader=other.device, follower=followers[0].device,
                         source="probe",
                         detail=f"{followers[0].describe()} / 残りをリーダーと判定")
        if len(leaders) == 1 and not followers:
            other = [p for p in probes if p is not leaders[0]][0]
            return Roles(leader=leaders[0].device, follower=other.device,
                         source="probe",
                         detail=f"{leaders[0].describe()} / 残りをフォロワーと判定")

    lines = "\n  ".join(p.describe() for p in probes) or "候補ポートなし"
    if len(followers) > 1:
        why = "12V 系が2本見えます（リーダーの電源が 12V になっていませんか）"
    elif len(leaders) > 1:
        why = "低電圧系が2本見えます（フォロワーの 12V 電源が入っていますか）"
    elif not probes:
        why = "SO-101 のドライバボード（CH343）が1本も見つかりません"
    else:
        why = "電圧からリーダー / フォロワーを確定できません"
    raise PortError(f"リーダー / フォロワーを判別できませんでした: {why}\n  {lines}")


# ==============================================================================
# Remembered mapping
# ==============================================================================
def load_cache(path: pathlib.Path = DEFAULT_CACHE) -> dict:
    try:
        data = json.loads(pathlib.Path(path).read_text(encoding="utf-8"))
        return data if isinstance(data, dict) else {}
    except Exception:
        return {}


def save_cache(roles: Roles, ports: Sequence[PortInfo],
               path: pathlib.Path = DEFAULT_CACHE) -> None:
    """Remember a *confirmed* mapping by USB identity. Best-effort; never raises."""
    if not roles.confirmed:
        return
    keys = port_keys(ports)
    by_device = {p.device: p for p in ports}
    payload = {"version": 1, "updated": time.strftime("%Y-%m-%dT%H:%M:%S")}
    for role, device in ((LEADER, roles.leader), (FOLLOWER, roles.follower)):
        info = by_device.get(device)
        payload[role] = {
            "key": keys.get(device, f"dev:{device}"),
            "device": device,
            "description": info.description if info else "",
        }
    try:
        p = pathlib.Path(path)
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
                     encoding="utf-8")
    except Exception:
        pass


def roles_from_cache(cache: dict, ports: Sequence[PortInfo]) -> Optional[Roles]:
    """Re-apply a remembered mapping to the ports present now (pure).

    Returns None unless both remembered identities are attached and distinct.
    """
    lead = (cache.get(LEADER) or {}).get("key")
    foll = (cache.get(FOLLOWER) or {}).get("key")
    if not lead or not foll or lead == foll:
        return None
    by_key = {k: d for d, k in port_keys(ports).items()}
    if lead not in by_key or foll not in by_key:
        return None
    return Roles(leader=by_key[lead], follower=by_key[foll], source="cache",
                 detail="前回確定した USB 位置から復元（ケーブルを差し替えていれば誤り）")


# ==============================================================================
# Probing (I/O)
# ==============================================================================
def _access_hint(err: str) -> str:
    low = err.lower()
    if "permission" in low or "access is denied" in low or "アクセス" in err:
        return "（他のプロセスが掴んでいます: コックピット / 校正 GUI / 統合サーバの二重起動）"
    if "input voltage error" in low:
        return "（サーボ EEPROM の電圧リミット: so101_check_voltage_limits.py 参照）"
    return ""


def probe_port(device: str, joints: Optional[Sequence[str]] = None) -> Probe:
    """Open `device`, read each servo's supply voltage, close. Never energises.

    Torque is explicitly left alone on disconnect — dropping torque on the
    follower would let the arm fall.
    """
    from .teleop_engine import JOINTS, make_bus
    names = list(joints or JOINTS)
    probe = Probe(device=device)
    try:
        bus = make_bus(device)
    except Exception as e:
        probe.error = f"{e}{_access_hint(str(e))}"
        return probe
    try:
        bus.connect(handshake=False)
    except Exception as e:
        probe.error = f"{e}{_access_hint(str(e))}"
        return probe
    try:
        volts: List[float] = []
        for n in names:
            try:
                raw = bus.read("Present_Voltage", n, normalize=False)
            except Exception:
                continue
            probe.motors.append(n)
            volts.append(raw / 10.0)       # register is 0.1 V units
        if volts:
            probe.ok = True
            probe.volt = statistics.median(volts)
        else:
            probe.error = "サーボが1軸も応答しません（電源 / 配線）"
    except Exception as e:
        probe.error = f"{e}{_access_hint(str(e))}"
    finally:
        try:
            bus.disconnect(disable_torque=False)
        except Exception:
            pass
    return probe


# ==============================================================================
# Resolution
# ==============================================================================
def resolve_ports(leader: Optional[str] = None, follower: Optional[str] = None,
                  cache_path: pathlib.Path = DEFAULT_CACHE,
                  log: Optional[Callable[[str], None]] = None) -> Roles:
    """Work out which COM port is the leader and which is the follower.

    `leader` / `follower` may name a port explicitly (or be None / "auto" to
    detect). Explicit names are trusted as given — they are the operator's
    assertion and skip probing entirely.

    Raises PortError when neither the probe nor the remembered mapping settles
    it; the message names each port and what it read.
    """
    say = log or (lambda _m: None)
    lead_fixed = leader if leader and leader.lower() != "auto" else None
    foll_fixed = follower if follower and follower.lower() != "auto" else None
    if lead_fixed and foll_fixed:
        return Roles(leader=lead_fixed, follower=foll_fixed, source="manual",
                     detail="コマンドライン指定")

    ports = list_so101_ports()
    if lead_fixed or foll_fixed:
        # One side pinned: the remaining SO-101 port is the other side.
        fixed, role = (lead_fixed, LEADER) if lead_fixed else (foll_fixed, FOLLOWER)
        rest = [p.device for p in ports if p.device != fixed]
        if len(rest) != 1:
            raise PortError(
                f"{role}={fixed} を指定しましたが、もう一方を特定できません"
                f"（他の SO-101 ポート: {rest or 'なし'}）")
        if role == LEADER:
            return Roles(leader=fixed, follower=rest[0], source="manual",
                         detail="リーダー指定 + 残り1本をフォロワー")
        return Roles(leader=rest[0], follower=fixed, source="manual",
                     detail="フォロワー指定 + 残り1本をリーダー")

    if len(ports) < 2:
        found = ", ".join(p.label() for p in ports) or "なし"
        raise PortError(
            f"SO-101 のポートが {len(ports)} 本しか見つかりません（{found}）。"
            "両アームの USB と電源を確認してください。")

    say(f"ポート判別中: {', '.join(p.device for p in ports)}")
    probes = [probe_port(p.device) for p in ports]
    try:
        roles = decide_roles(probes)
    except PortError as e:
        cached = roles_from_cache(load_cache(cache_path), ports)
        if cached is None:
            raise
        say(f"⚠ 電圧で判別できず、記憶した USB 位置で割り当てます: {e}")
        say(f"リーダー={cached.leader} / フォロワー={cached.follower}"
            "（ケーブルを差し替えた場合は誤りの可能性があります）")
        return cached
    say(f"判別 OK — リーダー={roles.leader} / フォロワー={roles.follower}")
    say(roles.detail)
    save_cache(roles, ports, cache_path)
    return roles


def wait_for_ports(leader: Optional[str] = None, follower: Optional[str] = None,
                   cache_path: pathlib.Path = DEFAULT_CACHE,
                   log: Optional[Callable[[str], None]] = None,
                   poll: float = 2.0, timeout: Optional[float] = None,
                   should_stop: Optional[Callable[[], bool]] = None,
                   sleep: Callable[[float], None] = time.sleep) -> Roles:
    """Block until both arms are identifiable, then return the assignment.

    Lets a server be started before the USB cables are plugged in. Repeated
    identical failures are logged once, so an unplugged rig does not fill the
    event log.
    """
    say = log or (lambda _m: None)
    deadline = None if timeout is None else time.monotonic() + timeout
    last = ""
    while True:
        if should_stop is not None and should_stop():
            raise PortError("中断されました")
        try:
            return resolve_ports(leader, follower, cache_path, log=say)
        except PortError as e:
            msg = str(e)
            if msg != last:
                say(msg.splitlines()[0])
                say(f"USB / 電源の接続を待っています（{poll:g}秒ごとに再検出）")
                last = msg
        if deadline is not None and time.monotonic() >= deadline:
            raise PortError(f"ポート検出がタイムアウトしました: {last}")
        sleep(poll)
