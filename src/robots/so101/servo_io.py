"""Fault-tolerant Feetech bus I/O for the hot loop.

Why not just lerobot's FeetechMotorsBus
---------------------------------------
- `read()` / `write()` raise RuntimeError whenever a servo reports a status bit
  (overload, overheat, ...). A servo in protection then breaks every call that
  touches it - including the torque-off of a relax sequence, which used to stop
  half way and leave the shoulder energized when the elbow was the one tripped.
- `sync_read()` goes through the SDK's GroupSyncRead, which throws the status
  byte away and, when one ID does not answer, also loses every ID after it.
- The SDK's readRx accepts the first packet whose ID matches without checking
  its length, so a late reply left in the receive buffer can be taken as the
  answer to the NEXT, unrelated read. Mixing our own short-deadline reads with
  SDK calls on one port is therefore unsafe: once the follower is connected,
  every transaction on it goes through this module.

Transaction hygiene (RawFeetechIO)
----------------------------------
- drop whatever is in the receive buffer, then send;
- accept only checksummed packets whose ID AND length match the request;
- own deadline (a silent servo costs ~8 ms, not lerobot's 50 ms);
- after a deadline, wait 2 ms and drop the buffer again so late bytes cannot
  leak into the next transaction.

Status bits are returned as data and never raised. Writes on the hot path are
sync writes (no reply to wait for).

`LegacyBusIO` offers the same interface over any lerobot-like bus
(sync_read / sync_write / read / write) for fakes and tests.
"""
from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Callable, Dict, Iterable, List, Optional, Protocol, Tuple

from . import sts3215
from .sts3215 import REG, ServoState

INST_READ = 0x02
INST_WRITE = 0x03
INST_SYNC_READ = 0x82
INST_SYNC_WRITE = 0x83
BROADCAST_ID = 0xFE

# signed registers and the bit that carries their sign
SIGN_BIT = {
    "Goal_Position": 15, "Goal_Velocity": 15, "Present_Position": 15,
    "Present_Velocity": 15, "Present_Load": 10, "Homing_Offset": 11,
    "Present_Current": 15, "Present_Target": 15,
}

# Per-cycle command window: Acceleration(41) Goal_Position(42-43) Goal_Time(44-45)
# Goal_Velocity(46-47) Torque_Limit(48-49). One sync write sets all of it; this is
# the SDK's SyncWritePosEx window plus the torque limit. SRAM, so writing it
# every cycle wears nothing.
COMMAND_ADDR = 41
COMMAND_LEN = 9


@dataclass
class Command:
    """What one servo is told this cycle."""
    goal: int
    velocity: int
    acceleration: int
    torque_limit: int = 1000


def encode_command(c: Command) -> List[int]:
    g = sts3215.encode_sign(int(c.goal), 15)
    v = sts3215.encode_sign(int(c.velocity), 15)
    tl = max(0, min(1000, int(c.torque_limit)))
    acc = max(0, min(254, int(c.acceleration)))
    return [acc, g & 0xFF, (g >> 8) & 0xFF, 0, 0, v & 0xFF, (v >> 8) & 0xFF,
            tl & 0xFF, (tl >> 8) & 0xFF]


class ServoIO(Protocol):
    """What the follower governor and the teleop engine need from a bus."""

    names: List[str]

    def read_telemetry(self, names: Optional[Iterable[str]] = None) -> Dict[str, ServoState]: ...
    def read_positions(self, names: Optional[Iterable[str]] = None) -> Dict[str, Optional[int]]: ...
    def sync_write(self, reg: str, values: Dict[str, int]) -> None: ...
    def write_commands(self, cmds: Dict[str, Command]) -> None: ...
    def read_reg(self, reg: str, name: str) -> Tuple[Optional[int], int]: ...
    def write_reg(self, reg: str, name: str, value: int) -> Tuple[bool, int]: ...


def _checksum(body: Iterable[int]) -> int:
    return (~sum(body)) & 0xFF


def _packet(sid: int, inst: int, params: List[int]) -> bytes:
    body = [sid, len(params) + 2, inst] + list(params)
    return bytes([0xFF, 0xFF] + body + [_checksum(body)])


def build_sync_read(addr: int, length: int, ids: List[int]) -> bytes:
    return _packet(BROADCAST_ID, INST_SYNC_READ, [addr, length] + list(ids))


def build_sync_write(addr: int, length: int, items: List[Tuple[int, List[int]]]) -> bytes:
    params: List[int] = [addr, length]
    for sid, data in items:
        params.append(sid)
        params.extend(list(data)[:length])
    return _packet(BROADCAST_ID, INST_SYNC_WRITE, params)


def build_read(sid: int, addr: int, length: int) -> bytes:
    return _packet(sid, INST_READ, [addr, length])


def build_write(sid: int, addr: int, data: List[int]) -> bytes:
    return _packet(sid, INST_WRITE, [addr] + list(data))


def parse_status_packets(buf: bytearray, length: int, wanted: Iterable[int]
                         ) -> Tuple[Dict[int, Tuple[bytes, int]], int]:
    """Pull every complete, checksummed status packet for `wanted` out of `buf`.

    Only packets whose ID is wanted AND whose length matches the request are
    returned ({id: (data, error_byte)}); echoed instructions, other replies and
    corrupt bytes are skipped. Returns (found, bytes_consumed); an incomplete
    tail is left for the next call.
    """
    wanted = set(wanted)
    found: Dict[int, Tuple[bytes, int]] = {}
    i = 0
    n = len(buf)
    while i + 4 <= n:
        if buf[i] != 0xFF or buf[i + 1] != 0xFF:
            i += 1
            continue
        pid, plen = buf[i + 2], buf[i + 3]
        if pid == 0xFF:              # FF FF FF ... : the real header starts one later
            i += 1
            continue
        total = plen + 4
        if plen < 2 or total > 250:
            i += 1
            continue
        if i + total > n:
            break                    # wait for the rest
        pkt = buf[i:i + total]
        if pkt[-1] != _checksum(pkt[2:-1]):
            i += 1
            continue
        if pid in wanted and plen == length + 2:
            found[pid] = (bytes(pkt[5:5 + length]), pkt[4])
        i += total                   # well-formed: an echo, another ID, or ours
    return found, i


def _u(data: bytes, length: int) -> int:
    return data[0] if length == 1 else data[0] | (data[1] << 8)


def _encode_value(reg: str, value: int) -> List[int]:
    length = REG[reg][1]
    v = int(value)
    sb = SIGN_BIT.get(reg)
    if sb is not None:
        v = sts3215.encode_sign(v, sb)
    return [v & 0xFF] if length == 1 else [v & 0xFF, (v >> 8) & 0xFF]


class RawFeetechIO:
    """Protocol-0 I/O on the serial port of an open lerobot FeetechMotorsBus.

    After construction, nothing else may talk on this port (see module doc).
    """

    def __init__(self, bus, motors: Optional[Dict[str, int]] = None, *,
                 base_timeout_s: float = 0.008,
                 clock: Callable[[], float] = time.perf_counter,
                 sleep: Callable[[float], None] = time.sleep):
        self.bus = bus
        self.ser = bus.port_handler.ser
        if motors is None:
            motors = {name: m.id for name, m in bus.motors.items()}
        self.ids = dict(motors)
        self.names = list(motors)
        self._by_id = {sid: n for n, sid in self.ids.items()}
        self.base_timeout_s = base_timeout_s
        self.clock = clock
        self.sleep = sleep
        self._last: Dict[str, ServoState] = {n: ServoState() for n in self.names}
        self.stats = {"transactions": 0, "timeouts": 0, "missing": {}, "last_read_ms": 0.0}

    # -- raw transaction ------------------------------------------------------
    def _transact(self, pkt: bytes, ids: List[int], length: int
                  ) -> Dict[int, Tuple[bytes, int]]:
        ser = self.ser
        ser.reset_input_buffer()
        ser.write(pkt)
        self.stats["transactions"] += 1
        if not ids:
            return {}
        expected = len(ids) * (length + 6)
        deadline = self.clock() + self.base_timeout_s + expected * 10e-6 * 1.5
        buf = bytearray()
        got: Dict[int, Tuple[bytes, int]] = {}
        echo_checked = False
        while len(got) < len(ids) and self.clock() < deadline:
            chunk = ser.read(ser.in_waiting or 1)
            if not chunk:
                continue
            buf.extend(chunk)
            if not echo_checked:
                # A half-duplex adapter may hand our own instruction back first.
                # A 2-byte READ instruction is shaped exactly like a 2-byte reply
                # (FF FF id 04 ..), so it must be removed, not parsed.
                if len(buf) < len(pkt) and bytes(buf) == pkt[:len(buf)]:
                    continue
                if bytes(buf[:len(pkt)]) == pkt:
                    del buf[:len(pkt)]
                echo_checked = True
            found, used = parse_status_packets(buf, length, ids)
            got.update(found)
            del buf[:used]
        if len(got) < len(ids):
            # a reply may still be on its way: let it land, then drop it so it
            # cannot be taken for the answer to the next request
            self.stats["timeouts"] += 1
            self.sleep(0.002)
            ser.reset_input_buffer()
        return got

    # -- reads ----------------------------------------------------------------
    def read_telemetry(self, names: Optional[Iterable[str]] = None) -> Dict[str, ServoState]:
        names = list(names) if names is not None else list(self.names)
        ids = [self.ids[n] for n in names]
        t0 = self.clock()
        raw = self._transact(build_sync_read(sts3215.TELEMETRY_ADDR, sts3215.TELEMETRY_LEN, ids),
                             ids, sts3215.TELEMETRY_LEN)
        now = self.clock()
        self.stats["last_read_ms"] = round((now - t0) * 1000.0, 2)
        out: Dict[str, ServoState] = {}
        for n in names:
            sid = self.ids[n]
            if sid in raw:
                data, err = raw[sid]
                st = sts3215.decode_telemetry(data, t=now, err=err)
            else:
                prev = self._last.get(n, ServoState())
                st = ServoState(**{**prev.__dict__, "ok": False, "age": prev.age + 1})
                self.stats["missing"][n] = self.stats["missing"].get(n, 0) + 1
            self._last[n] = st
            out[n] = st
        return out

    def read_positions(self, names: Optional[Iterable[str]] = None) -> Dict[str, Optional[int]]:
        names = list(names) if names is not None else list(self.names)
        ids = [self.ids[n] for n in names]
        raw = self._transact(build_sync_read(REG["Present_Position"][0], 2, ids), ids, 2)
        out: Dict[str, Optional[int]] = {}
        for n in names:
            sid = self.ids[n]
            if sid in raw:
                out[n] = sts3215.decode_sign(_u(raw[sid][0], 2), 15)
            else:
                out[n] = None
                self.stats["missing"][n] = self.stats["missing"].get(n, 0) + 1
        return out

    def read_reg(self, reg: str, name: str) -> Tuple[Optional[int], int]:
        addr, length = REG[reg]
        sid = self.ids[name]
        raw = self._transact(build_read(sid, addr, length), [sid], length)
        if sid not in raw:
            return None, 0
        data, err = raw[sid]
        v = _u(data, length)
        sb = SIGN_BIT.get(reg)
        return (sts3215.decode_sign(v, sb) if sb is not None else v), err

    # -- writes ---------------------------------------------------------------
    def write_reg(self, reg: str, name: str, value: int) -> Tuple[bool, int]:
        """Acknowledged single write. Status bits do not make it fail."""
        addr, _length = REG[reg]
        sid = self.ids[name]
        raw = self._transact(build_write(sid, addr, _encode_value(reg, value)), [sid], 0)
        if sid not in raw:
            return False, 0
        return True, raw[sid][1]

    def sync_write(self, reg: str, values: Dict[str, int]) -> None:
        """Unacknowledged write to many servos (a tripped servo cannot block it)."""
        if not values:
            return
        addr, length = REG[reg]
        items = [(self.ids[n], _encode_value(reg, v)) for n, v in values.items()]
        self._transact(build_sync_write(addr, length, items), [], 0)

    def write_commands(self, cmds: Dict[str, Command]) -> None:
        if not cmds:
            return
        items = [(self.ids[n], encode_command(c)) for n, c in cmds.items()]
        self._transact(build_sync_write(COMMAND_ADDR, COMMAND_LEN, items), [], 0)


class LegacyBusIO:
    """ServoIO on any lerobot-like bus (sync_read / sync_write / read / write).

    Slower (one register per transaction) and it cannot see status bytes, but
    it lets the engine run on the unit tests' fake buses. Errors from a single
    servo are isolated to it.
    """

    _FIELDS = (("Present_Velocity", "vel"), ("Present_Load", "load"),
               ("Present_Voltage", "volt"), ("Present_Temperature", "temp"),
               ("Status", "status"), ("Present_Current", "current"))

    def __init__(self, bus, names: Optional[List[str]] = None,
                 clock: Callable[[], float] = time.perf_counter):
        self.bus = bus
        if names is None:
            motors = getattr(bus, "motors", None)
            names = list(motors) if motors else []
        self.names = list(names)
        self.clock = clock
        self._last: Dict[str, ServoState] = {n: ServoState() for n in self.names}
        self.stats = {"transactions": 0, "timeouts": 0, "missing": {}, "last_read_ms": 0.0}

    def read_positions(self, names: Optional[Iterable[str]] = None) -> Dict[str, Optional[int]]:
        names = list(names) if names is not None else list(self.names)
        try:
            pos = self.bus.sync_read("Present_Position", normalize=False, num_retry=1)
            return {n: (int(pos[n]) if n in pos else None) for n in names}
        except Exception:
            out: Dict[str, Optional[int]] = {}
            for n in names:
                try:
                    out[n] = int(self.bus.read("Present_Position", n, normalize=False))
                except Exception:
                    out[n] = None
            return out

    def read_telemetry(self, names: Optional[Iterable[str]] = None) -> Dict[str, ServoState]:
        names = list(names) if names is not None else list(self.names)
        now = self.clock()
        pos = self.read_positions(names)
        out: Dict[str, ServoState] = {}
        for n in names:
            prev = self._last.get(n, ServoState())
            if pos.get(n) is None:
                st = ServoState(**{**prev.__dict__, "ok": False, "age": prev.age + 1})
                self.stats["missing"][n] = self.stats["missing"].get(n, 0) + 1
            else:
                st = ServoState(ok=True, pos=int(pos[n]), t=now)
                for reg, attr in self._FIELDS:
                    try:
                        v = self.bus.read(reg, n, normalize=False)
                        if reg == "Present_Voltage":
                            v = v / 10.0
                        elif reg == "Present_Current":
                            v = v * sts3215.CURRENT_LSB_A
                    except Exception:
                        v = getattr(prev, attr)
                    setattr(st, attr, v)
                st.faults = sts3215.status_names(int(st.status))
            self._last[n] = st
            out[n] = st
        return out

    def sync_write(self, reg: str, values: Dict[str, int]) -> None:
        if values:
            self.bus.sync_write(reg, {n: int(v) for n, v in values.items()}, normalize=False)

    def write_commands(self, cmds: Dict[str, Command]) -> None:
        if not cmds:
            return
        self.sync_write("Acceleration", {n: c.acceleration for n, c in cmds.items()})
        self.sync_write("Goal_Velocity", {n: c.velocity for n, c in cmds.items()})
        self.sync_write("Torque_Limit", {n: c.torque_limit for n, c in cmds.items()})
        self.sync_write("Goal_Position", {n: c.goal for n, c in cmds.items()})

    def read_reg(self, reg: str, name: str) -> Tuple[Optional[int], int]:
        try:
            return int(self.bus.read(reg, name, normalize=False)), 0
        except Exception:
            return None, 0

    def write_reg(self, reg: str, name: str, value: int) -> Tuple[bool, int]:
        try:
            self.bus.write(reg, name, int(value), normalize=False)
            return True, 0
        except Exception:
            return False, 0


def wrap_bus(bus, names: Optional[List[str]] = None):
    """Best available ServoIO for `bus`: the bus itself if it already speaks
    ServoIO (the simulated follower), raw protocol if it has a real serial
    port, the legacy adapter otherwise (fakes, mocks)."""
    if hasattr(bus, "read_telemetry") and hasattr(bus, "write_commands"):
        return bus
    port = getattr(bus, "port_handler", None)
    if port is not None and getattr(port, "ser", None) is not None:
        motors = None
        if names is not None and hasattr(bus, "motors"):
            motors = {n: bus.motors[n].id for n in names}
        return RawFeetechIO(bus, motors)
    return LegacyBusIO(bus, names)
