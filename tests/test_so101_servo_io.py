"""Protocol-level tests for the fault-tolerant Feetech I/O (no hardware).

A fake serial port with its own clock answers the way an STS3215 chain does:
one status packet per servo after a latency, optionally preceded by the echo
of the instruction (half-duplex boards). Servos can go silent, corrupt a byte,
answer late, or report protection bits.

Run: python -m unittest discover tests -p "test_so101_servo_io.py"
"""
import pathlib
import sys
import unittest

ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "src"))

from robots.so101 import sts3215  # noqa: E402
from robots.so101.servo_io import (  # noqa: E402
    COMMAND_ADDR, COMMAND_LEN, Command, LegacyBusIO, RawFeetechIO, _checksum,
    build_sync_read, build_sync_write, encode_command, parse_status_packets, wrap_bus,
)
from robots.so101.sts3215 import REG, ServoState  # noqa: E402

NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
IDS = {n: i + 1 for i, n in enumerate(NAMES)}


def status_packet(sid, data, err=0):
    body = [sid, len(data) + 2, err] + list(data)
    return bytes([0xFF, 0xFF] + body + [_checksum(body)])


class FakeSerial:
    """Servo chain behind a half-duplex adapter, with a virtual clock."""

    LATENCY = 0.0015      # first byte after the request
    TICK = 0.0002         # time that passes per read() poll

    def __init__(self, states, echo=True):
        self.states = states            # id -> ServoState
        self.regs = {sid: {} for sid in states}
        self.echo = echo
        self.silent = set()
        self.corrupt = set()
        self.late = {}                  # id -> extra delay (s)
        self.errbits = {}
        self.now = 0.0
        self.pending = []               # (arrival_time, bytes)
        self.rx = bytearray()
        self.written = []

    # clock / sleep handed to RawFeetechIO
    def clock(self):
        return self.now

    def sleep(self, s):
        self.now += s
        self._deliver()

    def _deliver(self):
        keep = []
        for at, data in self.pending:
            if at <= self.now:
                self.rx.extend(data)
            else:
                keep.append((at, data))
        self.pending = keep

    # pyserial surface
    @property
    def in_waiting(self):
        self._deliver()
        return len(self.rx)

    def read(self, n):
        self.now += self.TICK
        self._deliver()
        out = bytes(self.rx[:n])
        del self.rx[:n]
        return out

    def reset_input_buffer(self):
        self._deliver()
        self.rx.clear()

    def _reply(self, sid, data, t):
        if sid in self.silent:
            return
        p = bytearray(status_packet(sid, data, self.errbits.get(sid, 0)))
        if sid in self.corrupt:
            p[-2] ^= 0x55
        self.pending.append((t + self.late.get(sid, 0.0), bytes(p)))

    def write(self, pkt):
        pkt = bytes(pkt)
        self.written.append(pkt)
        if self.echo:
            self.rx.extend(pkt)
        sid, inst = pkt[2], pkt[4]
        t = self.now + self.LATENCY
        if inst == 0x82:                                   # sync read
            addr, length = pkt[5], pkt[6]
            for k, rid in enumerate(pkt[7:-1]):
                window = sts3215.encode_telemetry(self.states[rid])
                self._reply(rid, window[addr - sts3215.TELEMETRY_ADDR:][:length], t + k * 2e-4)
        elif inst == 0x02:                                 # read
            addr, length = pkt[5], pkt[6]
            v = self.regs[sid].get(addr, addr)             # unknown regs read back their address
            data = [v & 0xFF] if length == 1 else [v & 0xFF, (v >> 8) & 0xFF]
            self._reply(sid, data, t)
        elif inst == 0x03:                                 # write
            addr = pkt[5]
            data = pkt[6:-1]
            self.regs[sid][addr] = data[0] if len(data) == 1 else data[0] | (data[1] << 8)
            self._reply(sid, b"", t)
        return len(pkt)


class FakePortHandler:
    def __init__(self, ser):
        self.ser = ser


class FakeMotor:
    def __init__(self, i):
        self.id = i


class FakeRawBus:
    def __init__(self, ser):
        self.port_handler = FakePortHandler(ser)
        self.motors = {n: FakeMotor(i) for n, i in IDS.items()}


def make_states():
    st = {}
    for n, i in IDS.items():
        st[i] = ServoState(ok=True, pos=2048 + 10 * i, vel=-5 * i, load=-(100 + i),
                           volt=12.2, temp=30 + i, status=0, moving=0,
                           target=2048, current=0.0065 * 20 * i)
    return st


def make_io(**kw):
    ser = FakeSerial(make_states(), **kw)
    io = RawFeetechIO(FakeRawBus(ser), base_timeout_s=0.004, clock=ser.clock, sleep=ser.sleep)
    return ser, io


class TestEncoding(unittest.TestCase):
    def test_sign_magnitude_round_trip(self):
        for bit in (10, 11, 15):
            for v in (0, 1, 500, -1, -500, (1 << bit) - 1, -((1 << bit) - 1)):
                self.assertEqual(sts3215.decode_sign(sts3215.encode_sign(v, bit), bit), v)

    def test_telemetry_window_round_trip(self):
        s = ServoState(ok=True, pos=-12, vel=345, load=-678, volt=12.1, temp=47,
                       status=0x20, moving=1, target=2100, current=-1.3)
        d = sts3215.decode_telemetry(sts3215.encode_telemetry(s))
        self.assertEqual((d.pos, d.vel, d.load, d.temp, d.status, d.moving, d.target),
                         (-12, 345, -678, 47, 0x20, 1, 2100))
        self.assertAlmostEqual(d.volt, 12.1, places=6)
        self.assertAlmostEqual(d.current, -1.3, delta=0.0065)
        self.assertEqual(d.faults, ("overload",))

    def test_packet_error_byte_merges_into_status(self):
        d = sts3215.decode_telemetry(sts3215.encode_telemetry(ServoState(ok=True)), err=0x08)
        self.assertIn("overcurrent", d.faults)

    def test_sync_read_packet_matches_the_protocol(self):
        pkt = build_sync_read(56, 15, [1, 2, 3])
        self.assertEqual(list(pkt[:7]), [0xFF, 0xFF, 0xFE, 7, 0x82, 56, 15])
        self.assertEqual(pkt[-1], _checksum(pkt[2:-1]))

    def test_sync_write_packet_matches_the_protocol(self):
        pkt = build_sync_write(42, 2, [(1, [0x00, 0x08]), (2, [0x10, 0x08])])
        self.assertEqual(pkt[3], (2 + 1) * 2 + 4)          # LEN = (L+1)*N + 4
        self.assertEqual(list(pkt[7:13]), [1, 0x00, 0x08, 2, 0x10, 0x08])

    def test_command_window_layout(self):
        # 41 acc, 42-43 goal, 44-45 time (0), 46-47 velocity, 48-49 torque limit
        self.assertEqual(COMMAND_ADDR, REG["Acceleration"][0])
        self.assertEqual(COMMAND_ADDR + COMMAND_LEN, REG["Torque_Limit"][0] + 2)
        self.assertEqual(encode_command(Command(goal=2100, velocity=512, acceleration=50,
                                                torque_limit=1000)),
                         [50, 0x34, 0x08, 0, 0, 0x00, 0x02, 0xE8, 0x03])


class TestParser(unittest.TestCase):
    def test_skips_echo_and_noise(self):
        echo = build_sync_read(56, 2, [1, 2])
        buf = bytearray(b"\x00\x13" + echo + status_packet(1, b"\x10\x08")
                        + status_packet(2, b"\x20\x08"))
        found, used = parse_status_packets(buf, 2, [1, 2])
        self.assertEqual(set(found), {1, 2})
        self.assertEqual(used, len(buf))

    def test_leaves_an_incomplete_tail(self):
        p = status_packet(1, b"\x10\x08")
        found, used = parse_status_packets(bytearray(p[:-2]), 2, [1])
        self.assertEqual((found, used), ({}, 0))

    def test_rejects_bad_checksum(self):
        p = bytearray(status_packet(1, b"\x10\x08"))
        p[-1] ^= 1
        self.assertEqual(parse_status_packets(p, 2, [1])[0], {})

    def test_rejects_a_reply_of_the_wrong_length(self):
        # e.g. a telemetry reply from the same servo is not the answer to a 2-byte read
        p = status_packet(1, bytes(15))
        self.assertEqual(parse_status_packets(bytearray(p), 2, [1])[0], {})


class TestRawIO(unittest.TestCase):
    def test_reads_every_servo_in_one_transaction(self):
        ser, io = make_io()
        t = io.read_telemetry()
        self.assertEqual(len(ser.written), 1)
        self.assertTrue(all(t[n].ok for n in NAMES))
        self.assertEqual(t["elbow_flex"].pos, 2048 + 30)
        self.assertEqual(t["elbow_flex"].load, -103)
        self.assertAlmostEqual(t["elbow_flex"].current, 0.0065 * 60, delta=1e-6)

    def test_a_silent_servo_costs_only_itself(self):
        ser, io = make_io()
        ser.silent.add(IDS["elbow_flex"])
        t = io.read_telemetry()
        self.assertFalse(t["elbow_flex"].ok)
        for n in NAMES:
            if n != "elbow_flex":
                self.assertTrue(t[n].ok, f"{n} was lost behind the silent elbow")
        self.assertEqual(t["elbow_flex"].age, 1)

    def test_a_silent_servo_costs_milliseconds_not_lerobots_50(self):
        ser, io = make_io()
        ser.silent.add(IDS["elbow_flex"])
        t0 = ser.now
        io.read_telemetry()
        self.assertLess(ser.now - t0, 0.012)

    def test_a_corrupt_packet_is_dropped_not_misread(self):
        ser, io = make_io()
        ser.corrupt.add(IDS["wrist_flex"])
        t = io.read_telemetry()
        self.assertFalse(t["wrist_flex"].ok)
        self.assertTrue(t["wrist_roll"].ok)

    def test_protection_bits_are_data_not_exceptions(self):
        ser, io = make_io()
        ser.errbits[IDS["elbow_flex"]] = 0x20
        t = io.read_telemetry()
        self.assertTrue(t["elbow_flex"].ok)
        self.assertEqual(t["elbow_flex"].faults, ("overload",))

    def test_works_without_echo(self):
        ser, io = make_io(echo=False)
        self.assertTrue(all(s.ok for s in io.read_telemetry().values()))

    def test_stale_values_survive_a_dropout(self):
        ser, io = make_io()
        first = io.read_telemetry()["shoulder_lift"]
        ser.silent.add(IDS["shoulder_lift"])
        again = io.read_telemetry()["shoulder_lift"]
        self.assertFalse(again.ok)
        self.assertEqual(again.pos, first.pos)

    def test_positions_only(self):
        ser, io = make_io()
        self.assertEqual(io.read_positions(["shoulder_pan", "gripper"]),
                         {"shoulder_pan": 2058, "gripper": 2108})

    def test_a_late_reply_never_leaks_into_the_next_read(self):
        ser, io = make_io()
        ser.late[IDS["elbow_flex"]] = 0.006          # lands after the deadline
        t = io.read_telemetry()
        self.assertFalse(t["elbow_flex"].ok)
        ser.late.clear()
        ser.regs[IDS["elbow_flex"]][REG["P_Coefficient"][0]] = 16
        v, _ = io.read_reg("P_Coefficient", "elbow_flex")
        self.assertEqual(v, 16, "a stale telemetry packet was taken as the register value")

    def test_read_and_write_register(self):
        ser, io = make_io()
        ok, err = io.write_reg("Torque_Limit", "gripper", 500)
        self.assertTrue(ok)
        self.assertEqual(err, 0)
        self.assertEqual(io.read_reg("Torque_Limit", "gripper")[0], 500)

    def test_status_bits_do_not_fail_a_write(self):
        ser, io = make_io()
        ser.errbits[IDS["elbow_flex"]] = 0x08
        ok, err = io.write_reg("Torque_Enable", "elbow_flex", 0)
        self.assertTrue(ok)
        self.assertEqual(err, 0x08)

    def test_signed_goal_write(self):
        ser, io = make_io()
        io.sync_write("Goal_Position", {"shoulder_pan": 2100, "gripper": -3})
        pkt = ser.written[-1]
        self.assertEqual(pkt[4], 0x83)
        self.assertEqual(list(pkt[-4:-1]), [6, 0x03, 0x80])   # -3 -> 0x8003

    def test_commands_are_one_sync_write(self):
        ser, io = make_io()
        io.write_commands({n: Command(2048, 512, 50) for n in NAMES})
        pkt = ser.written[-1]
        self.assertEqual((pkt[4], pkt[5], pkt[6]), (0x83, COMMAND_ADDR, COMMAND_LEN))
        self.assertEqual(len(ser.written), 1)


class FakeLegacyBus:
    def __init__(self):
        self.pos = {n: 2048 for n in NAMES}
        self.sick = set()
        self.writes = []

    def sync_read(self, reg, normalize=False, num_retry=1):
        return dict(self.pos)

    def sync_write(self, reg, values, normalize=False):
        self.writes.append((reg, dict(values)))

    def read(self, reg, motor, normalize=False):
        if motor in self.sick and reg != "Present_Position":
            raise RuntimeError("[RxPacketError] Overload error!")
        return {"Present_Voltage": 121, "Present_Temperature": 33,
                "Present_Load": 250, "Present_Current": 40}.get(reg, 0)

    def write(self, reg, motor, value, normalize=False):
        if motor in self.sick:
            raise RuntimeError("[RxPacketError] Overload error!")


class TestLegacyAdapter(unittest.TestCase):
    def test_wrap_picks_the_adapter_for_fakes(self):
        self.assertIsInstance(wrap_bus(FakeLegacyBus(), NAMES), LegacyBusIO)

    def test_wrap_picks_raw_for_a_real_port(self):
        self.assertIsInstance(wrap_bus(FakeRawBus(FakeSerial(make_states()))), RawFeetechIO)

    def test_units(self):
        t = LegacyBusIO(FakeLegacyBus(), NAMES).read_telemetry()["elbow_flex"]
        self.assertAlmostEqual(t.volt, 12.1)
        self.assertAlmostEqual(t.current, 0.26, places=6)
        self.assertEqual(t.load, 250)

    def test_a_sick_servo_does_not_raise(self):
        bus = FakeLegacyBus()
        bus.sick.add("elbow_flex")
        io = LegacyBusIO(bus, NAMES)
        self.assertTrue(io.read_telemetry()["elbow_flex"].ok)   # position still answers
        ok, _ = io.write_reg("Torque_Limit", "elbow_flex", 300)
        self.assertFalse(ok)

    def test_commands_fan_out_to_registers(self):
        bus = FakeLegacyBus()
        LegacyBusIO(bus, NAMES).write_commands({"gripper": Command(2000, 400, 40, 500)})
        regs = [r for r, _ in bus.writes]
        self.assertEqual(regs[-1], "Goal_Position")              # goal goes last
        self.assertIn("Torque_Limit", regs)


if __name__ == "__main__":
    unittest.main()
