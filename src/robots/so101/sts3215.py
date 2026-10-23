"""Feetech STS3215 facts: register map, units, status bits, electrical specs.

Pure data + decoding helpers (no I/O, stdlib only). Every number here is from
a primary source; uncertain ones say so. Sources (fetched 2026-10-24):

- [F]   Feetech STS memory table (official wiki, 2026-10-17)
        https://github.com/ftservo/ftservo-wiki/blob/HEAD/docs/reference/parameter/memory-sts.md
- [D47] STS3215 12V datasheet (C047, Seeed mirror)
        https://files.seeedstudio.com/products/Feetech/108090003_FEETECH_ST-3215-C047-Datasheet.pdf
- [B]   Feetech actuator identification model (error_gain = 0.163)
        https://github.com/ftservo/ftservo-bam/blob/HEAD/bam/feetech/actuator.py
- [SDK] scservo_sdk protocol_packet_handler.py (error bit names)

Anything that controls the arm should import these names instead of repeating
magic numbers, so a correction lands everywhere at once.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Optional, Tuple

# --- register map (address, size) [F] -------------------------------------
REG: Dict[str, Tuple[int, int]] = {
    # EEPROM (persisted only when Lock(55) == 0; with Lock == 1 a write takes
    # effect immediately and is lost at power-off [F])
    "Firmware_Major": (0, 1),
    "Firmware_Minor": (1, 1),
    "Model_Number": (3, 2),
    "ID": (5, 1),
    "Baud_Rate": (6, 1),
    "Return_Delay_Time": (7, 1),
    "Response_Status_Level": (8, 1),
    "Min_Position_Limit": (9, 2),
    "Max_Position_Limit": (11, 2),
    "Max_Temperature_Limit": (13, 1),     # degC, default 70
    "Max_Voltage_Limit": (14, 1),         # 0.1 V
    "Min_Voltage_Limit": (15, 1),         # 0.1 V
    "Max_Torque_Limit": (16, 2),          # 0.1 %, copied into Torque_Limit at power-on
    "Phase": (18, 1),
    "Unloading_Condition": (19, 1),       # protection enable bits (STATUS_BITS)
    "LED_Alarm_Condition": (20, 1),
    "P_Coefficient": (21, 1),             # duty[0.1%] ~= (P/4) * error[step]  (inferred, see K_DUTY)
    "D_Coefficient": (22, 1),
    "I_Coefficient": (23, 1),
    "Minimum_Startup_Force": (24, 2),
    "CW_Dead_Zone": (26, 1),
    "CCW_Dead_Zone": (27, 1),
    "Protection_Current": (28, 2),        # 6.5 mA units
    "Angular_Resolution": (30, 1),
    "Homing_Offset": (31, 2),             # Present = raw - offset (BIT11 sign)
    "Operating_Mode": (33, 1),            # 0 position, 1 velocity, 2 PWM, 3 step
    "Protective_Torque": (34, 1),         # % of max torque left after an overload trip
    "Protection_Time": (35, 1),           # 10 ms units, overload must persist this long
    "Overload_Torque": (36, 1),           # % threshold that starts the overload timer
    "Velocity_P": (37, 1),
    "Over_Current_Protection_Time": (38, 1),  # 10 ms units
    "Velocity_I": (39, 1),
    # SRAM
    "Torque_Enable": (40, 1),
    "Acceleration": (41, 1),              # 100 step/s^2 units; 0 = maximum
    "Goal_Position": (42, 2),             # BIT15 sign
    "Goal_Time": (44, 2),
    "Goal_Velocity": (46, 2),             # step/s (Phase BIT2 = 1, factory default 12)
    "Torque_Limit": (48, 2),              # 0.1 % of stall torque
    "Lock": (55, 1),
    "Present_Position": (56, 2),          # BIT15 sign
    "Present_Velocity": (58, 2),          # step/s, BIT15 sign
    "Present_Load": (60, 2),              # drive duty, 0.1 %, BIT10 sign
    "Present_Voltage": (62, 1),           # 0.1 V
    "Present_Temperature": (63, 1),       # degC
    "Status": (65, 1),                    # STATUS_BITS, 1 = fault
    "Moving": (66, 1),
    "Present_Target": (67, 2),            # profile target per [F]; lerobot names 71 instead (unverified)
    "Present_Current": (69, 2),           # 6.5 mA units, BIT15 sign (per SDK)
}

# One sync read of this window returns everything the follower governor needs.
TELEMETRY_ADDR = 56
TELEMETRY_LEN = 15            # 56 .. 70 inclusive

# --- status / protection bits [F][SDK] --------------------------------------
STATUS_BITS = {
    0x01: "voltage",
    0x02: "sensor",           # magnetic encoder ("Angle sen" in the SDK)
    0x04: "overheat",
    0x08: "overcurrent",      # "OverEle" in the SDK
    0x20: "overload",
}
STATUS_JA = {
    "voltage": "電圧",
    "sensor": "エンコーダ",
    "overheat": "過熱",
    "overcurrent": "過電流",
    "overload": "過負荷",
}

# --- 12 V STS3215 electrical / mechanical specs [D47] -------------------------
SUPPLY_NOMINAL_V = 12.0
STALL_TORQUE_NM = 2.94        # 30 kg*cm @ 12 V
STALL_CURRENT_A = 2.7
NO_LOAD_RPM = 45.0            # 0.222 s / 60 deg
NO_LOAD_CURRENT_A = 0.18
RATED_TORQUE_NM = 0.98        # 10 kg*cm, continuous
RATED_CURRENT_A = 0.9
GEAR_RATIO = 345
BACKLASH_DEG = 0.5
TICKS_PER_REV = 4096
OVERCURRENT_TRIP_A = 2.0      # datasheet: > 2 A for 2 s; register default 3.25 A (conflict)
OVERHEAT_TRIP_C = 70

# Continuous effort the motor can dissipate, as a fraction of stall duty: the
# datasheet rates 10 of 30 kg*cm continuous.
CONTINUOUS_DUTY = RATED_TORQUE_NM / STALL_TORQUE_NM   # ~0.33

CURRENT_LSB_A = 0.0065
VOLTAGE_LSB_V = 0.1
LOAD_FULL_SCALE = 1000        # Present_Load / Torque_Limit full scale (100.0 %)


def k_duty_per_tick(p_coefficient: int) -> float:
    """Position-loop gain in duty-permille per tick of error.

    duty[0.1%] ~= (P/4) * error[step]: the memory table annotates P with
    "(1/4)" [F], and Feetech's identified model uses error_gain = 0.163 =
    (1/4) * 4096/(2*pi) / 1000 [B]. Not yet measured on this arm; the
    governor closes its loop on the measured load, so a wrong value changes
    its speed, not where it settles.
    """
    return max(0.25, float(p_coefficient) / 4.0)


# --- decoding helpers -------------------------------------------------------
def decode_sign(value: int, sign_bit: int) -> int:
    """Feetech sign-magnitude -> signed int."""
    mask = 1 << sign_bit
    if value & mask:
        return -(value & (mask - 1))
    return value & (mask - 1)


def encode_sign(value: int, sign_bit: int) -> int:
    """signed int -> Feetech sign-magnitude."""
    if value < 0:
        return (-value & ((1 << sign_bit) - 1)) | (1 << sign_bit)
    return value & ((1 << sign_bit) - 1)


def status_names(bits: int) -> Tuple[str, ...]:
    return tuple(name for bit, name in STATUS_BITS.items() if bits & bit)


def ticks_to_deg(ticks: float) -> float:
    return float(ticks) * 360.0 / TICKS_PER_REV


def deg_to_ticks(deg: float) -> float:
    return float(deg) * TICKS_PER_REV / 360.0


@dataclass
class ServoState:
    """One servo's telemetry window (addresses 56..70), decoded.

    `ok` is False when the servo did not answer this cycle; the other fields
    then hold the last good values (see `age`) so the governor can tell a stale
    reading from a fresh one.
    """
    ok: bool = False
    pos: int = 0              # homed ticks
    vel: int = 0              # step/s
    load: int = 0             # duty permille, signed
    volt: float = 0.0         # V
    temp: int = 0             # degC
    status: int = 0           # STATUS_BITS
    moving: int = 0
    target: int = 0           # profile target (address 67, firmware dependent)
    current: float = 0.0      # A, signed
    err: int = 0              # error byte of the status packet (same bits as `status`)
    t: float = 0.0            # host time of the read (perf_counter)
    age: int = 0              # consecutive cycles without a fresh read
    faults: Tuple[str, ...] = field(default_factory=tuple)

    @property
    def vel_dps(self) -> float:
        return ticks_to_deg(self.vel)

    @property
    def duty(self) -> float:
        """|Present_Load| as a 0..1 fraction of full drive."""
        return min(1.0, abs(self.load) / LOAD_FULL_SCALE)


def decode_telemetry(data: bytes, t: float = 0.0, err: int = 0) -> ServoState:
    """Decode the 15-byte window starting at TELEMETRY_ADDR."""
    if len(data) < TELEMETRY_LEN:
        raise ValueError(f"telemetry window needs {TELEMETRY_LEN} bytes, got {len(data)}")

    def u16(off: int) -> int:
        return data[off] | (data[off + 1] << 8)

    status = data[65 - TELEMETRY_ADDR] | (err & 0x2F)
    return ServoState(
        ok=True,
        pos=decode_sign(u16(0), 15),
        vel=decode_sign(u16(2), 15),
        load=decode_sign(u16(4), 10),
        volt=data[6] * VOLTAGE_LSB_V,
        temp=data[7],
        status=status,
        moving=data[66 - TELEMETRY_ADDR],
        target=decode_sign(u16(67 - TELEMETRY_ADDR), 15),
        current=decode_sign(u16(69 - TELEMETRY_ADDR), 15) * CURRENT_LSB_A,
        err=err,
        t=t,
        age=0,
        faults=status_names(status),
    )


def encode_telemetry(st: ServoState) -> bytes:
    """Inverse of decode_telemetry (used by the simulated bus)."""
    out = bytearray(TELEMETRY_LEN)

    def put16(off: int, v: int) -> None:
        out[off] = v & 0xFF
        out[off + 1] = (v >> 8) & 0xFF

    put16(0, encode_sign(int(st.pos), 15))
    put16(2, encode_sign(int(st.vel), 15))
    put16(4, encode_sign(int(st.load), 10))
    out[6] = max(0, min(255, int(round(st.volt / VOLTAGE_LSB_V))))
    out[7] = max(0, min(255, int(st.temp)))
    out[65 - TELEMETRY_ADDR] = st.status & 0xFF
    out[66 - TELEMETRY_ADDR] = st.moving & 0xFF
    put16(67 - TELEMETRY_ADDR, encode_sign(int(st.target), 15))
    put16(69 - TELEMETRY_ADDR, encode_sign(int(round(st.current / CURRENT_LSB_A)), 15))
    return bytes(out)


def describe_faults(faults) -> str:
    return "・".join(STATUS_JA.get(f, f) for f in faults)
