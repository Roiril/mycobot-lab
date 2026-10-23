"""Simulated SO-101 follower: STS3215 servo physics on the MuJoCo arm.

What it is for
--------------
Validating the follower control logic (teleop_engine + effort governor) and the
cockpit without hardware: the same code that drives the real bus drives this
one, through the ServoIO interface (read_telemetry / read_positions /
sync_write / read_reg / write_reg).

It is a MODEL. Use it to check that the logic does what it should (does a
blocked joint yield, does an overload trip happen, how does temperature move
relative to effort) - not to claim real-world numbers. Parameters marked
ASSUMED below are not in any datasheet.

Per servo (1 kHz)
-----------------
- Internal trapezoidal profile: the setpoint chases Goal_Position with
  |v| <= Goal_Velocity [step/s] and |a| <= Acceleration * 100 [step/s^2]
  (0 means "maximum" for both) and is re-planned from its own state whenever a
  new goal arrives.
- Position loop: duty[permille] = (P/4) * (setpoint - pos) + D term, clamped to
  Torque_Limit and to whatever a protection leaves. (P/4 is Feetech's
  annotation + identified error gain; the D scaling is ASSUMED.)
- DC motor behind a 1/345 gear, output-side constants from the 12 V datasheet
  (stall 2.94 N*m / 2.7 A, no-load 45 rpm / 0.18 A):
      I = (duty * V - Ke * w) / R,  tau = Kt * I
  R = 12/2.7 = 4.44 ohm, Ke = 2.38 V*s/rad, Kt = 1.09 N*m/A. Their product
  Kt*Ke/R = 0.58 N*m*s/rad is the back-EMF damping the vendored MJCF calls
  joint damping (0.60) - so the MJCF damping is switched off here to avoid
  counting it twice.
- Protections per the datasheet / memory table:
    overload     |duty| > Overload_Torque % for Protection_Time -> output
                 limited to Protective_Torque %; a new goal clears it
    overcurrent  |I| > 2.0 A for Over_Current_Protection_Time -> output off;
                 a new goal clears it
    overheat     T > Max_Temperature_Limit -> output off until 5 C below
  Whether "80 %" is measured against full scale or against Torque_Limit is not
  documented; `overload_vs_torque_limit` selects the interpretation.
- Temperature: one thermal node, P = I^2 R, R_th = 3 C/W, C_th = 29 J/C
  (ASSUMED; fits a 15 kg*cm static hold reaching +23 C in 10 min and the
  observed elbow overload reaching 82 C within a couple of minutes).

Arm and world
-------------
MuJoCo integrates the vendored SO-101 model with the servo torques applied as
generalized forces (the MJCF position actuators are zeroed). A table plane can
push back on the gripper, mechanical stops can be made narrower than the
calibrated range, and a payload can hang at the TCP. The 12 V / 5 A supply sags
with total current.
"""
from __future__ import annotations

import math
import pathlib
import threading
import time
from dataclasses import dataclass, field
from typing import Callable, Dict, Iterable, List, Optional, Tuple

from .. import profile, sts3215
from ..sts3215 import ServoState

_MODEL = pathlib.Path(__file__).with_name("so101_new_calib.xml")
JOINTS = list(profile.JOINT_NAMES) + [profile.GRIPPER_NAME]
TICKS_PER_RAD = (sts3215.TICKS_PER_REV - 1) / (2.0 * math.pi)   # lerobot DEGREES uses 4095


@dataclass
class MotorParams:
    supply_v: float = 12.2              # open-circuit supply (measured on this rig)
    R: float = 12.0 / sts3215.STALL_CURRENT_A
    Ke: float = (12.0 - sts3215.NO_LOAD_CURRENT_A * 12.0 / sts3215.STALL_CURRENT_A) / (
        sts3215.NO_LOAD_RPM * 2 * math.pi / 60)
    Kt: float = sts3215.STALL_TORQUE_NM / sts3215.STALL_CURRENT_A
    coulomb_nm: float = 0.05            # ~ MJCF frictionloss 0.052
    viscous_nms: float = 0.031          # rest of the no-load current
    d_gain_per_unit: float = 0.0035     # ASSUMED: permille per (step/s) per D unit
    overcurrent_a: float = sts3215.OVERCURRENT_TRIP_A
    r_th: float = 3.0                   # ASSUMED, C/W
    c_th: float = 29.0                  # ASSUMED, J/C
    ambient_c: float = 27.0
    overload_vs_torque_limit: bool = False


@dataclass
class ServoRegs:
    torque_enable: int = 0
    p: int = 16
    d: int = 32
    i: int = 0
    acceleration: int = 254
    goal_position: int = 2048
    goal_velocity: int = 0
    torque_limit: int = 1000
    max_torque_limit: int = 1000
    overload_torque: int = 80
    protection_time: int = 200
    protective_torque: int = 20
    protection_current: int = 500
    over_current_protection_time: int = 200
    max_temperature: int = 70
    min_voltage: int = 40
    max_voltage: int = 140
    lock: int = 1


_REG_ATTR = {
    "Torque_Enable": "torque_enable", "P_Coefficient": "p", "D_Coefficient": "d",
    "I_Coefficient": "i", "Acceleration": "acceleration", "Goal_Position": "goal_position",
    "Goal_Velocity": "goal_velocity", "Torque_Limit": "torque_limit",
    "Max_Torque_Limit": "max_torque_limit", "Overload_Torque": "overload_torque",
    "Protection_Time": "protection_time", "Protective_Torque": "protective_torque",
    "Protection_Current": "protection_current",
    "Over_Current_Protection_Time": "over_current_protection_time",
    "Max_Temperature_Limit": "max_temperature", "Min_Voltage_Limit": "min_voltage",
    "Max_Voltage_Limit": "max_voltage", "Lock": "lock",
}


class SimServo:
    """One STS3215: profile + position loop + motor + protections + heat."""

    def __init__(self, name: str, sid: int, mp: MotorParams, mid_ticks: float):
        self.name = name
        self.id = sid
        self.mp = mp
        self.mid = mid_ticks
        self.regs = ServoRegs()
        self.setpoint = mid_ticks
        self.sp_vel = 0.0
        self.temp_c = mp.ambient_c
        self.duty = 0.0                  # permille, signed
        self.current = 0.0               # A
        self.overload = False
        self.overcurrent = False
        self.overheat = False
        self._ol_timer = 0.0
        self._oc_timer = 0.0
        self.trips = {"overload": 0, "overcurrent": 0, "overheat": 0}
        self.peak_temp = self.temp_c

    # -- registers -------------------------------------------------------------
    def write(self, reg: str, value: int, pos_ticks: float) -> None:
        attr = _REG_ATTR.get(reg)
        if attr is None:
            return
        value = int(value)
        if reg == "Torque_Enable" and value and not self.regs.torque_enable:
            # enabling torque holds where the shaft is
            self.setpoint = pos_ticks
            self.sp_vel = 0.0
        if reg == "Goal_Position":
            # a new position command clears overload / overcurrent [datasheet]
            self.overload = False
            self.overcurrent = False
            self._ol_timer = 0.0
            self._oc_timer = 0.0
        setattr(self.regs, attr, value)

    def read(self, reg: str):
        attr = _REG_ATTR.get(reg)
        return None if attr is None else getattr(self.regs, attr)

    # -- 1 kHz update ----------------------------------------------------------
    def _profile(self, dt: float) -> None:
        r = self.regs
        vmax = float(r.goal_velocity) if r.goal_velocity > 0 else 3400.0
        amax = float(r.acceleration) * 100.0 if r.acceleration > 0 else 2.0e5
        goal = float(r.goal_position)
        err = goal - self.setpoint
        if err == 0.0 and self.sp_vel == 0.0:
            return
        # fastest velocity from which we can still stop at the goal
        v_des = math.copysign(min(vmax, math.sqrt(2.0 * amax * abs(err))), err)
        dv = max(-amax * dt, min(amax * dt, v_des - self.sp_vel))
        self.sp_vel += dv
        new_sp = self.setpoint + self.sp_vel * dt
        if (goal - new_sp) * err <= 0.0:
            # arrived, or would step past it while closing in (the deceleration
            # law keeps that last step small)
            self.setpoint = goal
            self.sp_vel = 0.0
        else:
            self.setpoint = new_sp

    def update(self, pos_ticks: float, vel_ticks: float, supply_v: float, dt: float) -> float:
        """Advance one control period; return the output torque (N*m)."""
        r, mp = self.regs, self.mp
        if not r.torque_enable:
            self.duty = 0.0
            self.setpoint = pos_ticks
            self.sp_vel = 0.0
            w = vel_ticks / TICKS_PER_RAD
            # coasting: the bridge is off, no current
            self.current = 0.0
            self._heat(dt)
            return 0.0
        self._profile(dt)
        e = self.setpoint - pos_ticks
        k = max(0.25, r.p / 4.0)
        duty = k * e + mp.d_gain_per_unit * r.d * (self.sp_vel - vel_ticks)
        limit = float(min(r.torque_limit, 1000))
        if self.overload:
            limit = min(limit, r.protective_torque * 10.0)
        if self.overcurrent or self.overheat:
            limit = 0.0
        duty = max(-limit, min(limit, duty))
        # overload watches the output; which full scale "80 %" refers to is not
        # documented, so both readings are available
        base = float(min(r.torque_limit, 1000)) if mp.overload_vs_torque_limit else 1000.0
        if not self.overload and abs(duty) > r.overload_torque / 100.0 * base:
            self._ol_timer += dt
            if self._ol_timer >= r.protection_time * 0.01:
                self.overload = True
                self.trips["overload"] += 1
        elif not self.overload:
            self._ol_timer = 0.0
        self.duty = duty
        w = vel_ticks / TICKS_PER_RAD
        current = (duty / 1000.0 * supply_v - mp.Ke * w) / mp.R
        self.current = current
        if not self.overcurrent and abs(current) > mp.overcurrent_a:
            self._oc_timer += dt
            if self._oc_timer >= r.over_current_protection_time * 0.01:
                self.overcurrent = True
                self.trips["overcurrent"] += 1
        elif not self.overcurrent:
            self._oc_timer = 0.0
        self._heat(dt)
        return mp.Kt * current

    def _heat(self, dt: float) -> None:
        mp = self.mp
        p_loss = self.current * self.current * mp.R
        self.temp_c += (p_loss - (self.temp_c - mp.ambient_c) / mp.r_th) / mp.c_th * dt
        self.peak_temp = max(self.peak_temp, self.temp_c)
        if not self.overheat and self.temp_c > self.regs.max_temperature:
            self.overheat = True
            self.trips["overheat"] += 1
        elif self.overheat and self.temp_c < self.regs.max_temperature - 5:
            self.overheat = False

    @property
    def status(self) -> int:
        bits = 0
        if self.overload:
            bits |= 0x20
        if self.overcurrent:
            bits |= 0x08
        if self.overheat:
            bits |= 0x04
        return bits


@dataclass
class World:
    table_z_m: Optional[float] = None      # table plane under the gripper (base frame)
    table_k: float = 4000.0                # N/m
    table_c: float = 40.0                  # N*s/m
    payload_kg: float = 0.0
    stops_deg: Dict[str, Tuple[float, float]] = field(default_factory=dict)  # narrower mech. stops


class SimArm:
    """MuJoCo SO-101 + six SimServos + a sagging 12 V supply."""

    def __init__(self, ranges: Optional[Dict[str, Tuple[int, int]]] = None,
                 motor: Optional[MotorParams] = None, world: Optional[World] = None,
                 start_deg: Optional[List[float]] = None):
        import mujoco
        self.mj = mujoco
        self.m = mujoco.MjModel.from_xml_path(str(_MODEL))
        self.d = mujoco.MjData(self.m)
        m = self.m
        m.opt.timestep = 0.001
        m.actuator_gainprm[:] = 0.0          # the servos below are the actuators
        m.actuator_biasprm[:] = 0.0
        m.dof_damping[:] = 0.0               # back-EMF damping is in the motor model
        self.motor = motor or MotorParams()
        self.world = world or World()
        self.ranges = ranges or {n: (1024, 3072) for n in JOINTS}
        self.servos: List[SimServo] = []
        for i, n in enumerate(JOINTS):
            lo, hi = self.ranges.get(n, (0, 4095))
            self.servos.append(SimServo(n, i + 1, self.motor, (lo + hi) / 2.0))
        for n, (lo, hi) in self.world.stops_deg.items():
            j = JOINTS.index(n)
            m.jnt_range[j] = [math.radians(lo), math.radians(hi)]
        q0 = start_deg or [0.0, -60.0, 60.0, 60.0, 0.0]
        for j, a in enumerate(q0):
            self.d.qpos[j] = math.radians(a)
        mujoco.mj_forward(m, self.d)
        for j, s in enumerate(self.servos):
            s.setpoint = self.ticks(j)
            s.regs.goal_position = int(round(s.setpoint))
        self._gripper_bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "gripper")
        self.t = 0.0
        self.supply_v = self.motor.supply_v
        self.contact_force = 0.0
        self.lock = threading.RLock()

    # -- units -----------------------------------------------------------------
    def ticks(self, j: int) -> float:
        return self.servos[j].mid + self.d.qpos[j] * TICKS_PER_RAD

    def vel_ticks(self, j: int) -> float:
        return self.d.qvel[j] * TICKS_PER_RAD

    def angles_deg(self) -> List[float]:
        return [math.degrees(self.d.qpos[j]) for j in range(5)]

    def tcp_world(self):
        import numpy as np
        R = self.d.xmat[self._gripper_bid].reshape(3, 3)
        off = np.array(profile.TOOL_TRANSFORM[0]) / 1000.0
        return self.d.xpos[self._gripper_bid] + R @ off

    # -- physics -----------------------------------------------------------------
    def _external(self) -> None:
        mj, m, d, w = self.mj, self.m, self.d, self.world
        d.qfrc_applied[:] = 0.0
        import numpy as np
        pt = None
        force = np.zeros(3)
        if w.payload_kg > 0:
            pt = self.tcp_world()
            force += np.array([0.0, 0.0, -9.81 * w.payload_kg])
        self.contact_force = 0.0
        if w.table_z_m is not None:
            tcp = self.tcp_world()
            pen = w.table_z_m - tcp[2]
            if pen > 0:
                jacp = np.zeros((3, m.nv))
                mj.mj_jac(m, d, jacp, None, tcp, self._gripper_bid)
                vz = (jacp @ d.qvel)[2]
                fz = max(0.0, w.table_k * pen - w.table_c * vz)
                self.contact_force = fz
                d.qfrc_applied[:] += jacp.T @ np.array([0.0, 0.0, fz])
        if pt is not None:
            jacp = np.zeros((3, m.nv))
            mj.mj_jac(m, d, jacp, None, pt, self._gripper_bid)
            d.qfrc_applied[:] += jacp.T @ force

    def step(self, dt: float = 0.001) -> None:
        with self.lock:
            m, d = self.m, self.d
            self._external()
            total_i = 0.0
            for j, s in enumerate(self.servos):
                tau = s.update(self.ticks(j), self.vel_ticks(j), self.supply_v, dt)
                visc = -self.motor.viscous_nms * d.qvel[j]
                d.qfrc_applied[j] += tau + visc
                total_i += abs(s.current)
            # 12 V / 5 A brick: ~0.15 ohm, then current limiting
            over = max(0.0, total_i - 5.0)
            self.supply_v = max(5.0, self.motor.supply_v - 0.15 * total_i - 2.0 * over)
            self.mj.mj_step(m, d)
            self.t += dt

    def advance(self, seconds: float) -> None:
        n = int(round(seconds / 0.001))
        for _ in range(n):
            self.step(0.001)

    def servo(self, name: str) -> SimServo:
        return self.servos[JOINTS.index(name)]

    def snapshot(self, name: str) -> ServoState:
        j = JOINTS.index(name)
        s = self.servos[j]
        st = ServoState(
            ok=True, pos=int(round(self.ticks(j))), vel=int(round(self.vel_ticks(j))),
            load=int(round(s.duty)), volt=round(self.supply_v, 1), temp=int(round(s.temp_c)),
            status=s.status, moving=int(abs(self.vel_ticks(j)) > 10),
            target=int(round(s.setpoint)), current=s.current, t=self.t)
        st.faults = sts3215.status_names(st.status)
        return st


class SimFollowerBus:
    """ServoIO over a SimArm. `clock` decides how far physics advances on each
    I/O call: wall time for the live cockpit, a manual clock for tests."""

    def __init__(self, arm: SimArm, clock: Optional[Callable[[], float]] = None,
                 realtime: bool = True):
        self.arm = arm
        self.names = list(JOINTS)
        self.realtime = realtime
        self._clock = clock or time.perf_counter
        self._t0 = self._clock()
        self.writes: List[Tuple[str, Dict[str, int]]] = []

    def _sync_time(self) -> None:
        if not self.realtime:
            return
        target = self._clock() - self._t0
        lag = target - self.arm.t
        if lag > 0.25:                      # the host stalled: don't burn CPU catching up
            self.arm.t = target - 0.25
            lag = 0.25
        if lag > 0:
            self.arm.advance(lag)

    def read_telemetry(self, names: Optional[Iterable[str]] = None) -> Dict[str, ServoState]:
        self._sync_time()
        with self.arm.lock:
            return {n: self.arm.snapshot(n) for n in (names or self.names)}

    def read_positions(self, names: Optional[Iterable[str]] = None) -> Dict[str, Optional[int]]:
        return {n: s.pos for n, s in self.read_telemetry(names).items()}

    def sync_write(self, reg: str, values: Dict[str, int]) -> None:
        self._sync_time()
        self.writes.append((reg, dict(values)))
        with self.arm.lock:
            for n, v in values.items():
                j = JOINTS.index(n)
                self.arm.servos[j].write(reg, int(v), self.arm.ticks(j))

    def write_commands(self, cmds) -> None:
        """Registers 41..49 in one go, like the real sync write (goal last: a
        new position command is what clears overload / over-current)."""
        if not cmds:
            return
        self._sync_time()
        self.writes.append(("commands", {n: c.goal for n, c in cmds.items()}))
        with self.arm.lock:
            for n, c in cmds.items():
                j = JOINTS.index(n)
                s = self.arm.servos[j]
                pos = self.arm.ticks(j)
                s.write("Acceleration", c.acceleration, pos)
                s.write("Goal_Velocity", c.velocity, pos)
                s.write("Torque_Limit", c.torque_limit, pos)
                s.write("Goal_Position", c.goal, pos)

    def read_reg(self, reg: str, name: str) -> Tuple[Optional[int], int]:
        with self.arm.lock:
            s = self.arm.servo(name)
            if reg == "Present_Position":
                return int(round(self.arm.ticks(JOINTS.index(name)))), s.status
            if reg == "Present_Temperature":
                return int(round(s.temp_c)), s.status
            if reg == "Present_Voltage":
                return int(round(self.arm.supply_v * 10)), s.status
            v = s.read(reg)
            return (None if v is None else int(v)), s.status

    def write_reg(self, reg: str, name: str, value: int) -> Tuple[bool, int]:
        self.sync_write(reg, {name: int(value)})
        return True, self.arm.servo(name).status

    # lerobot-like lifecycle so the cockpit can treat it as a bus
    def connect(self, handshake: bool = True) -> None:
        pass

    def disconnect(self, disable_torque: bool = True) -> None:
        if disable_torque:
            self.sync_write("Torque_Enable", {n: 0 for n in self.names})


class ScriptedLeaderBus:
    """A leader arm whose joints follow f(t) -> {joint: deg} (torque off)."""

    def __init__(self, ranges: Dict[str, Tuple[int, int]],
                 path: Callable[[float], Dict[str, float]],
                 clock: Optional[Callable[[], float]] = None):
        self.ranges = ranges
        self.path = path
        self.names = list(JOINTS)
        self._clock = clock or time.perf_counter
        self._t0 = self._clock()

    def now(self) -> float:
        return self._clock() - self._t0

    def ticks(self) -> Dict[str, int]:
        deg = self.path(self.now())
        out = {}
        for n in self.names:
            lo, hi = self.ranges.get(n, (0, 4095))
            mid = (lo + hi) / 2.0
            out[n] = int(round(mid + deg.get(n, 0.0) * (sts3215.TICKS_PER_REV - 1) / 360.0))
        return out

    def read_positions(self, names: Optional[Iterable[str]] = None) -> Dict[str, Optional[int]]:
        t = self.ticks()
        return {n: t[n] for n in (names or self.names)}

    def read_telemetry(self, names: Optional[Iterable[str]] = None) -> Dict[str, ServoState]:
        t = self.ticks()
        return {n: ServoState(ok=True, pos=t[n], volt=7.4, temp=28) for n in (names or self.names)}

    def sync_write(self, reg: str, values: Dict[str, int]) -> None:
        pass

    def write_commands(self, cmds) -> None:
        pass

    def read_reg(self, reg: str, name: str) -> Tuple[Optional[int], int]:
        if reg == "Present_Voltage":
            return 74, 0
        if reg == "Present_Position":
            return self.ticks()[name], 0
        return 0, 0

    def write_reg(self, reg: str, name: str, value: int) -> Tuple[bool, int]:
        return True, 0

    def connect(self, handshake: bool = True) -> None:
        pass

    def disconnect(self, disable_torque: bool = True) -> None:
        pass
