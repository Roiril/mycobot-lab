"""SO-101 on a floor: every link can touch the ground, not just the TCP.

`SimArm` (servo_sim.py) models the six STS3215 servos and lets MuJoCo integrate
the arm, but its only contact is a spring plane under the gripper TCP. The web
simulator puts the arm on a visible floor, so a forearm or the wrist servo that
reaches the floor must be stopped by it too.

How the contact works
---------------------
Each collision mesh is reduced to the vertices that are extreme along a set of
directions (a sampled convex hull, ~40 points per mesh). Every step, each point
below the floor plane gets a penalty force

    Fn = k * depth - c * v_z            (only when > 0)
    Ft = -min(mu * Fn, c_t * |v_t|) * v_t/|v_t|   (Coulomb, with a viscous ramp)

applied through the point Jacobian, like SimArm's table. The base is fixed to
the floor, so its own geometry is skipped. Links do not collide with each other.

The floor height defaults to the bottom of the base meshes, so the arm stands on
it at z = 0 of the floor.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional

import numpy as np

from .servo_sim import JOINTS, MotorParams, SimArm, World


@dataclass
class FloorParams:
    enabled: bool = True
    k: float = 3000.0          # N/m per contact point
    c: float = 30.0            # N*s/m per contact point
    mu: float = 0.6            # friction coefficient (PLA on a desk, ASSUMED)
    c_t: float = 60.0          # N*s/m, tangential viscous ramp below the Coulomb cap
    directions: int = 42       # sample directions per mesh


def _fibonacci_dirs(n: int) -> np.ndarray:
    i = np.arange(n) + 0.5
    phi = np.arccos(1 - 2 * i / n)
    theta = math.pi * (1 + 5 ** 0.5) * i
    return np.stack([np.cos(theta) * np.sin(phi), np.sin(theta) * np.sin(phi), np.cos(phi)], 1)


class FloorArm(SimArm):
    """SimArm + a floor plane every link collides with."""

    def __init__(self, ranges=None, motor: Optional[MotorParams] = None,
                 world: Optional[World] = None, start_deg: Optional[List[float]] = None,
                 floor: Optional[FloorParams] = None):
        super().__init__(ranges=ranges, motor=motor, world=world or World(), start_deg=start_deg)
        self.floor = floor or FloorParams()
        mj, m, d = self.mj, self.m, self.d
        base_bid = mj.mj_name2id(m, mj.mjtObj.mjOBJ_BODY, "base")
        dirs = _fibonacci_dirs(self.floor.directions)
        geoms, points = [], []
        base_min = math.inf
        for g in range(m.ngeom):
            if m.geom_type[g] != mj.mjtGeom.mjGEOM_MESH:
                continue
            mid = m.geom_dataid[g]
            adr, n = m.mesh_vertadr[mid], m.mesh_vertnum[mid]
            verts = np.array(m.mesh_vert[adr:adr + n], dtype=float)
            if m.geom_bodyid[g] == base_bid:
                R = d.geom_xmat[g].reshape(3, 3)
                base_min = min(base_min, float((verts @ R.T + d.geom_xpos[g])[:, 2].min()))
                continue
            if m.geom_group[g] != 3:            # collision copies only (visuals duplicate them)
                continue
            idx = np.unique(np.argmax(verts @ dirs.T, axis=0))
            for p in verts[idx]:
                geoms.append(g)
                points.append(p)
        self._cg = np.array(geoms, dtype=int)
        self._cp = np.array(points, dtype=float)
        self._cbody = np.array([m.geom_bodyid[g] for g in geoms], dtype=int)
        self.floor_z = base_min if math.isfinite(base_min) else 0.0
        self.contacts: List[List[float]] = []      # [x, y, z, Fn] of touching points
        self.floor_force = 0.0                     # N, total normal force

    def contact_points_world(self) -> np.ndarray:
        d = self.d
        R = d.geom_xmat[self._cg].reshape(-1, 3, 3)
        return np.einsum("nij,nj->ni", R, self._cp) + d.geom_xpos[self._cg]

    def _external(self) -> None:
        super()._external()
        self.contacts = []
        self.floor_force = 0.0
        f = self.floor
        if not f.enabled or len(self._cp) == 0:
            return
        mj, m, d = self.mj, self.m, self.d
        pts = self.contact_points_world()
        hit = np.nonzero(pts[:, 2] < self.floor_z)[0]
        if len(hit) == 0:
            return
        jacp = np.zeros((3, m.nv))
        for i in hit:
            p = pts[i]
            jacp[:] = 0.0
            mj.mj_jac(m, d, jacp, None, p, int(self._cbody[i]))
            v = jacp @ d.qvel
            depth = self.floor_z - p[2]
            fn = max(0.0, f.k * depth - f.c * v[2])
            if fn <= 0.0:
                continue
            vt = np.array([v[0], v[1], 0.0])
            sp = float(np.linalg.norm(vt))
            ft = np.zeros(3)
            if sp > 1e-9:
                ft = -min(f.mu * fn, f.c_t * sp) * vt / sp
            force = ft + np.array([0.0, 0.0, fn])
            d.qfrc_applied[:] += jacp.T @ force
            self.contacts.append([float(p[0]), float(p[1]), float(self.floor_z), fn])
            self.floor_force += fn


__all__ = ["FloorArm", "FloorParams", "JOINTS"]
