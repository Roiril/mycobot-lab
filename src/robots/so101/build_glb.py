"""Convert the vendored SO-101 per-link STL visuals into one GLB per URDF link.

Reproducible asset build step (Phase 0 of the SO-101 integration plan). The
SO-101 URDF groups several STL meshes per link (structural part + servo +
holders), each with its own <visual><origin>. This bakes those origins in and
merges each link's visuals into a single GLB expressed in the LINK-LOCAL frame,
scaled to MILLIMETERS so a GLB can be dropped straight onto the corresponding
kinematics.link_frames() node in the three.js viewport (which works in mm).

Usage:  python -m robots.so101.build_glb     (from the src/ dir on sys.path)
   or:  python src/robots/so101/build_glb.py

Inputs : so101_new_calib.urdf (+ sim/assets/*.stl)
Output : meshes_glb/<link_name>.glb  (+ a manifest.json mapping link -> glb)
Deps   : trimesh (requirements-sim.txt). Source assets are Apache-2.0
         (TheRobotStudio/SO-ARM100).
"""
from __future__ import annotations
import json
import math
import pathlib
import xml.etree.ElementTree as ET

import numpy as np
import trimesh

HERE = pathlib.Path(__file__).resolve().parent
URDF = HERE / "so101_new_calib.urdf"
ASSET_DIR = HERE / "sim" / "assets"   # where the .stl files live
OUT_DIR = HERE / "meshes_glb"
MM = 1000.0


def _rpy_to_matrix(rx, ry, rz):
    cx, sx = math.cos(rx), math.sin(rx)
    cy, sy = math.cos(ry), math.sin(ry)
    cz, sz = math.cos(rz), math.sin(rz)
    R = np.array([
        [cy * cz, sx * sy * cz - cx * sz, cx * sy * cz + sx * sz],
        [cy * sz, sx * sy * sz + cx * cz, cx * sy * sz - sx * cz],
        [-sy, sx * cy, cx * cy],
    ])
    return R


def _origin_tf(xyz, rpy):
    T = np.eye(4)
    T[:3, :3] = _rpy_to_matrix(*rpy)
    T[:3, 3] = xyz
    return T


def build() -> dict:
    OUT_DIR.mkdir(exist_ok=True)
    root = ET.parse(URDF).getroot()
    manifest = {}
    for link in root.findall("link"):
        name = link.get("name")
        parts = []
        for vis in link.findall("visual"):
            mesh_el = vis.find("geometry/mesh")
            if mesh_el is None:
                continue
            fn = pathlib.Path(mesh_el.get("filename")).name  # strip assets/ prefix
            stl = ASSET_DIR / fn
            if not stl.exists():
                raise FileNotFoundError(stl)
            m = trimesh.load_mesh(stl)
            o = vis.find("origin")
            if o is not None:
                xyz = [float(v) for v in (o.get("xyz") or "0 0 0").split()]
                rpy = [float(v) for v in (o.get("rpy") or "0 0 0").split()]
                m.apply_transform(_origin_tf(xyz, rpy))
            m.apply_scale(MM)  # meters -> mm to match kinematics frames
            parts.append(m)
        if not parts:
            continue
        merged = trimesh.util.concatenate(parts)
        out = OUT_DIR / f"{name}.glb"
        merged.export(out)
        manifest[name] = {"glb": out.name, "vertices": int(len(merged.vertices)),
                          "faces": int(len(merged.faces))}
    (OUT_DIR / "manifest.json").write_text(json.dumps(manifest, indent=2))
    return manifest


if __name__ == "__main__":
    mani = build()
    for k, v in mani.items():
        print(f"{k:28s} -> {v['glb']:32s} {v['vertices']:>7} v {v['faces']:>7} f")
