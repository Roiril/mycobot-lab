# Vendored SO-101 simulation assets

The files in this directory and `assets/` are vendored from:

- **TheRobotStudio / SO-ARM100** — https://github.com/TheRobotStudio/SO-ARM100
- Path: `Simulation/SO101/` (downloaded 2026-06-04)
- License: **Apache-2.0**

Vendored so the SO-101 sim works offline (develop-before-hardware) without a
network dependency on the upstream repo.

Files:
- `scene.xml`, `so101_new_calib.xml`, `joints_properties.xml` — MuJoCo MJCF model
- `assets/*.stl` — per-link visual/collision meshes (the URDF/MJCF reference these)

Derived artifacts (NOT committed; regenerate locally):
- `preview_pose.png` — `mujoco_sim.So101Sim.save_png`
- `../meshes_glb/*.glb` — `python -m robots.so101.build_glb`
