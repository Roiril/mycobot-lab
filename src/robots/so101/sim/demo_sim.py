"""Offline end-to-end demo: drive the SO-101 through a Cartesian trajectory in
MuJoCo, entirely without hardware, and render it to an animated GIF.

Exercises the full SO-101 stack together: controller -> IK -> safety -> driver
-> MuJoCo. This is the "watch it move before the hardware arrives" deliverable.

Run: python -m robots.so101.sim.demo_sim
Output: src/robots/so101/sim/demo.gif (+ demo_final.png). Both gitignored.
"""
from __future__ import annotations
import pathlib

from PIL import Image

from ..controller import So101Controller
from ..kinematics import end_effector
from .mujoco_sim import MujocoSo101Driver

HERE = pathlib.Path(__file__).resolve().parent
W, H = 480, 360


def main() -> None:
    driver = MujocoSo101Driver()
    driver.connect()
    ctrl = So101Controller(driver)

    frames = []

    def grab(_angles=None):
        arr = driver.render(width=W, height=H)
        frames.append(Image.fromarray(arr))

    # Reachable Cartesian waypoints (derived from valid joint poses so IK is
    # guaranteed a solution), plus gripper open/close, then home.
    targets = [
        ("reach right", end_effector([45, -35, 45, -15, 0]), 100.0),
        ("reach left", end_effector([-45, -35, 45, -15, 0]), 100.0),
        ("reach high", end_effector([0, -70, 50, -10, 0]), 20.0),
        ("reach low/forward", end_effector([0, -20, 35, 10, 0]), 100.0),
    ]

    grab()
    for label, xyz, grip in targets:
        ok, msg = ctrl.move_to_position(xyz, gripper=grip, on_step=grab)
        print(f"{label:20s} {'OK' if ok else 'FAIL'}  tip={[round(c) for c in xyz]}  {('' if ok else msg)}")
    ok, _ = ctrl.home(on_step=grab)
    print(f"{'home':20s} {'OK' if ok else 'FAIL'}")

    gif = HERE / "demo.gif"
    frames[0].save(gif, save_all=True, append_images=frames[1:], duration=60, loop=0)
    frames[-1].save(HERE / "demo_final.png")
    print(f"\n{len(frames)} frames -> {gif}")


if __name__ == "__main__":
    main()
