"""Basic motion demo. Same shape as the proven sequence in cogni-storage/scripts/arm_move.py."""
import sys, pathlib, time
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "src"))

from arm import Arm, poses

a = Arm()
print("port:", a.port, "version:", a.version())
a.power_on()
print("angles@start:", a.angles())

a.move(poses.HOME, speed=25)
a.move_joint(1, 20, speed=25)
a.move_joint(1, -20, speed=25)
a.move_joint(1, 0, speed=25)
a.move(poses.WAVE_A, speed=35)
a.move(poses.WAVE_B, speed=35)
a.move(poses.HOME, speed=25)

print("angles@end :", a.angles())
print("done")
