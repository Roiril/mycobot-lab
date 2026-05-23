"""Connection / status check. Run before any motion script."""
import sys, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "src"))

from arm import Arm

a = Arm()
print("port      :", a.port)
print("version   :", a.version())
print("powered   :", a.mc.is_power_on())
print("angles    :", a.angles())
print("coords    :", a.coords())
