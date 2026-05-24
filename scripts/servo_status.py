"""Dump per-servo status, errors, temperatures, voltages.

Run when something weird happens (limp arm, overload, etc.) to see what
firmware-level fault flags are set. The HTTP server hogs COM so this stops
the server briefly.

Usage:
  # while server is running:
  python scripts/servo_status.py --via-http   (limited info)
  # standalone (server must be stopped):
  python scripts/servo_status.py
"""
import sys, time, pathlib, argparse
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "src"))

def via_arm():
    from arm.client import Arm
    a = Arm()
    mc = a.mc
    print(f"port             : {a.port}")
    print(f"power            : {mc.is_power_on()}")
    print(f"angles           : {mc.get_angles()}")
    try: print(f"servo_status     : {mc.get_servo_status()}")
    except Exception as e: print(f"servo_status     : err {e}")
    try: print(f"robot_status     : {mc.get_robot_status()}")
    except Exception as e: print(f"robot_status     : err {e}")
    try: print(f"servo_temps (°C) : {mc.get_servo_temps()}")
    except Exception as e: print(f"servo_temps      : err {e}")
    try: print(f"servo_voltages   : {mc.get_servo_voltages()}")
    except Exception as e: print(f"servo_voltages   : err {e}")
    try: print(f"servo_currents   : {mc.get_servo_currents()}")
    except Exception as e: print(f"servo_currents   : err {e}")
    try: print(f"servo_speeds     : {mc.get_servo_speeds()}")
    except Exception as e: print(f"servo_speeds     : err {e}")
    try: print(f"servo_enabled    : {[mc.is_servo_enable(i) for i in range(1,7)]}")
    except Exception as e: print(f"servo_enabled    : err {e}")
    try:
        print("--- error queue (read_next_error x6) ---")
        for _ in range(6):
            err = mc.read_next_error()
            print(f"  {err}")
    except Exception as e:
        print(f"  err {e}")
    try: print(f"error_information: {mc.get_error_information()}")
    except Exception as e: print(f"error_information: err {e}")
    a.close()

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--clear", action="store_true", help="call clear_error_information after dump")
    args = ap.parse_args()
    via_arm()
    if args.clear:
        from arm.client import Arm
        a = Arm()
        a.mc.clear_error_information()
        print("[cleared error_information]")
        a.close()

if __name__ == "__main__":
    main()
