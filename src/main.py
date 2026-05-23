"""Demo entry point: home -> wave -> home."""
from arm import Arm, poses


def main() -> None:
    arm = Arm()
    print("port:", arm.port, "version:", arm.version())
    arm.power_on()
    print("angles:", arm.angles())

    arm.move(poses.HOME, speed=25)
    arm.move(poses.WAVE_A, speed=35)
    arm.move(poses.WAVE_B, speed=35)
    arm.move(poses.HOME, speed=25)
    print("final:", arm.angles())


if __name__ == "__main__":
    main()
