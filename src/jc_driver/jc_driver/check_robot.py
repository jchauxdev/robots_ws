"""Diagnostico rapido del enlace serie con el JetCobot (sin ROS).

    ros2 run jc_driver check_robot --port /dev/ttyUSB0 --baud 1000000
"""
import argparse
import time


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=1000000)
    ap.add_argument("--simulate", action="store_true")
    a = ap.parse_args()
    if a.simulate:
        from jc_driver.fake_mycobot import FakeMyCobot280 as Robot
    else:
        import pymycobot
        from pymycobot import MyCobot280 as Robot
        print("pymycobot", pymycobot.__version__)
    mc = Robot(a.port, a.baud)
    time.sleep(0.2)
    for name in ["is_power_on", "get_angles", "get_coords", "get_gripper_value", "get_fresh_mode"]:
        fn = getattr(mc, name, None)
        try:
            print(f"{name:20s} -> {fn() if fn else 'no disponible'}")
        except Exception as e:
            print(f"{name:20s} -> ERROR {e}")
        time.sleep(0.05)


if __name__ == "__main__":
    main()
