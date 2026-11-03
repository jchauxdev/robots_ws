"""El cliente RoboClaw de mr_base habla con el emulador por un pseudo-terminal real."""
import os
import threading
import time

import pytest
import rclpy

from mr_base.roboclaw import RoboClaw


@pytest.fixture(scope='module')
def link():
    path = f'/tmp/roboclaw_test_{os.getpid()}'
    rclpy.init(args=['--ros-args', '-p', f'link:={path}', '-p', 'true_wheel_diameter:=0.09'])
    from mr_sim.roboclaw_emulator import Emulator
    node = Emulator()
    ex = rclpy.executors.SingleThreadedExecutor()
    ex.add_node(node)
    th = threading.Thread(target=ex.spin, daemon=True)
    th.start()
    yield path
    ex.shutdown(timeout_sec=1.0)
    node.destroy_node()
    rclpy.try_shutdown()


def test_roundtrip(link):
    with RoboClaw(link, 38400, 128) as rc:
        assert 'Roboclaw' in rc.read_version()
        assert rc.read_main_battery() == pytest.approx(12.1)
        assert rc.read_status() == 0
        assert rc.read_velocity_pid(1)['qpps'] == 6000
        assert rc.reset_encoders()
        assert rc.speed_accel_m1m2(20000, 1000, -1000)
        time.sleep(1.0)
        e1, e2 = rc.read_encoder(1)[0], rc.read_encoder(2)[0]
        s1, s2 = rc.read_speed(1), rc.read_speed(2)
        assert 700 < e1 < 1000 and -1000 < e2 < -700          # ~1 s a 1000 QPPS (menos la rampa)
        assert s1 == pytest.approx(1000, abs=30) and s2 == pytest.approx(-1000, abs=30)
        assert rc.stop()
        assert rc.crc_errors == 0
