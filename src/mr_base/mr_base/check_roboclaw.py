"""Diagnostico del RoboClaw sin ROS: version, baterias, PID/QPPS y encoders en vivo.

  ros2 run mr_base check_roboclaw --port /dev/roboclaw
  ros2 run mr_base check_roboclaw --port /dev/roboclaw --probar   # gira cada motor 2 s a baja velocidad

Con --probar: levante el robot (ruedas en el aire). Cada rueda debe girar HACIA ADELANTE y su encoder
debe AUMENTAR; si no, corrija left_motor / invert_left / invert_right en config/mr_base.yaml (DA-12.1).
"""
import argparse
import time

from .roboclaw import RoboClaw, decode_status


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', default='/dev/roboclaw')
    ap.add_argument('--baud', type=int, default=38400)
    ap.add_argument('--address', type=int, default=128)
    ap.add_argument('--probar', action='store_true', help='mueve cada motor al 10 %% de QPPS durante 2 s')
    ap.add_argument('--segundos', type=float, default=10.0, help='tiempo de monitoreo de encoders')
    a = ap.parse_args()
    with RoboClaw(a.port, a.baud, a.address) as rc:
        print('Version      :', rc.read_version())
        print('Bat. principal:', rc.read_main_battery(), 'V   logica:', rc.read_logic_battery(), 'V')
        print('Temperatura  :', rc.read_temperature(), 'C')
        st = rc.read_status()
        if st is not None:
            e, w = decode_status(st)
            print(f'Estado       : 0x{st:08X} errores={e or "-"} avisos={w or "-"}')
        pids = {}
        for m in (1, 2):
            pids[m] = rc.read_velocity_pid(m)
            print(f'PID vel. M{m} :', pids[m])
        if a.probar:
            for m in (1, 2):
                qpps = int(0.1 * ((pids[m] or {}).get('qpps') or 3000))
                e0 = rc.read_encoder(m)[0]
                rc.speed_accel_m1m2(qpps * 2, qpps if m == 1 else 0, qpps if m == 2 else 0)
                time.sleep(2.0)
                rc.stop()
                time.sleep(0.5)
                e1 = rc.read_encoder(m)[0]
                print(f'M{m}: velocidad +{qpps} QPPS -> encoder {e1 - e0:+d} cuentas '
                      f'({"OK, mismo signo" if e1 - e0 > 0 else "REVISAR: encoder invertido o sin senal"})')
        print(f'Monitoreo de encoders {a.segundos:.0f} s (gire las ruedas a mano)...')
        t0 = time.time()
        while time.time() - t0 < a.segundos:
            e1, e2 = rc.read_encoder(1), rc.read_encoder(2)
            print(f'\rM1={e1[0] if e1 else "?":>10}  M2={e2[0] if e2 else "?":>10}', end='', flush=True)
            time.sleep(0.2)
        print()


if __name__ == '__main__':
    main()
