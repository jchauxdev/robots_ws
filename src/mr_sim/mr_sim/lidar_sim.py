"""Simulador de lidar 2D tipo RPLidar A1 (trazado de rayos sobre el mapa de verdad terreno).

Entrada : /sim/ground_truth (pose real del robot que integra el emulador) y un mapa PNG/YAML (marco world).
Salida  : /scan (sensor_msgs/LaserScan) en el marco 'laser', con la misma convencion que rplidar_ros
          (angle_min = -pi, angle_max = pi, sentido antihorario) y ruido gaussiano.
No necesita GPU: sirve para validar SLAM y Nav2 en cualquier PC (Gazebo + gpu_lidar requiere GPU).
"""
import math
from pathlib import Path

import numpy as np
import rclpy
import yaml
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class LidarSim(Node):
    def __init__(self):
        super().__init__('lidar_sim')
        p = self.declare_parameter
        map_yaml = Path(p('world_map', '').value)
        self.n = int(p('samples', 360).value)
        self.rmin = float(p('range_min', 0.15).value)
        self.rmax = float(p('range_max', 12.0).value)
        self.std = float(p('noise_std', 0.01).value)
        self.rate = float(p('rate', 7.0).value)
        self.frame = p('frame_id', 'laser').value
        # pose del lidar en base_footprint (debe coincidir con el URDF: lidar_joint + laser_joint)
        self.lx = float(p('laser_x', 0.07775).value)
        self.ly = float(p('laser_y', 0.0).value)
        self.lyaw = float(p('laser_yaw', math.pi).value)

        meta = yaml.safe_load(map_yaml.read_text())
        from PIL import Image
        img = np.array(Image.open(map_yaml.parent / meta['image']).convert('L'))
        self.res = float(meta['resolution'])
        self.ox, self.oy = meta['origin'][0], meta['origin'][1]
        self.occ = np.flipud(img < 128)          # fila 0 = y minima
        self.h, self.w = self.occ.shape
        self.pose = None
        self.rng = np.random.default_rng(7)
        self.angles = -math.pi + np.arange(self.n) * (2 * math.pi / self.n)
        step = self.res * 0.5
        self.steps = np.arange(self.rmin, self.rmax, step)
        self.create_subscription(Odometry, '/sim/ground_truth', self.on_truth, 10)
        self.pub = self.create_publisher(LaserScan, 'scan', qos_profile_sensor_data)
        self.create_timer(1.0 / self.rate, self.tick)
        self.get_logger().info(f'Lidar simulado: {self.n} haces, {self.rate} Hz, mapa {map_yaml.name}')

    def on_truth(self, m):
        q = m.pose.pose.orientation
        self.pose = (m.pose.pose.position.x, m.pose.pose.position.y, 2 * math.atan2(q.z, q.w))

    def tick(self):
        if self.pose is None:
            return
        x, y, th = self.pose
        lx = x + self.lx * math.cos(th) - self.ly * math.sin(th)
        ly = y + self.lx * math.sin(th) + self.ly * math.cos(th)
        a = th + self.lyaw + self.angles                       # angulo global de cada haz
        px = lx + np.outer(np.cos(a), self.steps)
        py = ly + np.outer(np.sin(a), self.steps)
        c = ((px - self.ox) / self.res).astype(int)
        r = ((py - self.oy) / self.res).astype(int)
        inside = (c >= 0) & (c < self.w) & (r >= 0) & (r < self.h)
        hit = np.zeros_like(inside)
        hit[inside] = self.occ[r[inside], c[inside]]
        first = np.where(hit.any(axis=1), hit.argmax(axis=1), -1)
        ranges = np.where(first >= 0, self.steps[np.clip(first, 0, None)], np.inf)
        ranges = ranges + np.where(np.isfinite(ranges), self.rng.normal(0, self.std, self.n), 0)
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame
        msg.angle_min, msg.angle_increment = -math.pi, 2 * math.pi / self.n
        msg.angle_max = msg.angle_min + (self.n - 1) * msg.angle_increment
        msg.scan_time, msg.time_increment = 1.0 / self.rate, (1.0 / self.rate) / self.n
        msg.range_min, msg.range_max = self.rmin, self.rmax
        msg.ranges = [float(v) for v in ranges]
        self.pub.publish(msg)


def main():
    rclpy.init()
    n = LidarSim()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
