"""Verifica que el xacro se expande y que la cadena de TF y la geometria son coherentes."""
import math
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

XACRO = Path(__file__).resolve().parents[1] / 'urdf' / 'mr_robot.urdf.xacro'


def expand(*args):
    out = subprocess.run(['xacro', str(XACRO), *args], check=True, capture_output=True, text=True).stdout
    return ET.fromstring(out)


def joints(root):
    return {j.get('name'): j for j in root.findall('joint')}


def test_tree_and_geometry():
    root = expand()
    links = {l.get('name') for l in root.findall('link')}
    for name in ['base_footprint', 'base_link', 'left_wheel_link', 'right_wheel_link', 'laser',
                 'rgb_camera_optical_frame', 'd435i_link']:
        assert name in links, name
    j = joints(root)
    # Ruedas tocan el suelo: z(base_link) + z(eje) == radio
    zb = float(j['base_joint'].find('origin').get('xyz').split()[2])
    zw = float(j['left_wheel_joint'].find('origin').get('xyz').split()[2])
    assert math.isclose(zb + zw, 0.045, abs_tol=1e-6)
    # Separacion de ruedas
    yl = float(j['left_wheel_joint'].find('origin').get('xyz').split()[1])
    yr = float(j['right_wheel_joint'].find('origin').get('xyz').split()[1])
    assert math.isclose(yl - yr, 0.2016, abs_tol=1e-6)
    for w in ('left_wheel_joint', 'right_wheel_joint'):
        assert j[w].find('axis').get('xyz') == '0 1 0'


def test_options():
    root = expand('use_realsense:=false', 'lidar_yaw:=0.0')
    links = {l.get('name') for l in root.findall('link')}
    assert 'd435i_link' not in links
    rpy = joints(root)['laser_joint'].find('origin').get('rpy').split()
    assert float(rpy[2]) == 0.0
