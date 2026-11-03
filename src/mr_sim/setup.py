from glob import glob
from setuptools import setup

package_name = 'mr_sim'
setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/worlds', glob('worlds/*')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Julian Rene Chaux Cedeno',
    maintainer_email='julianrenechaux@gmail.com',
    description='Emulador de RoboClaw, simulador de lidar 2D y mundo de laboratorio (sin GPU) y para Gazebo Fortress.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'roboclaw_emulator = mr_sim.roboclaw_emulator:main',
        'lidar_sim = mr_sim.lidar_sim:main',
        'world_gen = mr_sim.world_gen:main',
    ]},
)
