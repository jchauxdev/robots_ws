from glob import glob
from setuptools import setup

package_name = 'mr_logistics'
setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    package_data={package_name: ['templates/*.html', 'static/*']},
    include_package_data=True,
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='Julian Rene Chaux Cedeno',
    maintainer_email='julianrenechaux@gmail.com',
    description='App web de logistica (Flask) y gestor de misiones robot movil + cobot.',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'mission_manager = mr_logistics.mission_manager:main',
        'web_app = mr_logistics.web_app:main',
        'pedido = mr_logistics.cli:main',
    ]},
)
