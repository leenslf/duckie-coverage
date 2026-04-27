from setuptools import setup
from glob import glob
import os

package_name = 'waypoint_nav'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Leen Said',
    maintainer_email='leensaid24@gmail.com',
    description='Nav2 waypoint navigation for Duckietown DM21 with VIO odometry from RTAB-Map.',
    license='MIT',
    entry_points={},
)
