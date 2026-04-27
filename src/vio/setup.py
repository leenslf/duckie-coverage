from setuptools import setup
from glob import glob
import os

package_name = 'vio'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Leen Said',
    maintainer_email='leensaid24@gmail.com',
    description='Visual-inertial odometry launch files for OAK-D Pro + rtabmap_ros.',
    license='MIT',
    entry_points={},
)
