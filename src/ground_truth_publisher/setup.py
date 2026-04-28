from setuptools import setup
import os
from glob import glob

package_name = 'ground_truth_publisher'

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
    description='Publishes robot pose ground truth from AprilTag detections via TF composition.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'ground_truth_publisher_node = ground_truth_publisher.ground_truth_publisher_node:main',
        ],
    },
)
