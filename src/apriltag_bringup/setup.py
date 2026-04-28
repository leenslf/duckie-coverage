from setuptools import setup

package_name = 'apriltag_bringup'

setup(
    name=package_name,
    version='0.0.1',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/apriltag.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Leen Said',
    maintainer_email='leensaid24@gmail.com',
    description='Launch AprilTag detection for OAK-D Pro',
    license='Apache-2.0',
    entry_points={},
)
