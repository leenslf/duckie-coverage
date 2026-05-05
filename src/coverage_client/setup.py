from setuptools import setup

package_name = 'coverage_client'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Leen Said',
    maintainer_email='leensaid24@gmail.com',
    description='One-shot CLI node that sends a NavigateCompleteCoverage action goal',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'coverage_client_node = coverage_client.coverage_client_node:main',
        ],
    },
)
