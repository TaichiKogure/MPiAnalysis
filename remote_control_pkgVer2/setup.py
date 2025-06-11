from setuptools import setup
import os
from glob import glob

package_name = 'remote_control_pkgVer2'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='MentorPi User',
    maintainer_email='user@example.com',
    description='Improved remote control package for MentorPi robot',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_control_node = remote_control_pkgVer2.robot_control_node:main',
            'pc_control_node = remote_control_pkgVer2.pc_control_node:main',
        ],
    },
)