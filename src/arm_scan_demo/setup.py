from setuptools import find_packages, setup
from glob import glob

package_name = 'arm_scan_demo'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'numpy', 'pyquaternion'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Arc-scan pose sweep demo using MoveIt2 (moveit_py) on ROS2 Humble',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move_group_interface = arm_scan_demo.move_group_interface:main',
        ],
    },
)
