from setuptools import setup
import os
from glob import glob

package_name = 'carla_vehicle_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, package_name + '.controllers'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nor',
    maintainer_email='nor@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'trajectory_tracker = carla_vehicle_control.trajectory_tracker:main',
        ],
    },
)