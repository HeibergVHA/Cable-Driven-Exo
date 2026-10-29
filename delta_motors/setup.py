from glob import glob
from setuptools import setup

package_name = 'delta_motors'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Anton Hvid',
    maintainer_email='hvidantonch@gmail.com',
    description='Simple ROS2 position control for DrEmpower CAN joint motors',
    license='MIT',
    entry_points={
        'console_scripts': [
            'motor_node = delta_motors.motor_node:main',
            'motor_cmd = delta_motors.motor_cmd:main',
        ],
    },
)
