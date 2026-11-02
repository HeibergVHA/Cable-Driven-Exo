from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'python_utils'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
        glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='CasKK',
    maintainer_email='casper.siem@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        'i2c_scanner_node = python_utils.i2c_scanner_node:main',
        'topic_plotter = python_utils.topic_plotter:main',
        ],
    },
)
