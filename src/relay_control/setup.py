import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'relay_control'

setup(
    name=package_name,
    version='0.6.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Yiyuan Lin',
    maintainer_email='yl3663@cornell.edu',
    description='Prescription-driven UV lamp control with navigation safety gates.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'uv_lamp_keyboard_node = relay_control.uv_lamp_keyboard_node:main',
            'uv_treatment_node = relay_control.uv_treatment_node:main',
            # Compatibility aliases retained for existing deployments.
            'relay_keyboard_node = relay_control.relay_keyboard_node:main',
            'gps_relay_node = relay_control.gps_relay_node:main',
        ],
    },
)
