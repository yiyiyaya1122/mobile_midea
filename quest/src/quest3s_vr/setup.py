from setuptools import setup
import os
from glob import glob

package_name = 'quest3s_vr'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='',
    maintainer_email='',
    description='',
    license='Apache 2.0',
    entry_points={
        'console_scripts': [
            'controller_node = quest3s_vr.controller_node:main',
            'hand_pub = quest3s_vr.hand_pub:main',
            'torso_chassis_vr_control = quest3s_vr.torso_chassis_vr_control:main'
        ],
    },
)
