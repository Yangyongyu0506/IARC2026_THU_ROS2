from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'iarc_main'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (f"share/{package_name}/config", glob("config/*")),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")), 
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yangy',
    maintainer_email='yang-yy24@mails.tsinghua.edu.cn',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'frametransformer_node = iarc_main.frametransformer_node:main',
            'odomtfbroadcaster_node = iarc_main.odomtfbroadcaster_node:main',
            'targetfeedback_node = iarc_main.targetfeedback_node:main',
        ],
    },
)
