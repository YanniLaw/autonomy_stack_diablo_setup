from setuptools import setup
import os
from glob import glob

package_name = 'diablo_dance'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Choreography + record/replay tools for DIABLO via /diablo/MotionCmd.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'diablo_dance_orchestrator = diablo_dance.dance_orchestrator:main',
            'diablo_cmd_recorder = diablo_dance.cmd_recorder:main',
        ],
    },
)
