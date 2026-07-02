from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'arena_2d'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
      (os.path.join('share', package_name, 'launch'), glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='matth',
    maintainer_email='223842537+AvelSense@users.noreply.github.com',
    description='2D visualisation tool for testing observers and controllers',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'viewer = arena_2d.viewer:main',
            'simulator_euler = arena_2d.simulator:main',
            'ground_truth = arena_2d.ground_truth:main',
        ],
    },
)