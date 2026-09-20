from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'shelving_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'resource'), glob('resource/*.pt')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hymi',
    maintainer_email='lhm6582@gmail.com',
    description='Shelf, empty-slot, and tray-book perception for the shelving robot.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'vision_manager = shelving_perception.vision_manager:main',
        ],
    },
)
