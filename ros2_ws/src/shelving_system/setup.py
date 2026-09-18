from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'shelving_system'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hymi',
    maintainer_email='lhm6582@gmail.com',
    description='Task management, YAML data, return machine simulation, and launch files.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            ('return_machine_node = shelving_system.return_machine_node:main'),
            ('task_manager_node = shelving_system.task_manager_node:main'),
            ('mock_navigation_server = shelving_system.mock_navigation_server:main'),
            ('mock_perception_server = shelving_system.mock_perception_server:main'),
            ('mock_manipulation_server = shelving_system.mock_manipulation_server:main')
        ],
    },
)
