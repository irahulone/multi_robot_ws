from setuptools import setup

import os

robot_id = os.getenv("ROBOT_ID")

package_name = 'locomotion_core'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vboxuser',
    maintainer_email='vboxuser@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            f'{robot_id}_movebase_kinematics = locomotion_core.movebase_kinematics:main',
            f'{robot_id}_cmd_roboteq = locomotion_core.cmd_roboteq:main',
        ],
    },
)
