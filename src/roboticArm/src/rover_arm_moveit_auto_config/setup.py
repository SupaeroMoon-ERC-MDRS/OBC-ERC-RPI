from setuptools import setup
import os
from glob import glob

package_name = 'rover_arm_moveit_auto_config'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='luc24',
    maintainer_email='luc24@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arm_controller = rover_arm_moveit_auto_config.arm_controller:main',
            'move_arm_example = rover_arm_moveit_auto_config.move_arm_example:main',
        ],
    },
) 