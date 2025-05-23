from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'arm_build'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf*')),
        (os.path.join('share', package_name, 'test'), glob('test/*')),
        (os.path.join('share', package_name, 'urdf/arm_meshes'), glob('urdf/arm_meshes/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'servo_command_node = arm_build.servo_command_node:main',
            'inverse_kinematics = arm_build.inverse_kinematics:main',
            'keyboard_arm = arm_build.keyboard_arm:main',
        ],
    },
)
