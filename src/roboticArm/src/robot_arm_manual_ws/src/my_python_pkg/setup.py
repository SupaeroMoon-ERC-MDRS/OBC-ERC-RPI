import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'my_python_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='michou',
    maintainer_email='michou@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'position_listener = my_python_pkg.position_listener:main',
            'joint_state_publisher_node = my_python_pkg.joint_state_publisher_node:main',
            'trajectory_planner = my_python_pkg.trajectory_planner:main',
            'position_listener_ik = my_python_pkg.position_listener_ik2:main',
            'servo_execution = my_python_pkg.servo_execution:main'
        ],
    },
)
