from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'mdrs_build'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        (os.path.join('lib', package_name), glob('mdrs_build/roboclaw_driver.py')),
        (os.path.join('lib', package_name), glob('mdrs_build/raspistatechecker.py')),
        (os.path.join('lib', package_name), glob('mdrs_build/udpcanpy*')),
        (os.path.join('lib', package_name), glob('mdrs_build/comms.dbc')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),


    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'steering_node = mdrs_build.steering_node:main',
            'diff_controller = mdrs_build.diff_controller:main',
            'roboclaw_node = mdrs_build.roboclaw_node:main',
            'encoder_node = mdrs_build.encoder_node:main',
            'remote_messages_node = mdrs_build.remote_messages_node:main',
            'servo_control = mdrs_build.servo_control:main',
            'keyboard_control = mdrs_build.keyboard_control:main',
            'arm_motion = mdrs_build.arm_motion:main',
            'aruco_pub = mdrs_build.aruco_pub:main',
            'marker_follower = mdrs_build.marker_follower:main',
            'stereo_camera = mdrs_build.stereo_camera:main',
            'camera_stream = mdrs_build.camera_stream:main',
        ],
    },
)
