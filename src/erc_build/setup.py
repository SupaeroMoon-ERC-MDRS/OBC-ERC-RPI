from setuptools import find_packages, setup

package_name = 'erc_build'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/*']),
        ('share/' + package_name + '/config', ['config/*.yaml']),
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
            'steering_node = erc_build.steering_node:main',
            'roboclaw_node = erc_build.roboclaw_node:main',
            'encoder_node = erc_build.encoder_node:main',
            'remote_messages_node = erc_build.remote_messages_node:main',
            'servo_control = erc_build.servo_control:main',
            'keyboard_control = erc_build.keyboard_control:main',
        ],
    },
)
