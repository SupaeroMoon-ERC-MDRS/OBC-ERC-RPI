from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Ackermann controller
        Node(
            package='arm_build',
            executable='inverse_kinematics',
            name='inverse_kinematics',
            output='screen'
        ),
        Node(
            package='arm_build',
            executable='servo_command_node',
            name='servo_command_node',
            output='screen'
        ),
        Node(
            package='arm_build',
            executable='keyboard_arm',
            name='keyboard_arm',
            output='screen'
        )
    ])