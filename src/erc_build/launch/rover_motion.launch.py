from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Ackermann controller
        Node(
            package='erc_build',
            executable='steering',
            name='steering',
            output='screen'
        ),

        # Roboclaw motor driver
        Node(
            package='erc_build',
            executable='roboclaw_node',
            name='roboclaw_node',
            output='screen'
        ),

        # Encoder-based odometry
        Node(
            package='erc_build',
            executable='encoder_node',
            name='encoder_node',
            output='screen'
        ),

        # Remote messages (e.g., joystick or high-level commands)
        Node(
            package='erc_build',
            executable='remote_messages_node',
            name='remote_node',
            output='screen'
        ),
        Node(
            package='erc_build',
            executable='servo_control',
            name='servo_control',
            output='screen'
        ),
    ])
