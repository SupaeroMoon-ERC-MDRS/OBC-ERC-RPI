from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Ackermann controller
        Node(
            package='mdrs_build',
            executable='diff_controller',
            name='diff_controller',
            output='screen'
        ),

        # Roboclaw motor driver
        Node(
            package='mdrs_build',
            executable='roboclaw_node',
            name='roboclaw_node',
            output='screen'
        ),

        # Remote messages (e.g., joystick or high-level commands)
        # Node(
        #     package='mdrs_build',
        #     executable='keyboard_control',
        #     name='keyboard_control',
        #     output='screen'
        # ),
        Node(
            package='mdrs_build',
            executable='remote_messages_node',
            name='remote_messages_node',
            output='screen'
        )
        # Node(
        #     package='mdrs_build',
        #     executable='servo_control',
        #     name='servo_control',
        #     output='screen'
        # ),
    ])
