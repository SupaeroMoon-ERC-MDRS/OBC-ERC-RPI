from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # steering controller
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


        # This function drives the arm
        Node(
            package='mdrs_build',
            executable='arm_motion',
            name='arm_motion',
            output='screen'
        ),
        # Communication between remote control and rover
        Node(
            package='mdrs_build',
            executable='remote_messages_node',
            name='remote_messages_node',
            output='screen'
        ),

        Node(
            package='mdrs_build',
            executable='weighing_ros',
            name='weighing_ros',
            output='screen'
        )
        # Backup in case remote not available
        # Node(
        #     package='mdrs_build',
        #     executable='keyboard_control',
        #     name='keyboard_control',
        #     output='screen'
        # ),
    ])
