from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_my_robot = get_package_share_directory('mdrs_build')

    camera_calib = os.path.join(pkg_my_robot, 'config', 'camera_calib.yaml')
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
        Node(
            package='mdrs_build',
            executable='camera_stream',
            name='camera_stream',
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
            executable='aruco_pub',
            parameters=[{'target_marker_id': 0, 'camera_calib_file': camera_calib}] 
        ),

        Node(
            package='mdrs_build',
            executable='marker_follower',
            name='marker_follower',
            output='screen'
        )

    ])