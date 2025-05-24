from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('arm_build')

    robot_description = Command(['xacro ', os.path.join(pkg_share, 'config', 'arm.urdf.xacro')])
    
    return LaunchDescription([
        # Load robot description
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),

        # MoveIt move_group node
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('moveit2'), 'launch', 'move_group.launch.py')
            ),
            launch_arguments={
                'robot_description': robot_description,
                'robot_description_semantic': os.path.join(pkg_share, 'config', 'my_rover.srdf'),
                'robot_description_planning': os.path.join(pkg_share, 'config', 'kinematics.yaml'),
                'controllers_configuration': os.path.join(pkg_share, 'config', 'controllers.yaml'),
            }.items()
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='log',
            arguments=['-d', os.path.join(pkg_share, 'rviz', 'my_rover.rviz')]
        ),

        # Custom servo command node
        Node(
            package='my_rover_moveit',
            executable='servo_command_node',
            name='servo_command_node',
            output='screen'
        ),
    ])
