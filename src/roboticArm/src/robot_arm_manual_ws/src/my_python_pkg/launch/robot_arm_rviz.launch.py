from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition

def generate_launch_description():
    urdf_file = PathJoinSubstitution([
        FindPackageShare('my_python_pkg'),
        'urdf',
        'robot_simple.urdf'
    ])

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': Command(['cat ', urdf_file])}]
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', PathJoinSubstitution([
                FindPackageShare('my_python_pkg'),
                'rviz',
                'robot_arm.rviz'
            ])]
        ),
        Node(
            package='my_python_pkg',
            executable='joint_state_publisher_node',
            name='custom_joint_state_publisher'
        )
    ])