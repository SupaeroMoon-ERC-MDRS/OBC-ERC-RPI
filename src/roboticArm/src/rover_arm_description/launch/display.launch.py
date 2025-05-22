from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition
import os


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Flag to enable joint_state_publisher_gui",
        )
    )

    # Initialize Arguments
    gui = LaunchConfiguration("gui")

    # Get the package directory
    pkg_dir = FindPackageShare('rover_arm_description')
    
    # Get the path to the URDF file
    urdf_file = os.path.join(pkg_dir, 'urdf', 'robot_simple.xacro')
    
    # Convert xacro to URDF
    robot_description = {"robot_description": urdf_file}

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("rover_arm_description"), "config", "display.rviz"]
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        condition=UnlessCondition(gui),
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        condition=IfCondition(gui),
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    nodes = [
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node,
    ]

    return LaunchDescription(declared_arguments + nodes) 