from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('rover_arm_description')
    
    # Get the path to the XACRO file
    xacro_file = os.path.join(pkg_dir, 'urdf', 'robot_simple.xacro')
    
    # Convert XACRO to URDF
    robot_description_raw = xacro.process_file(xacro_file).toxml()
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_raw,
            'publish_frequency': 30.0,
            'frame_prefix': ''
        }]
    )

    # Joint State Publisher GUI
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        parameters=[{
            'source_list': ['joint_states'],
            'rate': 30
        }]
    )

    # RViz2
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_dir, 'config', 'display.rviz')]
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz2
    ]) 