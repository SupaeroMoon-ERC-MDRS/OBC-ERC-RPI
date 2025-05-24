from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    moveit_config_path = get_package_share_directory('rover_arm_moveit_auto_config')
    demo_launch = os.path.join(moveit_config_path, 'launch', 'demo.launch.py')

    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(demo_launch)),
        Node(
            package='rover_arm_moveit_auto_config',
            executable='target_pose_publisher',
            output='screen'
        ),
        Node(
            package='rover_arm_moveit_auto_config',
            executable='moveit_planner_node',
            output='screen'
        ),
    ])
