from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("my_robot")
        .robot_description(file_path="config/urdf/robot.urdf.xacro")
        .moveit_cpp()
        .to_moveit_configs()
    )

    return LaunchDescription([
        moveit_config.to_launch_description(),
    ])