#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from moveit2_py import MoveIt2
from moveit_msgs.msg import DisplayTrajectory
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64MultiArray
import numpy as np

class ArmController(Node):
    def __init__(self):
        super().__init__('arm_controller')
        
        # Initialize MoveIt2
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'],
            base_link_name='base_link',
            end_effector_name='end_effector_link',
            group_name='manipulator'
        )
        
        # Create publisher for servo commands
        self.servo_pub = self.create_publisher(Float64MultiArray, 'servos_commands', 10)
        
        self.get_logger().info('Arm Controller initialized')

    def move_to_position(self, position, orientation=None):
        """
        Move the arm to a specified position and orientation
        position: [x, y, z] in meters
        orientation: [roll, pitch, yaw] in radians (optional)
        """
        # Create pose target
        pose = Pose()
        pose.position.x = position[0]
        pose.position.y = position[1]
        pose.position.z = position[2]
        
        if orientation is not None:
            # Convert roll, pitch, yaw to quaternion
            q = self._euler_to_quaternion(orientation[0], orientation[1], orientation[2])
            pose.orientation.x = q[0]
            pose.orientation.y = q[1]
            pose.orientation.z = q[2]
            pose.orientation.w = q[3]
        else:
            # Default orientation
            pose.orientation.w = 1.0

        # Plan and execute
        success = self.moveit2.move_to_pose(pose)
        
        if success:
            # Get joint positions after movement
            joint_positions = self.moveit2.get_current_joint_values()
            # Convert joint positions to servo commands and publish
            self._publish_servo_commands(joint_positions)
            return True
        return False

    def _publish_servo_commands(self, joint_positions):
        """
        Convert joint positions to servo commands and publish them
        joint_positions: list of joint angles in radians
        """
        # Convert radians to servo angles (assuming servos use 0-180 degrees)
        servo_commands = [np.degrees(pos) for pos in joint_positions]
        
        # Create and publish message
        msg = Float64MultiArray()
        msg.data = servo_commands
        self.servo_pub.publish(msg)
        self.get_logger().info(f'Published servo commands: {servo_commands}')

    def _euler_to_quaternion(self, roll, pitch, yaw):
        """
        Convert Euler angles to quaternion
        """
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy

        return [qx, qy, qz, qw]

def main(args=None):
    rclpy.init(args=args)
    arm_controller = ArmController()
    
    try:
        rclpy.spin(arm_controller)
    except KeyboardInterrupt:
        pass
    finally:
        arm_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 