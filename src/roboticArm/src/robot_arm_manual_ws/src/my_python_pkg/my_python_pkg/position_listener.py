#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Float64MultiArray
from my_python_pkg.inverse_kinematics_solver import compute_joint_angles

class PositionListener(Node):
    def __init__(self):
        super().__init__('position_listener')
        self.subscription = self.create_subscription(Point, '/target_position', self.listener_callback, 10)
        self.publisher = self.create_publisher(Float64MultiArray, '/servos_commands', 10)

    def listener_callback(self, msg):
        joint_angles = compute_joint_angles(msg.x, msg.y, msg.z)
        if joint_angles:
            command = Float64MultiArray(data=joint_angles)
            self.publisher.publish(command)
            self.get_logger().info(f'Published angles: {joint_angles}')
        else:
            self.get_logger().warn('Target out of reach or IK failed')

def main(args=None):
    rclpy.init(args=args)
    node = PositionListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()