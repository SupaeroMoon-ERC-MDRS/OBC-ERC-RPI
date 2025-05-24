#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
import board
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

class ServoCommandNode(Node):
    def __init__(self):
        super().__init__('servo_command_node')
        self.get_logger().info("Initializing servo controller...")

        # Setup I2C and PCA9685
        i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = PCA9685(i2c, address=0x43)
        self.pca.frequency = 50

        # Create servo objects
        self.servos = [
            servo.Servo(self.pca.channels[0], min_pulse=500, max_pulse=2500),
            servo.Servo(self.pca.channels[1], min_pulse=500, max_pulse=2500),
            servo.Servo(self.pca.channels[2], min_pulse=500, max_pulse=2500),
            servo.Servo(self.pca.channels[3], min_pulse=500, max_pulse=2500),
        ]

        self.subscription = self.create_subscription(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            self.trajectory_callback,
            10
        )
        self.get_logger().info("ServoCommandNode ready and listening for joint trajectories")

    def trajectory_callback(self, msg):
        if not msg.points:
            self.get_logger().warn("Received JointTrajectory with no points")
            return

        joint_positions = msg.points[0].positions  # First trajectory point
        self.get_logger().info(f"Received joint positions: {joint_positions}")

        for i, position in enumerate(joint_positions):
            if i < len(self.servos):
                # Convert radians to degrees (adjust as needed for your robot)
                angle_deg = max(0, min(180, position * 180.0 / 3.1415))
                self.servos[i].angle = angle_deg
                self.get_logger().info(f"Set servo[{i}] to {angle_deg:.1f} degrees")
            else:
                self.get_logger().warn(f"Received command for joint {i}, but only {len(self.servos)} servos available")

def main(args=None):
    rclpy.init(args=args)
    node = ServoCommandNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
