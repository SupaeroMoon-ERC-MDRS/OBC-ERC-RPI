#!/usr/bin/env python3
import rclpy
import time
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Header
import numpy as np

class TrajectoryPlanner(Node):
    def __init__(self):
        super().__init__('trajectory_planner')

        self.joint_names = [
            'joint2',        # rotation Z
            'joint3',    # rotation X
            'joint4',       # rotation X
            'joint5',       # rotation X
            'end_effector_rotation',         # rotation Y
        ]

        # self.joint_names = [
        #     'Base link',        # rotation Z
        #     'joint1',    # rotation X
        #     'joint2',       # rotation X
        #     'joint3',       # rotation X
        #     'joint4',       # rotation X
        #     'joint5',       # rotation X
        #     'end_effector_rotation',         # rotation Y
        #     'left_finger_joint',         # end effector
        # ]

        self.current_position = np.zeros(len(self.joint_names))  # valeurs actuelles
        self.target_position = None

        self.publisher_ = self.create_publisher(JointState, '/joint_states', 10)
        self.publisher_servo = self.create_publisher(JointState, '/servos_execution_commands', 10)
        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/servos_commands',
            self.target_callback,
            10
        )

        self.timer = self.create_timer(0.5, self.timer_callback)  # 20 Hz

    def target_callback(self, msg):
        print(len(msg.data))
        print(len(self.joint_names))
        print(msg.data)
        print(self.joint_names)
        if len(msg.data) != len(self.joint_names):
            self.get_logger().error("Mauvaise taille de commande")
            return
        self.target_position = np.array(msg.data)
        self.get_logger().info(f"Nouvelle cible : {msg.data}")

    def timer_callback(self):
        if self.target_position is None:
            return

        # Interpolation linéaire simple
        direction = self.target_position - self.current_position
        step = 0.02  # Ajuste ce pas selon la vitesse souhaitée

        if np.linalg.norm(direction) < step:
            self.current_position = self.target_position
            self.target_position = None
        else:
            self.current_position += step * direction / np.linalg.norm(direction)

        # Publie la position intermédiaire
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.current_position.tolist()

        self.publisher_.publish(msg)
        self.publisher_servo.publish(msg)
        time.sleep(0.5)

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPlanner()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()