#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from std_msgs.msg import Float64MultiArray  # Format des commandes servos
from builtin_interfaces.msg import Time

class ServosToJointStates(Node):
    def __init__(self):
        super().__init__('servos_to_joint_states')

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


        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/servos_commands',
            self.command_callback,
            10
        )

        self.publisher = self.create_publisher(JointState, '/joint_states', 10)

    def command_callback(self, msg):
        if len(msg.data) != len(self.joint_names):
            self.get_logger().error(f"Nombre de valeurs incorrect : {len(msg.data)} au lieu de {len(self.joint_names)}")
            return

        joint_state_msg = JointState()
        joint_state_msg.header = Header()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = self.joint_names
        joint_state_msg.position = msg.data

        self.publisher.publish(joint_state_msg)
        self.get_logger().info(f"Positions publiées : {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = ServosToJointStates()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()