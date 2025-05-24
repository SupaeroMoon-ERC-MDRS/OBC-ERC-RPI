#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Float64MultiArray
from my_python_pkg.inverse_kinematics_solver import compute_joint_angles

from ikpy.chain import Chain
from ikpy.utils import plot as plot_utils
import numpy as np
import os

from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

class PositionListener(Node):
    def __init__(self):
        super().__init__('position_listener')

        # urdf_path = PathJoinSubstitution([
        #     FindPackageShare('my_python_pkg'),
        #     'urdf',
        #     'robot_simple.urdf'
        # ])

        # Charger le modèle URDF
        # urdf_path = os.path.join(os.path.dirname(__file__), '..', 'urdf', 'robot_simple.urdf')
        urdf_path = os.path.join("/", "home", "michou", "robot_arm_manual_ws","install","my_python_pkg","share","my_python_pkg","urdf", 'robot_simple.urdf')
        self.robot_chain = Chain.from_urdf_file(urdf_path)

        # Lister les joints (tous les liens, y compris fixes)
        print("=== Tous les liens de la chaîne ===")
        for i, link in enumerate(self.robot_chain.links):
            print(f"{i}: {link.name} (type: {'active' if self.robot_chain.active_links_mask[i] else 'fixed'})")

        # Si tu veux uniquement les joints actifs :
        print("\n=== Joints actifs (utilisés pour IK) ===")
        for i, (link, is_active) in enumerate(zip(self.robot_chain.links, self.robot_chain.active_links_mask)):
            if is_active:
                print(f"{i}: {link.name}")

        self.publisher = self.create_publisher(Float64MultiArray, '/servos_commands', 10)
        self.subscription = self.create_subscription(Point, '/target_position', self.listener_callback, 10)

        self.last_angles = None

    def listener_callback(self, msg):
        # Définir la cible en 3D
        target_position = [msg.x, msg.y, msg.z]
        self.get_logger().info(f"Cible reçue : {target_position}")

        # Résolution de l’IK
        ik_solution = self.robot_chain.inverse_kinematics(target_position)

        # Extraire les angles d’articulations utiles (adapter à ton URDF)
        servo_angles = ik_solution[1:6]  # ajuster en fonction des joints actifs

        if self.last_angles is None or not np.allclose(self.last_angles, servo_angles, atol=0.01):
            msg_out = Float64MultiArray()
            msg_out.data = servo_angles.tolist()
            self.publisher.publish(msg_out)
            self.get_logger().info(f"Angles publiés : {msg_out.data}")
            self.last_angles = servo_angles

def main(args=None):
    rclpy.init(args=args)
    node = PositionListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()