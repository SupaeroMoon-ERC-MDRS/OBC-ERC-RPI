# target_pose_publisher.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class TargetPublisher(Node):
    def __init__(self):
        super().__init__('target_pose_publisher')
        self.publisher_ = self.create_publisher(PoseStamped, 'target_pose', 10)
        self.timer = self.create_timer(2.0, self.timer_callback)

    def timer_callback(self):
        msg = PoseStamped()
        msg.header.frame_id = "base_link"
        msg.pose.position.x = 0.4
        msg.pose.position.y = 0.2
        msg.pose.position.z = 0.3
        msg.pose.orientation.w = 1.0
        self.publisher_.publish(msg)
        self.get_logger().info("Published target pose")

rclpy.init()
node = TargetPublisher()
rclpy.spin(node)
rclpy.shutdown()