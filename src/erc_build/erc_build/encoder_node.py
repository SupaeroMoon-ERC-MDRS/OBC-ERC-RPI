import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from tf_transformations import quaternion_from_euler
import time
import numpy as np
from roboclaw_driver import Roboclaw

class EncoderNode(Node):
    def __init__(self):
        super().__init__('encoder_odometry_node')

        # Setup publisher
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.timer = self.create_timer(0.05, self.update_odometry)  # 20 Hz

        # Roboclaw Setup
        self.addresses = [128, 129, 130]
        self.roboclaw = Roboclaw("/dev/ttyAMA0", 115200)
        self.roboclaw.Open()

        # Robot parameters
        self.TICKS_PER_REV = 752
        self.GEAR_RATIO = 26.9
        self.WHEEL_RADIUS = 0.19/2  # meters
        self.TICKS_PER_METER = self.TICKS_PER_REV * self.GEAR_RATIO / (2 * np.pi * self.WHEEL_RADIUS)
        self.BASE_WIDTH = 0.33  # Distance between wheels (L/R), meters

        # Robot pose
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Encoder tracking
        self.prev_left = 0
        self.prev_right = 0
        self.last_time = time.time()

        # Initial encoder read
        self.prev_left = self.get_average_encoder(side='left')
        self.prev_right = self.get_average_encoder(side='right')

        self.get_logger().info("Encoder Odometry Node Initialized")

    def get_average_encoder(self, side='left'):
        encoders = []
        for i, addr in enumerate(self.addresses):
            if side == 'left':
                result = self.roboclaw.ReadEncM2(addr)
            else:
                result = self.roboclaw.ReadEncM1(addr)
            if result[0]:  # success
                encoders.append(result[1])
        return np.mean(encoders) if encoders else 0

    def update_odometry(self):
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        # Get current encoder readings
        left_ticks = self.get_average_encoder('left')
        right_ticks = self.get_average_encoder('right')

        delta_left = (left_ticks - self.prev_left) / self.TICKS_PER_METER
        delta_right = (right_ticks - self.prev_right) / self.TICKS_PER_METER

        self.prev_left = left_ticks
        self.prev_right = right_ticks

        # Compute odometry
        delta_s = (delta_right + delta_left) / 2.0
        delta_theta = (delta_right - delta_left) / self.BASE_WIDTH

        if abs(delta_theta) < 1e-6:
            delta_x = delta_s * np.cos(self.theta)
            delta_y = delta_s * np.sin(self.theta)
        else:
            radius = delta_s / delta_theta
            delta_x = radius * (np.sin(self.theta + delta_theta) - np.sin(self.theta))
            delta_y = -radius * (np.cos(self.theta + delta_theta) - np.cos(self.theta))

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta
        self.theta = self.normalize_angle(self.theta)

        # Publish odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_footprint"

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        quat = quaternion_from_euler(0, 0, self.theta)
        odom_msg.pose.pose.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])

        odom_msg.twist.twist.linear.x = delta_s / dt if dt > 0 else 0.0
        odom_msg.twist.twist.angular.z = delta_theta / dt if dt > 0 else 0.0

        self.odom_pub.publish(odom_msg)

    def normalize_angle(self, angle):
        while angle > np.pi:
            angle -= 2.0 * np.pi
        while angle < -np.pi:
            angle += 2.0 * np.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    node = EncoderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
