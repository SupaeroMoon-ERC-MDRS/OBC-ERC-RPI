#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
import math

class SimpleMarkerFollower(Node):
    """
    A lightweight marker follower that does NOT use Nav2.
    It subscribes to the marker pose (relative to camera) and directly
    publishes velocity commands (Twist) to follow the marker.
    """
    def __init__(self):
        super().__init__('simple_marker_follower')

        # Parameters
        self.safe_follow_distance = self.declare_parameter('safe_follow_distance', 0.8).value
        self.max_linear_vel = self.declare_parameter('max_linear_vel', 0.5).value
        self.max_angular_vel = self.declare_parameter('max_angular_vel', 1.0).value
        
        # Gains for the proportional controller
        self.kp_dist = self.declare_parameter('kp_dist', 0.5).value
        self.kp_ang = self.declare_parameter('kp_ang', 1.5).value

        # Topics
        self.sub = self.create_subscription(
            PoseStamped,
            '/aruco_marker_pose',
            self.marker_cb,
            10
        )
        self.pub_vel = self.create_publisher(Twist, '/cmd_vel', 10)

        # Safety: stop if no marker seen for a while
        self.last_marker_time = self.get_clock().now()
        self.timer = self.create_timer(0.1, self.control_loop)
        
        # State
        self.target_linear = 0.0
        self.target_angular = 0.0
        self.marker_visible = False

        self.get_logger().info('SimpleMarkerFollower started. Tracking /aruco_marker_pose directly.')

    def marker_cb(self, msg: PoseStamped):
        """
        Callback when a marker pose is received in camera/robot frame.
        We assume the publisher sends pose where:
        X = Forward distance
        Y = Lateral (Left)
        Z = Vertical
        """
        self.marker_visible = True
        self.last_marker_time = self.get_clock().now()

        # 1. Calculate errors
        # Distance error: we want to be at 'safe_follow_distance'
        current_dist = msg.pose.position.x
        dist_error = current_dist - self.safe_follow_distance

        # Angle error: we want to center the marker (Y=0)
        # Using atan2 to get the angle in radians
        angle_error = math.atan2(msg.pose.position.y, msg.pose.position.x)

        # 2. Compute control commands (P-controller)
        lin_cmd = self.kp_dist * dist_error
        ang_cmd = self.kp_ang * angle_error

        # 3. Apply limits
        # If we are too close (negative error), valid to backup (negative vel)
        # but usually we clamp max speed magnitude
        lin_cmd = max(min(lin_cmd, self.max_linear_vel), -self.max_linear_vel)
        ang_cmd = max(min(ang_cmd, self.max_angular_vel), -self.max_angular_vel)

        # Deadband / Stop if very close to target
        if abs(dist_error) < 0.05:
            lin_cmd = 0.0
        if abs(angle_error) < 0.05:
            ang_cmd = 0.0

        self.target_linear = lin_cmd
        self.target_angular = ang_cmd

    def control_loop(self):
        """
        Publishes the latest calculated velocity, or stops if marker is lost.
        """
        # Check timeout (e.g. 0.5 seconds without marker)
        if (self.get_clock().now() - self.last_marker_time).nanoseconds > 0.5 * 1e9:
            self.marker_visible = False
            self.target_linear = 0.0
            self.target_angular = 0.0
            # Optional: print once that we lost the marker
            # self.get_logger().info('Marker lost, stopping...', throttle_duration_sec=2.0)

        # Publish command
        twist = Twist()
        twist.linear.x = float(self.target_linear)
        twist.angular.z = float(self.target_angular)
        self.pub_vel.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleMarkerFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        node.pub_vel.publish(twist)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
