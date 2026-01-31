#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
import math
import time
class SimpleMarkerFollower(Node):
    """
    A lightweight marker follower that does NOT use Nav2.
    It subscribes to the marker pose (relative to camera) and directly
    publishes velocity commands (Twist) to follow the marker.
    """
    def __init__(self):
        super().__init__('simple_marker_follower')

        # Parameters
        self.safe_follow_distance = 0.4
        self.max_linear_vel = 5.0
        self.max_angular_vel = 10.0
        
        # Gains for the proportional controller
        self.kp_dist = 1.5
        self.kp_ang = 1.5
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
        self.marker_visible = True
        self.last_marker_time = self.get_clock().now()

        # --- COORDINATE TRANSFORMATION ---
        # Camera Optical Frame: Z=Forward, X=Right, Y=Down
        # Robot Body Frame:     X=Forward, Y=Left,  Z=Up
        
        # 1. Get Forward Distance (Camera Z)
        # We subtract safe_follow_distance from Z
        forward_dist = msg.pose.position.x
        dist_error = forward_dist - self.safe_follow_distance

        # 2. Get Lateral Error (Robot Y is Camera -X)
        lateral_error = msg.pose.position.y      
        # 3. Calculate Angle to Marker (Yaw)
        # atan2(Y, X) -> atan2(Left, Forward)
        angle_error = math.atan2(lateral_error, forward_dist)

        # --- CONTROL LOGIC (Point & Shoot) ---
        heading_tolerance = 0.5 # Radians

        if abs(angle_error) > heading_tolerance:
            # Turn in place if not facing target
            lin_cmd = 0.0
            ang_cmd = self.kp_ang * angle_error
        else:
            # Drive and steer
            lin_cmd = self.kp_dist * dist_error
            ang_cmd = self.kp_ang * angle_error

        # Limits
        lin_cmd = max(min(lin_cmd, self.max_linear_vel), -self.max_linear_vel)
        ang_cmd = max(min(ang_cmd, self.max_angular_vel), -self.max_angular_vel)

        self.target_linear = lin_cmd
        self.target_angular = ang_cmd
        self.get_logger().info(f'Computed target linear: {lin_cmd}, angular: {ang_cmd}')

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
            self.get_logger().info('Marker lost, stopping...', throttle_duration_sec=2.0)

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
        pass
    finally:
        # 1. Create the Stop Command
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        
        # 2. Publish it multiple times just to be safe
        node.get_logger().info("Sending STOP command...")
        for _ in range(3):
            node.pub_vel.publish(twist)
            time.sleep(0.1)  # <--- THE FIX: Wait for it to send
        
        # 3. NOW you can shut down
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
