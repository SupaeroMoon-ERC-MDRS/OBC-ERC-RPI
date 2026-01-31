## To create a ROS node to process incoming messages from the remote control
"""Untested code for now"""
import rclpy
from rclpy.node import Node
from roboclaw_driver import Roboclaw
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler

from std_msgs.msg import Float64MultiArray
import numpy as np
import time


class RoboclawNode(Node):
    def __init__(self):
        super().__init__('roboclaw_speed_node')
        self.subscription = self.create_subscription(Float64MultiArray, '/wheel_controller/commands',
            self.cmd_vel_motors, 10
        )
        self.odom_pub = self.create_publisher(Odometry, '/enc_odom', 10)


        # logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")

        baud_rate = 115200
        dev_name1 = "/dev/ttyAMA0"  # change


        self.robo = Roboclaw(dev_name1, baud_rate)
        self.addresses = [int(128), int(129), int(130)]  # change
        self.gear_ratio = 50
        self.wheel_radius = 0.1/2
        self.ticks_per_rev = 64*self.gear_ratio
        self.conversion_factor = 1/50
        self.accel = int(16383/2)

        print("Starting motor drives")

        try:
            self.robo.Open()
            self.get_logger().info("Successfully opened serial communications")
        except Exception as e:
            self.get_logger().error("Could not connect to Roboclaw: %s", e)
            raise e

        for address in self.addresses:
            try:
                self.get_logger().info(f"Attempting to talk to motor controller {address} through serial port {dev_name1} at a {baud_rate} baud_rate.")
                version = self.robo.ReadVersion(address)
                self.get_logger().info(f"response for RC at {address}: {version}")
                if version[0]:
                    self.get_logger().info(f"Roboclaw Version: {repr(version[1])}")
                else:
                    self.get_logger().warn("Could not get version from Roboclaw")
            except Exception as e:
                self.get_logger().error("Could not connect to Roboclaw: %s", e)
                raise e
            self.robo.ForwardM1(address, 0)
            self.robo.ForwardM2(address, 0)
            self.robo.ResetEncoders(address)

        # self.ticks_per_meter = 4342.2  # to be tested
        self.BASE_WIDTH = 0.446  # to be checked
        self.ticks_per_meter = self.ticks_per_rev / (2 * np.pi * self.wheel_radius)
        # 509554

        # Robot pose
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Encoder tracking
        self.prev_left = 0
        self.prev_right = 0
        self.last_time = time.time()

        # Initial encoder read
        self.prev_left = self.get_single_encoder(side='left', addr=130)
        self.prev_right = self.get_single_encoder(side='right', addr=130)

        self.last_set_speed_time = time.time()
        self.get_logger().info("Roboclaw Node Initialized")

    def cmd_vel_motors(self, msg):
        """Handle velocity commands for multiple differential drive motors"""
        try:
            vel_fl = msg.data[0]
            vel_fr = msg.data[1]
            vel_rl = msg.data[2]
            vel_rr = msg.data[3]
            vel_ml = msg.data[4]
            vel_mr = msg.data[5]
            rights = [vel_fr, vel_rr, vel_mr]
            lefts = [vel_fl, vel_rl, vel_ml]
            
            for i, address in enumerate(self.addresses): # Double check order of motors
                right_speed = rights[i]
                left_speed = lefts[i]

                # Ticks conversion
                qppsm1 = self.vel_to_qpps(left_speed)
                qppsm2 = self.vel_to_qpps(right_speed)
                self.get_logger().debug(f"Attempting to set motor {address} to speed {qppsm1} (M1), {qppsm2} (M2)")
                self.robo.DutyAccelM1(address, self.accel, qppsm1)
                self.robo.DutyAccelM2(address, self.accel, qppsm2)
                self.get_logger().debug(f"Motor {address} set to speed: {qppsm1} (M1), {qppsm2} (M2)")

                # Update timestamp
                self.last_set_speed_time = time.time()
            self.update_odometry()
        except Exception as e:
            self.get_logger().error(f"Motor command failed: {str(e)}")
            self.shutdown()

    def vel_to_qpps(self, vel):
        return int(vel * self.gear_ratio * self.conversion_factor * self.ticks_per_rev / (2 * np.pi * self.wheel_radius)) 
    # TODO: need clean shutdown so motors stop even if new msgs are arriving

    def get_average_encoder(self, side='left'):
        encoders = []
        for _, addr in enumerate(self.addresses):
            if side == 'left':
                result = self.robo.ReadEncM2(addr)
            else:
                result = self.robo.ReadEncM1(addr)
            if result[0]:  # success
                encoders.append(result[1])
        return np.mean(encoders) if encoders else 0

    def get_single_encoder(self, side='left', addr=130):
        self.get_logger().debug(f"Getting encoder for side {side} at address {addr}")

        # for i, addr in enumerate(self.addresses):
        if side == 'left':
            result = self.robo.ReadEncM1(addr)
            self.get_logger().debug(f"Got left encoder values: {result}")
        else:
            result = self.robo.ReadEncM2(addr)
            self.get_logger().debug(f"Got right encoder values: {result}")
        if result[1]:  # success
            return result[0]
        else:
            self.get_logger().warn(f"Failed to get encoder values.")
            return 0
    
    def update_odometry(self):
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        # Get current encoder readings
        left_ticks = self.get_single_encoder('left', 130)
        right_ticks = self.get_single_encoder('right', 130)

        delta_left = (left_ticks - self.prev_left) / self.ticks_per_meter
        delta_right = (right_ticks - self.prev_right) / self.ticks_per_meter

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

    def shutdown(self):
        self.get_logger().info("Shutting down")
        try:
            for address in self.addresses:
                self.robo.ForwardM1(address, 0)
                self.robo.ForwardM2(address, 0)
        except OSError:
            self.get_logger().info("Shutdown did not work trying again")
            try:
                for address in self.addresses:
                    self.robo.ForwardM1(address, 0)
                    self.robo.ForwardM2(address, 0)
            except OSError as e:
                self.get_logger().error("Could not shutdown motors!!!!")
                self.get_logger().error(e)


def main(args=None):
    rclpy.init(args=args)
    node = RoboclawNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
