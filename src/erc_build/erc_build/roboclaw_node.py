## To create a ROS node to process incoming messages from the remote control
"""Untested code for now"""
import rclpy
from rclpy.node import Node
from roboclaw_driver import Roboclaw
from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler
from geometry_msgs.msg import Quaternion

from std_msgs.msg import Float64MultiArray
import numpy as np
import time


class RoboclawNode(Node):
    def __init__(self):
        super().__init__('roboclaw_speed_node')
        self.subscription = self.create_subscription(Float64MultiArray, '/wheel_controller/commands',
            self.cmd_vel_motors, 10
        )
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        
        # logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")

        baud_rate = 115200
        dev_name1 = "/dev/ttyAMA0"  # change


        self.robo = Roboclaw(dev_name1, baud_rate)
        self.addresses = [int(128), int(129), int(130)]  # change
        self.gear_ratio = 26.9
        self.WHEEL_RADIUS = 0.19/2
        self.TICKS_PER_REV = 752
        self.meters_per_tick = (2 * np.pi * self.WHEEL_RADIUS) / self.TICKS_PER_REV
        self.conversion_factor = 1/50
        self.accel = int(16383/2)
        self.prev_left_ticks = [None] * len(self.addresses)
        self.prev_right_ticks = [None] * len(self.addresses)
        self.encoder_ok = [True] * len(self.addresses)
        self.max_tick_jump = 20000   # adjust for your encoder resolution
        
        # Encoder tracking
        self.prev_left_ticks = [None] * len(self.addresses)
        self.prev_right_ticks = [None] * len(self.addresses)
        self.encoder_ok = [True] * len(self.addresses)
        self.max_tick_jump = 20000

        # Odometry state - FIXED: Initialize these!
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_odom_time = time.time()

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

        self.MAX_SPEED = 2.0  # to be tested
        # self.TICKS_PER_METER = 4342.2  # to be tested
        self.BASE_WIDTH = 0.33  # to be checked
        self.last_set_speed_time = time.time()
        self.get_logger().info("Roboclaw Node Initialized")
        self.timer = self.create_timer(0.02, self.publish_odom)

    def cmd_vel_motors(self, msg):
        """Handle velocity commands for multiple differential drive motors"""
        try:
            vel_fl = msg.data[0]
            vel_fr = msg.data[2]
            vel_rl = msg.data[4]
            vel_rr = msg.data[6]
            vel_ml = msg.data[8]
            vel_mr = msg.data[10]
            rights = [vel_fr, vel_mr, vel_rr]
            lefts = [vel_fl, vel_ml, vel_rl]
            
            for i, address in enumerate(self.addresses): # Double check order of motors
                right_speed = rights[i]
                left_speed = lefts[i]

                # Ticks conversion
                qppsm1 = self.vel_to_qpps(right_speed)
                qppsm2 = self.vel_to_qpps(left_speed)
                self.get_logger().info(f"Attempting to set motor {address} to speed {qppsm1} (M1), {qppsm2} (M2)")
                self.robo.DutyAccelM1(address, self.accel, qppsm1)
                self.robo.DutyAccelM2(address, self.accel, qppsm2)
                self.get_logger().info(f"Motor {address} set to speed: {qppsm1} (M1), {qppsm2} (M2)")

                # Update timestamp
                self.last_set_speed_time = time.time()

        except Exception as e:
            self.get_logger().error(f"Motor command failed: {str(e)}")
            self.shutdown()

    def vel_to_qpps(self, vel):
        return int(vel * self.gear_ratio * self.conversion_factor * self.TICKS_PER_REV / (2 * np.pi * self.WHEEL_RADIUS)) 
    # TODO: need clean shutdown so motors stop even if new msgs are arriving

    def publish_odom(self):
        """Publish odometry from encoder readings"""
        try:
            right_displacements = []
            left_displacements = []
            
            now = time.time()
            dt = now - self.last_odom_time
            
            if dt <= 0:
                return
            
            # Read all encoders
            for i, address in enumerate(self.addresses):
                enc_right, status_right = self.robo.ReadEncM1(address)  # M1 = right
                enc_left, status_left = self.robo.ReadEncM2(address)    # M2 = left

                # --- PROCESS RIGHT ENCODER (M1) ---
                if enc_right == 0 and status_right == 0:
                    # Failed read
                    self.get_logger().warn(f"Encoder M1 (right) on {address} FAILED")
                    
                elif self.prev_right_ticks[i] is not None:
                    # Check for unreasonable jumps
                    d_ticks = enc_right - self.prev_right_ticks[i]
                    
                    if abs(d_ticks) > self.max_tick_jump:
                        self.get_logger().warn(
                            f"Encoder M1 (right) on {address} JUMP: {d_ticks} ticks"
                        )
                    else:
                        # Valid reading
                        displacement = d_ticks * self.meters_per_tick
                        right_displacements.append(displacement)
                
                # Update previous value
                self.prev_right_ticks[i] = enc_right

                # --- PROCESS LEFT ENCODER (M2) ---
                if enc_left == 0 and status_left == 0:
                    # Failed read
                    self.get_logger().warn(f"Encoder M2 (left) on {address} FAILED")
                    
                elif self.prev_left_ticks[i] is not None:
                    # Check for unreasonable jumps
                    d_ticks = enc_left - self.prev_left_ticks[i]
                    
                    if abs(d_ticks) > self.max_tick_jump:
                        self.get_logger().warn(
                            f"Encoder M2 (left) on {address} JUMP: {d_ticks} ticks"
                        )
                    else:
                        # Valid reading
                        displacement = d_ticks * self.meters_per_tick
                        left_displacements.append(displacement)
                
                # Update previous value
                self.prev_left_ticks[i] = enc_left

            # Check if we have enough valid readings
            if len(left_displacements) < 1 or len(right_displacements) < 1:
                self.get_logger().error("Too many encoder failures — cannot compute odom")
                return

            # Average displacements from all valid encoders
            dist_left = sum(left_displacements) / len(left_displacements)
            dist_right = sum(right_displacements) / len(right_displacements)

            # --- DIFFERENTIAL DRIVE ODOMETRY ---
            # Forward displacement
            d_center = (dist_right + dist_left) / 2.0
            
            # Angular displacement
            d_theta = (dist_right - dist_left) / self.BASE_WIDTH
            
            # Update pose
            if abs(d_theta) < 1e-6:
                # Straight motion
                dx = d_center * np.cos(self.theta)
                dy = d_center * np.sin(self.theta)
            else:
                # Arc motion
                R = d_center / d_theta
                dx = R * (np.sin(self.theta + d_theta) - np.sin(self.theta))
                dy = -R * (np.cos(self.theta + d_theta) - np.cos(self.theta))
            
            self.x += dx
            self.y += dy
            self.theta += d_theta
            self.theta = np.arctan2(np.sin(self.theta), np.cos(self.theta))  # Normalize

            # Calculate velocities
            v = d_center / dt
            w = d_theta / dt

            # --- PUBLISH ODOMETRY MESSAGE ---
            msg = Odometry()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "odom"
            msg.child_frame_id = "base_footprint"

            # Position
            msg.pose.pose.position.x = self.x
            msg.pose.pose.position.y = self.y
            msg.pose.pose.position.z = 0.0

            # Orientation - FIXED: Create Quaternion message properly
            quat_array = quaternion_from_euler(0, 0, self.theta)
            msg.pose.pose.orientation = Quaternion(
                x=quat_array[0],
                y=quat_array[1],
                z=quat_array[2],
                w=quat_array[3]
            )

            # Velocity
            msg.twist.twist.linear.x = v
            msg.twist.twist.linear.y = 0.0
            msg.twist.twist.angular.z = w

            # Covariance (basic estimates)
            msg.pose.covariance[0] = 0.001   # x variance
            msg.pose.covariance[7] = 0.001   # y variance
            msg.pose.covariance[35] = 0.01   # yaw variance

            msg.twist.covariance[0] = 0.01   # vx variance
            msg.twist.covariance[35] = 0.1   # vyaw variance

            self.odom_pub.publish(msg)
            
            # Update time
            self.last_odom_time = now

        except Exception as e:
            self.get_logger().error(f"Odometry publish failed: {str(e)}")


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
