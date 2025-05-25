## To create a ROS node to process incoming messages from the remote control
"""Untested code for now"""
import rclpy
from rclpy.node import Node
from roboclaw_driver import Roboclaw

from std_msgs.msg import Float64MultiArray
import numpy as np
import time


class RoboclawNode(Node):
    def __init__(self):
        super().__init__('roboclaw_speed_node')
        self.subscription = self.create_subscription(Float64MultiArray, '/wheel_controller/commands',
            self.cmd_vel_motors, 10
        )

        # logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")

        baud_rate = 115200
        dev_name1 = "/dev/ttyAMA0"  # change


        self.robo = Roboclaw(dev_name1, baud_rate)
        self.addresses = [int(128), int(129), int(130)]  # change
        self.gear_ratio = 26.9
        self.wheel_radius = 0.19/2
        self.ticks_per_rev = 752
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

        self.MAX_SPEED = 2.0  # to be tested
        # self.TICKS_PER_METER = 4342.2  # to be tested
        self.BASE_WIDTH = 0.33  # to be checked
        self.last_set_speed_time = time.time()
        self.get_logger().info("Roboclaw Node Initialized")

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
                self.robo.DutyAccelM1(address, self.accel, qppsm1)
                self.robo.DutyAccelM2(address, self.accel, qppsm2)
                self.get_logger().info(f"Motor {address} set to speed: {qppsm1} (M1), {qppsm2} (M2)")

                # Update timestamp
                self.last_set_speed_time = time.time()

        except Exception as e:
            self.get_logger().error(f"Motor command failed: {str(e)}")
            self.shutdown()

    def vel_to_qpps(self, vel):
        return int(vel * self.gear_ratio * self.conversion_factor * self.ticks_per_rev / (2 * np.pi * self.wheel_radius)) 
    # TODO: need clean shutdown so motors stop even if new msgs are arriving

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
