import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
# Init Adafruit PCA9685
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo
import board
import busio

class IKServoController(Node):
    def __init__(self):
        super().__init__('inverse_kinematics')

        self.get_logger().info("Initializing I2C for PCA9685 and Servos")
        i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = PCA9685(i2c, address=0x43)
        self.pca.frequency = 50

        # Create servo objects
        self.base = servo.Servo(self.pca.channels[0], actuation_range=270, min_pulse=500, max_pulse=2500)
        self.theta_1 = servo.Servo(self.pca.channels[1], actuation_range=270, min_pulse=500, max_pulse=2500)
        self.theta_2 = servo.Servo(self.pca.channels[2], actuation_range=270, min_pulse=500, max_pulse=2500)
        self.gripper = servo.Servo(self.pca.channels[3], actuation_range=270, min_pulse=500, max_pulse=2500)
        
        self.get_logger().info("Homing Arm")
        # Arm lengths
        self.l1 = 0.11059
        self.l2 = 0.18052
        self.l3 = 0.16994

        self.total_reach = self.l2 + self.l3 # 0.51012
        self.current_x = 0.0
        self.current_y = self.total_reach - 0.05
        theta_1, theta_2 = self.compute_ik(dir='up')


        self.base_curr = 0.0
        self.theta_1_curr = theta_1
        self.theta_2_curr = theta_2
        self.gripper_curr = 90.0

        self.subscription = self.create_subscription(
            Twist,
            '/cmd_move_arm',
            self.cmd_callback,
            10
        )

        self.get_logger().info("IK Servo Controller node started.")

    def cmd_callback(self, msg):

        # Case 1: arm fwd/bkwd
        if abs(msg.linear.x) > 1e-4 and abs(msg.linear.y) < 1e-4:
            dz = msg.linear.x
            x = self.current_x + dz
            y = self.current_y
            d = math.sqrt(x**2 + y**2)
            if abs(d) > self.total_reach:
                self.get_logger().info(f"Current x is {x}, current y is {y}, total reach is {d}")
                self.get_logger().info("Max reach is " + str(self.total_reach))
                self.get_logger().info("Target position is out of reach.")
                return None
            else:
                self.current_x = x
                self.get_logger().info(f"Moving in X {x}")
                # if self.current_x >= 0:
                #     self.compute_ik(dir='up')
                # else:
                #     self.compute_ik(dir='down')

        # Case 2: arm up/down
        if abs(msg.linear.y) > 1e-4 and abs(msg.linear.x) < 1e-4:
            dz = msg.linear.y
            x = self.current_x
            y = self.current_y + dz
            d = math.sqrt(x**2 + y**2)
            if abs(d) > self.total_reach:
                self.get_logger().info(f"Current x is {x}, current y is {y}, total reach is {d}")
                self.get_logger().info("Max reach is " + str(self.total_reach))
                self.get_logger().info("Target position is out of reach.")
                return None
            else:
                self.current_y = y
                self.get_logger().info(f"Moving in Y {y}")
                # if self.current_x >= 0:
                #     self.compute_ik(dir='up')
                # else:
                #     self.compute_ik(dir='down')

        # Case 3: BOTH
        if abs(msg.linear.y) > 1e-4 and abs(msg.linear.x) > 1e-4:
            dx = msg.linear.x
            dy = msg.linear.y
            x = self.current_x + dx
            y = self.current_y + dy
            d = math.sqrt(x**2 + y**2)
            if abs(d) > self.total_reach:
                self.get_logger().info(f"Current x is {x}, current y is {y}, total reach is {d}")
                self.get_logger().info("Max reach is " + str(self.total_reach))
                self.get_logger().info("Target position is out of reach.")
                return None
            else:
                self.current_x = x
                self.current_y = y
                self.get_logger().info(f"Moving in BOTH X Y {x, y}")
                # if self.current_x >= 0:
                #     self.compute_ik(dir='up')
                # else:
                #     self.compute_ik(dir='down')

        # Case 3: arm base rotation
        if abs(msg.angular.x) > 1e-4:
            dtheta = msg.angular.x * 10
            self.base_curr += dtheta
            self.send_to_servo(self.base_curr, self.base)

        # Case 4: arm gripper open/close
        if abs(msg.linear.z) > 1e-4:
            dz = msg.linear.z * 3
            self.gripper_curr += dz
            self.send_to_servo(self.gripper_curr, self.gripper)
        else:
            self.get_logger().info("No arm movement command received.")


    def compute_ik(self, dir='up'):
        self.get_logger().info(f"Computing IK for current_x: {self.current_x}, current_y: {self.current_y}")
        y = self.current_y  # Adjust for base height # 0.30046
        x = self.current_x # 0
        d = math.sqrt(x**2 + y**2)  # 0.30046
        try:
            q2 = math.acos((self.l2**2 + self.l3**2 - d**2) / (2 * self.l2 * self.l3))
            if dir == 'up':
                q2 = -q2
            
            q1 = math.atan2(y, x) - math.atan2(self.l3 * math.sin(q2), self.l2 + self.l3 * math.cos(q2))
            q1_deg = math.degrees(q1)
            q2_deg = math.degrees(q2)

            self.theta_1_curr = q1_deg # updates current
            self.theta_2_curr = q2_deg

            theta_1_servo = 135 - 90 + q1_deg # Example mapping, adjust sign as needed
            theta_2_servo = 270 - (135 + q2_deg)

            self.get_logger().info(f"Computed angles: theta_1={theta_1_servo:.1f}, theta_2={theta_2_servo:.1f}")
            self.send_to_servo(theta_1_servo, self.theta_1)
            self.send_to_servo(theta_2_servo, self.theta_2)

        except ValueError as e:
            self.get_logger().error(f"Error computing IK: {e}")
            self.get_logger().info(f"Input values: x={x}, y={y}, d={d}")
            self.get_logger().info(f"differential: {self.l2**2 + self.l3**2 - d**2}")
            q1, q2 = None, None

        return q1, q2

    def send_to_servo(self, q1, servo1):

        a1 = max(0, min(270, q1))
        self.get_logger().info(f"Setting angles: servo={a1:.1f}")
        servo1.angle = a1

    def go_home(self):
        self.get_logger().info("Returning to home position.")
        self.send_to_servo(135.0, self.base)
        self.send_to_servo(135.0, self.theta_1)
        self.send_to_servo(135.0, self.theta_2)
        self.send_to_servo(90.0, self.gripper)
    
def main(args=None):
    rclpy.init(args=args)
    inv_kin = IKServoController()
    try:
        rclpy.spin(inv_kin)
    except KeyboardInterrupt:
        pass
    finally:
        inv_kin.go_home()
        inv_kin.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()