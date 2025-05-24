import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from adafruit_servokit import ServoKit  # Example; replace with your actual hardware library
import math
# Init Adafruit PCA9685
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo
import board
import busio

class IKServoController(Node):
    def __init__(self):
        super().__init__('inverse_kinematics')

        self.current_x = 0.1
        self.current_y = 0.1 # Total height of arm goes here
        self.base_curr = 0.0
        self.theta_1_curr = 0.0
        self.theta_2_curr = 0.0
        self.wrist_link_curr = 0.0
        self.wrist_rot_curr = 0.0
        self.gripper_curr = 90.0

        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_callback,
            10
        )

        
        i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = PCA9685(i2c, address=0x43)
        self.pca.frequency = 50

        # Create servo objects
        self.base = servo.Servo(self.pca.channels[8], min_pulse=500, max_pulse=2500)
        self.theta_1 = servo.Servo(self.pca.channels[9], min_pulse=500, max_pulse=2500)
        self.theta_2 = servo.Servo(self.pca.channels[10], min_pulse=500, max_pulse=2500)
        self.wrist_link = servo.Servo(self.pca.channels[11], min_pulse=500, max_pulse=2500)
        self.wrist_rot = servo.Servo(self.pca.channels[4], min_pulse=500, max_pulse=2500)
        self.gripper = servo.Servo(self.pca.channels[5], min_pulse=500, max_pulse=2500)
        

        # # Delete this to test physically
        # self.base = 0
        # self.theta_1 = 0
        # self.theta_2 = 0
        # self.wrist_link = 0
        # self.wrist_rot = 0
        # self.gripper = 0

        # Arm lengths
        self.l1 = 0.1752
        self.l2 = 0.16089
        self.l3 = 0.21823

        self.get_logger().info("IK Servo Controller node started.")

    def cmd_callback(self, msg):
        if abs(msg.linear.x) > 1e-4 or abs(msg.linear.y) > 1e-4:
            dx = msg.linear.x * 0.01
            dy = msg.linear.y * 0.01

            new_x = self.current_x + dx
            new_y = self.current_y + dy

            result = self.compute_ik(new_x, new_y)

            if result:
                q1, q2 = result
                self.send_to_servo(q1, self.theta_1)
                self.send_to_servo(q2, self.theta_2)
                self.current_x += dx
                self.current_y += dy
            else:
                self.get_logger().warn(f"Unreachable target: ({new_x:.2f}, {new_y:.2f})")
        
        if abs(msg.linear.z) > 1e-4: # open/close grip
            dz = msg.linear.z * 0.01
            self.gripper_curr += dz
            self.send_to_servo(self.gripper_curr, self.gripper)
            # TODO: Implement servo limits

        if abs(msg.angular.x) > 1e-4: # rotate base
            dtheta = msg.angular.x * 0.5
            self.base_curr += dtheta
            self.send_to_servo(self.base_curr, self.base)

        if abs(msg.angular.y) > 1e-4: # rotate arm
            dtheta = msg.angular.y * 0.5
            self.wrist_rot_curr += dtheta
            self.send_to_servo(self.wrist_rot_curr, self.wrist_rot)

        if abs(msg.angular.z) > 1e-4: # tilt wrist
            dtheta = msg.angular.z * 0.5
            self.wrist_link_curr += dtheta
            self.send_to_servo(self.wrist_link_curr, self.wrist_link)


    def compute_ik(self, x, y):
        d = (x**2 + y**2 - self.l1**2 - self.l2**2) / (2 * self.l1 * self.l2)
        if abs(d) > 1.0:
            return None
        q2 = math.acos(d)
        q1 = math.atan2(y, x) - math.atan2(self.l2 * math.sin(q2), self.l1 + self.l2 * math.cos(q2))
        return q1, q2

    def send_to_servo(self, q1, servo1):
        # Convert radians to degrees and map to servo angle range
        a1 = math.degrees(q1) * 180 / 300

        # Clamp to [0, 180] as needed for hobby servos
        a1 = max(0, min(180, a1))  # Offset for center position

        self.get_logger().info(f"Setting angles: servo0={a1:.1f}")
        servo1.angle = a1

def main(args=None):
    rclpy.init(args=args)
    inv_kin = IKServoController()
    try:
        rclpy.spin(inv_kin)
    except KeyboardInterrupt:
        pass
    finally:
        inv_kin.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()