import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

import board
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo


class ServoSteeringNode(Node):
    def __init__(self):
        super().__init__('servo_steering_node')

        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/wheel_controller/commands',
            self.servo_callback,
            10
        )

        # Setup I2C and PCA9685
        i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = PCA9685(i2c, address=0x43)
        self.pca.frequency = 50

        # Setup four servos
        self.servos = {
            'fl': servo.Servo(self.pca.channels[0]),  # front-left
            'fr': servo.Servo(self.pca.channels[1]),  # front-right
            'rl': servo.Servo(self.pca.channels[2]),  # rear-left
            'rr': servo.Servo(self.pca.channels[3])   # rear-right
        }

        self.get_logger().info('Servo Steering Node initialized.')

    def servo_callback(self, msg):
        try:
            # Extract angles from the incoming message
            angle_fl = msg.data[1]
            angle_fr = msg.data[3]
            angle_rl = msg.data[9]
            angle_rr = msg.data[11]

            # Clamp and set angles
            self.servos['fl'].angle = self.clamp_angle(angle_fl)
            self.servos['fr'].angle = self.clamp_angle(angle_fr)
            self.servos['rl'].angle = self.clamp_angle(angle_rl)
            self.servos['rr'].angle = self.clamp_angle(angle_rr)

        except IndexError:
            self.get_logger().warn('Received malformed servo command message.')
        except Exception as e:
            self.get_logger().error(f'Servo command failed: {str(e)}')

    def clamp_angle(self, angle):
        return max(0, min(180, angle))


def main(args=None):
    rclpy.init(args=args)
    node = ServoSteeringNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
