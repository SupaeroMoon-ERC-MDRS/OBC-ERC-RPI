import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import curses

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
            'fl': servo.Servo(self.pca.channels[2], min_pulse=500, max_pulse=2500),  # front-left
            'fr': servo.Servo(self.pca.channels[1], min_pulse=500, max_pulse=2500),  # front-right
            'rl': servo.Servo(self.pca.channels[3], min_pulse=500, max_pulse=2500),  # rear-left
            'rr': servo.Servo(self.pca.channels[0], min_pulse=500, max_pulse=2500)   # rear-right
        }
        self.servo_zero = {
            "fl": 44.0,
            "fr": 261.0,
            "rl": 247.0,
            "rr": 41.0  
        }

        # self.calibrate_servos()
        self.get_logger().info('Servo Steering Node initialized.')

    def calibrate_servos(self):
        choice = input("Would you like to calibrate? [y/N] ").strip().lower()
        if choice not in ["y", "yes"]:
            print("Skipping calibration. Using defaults.")
            return

        def curses_main(stdscr):
            curses.curs_set(0)
            stdscr.nodelay(False)
            stdscr.timeout(-1)

            for servo_name, value in self.servo_zero.items():
                while True:
                    stdscr.clear()
                    stdscr.addstr(0, 0, f"Calibrating servo {servo_name.upper()}")
                    stdscr.addstr(2, 0, "Use UP/DOWN arrows to adjust, ENTER to confirm.")
                    stdscr.addstr(4, 0, f"Current value: {value:.1f}°")
                    stdscr.refresh()

                    key = stdscr.getch()
                    if key == curses.KEY_UP:
                        value += 1.0
                        self.servos[servo_name].angle = value
                    elif key == curses.KEY_DOWN:
                        value -= 1.0
                        self.servos[servo_name].angle = value
                    elif key in [10, 13]:  # Enter key
                        break

                self.servo_zero[servo_name] = value

        curses.wrapper(curses_main)
        print("Calibration complete. Final values:")
        print(self.servo_zero)

    def servo_callback(self, msg):
        try:
            # Extract angles from the incoming message
            angle_fl = self.servo_zero["fl"] + msg.data[1]
            angle_fr = self.servo_zero["fr"] + msg.data[3]
            angle_rl = self.servo_zero["rl"] + msg.data[9]
            angle_rr = self.servo_zero["rr"] + msg.data[11]

            # Clamp and set angles
            fl = self.clamp_angle(angle_fl * 180/300)
            fr = self.clamp_angle(angle_fr * 180/300)
            rl = self.clamp_angle(angle_rl * 180/300)
            rr = self.clamp_angle(angle_rr * 180/300)

            self.get_logger().info(f'Setting angles: FL={fl}, FR={fr}, RL={rl}, RR={rr}')

            self.servos['fl'].angle = fl
            self.servos['fr'].angle = fr
            self.servos['rl'].angle = rl
            self.servos['rr'].angle = rr

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
