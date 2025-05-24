import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty

class KeyboardTeleopNode(Node):
    def __init__(self):
        super().__init__('keyboard_arm')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('Keyboard Teleop Node Started')
        self.run()

    def run(self):
        self.get_logger().info("Press 'w' to send +1 Twist, 's' to send -1 Twist, 'q' to quit.")
        while rclpy.ok():
            key = self.get_key()
            if key == 'a': 
                self.publish_twist([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            elif key == 'w':
                self.publish_twist([1.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            elif key == 's':
                self.publish_twist([-1.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            elif key == 'e':
                self.publish_twist([0.0, 1.0, 0.0, 0.0, 0.0, 0.0])
            elif key == 'd':
                self.publish_twist([0.0, -1.0, 0.0, 0.0, 0.0, 0.0])
            elif key == 'r': # Gripper open
                self.publish_twist([0.0, 0.0, 1.0, 0.0, 0.0, 0.0])
            elif key == 'f': # Gripper Close
                self.publish_twist([0.0, 0.0, -1.0, 0.0, 0.0, 0.0])
            elif key == 't': # Base +ve
                self.publish_twist([0.0, 0.0, 0.0, 1.0, 0.0, 0.0])
            elif key == 'g': # Base -ve
                self.publish_twist([0.0, 0.0, 0.0, -1.0, 0.0, 0.0])
            elif key == 'y': # wrist CW
                self.publish_twist([0.0, 0.0, 0.0, 0.0, 1.0, 0.0])
            elif key == 'h': # Wrist CCW
                self.publish_twist([0.0, 0.0, 0.0, 0.0, -1.0, 0.0])
            elif key == 'u': # tilt Wrist
                self.publish_twist([0.0, 0.0, 0.0, 0.0, 0.0, 1.0])
            elif key == 'j': # tilt wrist
                self.publish_twist([0.0, 0.0, 0.0, 0.0, 0.0, -1.0])
            elif key == 'q':
                self.get_logger().info('Exiting...')
                break

    def publish_twist(self, values):
        twist = Twist()
        twist.linear.x = values[0]
        twist.linear.y = values[1]
        twist.linear.z = values[2]
        twist.angular.x = values[3]
        twist.angular.y = values[4]
        twist.angular.z = values[5]
        self.publisher_.publish(twist)
        self.get_logger().info(f'Published Twist: {values}')

    def get_key(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

def main(args=None):
    rclpy.init(args=args)
    keyboard_arm = KeyboardTeleopNode()
    try:
        rclpy.spin(keyboard_arm)
    except KeyboardInterrupt:
        pass
    finally:
        keyboard_arm.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
