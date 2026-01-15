import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import math

# Rover geometry
WHEEL_BASE = 0.199   # Distance between front and rear axles (meters)
TRACK_WIDTH = 0.446  # Distance between left and right wheels (meters)
WHEEL_RADIUS = 0.05  # Optional: if converting velocity to angular speed

class SixWheelFourWSController(Node):
    def __init__(self):
        super().__init__('six_wheel_fourws_controller')

        self.cmd_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 1)
        self.wheel_pub = self.create_publisher(Float64MultiArray, '/wheel_controller/commands', 1)
        self.get_logger().info('Six Wheel Four Wheel Steering Controller Node initialized.')

        self.cmd_vel_callback(Twist())
        self.get_logger().info('Six Wheel Four Wheel Steering Controller Zeroed.')


    def cmd_vel_callback(self, msg):
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z

        # Initialize output
        cmd = Float64MultiArray()

        # Default: no turning → all angles = 0
        angle_fl = 0.0 #2
        angle_fr = 0.0 #1
        angle_rl = 0.0 #3
        angle_rr = 0.0 #0 +ve CCW
        vel_fl = vel_fr = vel_ml = vel_mr = vel_rl = vel_rr = 0.0

        if abs(angular_vel) > 1e-5 and abs(linear_vel) > 1e-1:
            turning_radius = linear_vel / angular_vel
            sign = 1 if linear_vel >= 0 else -1

            # Calculate individual steering angles (4WS geometry)
            radius_fl = turning_radius - (TRACK_WIDTH / 2)
            radius_fr = turning_radius + (TRACK_WIDTH / 2)
            radius_rl = turning_radius - (TRACK_WIDTH / 2)
            radius_rr = turning_radius + (TRACK_WIDTH / 2)

            angle_fl = math.degrees(math.atan(sign * WHEEL_BASE / radius_fl))
            angle_fr = math.degrees(math.atan(sign * WHEEL_BASE / radius_fr))
            angle_rl = math.degrees(math.atan(-sign * WHEEL_BASE / radius_rl))  # rear steering typically opposite for tighter turns
            angle_rr = math.degrees(math.atan(-sign * WHEEL_BASE / radius_rr))

            vel_rl = vel_rr = vel_fl = vel_fr = vel_ml = vel_mr = linear_vel
            self.get_logger().info(f"Calculated angles: FL={angle_fl}, FR={angle_fr}, RL={angle_rl}, RR={angle_rr}")
        else:
            # Straight motion or no motion
            vel_rl = vel_rr = vel_fl = vel_fr = vel_ml = vel_mr = linear_vel

        # Fill message in specified format
        self.get_logger().info(f"Publishing wheel commands: FL={angle_fl}, FR={angle_fr}, RL={angle_rl}, RR={angle_rr}, Vel={linear_vel}")
        self.get_logger().info(f"Wheel velocities: FL={vel_fl}, FR={vel_fr}, ML={vel_ml}, MR={vel_mr}, RL={vel_rl}, RR={vel_rr}")
        cmd.data = [
            vel_fl, angle_fl,
            vel_fr, angle_fr,
            vel_ml, 0.0,
            vel_mr, 0.0,
            vel_rl, angle_rl,
            vel_rr, angle_rr
        ]

        self.wheel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = SixWheelFourWSController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.wheel_pub.publish([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # Stop all wheels on shutdown
    finally:
        node.wheel_pub.publish([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # Stop all wheels on shutdown
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
