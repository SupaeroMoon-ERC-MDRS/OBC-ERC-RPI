import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from tf_transformations import quaternion_from_euler
import math

ROVER_WHEEL_RADIUS = 0.05 # Needs to be updated
TRACK_WIDTH = 0.446  # Needs to be updated
WHEEL_BASE = 0.199

class DifferentialDriveController(Node):
    def __init__(self):
        super().__init__('controller')
        self.motor_wheel_pub = self.create_publisher(Float64MultiArray, '/wheel_controller/commands', 1)
        self.odom_pub = self.create_publisher(Odometry, 'odom_sim', 10)
        self.sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 1)
        
        self.x_position = 0.0
        self.y_position = 0.0
        self.theta = 0.0
        self.left_vel = 0.0
        self.right_vel = 0.0

        self.lf = 0.0
        self.lm = 0.0
        self.lr = 0.0
        self.rf = 0.0
        self.rm = 0.0
        self.rr = 0.0

        self.last_time = self.get_clock().now()

    def cmd_vel_callback(self, msg):
        linear_velocity = -msg.linear.x
        angular_velocity = msg.angular.z

        self.lf, self.lm, self.lr, self.rf, self.rm, self.rr = self.calculate_wheel_speeds(linear_velocity, angular_velocity)

        # self.left_vel = (linear_velocity - (angular_velocity * TRACK_WIDTH / 2)) / ROVER_WHEEL_RADIUS
        # self.right_vel = (linear_velocity + (angular_velocity * TRACK_WIDTH / 2)) / ROVER_WHEEL_RADIUS

        self.publish_wheel_commands()
        self.update_odometry()

    def publish_wheel_commands(self):
        wheel_msg = Float64MultiArray()
        wheel_msg.data = [self.lf, self.rf*1.25, self.lr*1.8, self.rr*1.03, self.lm, self.rm] #swap rf and rr once wiring is fixed
        self.motor_wheel_pub.publish(wheel_msg)

    def calculate_wheel_speeds(self, base_speed, angular_velocity):
        L = WHEEL_BASE  # distance to front or rear from center
        W = TRACK_WIDTH

        if abs(angular_velocity) > 0.001:  # avoid division by zero
            R = base_speed / angular_velocity
        else:
            R = float('inf')
        
        # Handle special cases
        if abs(base_speed) < 0.001 and R != float('inf'):  # Pivot turn
            # For pivot, just use differential speeds
            self.get_logger().info("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
            self.get_logger().info(f"STARTING PIVOT TURN at speed {angular_velocity}")
            self.get_logger().info("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
            turn_speed = angular_velocity * ROVER_WHEEL_RADIUS
            left_vel = -turn_speed
            right_vel = turn_speed
            return (left_vel, left_vel, left_vel, right_vel, right_vel, right_vel)
        
        if abs(R) > 1000:  # Essentially straight
            return (base_speed, base_speed, base_speed, base_speed, base_speed, base_speed)
        
        # Middle wheels (baseline)
        left_middle = base_speed * (R + W/2) / R
        right_middle = base_speed * (R - W/2) / R
        
        # Front/rear correction factor
        correction = math.sqrt(1 + (L/R)**2)
        
        left_front = left_middle * correction
        left_rear = left_middle * correction
        right_front = right_middle * correction
        right_rear = right_middle * correction
        
        return (left_front, left_middle, left_rear, 
                right_front, right_middle, right_rear)

    def update_odometry(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds * 1e-9
        self.last_time = current_time
        
        # Average all wheels on each side
        # Assuming you have: left_front_vel, left_middle_vel, left_rear_vel, etc.
        left_avg_vel = (self.lf + self.lm + self.lr) / 3.0
        right_avg_vel = (self.rf + self.rm + self.rr) / 3.0
        
        # Calculate velocities from wheel velocities
        linear_velocity = (left_avg_vel + right_avg_vel) * ROVER_WHEEL_RADIUS / 2.0
        angular_velocity = (right_avg_vel - left_avg_vel) * ROVER_WHEEL_RADIUS / TRACK_WIDTH
        
        # Update pose
        self.x_position += linear_velocity * math.cos(self.theta) * dt
        self.y_position += linear_velocity * math.sin(self.theta) * dt
        self.theta += angular_velocity * dt
        
        # Create and publish odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = 'odom_sim'
        odom_msg.child_frame_id = 'base_footprint'
        
        odom_msg.pose.pose.position.x = self.x_position
        odom_msg.pose.pose.position.y = self.y_position
        odom_msg.pose.pose.position.z = 0.0
        
        quat = quaternion_from_euler(0, 0, self.theta)
        odom_msg.pose.pose.orientation.x = quat[0]
        odom_msg.pose.pose.orientation.y = quat[1]
        odom_msg.pose.pose.orientation.z = quat[2]
        odom_msg.pose.pose.orientation.w = quat[3]
        
        # Set velocity in the odometry message
        odom_msg.twist.twist.linear.x = linear_velocity
        odom_msg.twist.twist.angular.z = angular_velocity
        
        self.odom_pub.publish(odom_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DifferentialDriveController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
