import time
import RPi.GPIO as GPIO
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16MultiArray
from hx711 import HX711


class WeightPublisher(Node):
    def __init__(self):
        super().__init__('weight_publisher')
        self.publisher_ = self.create_publisher(UInt16MultiArray, 'weight', 10)
        self.timer = self.create_timer(1, self.timer_callback)  # 2 Hz
        
        # Setup HX711
        self.hx = HX711(16, 20)
        self.hx.set_reading_format("MSB", "MSB")
        self.hx.set_reference_unit(492) 
        self.hx.reset()
        self.hx.tare(200)
        
        self.get_logger().info("Scale ready! Place items to weigh...")
        self.get_logger().info("Publishing weight to /weight topic...")
    
    def timer_callback(self):
        weight = int(self.hx.get_weight(5))  # Average of 5 readings
        self.get_logger().info(f"Weight: {weight}g")
        
        # Publish to ROS topic as [weight_g, status_value]
        msg = UInt16MultiArray()
        if weight <= 511 and weight >= 0 :
            msg.data = [weight, 0]  # weight in grams (uint16), status value (0-511)
        elif weight > -1 and weight <=0 :
            msg.data = [weight + 1, 0]
        else :
            msg.data = [weight, 1]
        self.publisher_.publish(msg)
        
        self.hx.power_down()
        self.hx.power_up()


def main(args=None):
    rclpy.init(args=args)
    weight_publisher = WeightPublisher()
    
    try:
        rclpy.spin(weight_publisher)
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        GPIO.cleanup()
        weight_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()