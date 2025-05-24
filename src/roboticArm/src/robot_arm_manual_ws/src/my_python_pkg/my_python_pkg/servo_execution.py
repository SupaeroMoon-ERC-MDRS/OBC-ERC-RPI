#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point
from std_msgs.msg import Float64MultiArray
import math
import time

# import time, board, busio, adafruit_ina260
# from adafruit_pca9685 import PCA9685
# from adafruit_motor import servo

# i2c = busio.I2C(board.SCL, board.SDA)
# pca = PCA9685(i2c, address = 0x43)
# ina260 = adafruit_ina260.INA260(i2c, address=0x40)

# pca.frequency = 50
# servo0 = servo.Servo(pca.channels[0]) #Base - First segment
# servo1 = servo.Servo(pca.channels[1]) #First segment- Second segment
# servo2 = servo.Servo(pca.channels[2]) #Second segment- Third segment
# servo3 = servo.Servo(pca.channels[3]) #Third segment- Fourth segment
# servo4 = servo.Servo(pca.channels[3]) #End effector rotation
# servo5 = servo.Servo(pca.channels[3]) #Gripper

# while True:
# 	print(ina260.voltage)
# 	servo0.angle = 0
# 	servo1.angle = 0
# 	#servo2.angle = 0
# 	#servo3.angle = 0
# 	time.sleep(0.5)
# 	servo0.angle = 180
# 	servo1.angle = 180
# 	#servo2.angle = 180
# 	#servo3.angle = 180
# 	time.sleep(0.5)

class Servo_execution(Node):
    def __init__(self):
        super().__init__('servos_execution')
        print("Servo node started !")
        self.subscription = self.create_subscription(JointState, '/servos_execution_commands', self.listener_callback, 10)
        self.subscription
        
    def listener_callback(self, msg):
        print("--- NEW COMMAND RECEIVED ---")

        print(msg)

        print("--- RADIANS ---")
        servo0_position = msg.position[0]
        print(servo0_position)
        servo1_position = msg.position[1]
        print(servo1_position)
        servo2_position = msg.position[2]
        print(servo2_position)
        servo3_position = msg.position[3]
        print(servo3_position)
        servo4_position = msg.position[4]
        print(servo4_position)

        print("--- DEGREE ---")
        servo0_position = math.degrees(msg.position[0])
        print(servo0_position)
        servo1_position = math.degrees(msg.position[1])
        print(servo1_position)
        servo2_position = math.degrees(msg.position[2])
        print(servo2_position)
        servo3_position = math.degrees(msg.position[3])
        print(servo3_position)
        servo4_position = math.degrees(msg.position[4])
        print(servo4_position)

        print("--- END NEW COMMAND ---")

        print("Voltage:")
        # print(ina260.voltage)
        # servo0.angle = servo0_position
        # servo1.angle = servo1_position
        # servo2.angle = servo2_position
        # servo3.angle = servo3_position
        # servo4.angle = servo4_position
        time.sleep(0.5)

def main(args=None):
    rclpy.init(args=args)
    node = Servo_execution()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()