#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class StereoCamera(Node):
    def __init__(self):
        super().__init__('stereo_camera')
        
        # Parameters
        self.declare_parameter('video_device', 0)
        self.declare_parameter('image_width', 2560)
        self.declare_parameter('image_height', 720)
        self.declare_parameter('framerate', 30)
        
        self.device_id = self.get_parameter('video_device').value
        self.width = self.get_parameter('image_width').value
        self.height = self.get_parameter('image_height').value
        self.framerate = self.get_parameter('framerate').value
        
        # Publishers
        self.pub_left = self.create_publisher(Image, '/camera/left/image_raw', 10)
        self.pub_right = self.create_publisher(Image, '/camera/right/image_raw', 10)
        
        # Open Camera
        self.get_logger().info(f"Opening video device {self.device_id}...")
        self.cap = cv2.VideoCapture(self.device_id)
        
        if not self.cap.isOpened():
             self.get_logger().error(f"Could not open video device {self.device_id}")
        
        # Set Resolution / FPS
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.framerate)
        
        # Check actual resolution
        actual_w = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        actual_h = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        self.get_logger().info(f"Camera opened: {actual_w}x{actual_h} at {self.cap.get(cv2.CAP_PROP_FPS)} FPS")

        self.bridge = CvBridge()
        
        # Timer
        self.timer = self.create_timer(1.0 / self.framerate, self.timer_callback)

    def timer_callback(self):
        if not self.cap.isOpened():
            return

        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Frame capture failed")
            return
            
        h, w, _ = frame.shape
        
        # Split Side-by-Side - assume input is combined stereo
        width_half = w // 2
        left_img = frame[0:h, 0:width_half]
        right_img = frame[0:h, width_half:w]
        
        # Convert to ROS Msg
        timestamp = self.get_clock().now().to_msg()
        
        if left_img.size > 0:
            msg_l = self.bridge.cv2_to_imgmsg(left_img, encoding='bgr8')
            msg_l.header.stamp = timestamp
            msg_l.header.frame_id = 'camera_left'
            self.pub_left.publish(msg_l)
        
        if right_img.size > 0:
            msg_r = self.bridge.cv2_to_imgmsg(right_img, encoding='bgr8')
            msg_r.header.stamp = timestamp
            msg_r.header.frame_id = 'camera_right'
            self.pub_right.publish(msg_r)

def main(args=None):
    rclpy.init(args=args)
    node = StereoCamera()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
