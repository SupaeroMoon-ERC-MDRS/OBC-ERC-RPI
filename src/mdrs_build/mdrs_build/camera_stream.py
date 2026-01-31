import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class UdpCamera(Node):
    def __init__(self):
        super().__init__('udp_camera_node')
        self.publisher_ = self.create_publisher(Image, 'camera/image_raw', 10)
        self.bridge = CvBridge()
        # add commandline argument to specify local IP or not
        # Connect to the stream coming from the Host
        # Change your VideoCapture line to this:
        self.address = f"http://ip:13000/stream.mjpg"
        self.cap = cv2.VideoCapture(self.address, cv2.CAP_FFMPEG)

        if not self.cap.isOpened():
            self.get_logger().error("Could not open video stream")

        self.timer = self.create_timer(0.033, self.timer_callback) # 30 FPS

    def timer_callback(self):
            ret, frame = self.cap.read()
            
            if ret:
                # Success - Publish the image
                self.get_logger().info("Received frame")

                msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = "camera_link"
                self.publisher_.publish(msg)
            else:
                # Failure - Try to reconnect instead of giving up
                self.get_logger().warn("Frame lost, attempting to reconnect...")
                self.cap.release()
                self.cap = cv2.VideoCapture(self.address, cv2.CAP_FFMPEG)


def main(args=None):
    rclpy.init(args=args)
    node = UdpCamera()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()