import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from time import sleep
from subprocess import run
import re
import socket as s


class UdpCamera(Node):
    def __init__(self):
        super().__init__('udp_camera_node')
        self.publisher_ = self.create_publisher(Image, 'camera/image_raw', 10)
        self.bridge = CvBridge()
        # add commandline argument to specify local IP or not
        # Connect to the stream coming from the Host
        # Change your VideoCapture line to this:
        _, _, _, wlan0_ips, _ = self.getIntf()
        self.ip = wlan0_ips
        self.address = f"http://{self.ip[0]}:13000/stream.mjpg"         
        self.cap = cv2.VideoCapture(self.address, cv2.CAP_FFMPEG)

        if not self.cap.isOpened():
            self.get_logger().error("Could not open video stream")

        self.timer = self.create_timer(0.033, self.timer_callback) # 30 FPS

    def timer_callback(self):
            ret, frame = self.cap.read()
            
            if ret:
                # Success - Publish the image
                # self.get_logger().info("Received frame")

                msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = "camera_link"
                self.publisher_.publish(msg)
            else:
                # Failure - Try to reconnect instead of giving up
                self.get_logger().warn("Frame lost, attempting to reconnect...")
                self.cap.release()
                self.cap = cv2.VideoCapture(self.address, cv2.CAP_FFMPEG)

    def getIntf(self):
        output = run(["ip", "a"], capture_output=True, text=True).stdout
        
        tmp = re.findall(r"[0-9]: [a-z0-9]*:", output)
        allifs = []
        for t in tmp:
            if t.__contains__("eth0") or t.__contains__("wlan0"):
                pass
            else:
                allifs.append(t)

        lines = output.split("\n")

        lines_if_eth0 = []
        is_eth0 = False
        lines_if_wlan0 = []
        is_wlan0 = False

        for line in lines:

            isifdec = False
            for ifn in allifs:
                if line.__contains__(ifn):
                    isifdec = True
                    break

            if line.__contains__("eth0"):
                is_eth0 = True
                is_wlan0 = False
            elif line.__contains__("wlan0"):
                is_wlan0 = True
                is_eth0 = False
            elif isifdec:
                is_wlan0 = False
                is_eth0 = False

            if is_eth0:
                lines_if_eth0.append(line)
            
            if is_wlan0:
                lines_if_wlan0.append(line)

        wlan0_ips = []
        wlan0_up = lines_if_wlan0[0].__contains__("state UP")
        for line in lines_if_wlan0:
            if line.__contains__("inet "):
                wlan0_ips.append(re.findall(r"[0-9]{1,3}.[0-9]{1,3}.[0-9]{1,3}.[0-9]{1,3}\/[0-9]{1,2}", line)[0].split('/')[0])

        eth0_ips = []
        eth0_up = lines_if_eth0[0].__contains__("state UP")
        for line in lines_if_eth0:
            if line.__contains__("inet "):
                eth0_ips.append(re.findall(r"[0-9]{1,3}.[0-9]{1,3}.[0-9]{1,3}.[0-9]{1,3}\/[0-9]{1,2}", line)[0].split('/')[0])

        msg = ""
        if eth0_up:
            msg += "The interface eth0 is up\n"
            for ip in eth0_ips:
                msg += f"IP for eth0: {ip}\n"
        else:
            msg += "The interface eth0 is not up\n"
            
        if wlan0_up:
            msg += "The interface wlan0 is up\n"
            for ip in wlan0_ips:
                msg += f"IP for wlan0: {ip}\n"
        else:
            msg += "The interface wlan0 is not up\n"

        print(msg)

        return eth0_up, eth0_ips, wlan0_up, wlan0_ips, msg


def main(args=None):
    rclpy.init(args=args)
    node = UdpCamera()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()