#basic telemetry node with examples for Odometry from encoder_node

from udpcanpy import NetworkHandler, RemoteControl, NavOdometry #to access the UPDCAN protocol
import rclpy
from rclpy.node import Node
import numpy as np

from std_msgs.msg import Float64, Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Quaternion

class TelemNode(Node):

    def __init__(self,protocol:str = "src/sim_mdrs/sim_mdrs/comms.dbc"): # string is placeholder to be replaced with actual path on device

         ## Create subscriptions for telemetry
        self.telem_sub = self.create_subscription(msg_type=Float64,topic="/telemetry",callback = self.rec_telem ,qos_profile=10) #might need multiple telemetry topics
        self.odom_sub = self.create_subscription(msg_type=Odometry,topic="/odom",callback = self.rec_odom ,qos_profile=10)

        self.nh = NetworkHandler()

        self.res = self.nh.parse(protocol)

        if self.res != 0:
            self.get_logger().error(f"Failed to parse UDPCAN protocol, error code: {self.res}")
            #raise StopWorthyException(f"UDPCAN Parsing error {self.res} - stopping rover")

        self.res = self.nh.init()
    
        if self.res != 0:
            if self.res == 1025:
                self.get_logger().error(f"Bind error - error code: {self.res}")
                #raise StopWorthyException(f"Bind error {self.res}, please check if another process is using this port")
            self.get_logger().error(f"nh Failed to init, error code:{self.res}")
            #raise RetryWorthyException(f"UDPCAN initiation error {self.res} - retrying")

        self.res = self.nh.start()
        if self.res != 0:
            self.get_logger().error(f"Failed to start thread, error code: {self.res}")
            raise RetryWorthyException(f"UDPCAN start error {self.res} - retrying")
        
        self.telem_target = self.nh.getTelemWrap() #higher order structure for telem messages
        self.telemessage = TelemMessage() #telem message object - still needs to be defined - might need several - can write to this object

        ## sample implementation with encoder odometry
        self.odom_wrap = self.nh.getOdometry() #need to check message definition with Dávid
        self.odomess = NavOdometry()

    def rec_telem(self):
        self.telem_data = self.telem_sub.read() #get telemetry data from subscriber

    def rec_odom(self):
        odom_now = self.odom_sub.read()
        self.x_pos = odom_now.pose.pose.position.x
        self.y_pos = odom_now.pose.pose.position.y
        self.z_pos = odom_now.pose.pose.position.z
        self.orientation = odom_now.pose.pose.orientation

        self.x_twist = odom_now.twist.twist.linear.x
        self.z_twist = odom_now.twist.twist.angular.z

        self.send_odom()


    def send_telem(self):
        #telem_data = self.telem_sub.read() #this is now done in telemetry callback function

        self.telemessage.variable = self.telem_data
        self.telem_target.update(self.telemessage)
        self.nh.pushTelemMessage()
        self.nh.flush() #ask David for condition 

    def send_odom(self):
        self.odomess.distance = np.sqrt(self.x_pos**2 + self.y_pos**2 + self.z_pos**2)
        self.odomess.speed = self.x_twist

        self.odomess.joint_0 = self.orientation.x
        self.odomess.joint_1 = self.orientation.y
        self.odomess.joint_2 = self.orientation.z
        self.odomess.joint_3 = self.orientation.w

        self.odom_wrap.update(self.odomess)
        self.nh.pushNavOdometry()
        self.nh.flush()

def main(args=None):
    rclpy.init(args=args)
    node = TelemNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()