## To create a ROS node to process incoming messages from the remote control
"""To be tested"""
from udpcanpy import NetworkHandler, RemoteControl, RaspiState, ServoCalibState, NavArm, NavLocomotion, ScienceWeight #to access the UPDCAN protocol
import rclpy
from rclpy.node import Node
from raspistatechecker import RaspiStateChecker

from std_msgs.msg import Float64, Bool, Float64MultiArray
from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Odometry
import numpy as np

class StopWorthyException(Exception):
    "Raised for crucial failures"
    pass
# feel free to change if feels redundant
class RetryWorthyException(Exception):
    "Rasied for less critical failures"
    pass
class EmStop(Exception):
    "Exception raised by emergency stop"
    pass



class RemoteComms(Node):

    def __init__(self,protocol:str = "src/mdrs_build/mdrs_build/comms.dbc"): # string is placeholder to be replaced with actual path on device
        super().__init__("RemoteComms") #placeholder name for now
        """TO ADD: Logger initialisation and parameter declaration (if needed)"""

        ## Create subscriptions and publishers
        self.cmd_vel_pub = self.create_publisher(msg_type=Twist,topic="/cmd_vel",qos_profile=10)
        self.cmd_arm_motion_pub = self.create_publisher(msg_type=Twist,topic="/cmd_move_arm",qos_profile=10)
        self.cmd_servo_calib_pub = self.create_publisher(msg_type=Float64MultiArray,topic="/wheel_controller/servo_calib_write",qos_profile=10)
        self.cmd_servo_calib_sub = self.create_subscription(msg_type=Float64MultiArray,topic="/wheel_controller/servo_calib_read",callback=self.sendCalibState,qos_profile=10)

        self.odom_sub = self.create_subscription(Odometry, '/enc_odom', self.process_odom, 10)
        self.motor_telem_sub = self.create_subscription(Float64MultiArray, '/wheel_controller/commands_telem', self.process_motor, 10)
        self.arm_telem_sub = self.create_subscription(Float64MultiArray, '/arm_telem', self.process_arm, 10)
        #self.cmd_arm_grip_pub = self.create_publisher(msg_type=Bool,topic="/cmd_grip_arm",qos_profile=10)
        # self.odom_sub = self.create_subscription(msg_type=Odometry,topic="/odom",callback = self.rec_odom ,qos_profile=10) #probably will need many telemetry topics #create callback func for subscription
        """The queue size has been set to 10 for now, but it can be changed as necessary"""
        
        ## Next step is to create interface with comms protocol to get remote input and store it for the node to publish
        
        # first creating NetworkHandler object
        self.nh = NetworkHandler()
        # then reading the UDPCAN rules from the file
        self.res = self.nh.parse(protocol)
        if self.res != 0:
            self.get_logger().error(f"Failed to parse UDPCAN protocol, error code: {self.res}")
            raise StopWorthyException(f"UDPCAN Parsing error {self.res} - stopping rover")

        self.res = self.nh.init(0)
        if self.res != 0:
            if self.res == 1025:
                self.get_logger().error(f"Bind error - error code: {self.res}")
                raise StopWorthyException(f"Bind error {self.res}, please check if another process is using this port")
            self.get_logger().error(f"nh Failed to init, error code:{self.res}")
            raise RetryWorthyException(f"UDPCAN initiation error {self.res} - retrying")
            
        
        self.res = self.nh.start()
        if self.res != 0:
            self.get_logger().error(f"Failed to start thread, error code: {self.res}")
            raise RetryWorthyException(f"UDPCAN start error {self.res} - retrying")
        

        self.remote = self.nh.getRemoteControl() #This creates the higher order structure that contains the data we need to access - MessageWrapper equivalent, I think
        self.data = RemoteControl() #object to eventually store the message data when accessed

        # self.odom_wrap = self.nh.getNavOdometry() #need to check message definition with Dávid
        # self.odomess = NavOdometry()

        self.raspihandle = self.nh.getRaspiState()
        self.raspi = RaspiState()
        self.rsc = RaspiStateChecker()

        self.servo_calib_handle = self.nh.getServoCalibState()
        self.servo_calib = ServoCalibState()

        self.nav_arm_handle = self.nh.getNavArm()
        self.nav_arm = NavArm()

        self.nav_locomotion_handle = self.nh.getNavLocomotion()
        self.nav_locomotion = NavLocomotion()

        self.science_weight_handle = self.nh.getScienceWeight()
        self.science_weight = ScienceWeight()
        
        self.e_stop = False #creating emergency stop attribute so that we know when that's been pressed


        """Q to ask: When writing in ROS, does everything that would otherwise be a regular variable, now become an attribute? - yes, for now"""
        ## Toggle to switch between different modes
        self.arm_mode = False #rover mode by default

        self.prev_cmd = [] #for remote control rising edge
        self.prev_cmd_arm = [] #for remote control rising edge

        ##initialise rover control variables - follow Emma's keyboard controls py file as template for updating and packaging as Twist
        self.lin_speed = 0.0
        self.ang_speed = 0.0
        
        # values by which to increment the speeds when a button is pressed
        self.lin_inc = 0.5
        self.ang_inc = 0.2

        # setting max limits
        self.max_lin_speed = 3.0
        self.max_ang_speed = 5.0
        self.max_servo_lin = 0.01

        # toggle for arm mode
        self.prev_toggle = [None, None]

        ##initialise arm control variables
        #initial motion directions
        self.lin_x = 0.0 #forward/back
        self.lin_y = 0.0 #up/down
        self.base = 0.0 #gripper open/close
        self.end_grip = 0.0 #wrist rotation

        self.ThumbLX = 127.0
        self.ThumbLY = 127.0
        self.ThumbRX = 127.0
        self.ThumbRY = 127.0
        self.ThumbDeadZone = 10.0
        self.ThumbDeadZone_arm = 20.0
        self.ThumbCenter = 127.0

        # Timer to run remote input method repeatedly once the Node is initialised
        self.timer = self.create_timer(0.2,self.remote_input)
        self.tempTimer = self.create_timer(0.01,self.sendTelemetryTemporary)
        self.flushtimer = self.create_timer(0.5,self.flushall)

        self.pushed = {
            "raspi": False,
            "loco": False,
            "arm": False,
        }

    def flushClear(self):
        self.nh.flush()
        self.pushed = {
            "raspi": False,
            "loco": False,
            "arm": False,
        }

    def flushall(self):
        #self.nh.pushRaspiState()
        if not self.pushed["loco"]:
            self.nh.pushNavLocomotion()
        
        if not self.pushed["arm"]:
            self.nav_arm.arm_active = self.arm_mode
            self.nav_arm_handle.update(self.nav_arm)
            self.nh.pushNavArm()

        self.flushClear()

    def sendTelemetryTemporary(self):
        if self.rsc.poll(self.raspi):
            if self.pushed["raspi"]:
                self.flushClear()
            self.raspihandle.update(self.raspi)
            self.nh.pushRaspiState()
            self.pushed["raspi"] = True

    def sendCalibState(self, msg):
        calib = ServoCalibState()
        calib.fl_calib = int(msg.data[0])
        calib.fr_calib = int(msg.data[1])
        calib.rl_calib = int(msg.data[2])
        calib.rr_calib = int(msg.data[3])
        self.servo_calib_handle.update(calib)
        self.nh.pushServoCalibState()

    def process_odom(self, msg: Odometry):
        self.nav_locomotion.odometry = (msg.pose.pose.position.x ** 2 + msg.pose.pose.position.y ** 2) ** 0.5

    def process_motor(self, msg: Float64MultiArray):
        self.nav_locomotion.motor_fl_target = msg.data[0]
        self.nav_locomotion.motor_fr_target = msg.data[1]
        self.nav_locomotion.motor_ml_target = msg.data[2]
        self.nav_locomotion.motor_mr_target = msg.data[3]
        self.nav_locomotion.motor_rl_target = msg.data[4]
        self.nav_locomotion.motor_rr_target = msg.data[5]
        if self.pushed["loco"]:
            self.flushClear()
        self.nav_locomotion_handle.update(self.nav_locomotion)
        self.nh.pushNavLocomotion()
        self.pushed["loco"] = True

    def process_arm(self, msg: Float64MultiArray):
        self.nav_arm.arm_active = self.arm_mode
        self.nav_arm.joint_0 = msg.data[0]
        self.nav_arm.joint_1 = msg.data[1]
        self.nav_arm.joint_2 = msg.data[2]
        self.nav_arm.joint_3 = msg.data[3]
        if self.pushed["arm"]:
            self.flushClear()
        self.nav_arm_handle.update(self.nav_arm)
        self.nh.pushNavArm()
        self.pushed["arm"] = True

    def remote_input(self):
        self.res = self.remote.access(self.data) #accesses message within remote and puts it into the data object (RemoteControl object)
        if self.res >= 1024:
            self.get_logger().error(f"Could not access remote data, error code: {self.res}")
            raise RetryWorthyException(f"Remote message access error {self.res} - retrying")
        elif self.res == 0: #no error code
            ## main method to access the input message from the remote control and publish to topic
            if self.data.e_stop or (self.data.l_left and self.data.r_right):
                self.get_logger().info(f"Received estop")
                self.emergency_stop()
            elif not self.e_stop:
                newLThumbX = self.data.thumb_left_x if abs(self.data.thumb_left_x - self.ThumbCenter) > self.ThumbDeadZone else self.ThumbCenter
                newLThumbY = self.data.thumb_left_y if abs(self.data.thumb_left_y - self.ThumbCenter) > self.ThumbDeadZone else self.ThumbCenter
                lThumbChange = self.ThumbLX != newLThumbX or self.ThumbLY != newLThumbY

                newRThumbX = self.data.thumb_right_x if abs(self.data.thumb_right_x - self.ThumbCenter) > self.ThumbDeadZone else self.ThumbCenter
                newRThumbY = self.data.thumb_right_y if abs(self.data.thumb_right_y - self.ThumbCenter) > self.ThumbDeadZone else self.ThumbCenter
                rThumbChange = self.ThumbRX != newRThumbX or self.ThumbRY != newRThumbY
                lThumbChange = self.ThumbLX != newLThumbX or self.ThumbLY != newLThumbY

                self.e_stop = self.data.e_stop # bool # overwriting emergency stop variable with actual input
                self.LB = self.data.l_bottom # bool
                self.LT = self.data.l_top # bool
                self.LR = self.data.l_right # bool
                self.LL = self.data.l_left # bool
                self.RB = self.data.r_bottom # bool
                self.RT = self.data.r_top # bool
                self.RR = self.data.r_right # bool
                self.RL = self.data.r_left # bool
                self.L1 = self.data.l_shoulder # bool # L1 on PS4, LS/LB on Xbox
                self.R1 = self.data.r_shoulder # bool # R1 on PS4, RS/RB on Xbox
                self.L2 = self.data.left_trigger # int 0-255 # L2 on PS4, LT on Xbox
                self.R2 = self.data.right_trigger # int 0-255 # R2 on PS4, RT on Xbox
                self.ThumbLX = newLThumbX # int 0-255
                self.ThumbLY = newLThumbY # int 0-255
                self.ThumbRX = newRThumbX # int 0-255 
                self.ThumbRY = newRThumbY # int 0-255

                # self.get_logger().error(self)

                # self.get_logger().error(f"Arm mode? {self.arm_mode}")

                # Mode toggle
                if self.L1 and self.R1:
                    if [self.L1,self.R1] != self.prev_toggle:
                        self.arm_mode = not self.arm_mode
                        self.get_logger().info("Toggling arm mode to " + str(self.arm_mode))
                
                self.prev_toggle = [self.L1,self.R1]

                if not self.arm_mode:
                    if [self.LT,self.LB,self.LL,self.LR,self.RB] != self.prev_cmd or rThumbChange or lThumbChange:
                        self.get_logger().info(f"Received ROVER command")
                        # self.print_remote_data()
                        self.rover_command()
                        self.prev_cmd = [self.LT,self.LB,self.LL,self.LR,self.RB]
                elif self.arm_mode:
                    # if [self.LT,self.LB,self.LL,self.LR] != self.prev_cmd_arm or lThumbChange: # Maybe remove this later so that we can have continuous arm movement
                        # self.get_logger().info(f"Received ARM command")
                        # self.print_remote_data()
                    self.arm_command()
                    self.prev_cmd_arm = [self.LT,self.LB,self.LL,self.LR]

        self.res = self.servo_calib_handle.access(self.servo_calib)
        if self.res >= 1024:
            self.get_logger().error(f"Could not access servo_calib data, error code: {self.res}")
            raise RetryWorthyException(f"ServoCalibState message access error {self.res} - retrying")
        elif self.res == 0:
            calib_command = Float64MultiArray()
            calib_command.data = [
                float(self.servo_calib.fl_calib),
                float(self.servo_calib.fr_calib),
                float(self.servo_calib.rl_calib),
                float(self.servo_calib.rr_calib),
            ]
            self.cmd_servo_calib_pub.publish(calib_command)


    def __repr__(self):
        return (f"=================\n\
                    LB: {self.LB}\n\
                    LT: {self.LT}\n\
                    LR: {self.LR}\n\
                    LL: {self.LL}\n\
                    RB: {self.RB}\n\
                    RT: {self.RT}\n\
                    RR: {self.RR}\n\
                    RL: {self.RL}\n\
                    LS: {self.L1}\n\
                    RS: {self.R1}\n\
                    LTrigger:{self.L2}\n\
                    RTrigger:{self.R2}\n\
                    ThumbLX: {self.ThumbLX}\n\
                    ThumbLY: {self.ThumbLY}\n\
                    ThumbRX: {self.ThumbRX}\n\
                    ThumbRY: {self.ThumbRY}\n")
    
    def print_remote_data(self):
        self.get_logger().info(f"=================\n\
                    LB: {self.data.l_bottom}\n\
                    LT: {self.data.l_top}\n\
                    LR: {self.data.l_right}\n\
                    LL: {self.data.l_left}\n\
                    RB: {self.data.r_bottom}\n\
                    RT: {self.data.r_top}\n\
                    RR: {self.data.r_right}\n\
                    RL: {self.data.r_left}\n\
                    LS: {self.data.l_shoulder}\n\
                    RS: {self.data.r_shoulder}\n\
                    LTrigger: {self.data.left_trigger}\n\
                    RTrigger: {self.data.right_trigger}\n\
                    ThumbLX: {self.data.thumb_left_x}\n\
                    ThumbLY: {self.data.thumb_left_y}\n\
                    ThumbRX: {self.data.thumb_right_x}\n\
                    ThumbRY: {self.data.thumb_right_y}\n\
                        ")
                

    def thumb_curve(self, x, dead_zone):
        if x > self.ThumbCenter + dead_zone:
            return (1.3 ** ((x - self.ThumbCenter - dead_zone) / 10) - 1) / (1.3 ** ((self.ThumbCenter - dead_zone) / 10) - 1)
        elif x < self.ThumbCenter - dead_zone:
            return -(1.3 ** ((self.ThumbCenter - x - dead_zone) / 10) - 1) / (1.3 ** ((self.ThumbCenter - dead_zone) / 10) - 1)
        else:
            return 0.0

    def rover_command(self):
        ## method to update and publish velocity commands in rover mode
        # update velocities based on new inputs
        if self.LT:
            self.lin_speed += self.lin_inc
        elif self.LB:
            self.lin_speed -= self.lin_inc
        elif self.LL:
            self.ang_speed += self.ang_inc
        elif self.LR:
            self.ang_speed -= self.ang_inc
        else:
            self.ang_speed = self.thumb_curve(self.ThumbLX, self.ThumbDeadZone) * self.max_ang_speed
            self.lin_speed = -self.thumb_curve(self.ThumbLY, self.ThumbDeadZone) * self.max_lin_speed

        # Clamp the speeds to their maximum values
        self.lin_speed = max(min(self.lin_speed, self.max_lin_speed), -self.max_lin_speed)
        self.ang_speed = max(min(self.ang_speed, self.max_ang_speed), -self.max_ang_speed)

        #print to debug
        self.get_logger().info(f"speeds calculated to send: {self.lin_speed,self.ang_speed}")
        #print to debug
        self.get_logger().info(f"speeds being sent: {self.lin_speed,self.ang_speed}")
        if self.RB: #normal stop button - values reset to zero before creating and publishing Twist
            self.lin_speed = 0.0
            self.ang_speed = 0.0
        
        # Twist message to store and send the current command values
        rov_cmd = Twist()
        rov_cmd.linear.x = self.lin_speed
        rov_cmd.angular.z = self.ang_speed
        self.cmd_vel_pub.publish(rov_cmd)

    def arm_command(self):
        
        arm_cmd = Twist()

        self.lin_x = 0.0 #forward/back
        self.lin_y = 0.0 #up/down
        self.base = 0.0 #gripper open/close
        self.end_grip = 0.0 #wrist rotation

        self.lin_x = self.thumb_curve(self.ThumbLX, self.ThumbDeadZone_arm) * self.max_servo_lin
        self.lin_y = -self.thumb_curve(self.ThumbLY, self.ThumbDeadZone_arm) * self.max_servo_lin
        if self.LL: # arm base left
            self.base = 1.0
        elif self.LR: # arm base right
            self.base = -1.0
        if self.LB: # grip open
            self.end_grip = -1.0
        elif self.LT: # grip close
            self.end_grip = 1.0

        #print to debug
        if self.lin_x != 0.0 or self.lin_y != 0.0 or self.base != 0.0 or self.end_grip != 0.0:
            self.get_logger().info(f"arm commands to send: {self.lin_x, self.lin_y, self.base, self.end_grip}")

        arm_cmd.linear.x = self.lin_x
        arm_cmd.linear.y = self.lin_y
        arm_cmd.linear.z = self.end_grip
        arm_cmd.angular.x = self.base
        arm_cmd.angular.y = 0.0
        arm_cmd.angular.z = 0.0
        self.cmd_arm_motion_pub.publish(arm_cmd)
        return

    
    def emergency_stop(self, direct = True):
        # Send 0 velocities when emergency stopped
        twist = Twist()
        self.cmd_vel_pub.publish(twist)

        movearm = Twist()
        self.cmd_arm_motion_pub.publish(movearm)
        # end_cmd = Bool()
        # end_cmd.data = False
        # self.cmd_arm_grip_pub.publish(end_cmd)
        if direct:
            raise EmStop("EMERGENCY! Stopping...")
        
    # def send_telem(self):
    #     telem_data = self.telem_sub.read() #get telemetry data from subscriber
    # 
    #     self.telemessage.variable = telem_data
    #     self.telem_target.update(self.telemessage)
    #     self.nh.pushTelemMessage()
    #     self.nh.flush()

    #def rec_odom(self, odom_now):
    #    self.x_pos = odom_now.pose.pose.position.x
    #    self.y_pos = odom_now.pose.pose.position.y
    #    self.z_pos = odom_now.pose.pose.position.z
    #    self.orientation = odom_now.pose.pose.orientation
    #
    #    self.x_twist = odom_now.twist.twist.linear.x
    #    self.z_twist = odom_now.twist.twist.angular.z
    #
    #    self.send_odom()
    #will eventually combine these two methods, but keeping them separate for now for testing
    # def send_odom(self):
    #     self.odomess.distance = np.sqrt(self.x_pos**2 + self.y_pos**2 + self.z_pos**2)
    #     self.odomess.speed = self.x_twist
    #     self.odomess.joint_0 = self.orientation.x
    #     self.odomess.joint_1 = self.orientation.y
    #     self.odomess.joint_2 = self.orientation.z
    #     self.odomess.joint_3 = self.orientation.w
    #     self.odom_wrap.update(self.odomess)
    #     self.nh.pushNavOdometry()
    #     if self.rsc.poll(self.raspi):
    #         self.raspihandle.update(self.raspi)
    #         self.nh.pushRaspiState()
    #     self.nh.flush()

def main(args=None):
    rclpy.init(args=args)
    node = RemoteComms()

    while True:

        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        except EmStop:
            node.get_logger().error("Emergency stop triggered")
            
            break
        except StopWorthyException as e:
            node.emergency_stop(direct = False)
            node.get_logger().error(f"Stopped because of error {e}")
            break
        except RetryWorthyException as e:
            node.get_logger().error(f"Encountered error: {e}. Trying again")
            pass
                # node.emergency_stop()
    node.nh.stop()
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()
