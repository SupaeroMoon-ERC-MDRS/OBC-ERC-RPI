import rclpy
from rclpy.node import Node

import cv2
import cv2.aruco as aruco
import numpy as np
import yaml

from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge

from tf_transformations import quaternion_from_matrix


def my_estimatePoseSingleMarkers(corners, marker_size, mtx, distortion):
    """
    Re-implementation of estimatePoseSingleMarkers using SOLVEPNP_IPPE_SQUARE.
    """
    marker_points = np.array(
        [
            [-marker_size / 2,  marker_size / 2, 0],
            [ marker_size / 2,  marker_size / 2, 0],
            [ marker_size / 2, -marker_size / 2, 0],
            [-marker_size / 2, -marker_size / 2, 0],
        ],
        dtype=np.float32,
    )

    rvecs = []
    tvecs = []
    trash = []

    for c in corners:
        nada, rvec, tvec = cv2.solvePnP(
            marker_points,
            c,
            mtx,
            distortion,
            useExtrinsicGuess=False,
            flags=cv2.SOLVEPNP_IPPE_SQUARE,
        )
        rvecs.append(rvec)
        tvecs.append(tvec)
        trash.append(nada)

    return rvecs, tvecs, trash


class ArucoPosePublisher(Node):
    def __init__(self):
        super().__init__('aruco_pose_publisher')

        # --- Parameters (you can also declare and set via YAML/params) ---
        # Marker side length in meters (15 cm example)
        self.marker_length = self.declare_parameter('marker_length', 0.15).value

        # need to be given the camera calib file, for matrix and distortion coefficients
        calib_file = self.declare_parameter('camera_calib_file', 'camera_calib.yaml').value
       
        self.image_topic = self.declare_parameter('image_topic', '/camera/image_raw').value
        self.camera_frame = self.declare_parameter('camera_frame', 'camera_link').value

        # TARGET marker ID (e.g. 23)
        self.target_marker_id = self.declare_parameter('target_marker_id', 23).value
        
        # MUST SET TARGET ID in Launch File or CLI
        #: ros2 run your_pkg aruco_pose_publisher --ros-args -p target_marker_id:=42

        # --- Load camera calibration ---
        with open(calib_file) as f:
            cam_calib_dist = yaml.load(f, Loader=yaml.Loader)
            
        self.mtx = np.array(cam_calib_dist['mtx'])
        self.dist = np.array(cam_calib_dist['dist'])

        self.calib_width = 4608
        self.calib_height = 2592

        # --- ArUco setup ---
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_100)
        self.parameters = aruco.DetectorParameters_create()
        # self.detector = aruco.ArucoDetector(self.aruco_dict, self.parameters)

        # --- ROS interfaces ---
        self.bridge = CvBridge()
        self.sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10
        )
        self.pose_pub = self.create_publisher(PoseStamped, '/aruco_marker_pose', 10)

        self.get_logger().info(f'ArucoPosePublisher started. Subscribing to {self.image_topic}')

    def image_callback(self, msg: Image):
        # Convert ROS Image to OpenCV
        try:
            raw_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'CvBridge error: {e}')
            return
        
        frame = cv2.rotate(raw_frame, cv2.ROTATE_180)

        h_stream, w_stream = frame.shape[:2]
        scale_x = w_stream / self.calib_width
        scale_y = h_stream / self.calib_height

        current_mtx = np.copy(self.mtx)
        
        # Scale Focal Lengths (fx, fy)
        current_mtx[0, 0] = 3846  # fx
        current_mtx[1, 1] = 3846  # fy
        
        # Scale & Flip Optical Center (cx, cy)
        # Because we rotated 180, the "center" pixel moves to the other side
        # New Center = Width - (Old_Center * Scale)
        current_mtx[0, 2] = w_stream /2
        current_mtx[1, 2] = h_stream /2

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect markers
        # corners, ids, rejected = self.detector.detectMarkers(gray)
        corners, ids, rejected = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)


        if ids is None or len(ids) == 0:
            return  # nothing to publish
        # else:
        #     self.get_logger().info(f'Marker detected. Id: {ids[0]}')

        # ids is typically shape (N, 1), so flatten it
        ids_flat = ids.flatten()

        # Find indices where detected ID == target_marker_id
        matches = np.where(ids_flat == self.target_marker_id)[0]

        if len(matches) == 0:
            # Target marker not in this frame; do nothing
            return

        # If multiple instances of same ID exist, just take the first match
        idx = int(matches[0])

        # Estimate pose *only* for the matched marker
        # (you can still pass all corners but just take idx after)
        rvecs, tvecs, _ = my_estimatePoseSingleMarkers(
            corners,
            self.marker_length,
            current_mtx,
            self.dist
        )

        rvec = rvecs[idx]
        tvec = tvecs[idx]

        # Remap translation, flip the y axis, original: x, -y, z
        # tvec is shape (3,1) or (3,)
        tvec_flat = tvec.flatten()  # ensure 1D

        # Map camera coordinates to ROS coordinates (X forward, Y left, Z up)
        ros_x = float(tvec_flat[2])      # forward
        ros_y = float(-tvec_flat[0])     # right -> left
        ros_z = float(-tvec_flat[1])     # down -> up

        tvec_remap = np.vstack((ros_x, ros_y, ros_z))

        # Rotation matrix from rvec
        R, _ = cv2.Rodrigues(rvec)

        # 4x4 transform matrix
        Tint = np.hstack((R, tvec_remap))
        T = np.vstack((Tint, np.array([0, 0, 0, 1], dtype=float)))

        # Quaternion from rotation matrix
        qx, qy, qz, qw = quaternion_from_matrix(T)

        # Build PoseStamped in camera frame
        pose_msg = PoseStamped()
        pose_msg.header.stamp = msg.header.stamp
        pose_msg.header.frame_id = self.camera_frame  # must match your camera TF frame

        pose_msg.pose.position.x = ros_x
        pose_msg.pose.position.y = ros_y
        pose_msg.pose.position.z = ros_z

        pose_msg.pose.orientation.x = float(qx)
        pose_msg.pose.orientation.y = float(qy)
        pose_msg.pose.orientation.z = float(qz)
        pose_msg.pose.orientation.w = float(qw)

        self.pose_pub.publish(pose_msg)

        # Optional: debug print
        self.get_logger().info(
            f'Marker {int(ids[idx])} pose in {self.camera_frame}: '
            f'pos=({pose_msg.pose.position.x:.2f}, '
            f'{pose_msg.pose.position.y:.2f}, '
            f'{pose_msg.pose.position.z:.2f})'
        )


def main(args=None):
    rclpy.init(args=args)
    node = ArucoPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
