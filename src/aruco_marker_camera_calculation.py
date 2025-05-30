
# marker_diff
x = 0.21
y = 0
z = -0.04

import rospy
import cv2
import cv2.aruco as aruco
import numpy as np
import pyrealsense2 as rs
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
import tf2_ros
import tf.transformations
SAVE_PATH = "/home/riot/kinova_gen3_lite/src/openDay_demo/config/camera_pose.txt"


# Aruco dictionary and parameters
ARUCO_DICT = aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
ARUCO_PARAMS = aruco.DetectorParameters()


rospy.init_node('aruco_marker_detector')
# Initialize OpenCV bridge
bridge = CvBridge()

# Set up a tf broadcaster
tf_broadcaster = tf2_ros.TransformBroadcaster()
static_broadcaster = tf2_ros.StaticTransformBroadcaster()   # <-- added


message = rospy.wait_for_message("/camera/color/camera_info", CameraInfo)
camera_matrix = np.array(message.K).reshape((3, 3))
dist_coeffs = np.array(message.D)




# Function to process each frame, 
########### This is camera_link to marker
def process_frame(msg):
    frame = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect Aruco markers
    corners, ids, rejected = aruco.detectMarkers(gray, ARUCO_DICT, parameters=ARUCO_PARAMS)

    if ids is not None:
        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, 0.05, camera_matrix, dist_coeffs)

        for i in range(len(ids)):
            aruco.drawDetectedMarkers(frame, corners)     
            cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], 0.05)

            # Broadcast the transformation
            trans = TransformStamped()
            
            trans.header.stamp = rospy.Time.now()
            trans.header.frame_id = "camera_link"
            trans.child_frame_id = f"aruco_marker_3"

            # Convert rotation vector to quaternion
            rot_matrix = cv2.Rodrigues(rvecs[i])[0]
            quat = tf.transformations.quaternion_from_matrix(np.vstack((np.hstack((rot_matrix, np.zeros((3, 1)))), [0, 0, 0, 1])))

            tx, ty, tz = tvecs[i][0]
            trans.transform.translation.x = tx
            trans.transform.translation.y = ty
            trans.transform.translation.z = tz
            trans.transform.rotation.x = quat[0]
            trans.transform.rotation.y = quat[1]
            trans.transform.rotation.z = quat[2]
            trans.transform.rotation.w = quat[3]
            print(trans)
            tf_broadcaster.sendTransform(trans)
            
            # **ADDED**: publish base_link → aruco_marker_frame
            base_tf = TransformStamped()
            base_tf.header.stamp    = rospy.Time.now()
            base_tf.header.frame_id = "base_link"
            base_tf.child_frame_id  = "aruco_marker_3"
            base_tf.transform.translation.x = x
            base_tf.transform.translation.y = y
            base_tf.transform.translation.z = z
            base_tf.transform.rotation.x = 0.0
            base_tf.transform.rotation.y = 0.0
            base_tf.transform.rotation.z = 0.0
            base_tf.transform.rotation.w = 1.0
            static_broadcaster.sendTransform(base_tf)
            
            # ----- **ADDED: save the same 4×4 matrix to file** -----
            T = np.eye(4)
            T[:3, :3] = rot_matrix
            T[0, 3] = tx
            T[1, 3] = ty
            T[2, 3] = tz
            np.savetxt(SAVE_PATH, T, fmt="%.10f")
            rospy.loginfo(f"Saved camera_link to aruco_marker_3 transform to {SAVE_PATH}")

    cv2.imshow('Aruco Marker Detection', frame)
    cv2.waitKey(1)

# Subscribe to the RealSense camera topic
image_sub = rospy.Subscriber('/camera/color/image_raw', Image, process_frame)

# Spin to keep the script running
rospy.spin()