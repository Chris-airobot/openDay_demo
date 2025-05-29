
# marker_diff
x = 0.2
y = 0
z = 0.04

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

# Aruco dictionary and parameters
ARUCO_DICT = aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
ARUCO_PARAMS = aruco.DetectorParameters()


rospy.init_node('aruco_marker_detector')

# Set up a tf broadcaster
tf_broadcaster = tf2_ros.TransformBroadcaster()

message = rospy.wait_for_message("/camera/color/camera_info", CameraInfo)
camera_matrix = np.array(message.K).reshape((3, 3))
dist_coeffs = np.array(message.D)

# Initialize OpenCV bridge
bridge = CvBridge()



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

            trans.transform.translation.x = tvecs[i][0][0]
            trans.transform.translation.y = tvecs[i][0][1]
            trans.transform.translation.z = tvecs[i][0][2]
            trans.transform.rotation.x = quat[0]
            trans.transform.rotation.y = quat[1]
            trans.transform.rotation.z = quat[2]
            trans.transform.rotation.w = quat[3]
            print(trans)
            tf_broadcaster.sendTransform(trans)

    cv2.imshow('Aruco Marker Detection', frame)
    cv2.waitKey(1)

# Subscribe to the RealSense camera topic
image_sub = rospy.Subscriber('/camera/color/image_raw', Image, process_frame)

# Spin to keep the script running
rospy.spin()