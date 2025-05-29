#!/usr/bin/env python3
import rospy
import tf2_ros
import geometry_msgs.msg
import tf.transformations as tf_trans
import numpy as np
import sys

def publish_transform():
    rospy.init_node('camera_to_robot_tf_broadcaster')

    br = tf2_ros.StaticTransformBroadcaster()


    transform = geometry_msgs.msg.TransformStamped()
    transform.header.stamp = rospy.Time.now()
    transform.header.frame_id =  "base_link"
    transform.child_frame_id = "aruco_marker_3"

    # Extract translation
    transform.transform.translation.x = 0.2 
    transform.transform.translation.y = 0
    transform.transform.translation.z = 0.04

    # Extract rotation matrix
    z_rotate_deg = 180
    z_rotate_rad = np.deg2rad(z_rotate_deg)

    # Create rotation matrix for the Z-axis rotation
    rot_z = tf_trans.rotation_matrix(z_rotate_rad, (0, 0, 1))

    # Convert the rotation matrix to a quaternion
    quat = tf_trans.quaternion_from_matrix(rot_z)

    transform.transform.rotation.x = quat[0]
    transform.transform.rotation.y = quat[1]
    transform.transform.rotation.z = quat[2]
    transform.transform.rotation.w = quat[3]

    br.sendTransform(transform)

    rospy.spin()

if __name__ == '__main__':
    try:
        publish_transform()
    except rospy.ROSInterruptException:
        pass
