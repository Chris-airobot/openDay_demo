#!/usr/bin/env python3

import rospy
import tf2_ros
import geometry_msgs.msg
import tf.transformations as tf_trans
import numpy as np
import sys

def publish_transform():
    rospy.init_node('camera_to_robot_tf_broadcaster')

    # # Retrieve parameters from the launch file
    # input_file = rospy.get_param('~input_file')
    # parent_frame = rospy.get_param('~parent_frame')
    # child_frame = rospy.get_param('~child_frame')

    br = tf2_ros.StaticTransformBroadcaster()

    # ----- 1) define your two known static transforms -----
    # T_base->aruco
    t1 = np.array([0.21, 0.0, -0.04])
    q1 = np.array([0.0, 0.0, 0.7071, 0.7071])  # [x, y, z, w]

    # T_camera->aruco (as in your second static_publisher)
    t2 = np.array([0.6150792305377963, 0.02226115583018931, 0.08599622850224437])
    q2 = np.array([0.4047853186694866, -0.39513742850322936, -0.5814433363751594, 0.5847554232073617])

    # ----- 2) build 4×4 matrices -----
    M1 = tf_trans.quaternion_matrix(q1)
    M1[:3, 3] = t1

    M2 = tf_trans.quaternion_matrix(q2)
    M2[:3, 3] = t2

    # ----- 3) invert M2 to get T_aruco->camera -----
    R2 = M2[:3, :3]
    t2_inv = -R2.T.dot(t2)
    M2_inv = np.eye(4)
    M2_inv[:3, :3] = R2.T
    M2_inv[:3, 3] = t2_inv

    # ----- 4) compose: T_base->camera = T_base->aruco · T_aruco->camera -----
    M_bc = M1.dot(M2_inv)

    # extract translation + quaternion
    trans_bc = M_bc[:3, 3]
    quat_bc = tf_trans.quaternion_from_matrix(M_bc)

    # ----- 5) publish as a static transform -----
    st = geometry_msgs.msg.TransformStamped()
    st.header.stamp = rospy.Time.now()
    st.header.frame_id = 'base_link'
    st.child_frame_id = 'camera_link'
    st.transform.translation.x = trans_bc[0]
    st.transform.translation.y = trans_bc[1]
    st.transform.translation.z = trans_bc[2]
    st.transform.rotation.x = quat_bc[0]
    st.transform.rotation.y = quat_bc[1]
    st.transform.rotation.z = quat_bc[2]
    st.transform.rotation.w = quat_bc[3]

    br.sendTransform(st)
    rospy.loginfo("Published static transform base_link → camera_link")
    rospy.spin()

if __name__ == '__main__':
    try:
        publish_transform()
    except rospy.ROSInterruptException:
        pass
