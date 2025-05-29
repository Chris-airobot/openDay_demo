#!/usr/bin/env python3
from typing import Dict, List
import pandas as pd
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped, Point, Quaternion, Pose
from scipy.spatial.transform import Rotation as R
import os
from tf import TransformListener
from pydantic import BaseModel, Field
from tf2_geometry_msgs import PoseStamped
import numpy as np
from tf.transformations import euler_from_quaternion
from scipy.spatial.transform import Rotation as R
import open3d as o3d



# Get loc of package on computer
ROOT_PATH = os.path.dirname(__file__)

def pose_difference(p_current, p_desired):
    """Inputs are supposed to be in a list of 7 elements, first 3 
    are translational elements, last 4 are quaternion elements

    Args:
        p_current: current pose of the end_effector
        p_desired: desired pose of the end_effector
    """
    
    delta_translation = np.array(p_desired[:3]) - np.array(p_current[:3])
    
    # Rotation Difference calculation
    current_rotation = R.from_euler('xyz', p_current[3:]).as_quat()
    desired_rotation = R.from_euler('xyz', p_desired[3:]).as_quat()
    
    
    delta_rotation = R.from_quat(desired_rotation) * R.from_quat(current_rotation).inv()
    delta_rotation= delta_rotation.as_euler('xyz')
    
    
    # delta_angles = euler_from_quaternion(delta_rotation_quat)
    # rotation_angles = [0, 0, 0]
    
    # for i in [0,1,2]:
    #     rotation_angles[i] = delta_angles[i]*180/np.pi

    return np.concatenate((delta_translation, delta_rotation), axis= None)


def get_pointcloud(color_img, depth_img, camera_intrinsics):

    # Get depth image size
    im_h = depth_img.shape[0]
    im_w = depth_img.shape[1]
    
    # Project depth into 3D point cloud in camera coordinates
    pix_x,pix_y = np.meshgrid(np.linspace(0,im_w-1,im_w), np.linspace(0,im_h-1,im_h))
    cam_pts_x = np.multiply(pix_x-camera_intrinsics[0][2],depth_img/camera_intrinsics[0][0])
    cam_pts_y = np.multiply(pix_y-camera_intrinsics[1][2],depth_img/camera_intrinsics[1][1])
    cam_pts_z = depth_img.copy()
    cam_pts_x.shape = (im_h*im_w,1)
    cam_pts_y.shape = (im_h*im_w,1)
    cam_pts_z.shape = (im_h*im_w,1)

    # Reshape image into colors for 3D point cloud
    rgb_pts_r = color_img[:,:,0]
    rgb_pts_g = color_img[:,:,1]
    rgb_pts_b = color_img[:,:,2]
    rgb_pts_r.shape = (im_h*im_w,1)
    rgb_pts_g.shape = (im_h*im_w,1)
    rgb_pts_b.shape = (im_h*im_w,1)

    cam_pts = np.concatenate((cam_pts_x, cam_pts_y, cam_pts_z), axis=1)
    rgb_pts = np.concatenate((rgb_pts_r, rgb_pts_g, rgb_pts_b), axis=1)

    return cam_pts, rgb_pts


def extract_pointcloud(color_img_1,depth_img_1, color_img_2,depth_img_2, intrinsics_1, intrinsics_2):
    
    cam_pts, rgb_pts = get_pointcloud(color_img_1,depth_img_1, intrinsics_1)
    # Create Open3D PointCloud object
    left_pcd = o3d.geometry.PointCloud()
    left_pcd.points = o3d.utility.Vector3dVector(cam_pts)
    left_pcd.colors = o3d.utility.Vector3dVector(rgb_pts / 255.0)  # Normalize colors to [0, 1]

    # o3d.io.write_point_cloud("/home/riot/kinova_gen3_lite/src/first_demo/pcds/left_output.ply", pcd)
    
    cam_pts, rgb_pts = get_pointcloud(color_img_2,depth_img_2, intrinsics_2)
    # Create Open3D PointCloud object
    right_pcd = o3d.geometry.PointCloud()
    right_pcd.points = o3d.utility.Vector3dVector(cam_pts)
    right_pcd.colors = o3d.utility.Vector3dVector(rgb_pts / 255.0)  # Normalize colors to [0, 1]

    # o3d.io.write_point_cloud("/home/riot/kinova_gen3_lite/src/first_demo/pcds/right_output.ply", pcd)
    
    return left_pcd, right_pcd


def init_tf_tree() -> TransformListener:
    """Launch the tf2 transform tree to ensure transforms are available for use"""
    tf_listener = TransformListener()

    transforms_available = False
    while not transforms_available:
        try:
            tf_listener.waitForTransform(
                "/base_link", "/base_link", rospy.Time(), rospy.Duration(0.1)
            )
            transforms_available = True
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            rospy.loginfo("Waiting for tf tree")

    return tf_listener

def transform_pose(input_pose, from_frame, to_frame):
    # Initialize a tf2 Buffer and TransformListener
    buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer)

    # Wait for the transformation to become available
    rospy.sleep(1.0)

    # Create a PoseStamped message from the input pose
    pose_stamped = PoseStamped()
    pose_stamped.header.frame_id = from_frame
    pose_stamped.pose = input_pose

    try:
        # Use the TransformListener to transform the pose to the target frame
        transformed_pose_stamped = buffer.transform(pose_stamped, to_frame)

        # Extract the transformed pose
        transformed_pose = transformed_pose_stamped.pose

        return transformed_pose

    except tf2_ros.TransformException as e:
        rospy.logwarn("Transform failed: {}".format(e))
        return None







# class TFFixer:
#     def __init__(self):
#         rospy.init_node("tool_frame_camlink_frame_transformer")

#         self.tfBuffer = tf2_ros.Buffer()
#         self.listener = tf2_ros.TransformListener(self.tfBuffer)

#         self.broadcast = tf2_ros.StaticTransformBroadcaster()

#         self.timeout = rospy.Duration(5.0)



#     def announce_transform(self, parent_frame, child_frame, published=False):
#         mode = "Published" if published else "Read"
#         rospy.loginfo(f"{mode} transform from {parent_frame} to {child_frame}!")

#     def main(self):
#         try:
#             T_color_cam_msg: TransformStamped = self.tfBuffer.lookup_transform(
#                 "camera_color_frame",
#                 "camera_link",
#                 rospy.Time(),
#                 self.timeout,
#             )
#             T_color_cam = numpify(T_color_cam_msg.transform)
#             self.announce_transform("camera_color_frame", "camera_link")

#             T_optic_color_msg: TransformStamped = self.tfBuffer.lookup_transform(
#                 "camera_color_optical_frame",
#                 "camera_color_frame",
#                 rospy.Time(),
#                 self.timeout,
#             )
#             T_optic_color = numpify(T_optic_color_msg.transform)
#             self.announce_transform("camera_color_optical_frame", "camera_color_frame")

#             T_tool0_optic_msg: TransformStamped = self.tfBuffer.lookup_transform(
#                 "tool0",
#                 "gt_color_optical",
#                 rospy.Time(),
#                 self.timeout,
#             )
#             T_tool0_optic = numpify(T_tool0_optic_msg.transform)
#             self.announce_transform("tool0", "gt_color_optical")

#         except (
#             tf2_ros.LookupException,
#             tf2_ros.ConnectivityException,
#             tf2_ros.ExtrapolationException,
#         ):
#             raise TimeoutError("Waited too long for a tf to be available!")

#         # Compute transform
#         T_tool0_cam = T_tool0_optic @ T_optic_color @ T_color_cam
#         # R_tool0_cam_quart = Quaternion(matrix=T_tool0_cam)
#         R_tool0_cam_quart = R.as_quat(R.from_matrix(T_tool0_cam[:3, :3]))

#         # Publish transform
#         t = TransformStamped()
#         t.header.frame_id = "tool0"
#         t.header.stamp = rospy.Time.now()
#         t.child_frame_id = "camera_link"
#         t.transform.translation.x = T_tool0_cam[0, 3]
#         t.transform.translation.y = T_tool0_cam[1, 3]
#         t.transform.translation.z = T_tool0_cam[2, 3]

#         t.transform.rotation.x = R_tool0_cam_quart[0]
#         t.transform.rotation.y = R_tool0_cam_quart[1]
#         t.transform.rotation.z = R_tool0_cam_quart[2]
#         t.transform.rotation.w = R_tool0_cam_quart[3]

#         self.broadcast.sendTransform(t)

#         self.announce_transform("tool0", "camera_link", published=True)

#         rospy.spin()


