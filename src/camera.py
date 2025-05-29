import socket
import numpy as np
import cv2
import os
import time
import struct
import matplotlib.pyplot as plt
import sys
from scipy.spatial.transform import Rotation as R
root_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../"))
sys.path.append(root_dir)
from utils import *
import open3d as o3d


class Camera(object):

    def __init__(self, cam_num=2):

        # Data options (change me)
        self.im_height = 720
        self.im_width = 1280
        self.tcp_host_ip = '127.0.0.1'
        self.tcp_port = 50002
        self.buffer_size = 4098 # 4 KiB

        # Connect to server
        self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.tcp_socket.connect((self.tcp_host_ip, self.tcp_port))
        
        
        self.cam_num = cam_num
        

        self.intrinsics_1 = None
        self.intrinsics_2 = None
        self.cam_pos_1 = np.loadtxt('/home/riot/kinova_gen3_lite/src/box_detection/cam_poses/camera_pose_left.txt', delimiter=' ')
        self.cam_pos_2 = np.loadtxt('/home/riot/kinova_gen3_lite/src/box_detection/cam_poses/camera_pose_right.txt', delimiter=' ')
        
        # self.get_data_two_images()
        
        
        
    
    
    def get_data_two_images(self):
        # Ping the server with anything
        self.tcp_socket.send(b'asdf')

        # Fetch TCP data:
        #     color camera intrinsics, 9 floats, number of bytes: 9 x 4
        #     depth scale for converting depth from uint16 to float, 1 float, number of bytes: 4
        #     depth image, self.im_width x self.im_height uint16, number of bytes: self.im_width x self.im_height x 2
        #     color image, self.im_width x self.im_height x 3 uint8, number of bytes: self.im_width x self.im_height x 3
        data = b''
        while len(data) < (10*4*2 + self.im_height*self.im_width*10):
            data += self.tcp_socket.recv(self.buffer_size)

        # Reorganize TCP data into color and depth frame
        self.intrinsics_1 = np.frombuffer(data[0:(9*4)], np.float32).reshape(3, 3)
        # print(f'after sending, intrinsic are :{self.intrinsics}')
        depth_scale_1 = np.frombuffer(data[(9*4):(10*4)], np.float32)[0]
        # print(f'after sending, depth_scale are :{depth_scale}')
        depth_img_1 = np.frombuffer(data[(10*4):((10*4)+self.im_width*self.im_height*2)], np.uint16).reshape(self.im_height, self.im_width)
        # print(f'after sending, depth_img are :{depth_img}')
        color_img_1 = np.frombuffer(data[((10*4)+self.im_width*self.im_height*2): 10*4 + self.im_width*self.im_height*2 + self.im_width*self.im_height*3], np.uint8).reshape(self.im_height, self.im_width, 3)
        # print(f'after sending, color_img are :{color_img}')
        fixed_1 = color_img_1.copy()
        fixed_1[:,:,0] = color_img_1 [:,:,-1] 
        fixed_1[:,:,-1] = color_img_1 [:,:,0] 
        
        depth_img_1 = depth_img_1.astype(float) * depth_scale_1
        
        existing_size = 10*4 + self.im_width*self.im_height*2 + self.im_width*self.im_height*3
        
        # Reorganize TCP data into color and depth frame
        self.intrinsics_2 = np.frombuffer(data[existing_size : existing_size + (9*4)], np.float32).reshape(3, 3)
        # print(f'after sending, intrinsic are :{self.intrinsics}')
        depth_scale_2 = np.frombuffer(data[existing_size + (9*4) : existing_size + (10*4)], np.float32)[0]
        # print(f'after sending, depth_scale are :{depth_scale}')
        depth_img_2 = np.frombuffer(data[existing_size + (10*4): existing_size + ((10*4)+self.im_width*self.im_height*2)], np.uint16).reshape(self.im_height, self.im_width)
        # print(f'after sending, depth_img are :{depth_img}')
        color_img_2 = np.frombuffer(data[existing_size + ((10*4)+self.im_width*self.im_height*2):], np.uint8).reshape(self.im_height, self.im_width, 3)
        # print(f'after sending, color_img are :{color_img}')
        fixed_2 = color_img_2.copy()
        fixed_2[:,:,0] = color_img_2 [:,:,-1] 
        fixed_2[:,:,-1] = color_img_2 [:,:,0] 
        
        depth_img_2 = depth_img_2.astype(float) * depth_scale_2
        
        return fixed_1, depth_img_1, fixed_2, depth_img_2
    
    
    # Get the pointcloud that removes the table 
    def combined_pointcloud(self):
        color_img_1, depth_img_1, color_img_2, depth_img_2 = self.get_data_two_images()
        
        left, right = extract_pointcloud(color_img_1, depth_img_1, color_img_2, depth_img_2, self.intrinsics_1, self.intrinsics_2)
    
        left.transform(self.cam_pos_1)
        right.transform(self.cam_pos_2)
        
        
        threshold = 0.02  # Distance threshold
        trans_init = np.identity(4)  # Initial transformation matrix

        # Apply ICP
        reg_p2p = o3d.pipelines.registration.registration_icp(
            left, right, threshold, trans_init,
            o3d.pipelines.registration.TransformationEstimationPointToPoint())

        # Transform pcd1 to align with pcd2
        left.transform(reg_p2p.transformation)

        combined_pcd = left + right
        
        
        
        voxel_size = 0.005  # Adjust based on your point cloud density
        pcd = combined_pcd.voxel_down_sample(voxel_size)


        # Iteratively remove planes (tables)
        plane_model, inliers = pcd.segment_plane(distance_threshold=0.01,
                                                                ransac_n=3,
                                                                num_iterations=1000)

        # [a, b, c, d] = plane_model

        # Extract inliers (the table) and outliers (the rest of the scene)
        # table_cloud = pcd.select_by_index(inliers)
        remaining_cloud = pcd.select_by_index(inliers, invert=True)

        # Visualize the current iteration's table point cloud
        # table_cloud.paint_uniform_color([1, 0, 0])  # Color the table red
        # o3d.visualization.draw_geometries([table_cloud], window_name=f'Table Point Cloud - Iteration {1}')

        # Visualize the remaining point cloud (without the table)
        # o3d.visualization.draw_geometries([remaining_cloud], window_name='Point Cloud without Table')
        return remaining_cloud
                
        
        
        
        
        
        
        
        
        
        
    
if __name__ == "__main__":
    camera = Camera(2)
    time.sleep(1) #
    start = time.time()
    camera.combined_pointcloud()
    end = time.time()
    print(f'total time is: {end-start}')