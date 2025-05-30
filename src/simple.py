#!/usr/bin/python3
# Add this to change the namespace, otherwise the robot model cannot be found
import os
os.environ["ROS_NAMESPACE"] = "/kinova_gen3_lite"
from typing import List, Union
import rospy
from geometry_msgs.msg import Pose, PoseArray, Quaternion, Vector3, Point
from gpd_ros.msg import GraspConfig, GraspConfigList
from laser_assembler.srv import AssembleScans2, AssembleScans2Request
from sensor_msgs.msg import PointCloud2, Image
from std_msgs.msg import Bool, Header
from robot_model import GEN3_LITE
from first_demo.srv import PCLFwdStatus
# from pathlib import Path
import numpy as np
import pyquaternion
from tf2_geometry_msgs import PoseStamped
import sensor_msgs.point_cloud2 as pc2
from camera import Camera
import tf.transformations as tf_trans
from cv_bridge import CvBridge, CvBridgeError
import cv2

class FirstDemo:
    def __init__(self):
        rospy.init_node("demo") 
        self.bridge = CvBridge()
        
        # Call the point cloud data service
        self.scan = rospy.ServiceProxy("/realsense_processing/stitch_pcd_service", AssembleScans2)
        
        # # indicates whether there are objects on the top of the table
        # self.left_object = None
        # self.right_object = None

        # Robot Object       
        self.robot = GEN3_LITE()
        self.robot.move_trajectories(self.robot.pre_grasp)
        self.grasp_process_timeout = rospy.Duration(5)
        
        # Publish the point cloud data into topic that GPD package which receives the input
        self.grasps_pub = rospy.Publisher("/cloud_to_get_grasps_on", PointCloud2, queue_size=1)


        self.grasp_list: Union[List[GraspConfig], None] = None

        
        # Subscribe to the gpd topic and get the grasp gesture data, save it into the grasp_list through call_back function
        self.gpd_sub = rospy.Subscriber("/detect_grasps/clustered_grasps", GraspConfigList, self.save_grasps)
        

        
        # Topic for visualization of the grasp pose
        self.pose_pub = rospy.Publisher("/pose_viz", PoseArray, queue_size=1)

        self.init_grasping_pose = None
        
        # Indicating which side to grasp
        self.left_work = True
        
        self.count = 0
        rospy.sleep(1)
        
            
    
        
    def scan_object(self) -> PointCloud2:
                
        '''
        Get the point cloud of the whole scene
        '''
        start_time = rospy.Time.now()
        rospy.sleep(2)
        end_time = rospy.Time.now()

        stitch_response = self.scan(AssembleScans2Request(begin=start_time, end=end_time))

            
        stitched_cloud: PointCloud2 = stitch_response.cloud
        
        return stitched_cloud
    
    

    def save_grasps(self, data: GraspConfigList):
        self.grasp_list = data.grasps
    
        
        
        
    def vector3ToNumpy(self, vec: Vector3) -> np.ndarray:
        return np.array([vec.x, vec.y, vec.z])        
        
        
        
    
    
    def find_grasps(self, start_time):
        while (self.grasp_list is None and 
               rospy.Time.now() - start_time < self.grasp_process_timeout):
            rospy.loginfo("waiting for left grasps")
            rospy.sleep(0.5)

            if self.grasp_list is None:
                rospy.logwarn("No grasps detected on pcl, restarting!!!")
                continue
        
        # Sort all grasps based on the gpd_ros's msg: GraspConfigList.grasps.score.data            
        self.grasp_list.sort(key=lambda x: x.score.data, reverse=True)
        grasp_performed = False
        for grasp in self.grasp_list:
            rot = np.zeros((3, 3))
            # grasp approach direction
            rot[:, 2] = self.vector3ToNumpy(grasp.approach)
            # hand closing direction
            rot[:, 0] = self.vector3ToNumpy(grasp.binormal)
            # hand axis
            rot[:, 1] = self.vector3ToNumpy(grasp.axis)
            
            
            # Turn the roll pitch yaw thing into the quaternion axis
            
            quat = pyquaternion.Quaternion(matrix=rot)
            pos: Point = grasp.position
            
            pos.z -= 0.02
            
            orientation = [180, 0, -90]
            
            
            
            
            
            # PointCloud offset position
            grasp_pose = Pose(
                position=pos,
                orientation=Quaternion(x=quat[1], y=quat[2], z=quat[3], w=quat[0]),
            )

            
            # Publish the grasp pose visualisation
            self.pose_pub.publish(
                PoseArray(header=Header(frame_id="base_link"), poses=[grasp_pose])
            )
            # res = input("Hit Enter to execute")
            
            
            # if res == 'q':
            #     continue
            # elif res == 'b':
            #     return None, True
            # input("Press Enter to go to pregrasp pose")
            grasp_move_done = self.robot.move_pose([pos.x, pos.y, 0.2], orientation)
            
   
                
            
            # input("Press Enter to go to grasp pose")
            grasp_move_done = self.robot.move_pose([pos.x, pos.y, pos.z], orientation)
            
            left = True if pos.y < 0.02 else False
            
            if grasp_move_done:
                grasp_performed = True
                break




        if not grasp_performed:
            rospy.logwarn("No valid grasps planned to, restarting!!")
            return grasp_performed, True
                    
        # Grasped the object
        self.robot.move_gripper(1)
        rospy.sleep(1)
        return grasp_performed, left
    
    
            
    
        
    def main(self):
        
        while not rospy.is_shutdown():
            self.robot.move_trajectories(self.robot.pre_grasp)
            
            start_time = rospy.Time.now()
            
            rospy.sleep(1)
            
            # Creates a bunch of grasping poses
            self.grasp_list = None
            # Get the point cloud data of the object
            pcd = self.scan_object()
            self.grasps_pub.publish(pcd)

            
            # Just wait for the grasping, while the callback function is running at the back

            executed, left = self.find_grasps(start_time)

            
            
            if not executed:
                rospy.logwarn("No valid grasps planned to, restarting!!")
                continue
                        
            # Grasped the object
            self.robot.move_gripper(1)
            rospy.sleep(1)
            
            self.robot.move_trajectories(self.robot.pre_grasp)
            self.robot.demo_release(left)
            self.robot.move_gripper(0)
            
            rospy.sleep(1)
            continue


if __name__ == '__main__':
    # params_path = "/home/jason/catkin_ws/src/ur_grasping/src/box_grasping/configs/base_config.yaml"
    
    try:
        grasper = FirstDemo()
        grasper.main()
        # grasper.robot.move_gripper(0.3)
        #[51 47 20]
    except KeyboardInterrupt:
        pass