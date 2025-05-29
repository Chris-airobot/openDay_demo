#!/usr/bin/python3
import os
os.environ["ROS_NAMESPACE"] = "/kinova_gen3_lite"
import sys
# Get the root directory (one level up from src)
root_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "../"))
sys.path.append(root_dir)
import numpy as np
from roboticstoolbox import *
from math import pi
import rospy
from kortex_driver.srv import *
from kortex_driver.msg import *
import spatialmath as sm
import roboticstoolbox as rtb
import time
import moveit_commander
import moveit_msgs.msg
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Point,Quaternion, PoseStamped, PoseArray
import pyrealsense2 as rs
import tf
import random
import copy

 # For simulation, kinova does not have the same function in real robots, so we use Moveit Setups
            
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial',
                anonymous=True)
# Group names of the robot, found it by running the actual robot
arm_group_name = 'arm'
gripper_group_name = 'gripper'

# Robot’s kinematic model and the robot’s current joint states
robot = moveit_commander.RobotCommander()
# Robot’s internal understanding of the surrounding world
scene = moveit_commander.PlanningSceneInterface(ns="/kinova_gen3_lite/")
# Interfaces for planning groups (group of joints) to plan an execute motions
arm_group = moveit_commander.MoveGroupCommander(arm_group_name, ns="/kinova_gen3_lite/")
gripper_group = moveit_commander.MoveGroupCommander(gripper_group_name, ns="/kinova_gen3_lite/")

# Ros publisher that is used to display trajectories in Rviz
display_trajectory_publisher = rospy.Publisher('/kinova_gen3_lite/move_group/display_planned_path',
                                        moveit_msgs.msg.DisplayTrajectory,
                                        queue_size=20)



# Used for reset the world 
# rospy.wait_for_service('/gazebo/reset_world')


rospy.loginfo("------- You are using MoveIt now -------")

def interpolate_waypoints(start_pose, end_pose, num_points=100):
    # Generate points along a curve (e.g., quadratic or cubic bezier curve)
    t = np.linspace(0, 1, num_points)
    
    # Example: Quadratic Bezier curve (Control points can be adjusted)
    control_point = Pose()
    control_point.position.x = (start_pose.position.x + end_pose.position.x) / 2
    control_point.position.y = (start_pose.position.y + end_pose.position.y) / 2   # Curvature height
    control_point.position.z = (start_pose.position.z + end_pose.position.z) / 2 + 0.2

    waypoints = []
    for ti in t:
        x = (1 - ti)**2 * start_pose.position.x + 2 * (1 - ti) * ti * control_point.position.x + ti**2 * end_pose.position.x
        y = (1 - ti)**2 * start_pose.position.y + 2 * (1 - ti) * ti * control_point.position.y + ti**2 * end_pose.position.y
        z = (1 - ti)**2 * start_pose.position.z + 2 * (1 - ti) * ti * control_point.position.z + ti**2 * end_pose.position.z
        
        waypoint = Pose()
        waypoint.position.x = x
        waypoint.position.y = y
        waypoint.position.z = z
        waypoint.orientation = start_pose.orientation  # Keep the same orientation
        waypoints.append(waypoint)
    print(waypoints)
    return waypoints

start_pose = arm_group.get_current_pose().pose
end_pose = arm_group.get_current_pose().pose
end_pose.position.y  = end_pose.position.y + 0.4
# waypoints = interpolate_waypoints(start_pose, end_pose,5)

waypoints = []
scale = 1
wpose = arm_group.get_current_pose().pose
wpose.position.z += scale * 0.1  # First move up (z)
wpose.position.y += scale * 0.3  # and sideways (y)
waypoints.append(copy.deepcopy(wpose))




# We want the Cartesian path to be interpolated at a resolution of 1 cm
# which is why we will specify 0.01 as the eef_step in Cartesian
# translation.  We will disable the jump threshold by setting it to 0.0 disabling:
(plan, fraction) = arm_group.compute_cartesian_path(
                                   waypoints,   # waypoints to follow
                                   1,        # eef_step
                                   0.0)         # jump_threshold


arm_group.set_max_velocity_scaling_factor(0.5)  # Set to 50% of max speed
display_trajectory = moveit_msgs.msg.DisplayTrajectory()
display_trajectory.trajectory_start = robot.get_current_state()
display_trajectory.trajectory.append(plan)
# Publish
display_trajectory_publisher.publish(display_trajectory)

arm_group.execute(plan, wait=True)