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



rad2deg = 180/pi
RIGHT_CAM_NO = '242222070936'
LEFT_CAM_NO = '243522073159'
MIDDLE_CAM_NO = '242322078188'


    
    
class GEN3_LITE(ERobot):

    def __init__(self, cam_num=1):

        # links, name, urdf_string, urdf_filepath = self.URDF_read("/home/riot/kinova_gen3_lite/src/ros_kortex/kortex_description/arms/gen3_lite/6dof/urdf/gen3_lite_macro.xacro")
        links, name, urdf_string, urdf_filepath = self.URDF_read("/home/riot/kinova_gen3_lite/src/ros_kortex/kortex_description/robots/gen3_lite_gen3_lite_2f.xacro")
    
        super().__init__(
            links,
            name='gen3-lite',
            urdf_string=urdf_string,
            urdf_filepath=urdf_filepath,
            manufacturer="Kinova",
            gripper_links=links[7]
        )
        cam_serial = MIDDLE_CAM_NO

        if cam_serial == LEFT_CAM_NO:
            pose_path = '/home/riot/kinova_gen3_lite/src/box_detection/cam_poses/camera_pose_left.txt'
            scale_path = '/home/riot/kinova_gen3_lite/src/heightmap/real/camera_depth_scale_left.txt'

        elif cam_serial == RIGHT_CAM_NO:
            pose_path = '/home/riot/kinova_gen3_lite/src/box_detection/cam_poses/camera_pose_right.txt'
            scale_path = '/home/riot/kinova_gen3_lite/src/heightmap/real/camera_depth_scale_right.txt'
            
        elif cam_serial == MIDDLE_CAM_NO:
            pose_path = '/home/riot/kinova_gen3_lite/src/ggcnn/cfg/cam_pose.txt'
            scale_path = '/home/riot/kinova_gen3_lite/src/ggcnn/cfg/cam_scale.txt'
            
            
        self.cam_pose = np.loadtxt(pose_path, delimiter=' ')
        self.cam_depth_scale = np.loadtxt(scale_path, delimiter=' ')
            
        if cam_num == 2:
            self.cam_pose_right = np.loadtxt('/home/riot/kinova_gen3_lite/src/box_detection/cam_poses/camera_pose_right.txt', delimiter=' ')
            self.cam_depth_scale_right = np.loadtxt('/home/riot/kinova_gen3_lite/src/heightmap/real/camera_depth_scale_right.txt', delimiter=' ')

        try:
            if rospy.get_param("/gazebo/time_step", None):
                self.is_simulation = True
            else:
                self.is_simulation = False
            
            
            self.built_in = True
            
            self.HOME_ACTION_IDENTIFIER = 2

            # Get node params
            self.robot_name = "kinova_gen3_lite"
            self.degrees_of_freedom = rospy.get_param("/" + self.robot_name + "/degrees_of_freedom", 6)
            self.is_gripper_present = rospy.get_param("/" + self.robot_name + "/is_gripper_present", True)

            rospy.loginfo("Using robot_name " + self.robot_name + " , robot has " + str(self.degrees_of_freedom) + " degrees of freedom and is_gripper_present is " + str(self.is_gripper_present))

            ##### Init the topics #####
            # Init the action topic subscriber
            self.action_topic_sub = rospy.Subscriber("/" + self.robot_name + "/action_topic", ActionNotification, self.cb_action_topic)
            self.last_action_notif_type = None
            
            
            
    
            ##### Init the services #####
            # Reset or clear fault states, allowing the robotic arm to recover from errors and resume normal operation
            clear_faults_full_name = '/' + self.robot_name + '/base/clear_faults'
            rospy.wait_for_service(clear_faults_full_name)
            self.clear_faults = rospy.ServiceProxy(clear_faults_full_name, Base_ClearFaults)

            joint_vel_cmd_full_name = '/' + self.robot_name + '/base/send_joint_speeds_command'
            rospy.wait_for_service(joint_vel_cmd_full_name)
            self.joint_vel_cmd = rospy.ServiceProxy(joint_vel_cmd_full_name, SendJointSpeedsCommand)
            
            read_action_full_name = '/' + self.robot_name + '/base/read_action'
            rospy.wait_for_service(read_action_full_name)
            self.read_action = rospy.ServiceProxy(read_action_full_name, ReadAction)
            
            execute_action_full_name = '/' + self.robot_name + '/base/execute_action'
            rospy.wait_for_service(execute_action_full_name)
            self.execute_action = rospy.ServiceProxy(execute_action_full_name, ExecuteAction)
            
            set_cartesian_reference_frame_full_name = '/' + self.robot_name + '/control_config/set_cartesian_reference_frame'
            rospy.wait_for_service(set_cartesian_reference_frame_full_name)
            self.set_cartesian_reference_frame = rospy.ServiceProxy(set_cartesian_reference_frame_full_name, SetCartesianReferenceFrame)
            
            send_gripper_command_full_name = '/' + self.robot_name + '/base/send_gripper_command'
            rospy.wait_for_service(send_gripper_command_full_name)
            self.send_gripper_command = rospy.ServiceProxy(send_gripper_command_full_name, SendGripperCommand)
            
            activate_publishing_of_action_notification_full_name = '/' + self.robot_name + '/base/activate_publishing_of_action_topic'
            rospy.wait_for_service(activate_publishing_of_action_notification_full_name)
            self.activate_publishing_of_action_notification = rospy.ServiceProxy(activate_publishing_of_action_notification_full_name, OnNotificationActionTopic)
            
            get_product_configuration_full_name = '/' + self.robot_name + '/base/get_product_configuration'
            rospy.wait_for_service(get_product_configuration_full_name)
            self.get_product_configuration = rospy.ServiceProxy(get_product_configuration_full_name, GetProductConfiguration)
            
            validate_waypoint_list_full_name = '/' + self.robot_name + '/base/validate_waypoint_list'
            rospy.wait_for_service(validate_waypoint_list_full_name)
            self.validate_waypoint_list = rospy.ServiceProxy(validate_waypoint_list_full_name, ValidateWaypointList)
        
        
            self.subscribe_to_a_robot_notification()
        except:
            self.is_init_success = False
        else:
            self.is_init_success = True
            
        if not self.is_simulation:
            
            rospy.loginfo("------- You are now in Real Robot Case -------")

        else:
            # For simulation, kinova does not have the same function in real robots, so we use Moveit Setups
            
            moveit_commander.roscpp_initialize(sys.argv)
        
            # Group names of the robot, found it by running the actual robot
            self.arm_group_name = 'arm'
            self.gripper_group_name = 'gripper'
            
            # Robot’s kinematic model and the robot’s current joint states
            self.robot = moveit_commander.RobotCommander()
            # Robot’s internal understanding of the surrounding world
            self.scene = moveit_commander.PlanningSceneInterface(ns="/kinova_gen3_lite/")
            # Interfaces for planning groups (group of joints) to plan an execute motions
            self.arm_group = moveit_commander.MoveGroupCommander(self.arm_group_name, ns="/kinova_gen3_lite/")
            self.gripper_group = moveit_commander.MoveGroupCommander(self.gripper_group_name, ns="/kinova_gen3_lite/")

            # Ros publisher that is used to display trajectories in Rviz
            self.display_trajectory_publisher = rospy.Publisher('/kinova_gen3_lite/move_group/display_planned_path',
                                                        moveit_msgs.msg.DisplayTrajectory,
                                                        queue_size=20)
            
            
            
            # Used for reset the world 
            # rospy.wait_for_service('/gazebo/reset_world')
            

            rospy.loginfo("------- You are using MoveIt now -------")

        
        self.ee_to_tool = sm.SE3.Tz(0.13) * sm.SE3.Rz(1.571)
        
        self.home = np.array([0   * pi/180, 
                              -16 * pi/180, 
                              75  * pi/180, 
                              0   * pi/180, 
                              -60 * pi/180, 
                              0   * pi/180])
        
        self.vertical = np.zeros(6)
        
        self.pre_grasp = np.array([ -28  * pi/180, 
                                     20  * pi/180, 
                                    123  * pi/180, 
                                     89  * pi/180, 
                                     74  * pi/180, 
                                     68  * pi/180])
        
        self.pre_right_grasp = np.array([ 13  * pi/180, 
                                           0  * pi/180, 
                                         120  * pi/180, 
                                         -90  * pi/180, 
                                         -60  * pi/180, 
                                         106  * pi/180])
        
        
        
        self.pre_push = np.array([ 13  * pi/180, 
                                  -32  * pi/180, 
                                  112  * pi/180, 
                                  -90  * pi/180, 
                                  -37  * pi/180, 
                                  -77  * pi/180])
        
        self.calibration =  np.array([0, 
                               -75 * pi/180, 
                               105 * pi/180, 
                               90 * pi/180, 
                               -90 * pi/180, 
                               -90 * pi/180])
        
        
        
        
        self.addconfiguration(
            "vertical", np.array([0, 0, 0, 0, 0, 0])
            
        )
        
        self.addconfiguration(
            "home", np.array([0   * pi/180, 
                              -16 * pi/180, 
                              75  * pi/180, 
                              0   * pi/180, 
                              -60 * pi/180, 
                              0   * pi/180])
            
        )
        
        self.addconfiguration(
            "pre", np.array([-48 * pi/180, 
                             38  * pi/180, 
                             137 * pi/180, 
                             94  * pi/180, 
                             74  * pi/180, 
                             39  * pi/180])
            
        )
        
        
        
        self.qdlim = np.array(
            [1, 1, 1, 1, 1, 1.57]
        )
        if self.is_simulation:
            self.init_scene()
    
    
    
    def init_pose(self):
        self.move_trajectories(self.pre_grasp)
        
        
    # def grasp_prep(self, left):
    #     """Pregrasp movement for the demo

    #     Args:
    #         left: a boolean for indicating whether working in the left workspace or right workspace
    #     """

    #     self.move_trajectories(self.pre_grasp)

        
        
    def init_scene(self):
        '''
        Adding a table under the robot to ensure it does not hit the table in reality
        '''
        table_size = [2, 2, 0.86]
        
        table_pose = PoseStamped()
        table_pose.header.frame_id = "base_link"
        table_pose.pose.position.x = 0  
        table_pose.pose.position.y = 0  
        table_pose.pose.position.z = -table_size[2]/2-0.00001 
        table_name = "table"
        self.scene.add_box(table_name, table_pose, size=table_size)    
      
      
        
    def cb_action_topic(self, notif: ActionNotification):
        self.last_action_notif_type = notif.action_event
    
    
    
    
    def unpack_plan(self, plan_tuple):
        """Function used to unpack the tuple returned when planning with move_group.
        This seems to be different than is was in ros melodic, so this function
        is needed to adapt the old code to the changes.

        Args:
            plan_tuple: A plan tuple containing the plan and other success data.

        Returns:
            If the planning was successful, a trajectory that can be directly used for
            visualization and motion. If unsuccessful, None is returned.
        """

        # plan_tuple[0] is the success boolean flag
        if plan_tuple[0]:
            return plan_tuple[1]  # The RobotTrajectory
        else:
            # If needed, the exact error code can be parsed from plan_tuple[3]
            return None
    
    
        
    def get_current_pose(self):
        """Get current robot's tool pose

        Returns:
            position: in xyz order, in meters
            orientation: in xyz order, in degrees
        """
        robot_data: BaseCyclic_Feedback = rospy.wait_for_message("/kinova_gen3_lite/base_feedback", BaseCyclic_Feedback, 1)
        
        position = [robot_data.base.tool_pose_x, 
                    robot_data.base.tool_pose_y,
                    robot_data.base.tool_pose_z]
        
        orientation = [robot_data.base.tool_pose_theta_x, 
                       robot_data.base.tool_pose_theta_y,
                       robot_data.base.tool_pose_theta_z]
        
        return position, orientation
        

    def get_current_joint_values(self):
        """
        Get the current joint values of the robot in radians

        Returns:
            joint values in radians
        """
        joints = np.zeros(self.degrees_of_freedom)
        robot_data: BaseCyclic_Feedback = rospy.wait_for_message("/kinova_gen3_lite/base_feedback", BaseCyclic_Feedback, 1)
        
        for i in range(0, self.degrees_of_freedom):
            joints[i] = robot_data.actuators[i].position * pi/180

        return joints
    
    
    
    def stop(self):
        zero_speeds = Base_JointSpeeds()

        for i in range(self.degrees_of_freedom):
            joint = JointSpeed()
            joint.joint_identifier = i
            joint.value = 0.0
            self.joint_vel_cmd(SendJointSpeedsCommandRequest(input=zero_speeds))
           
    
    
    
    def move_pose(self, position, orientation):
        """Move the robot to a pose

        Args:
            position: x,y,z based on the /tool_frame
            orientation: r,p,y in degrees
        """
        
        pose = self.visualize_pose(position, orientation)
        
        # res = input("Check Rviz, is it possible to do the pose?")
        # if res == "n":
        #     quit()
            
        if self.is_simulation:
            self.arm_group.set_joint_value_target(pose, True)
            plan_tuple = self.arm_group.plan()
            plan = self.unpack_plan(plan_tuple)
            answer = input("Press Enter to proceed or q to replot")
            self.arm_group.set_pose_target(pose)
            if answer != "q":
                attempted = self.arm_group.execute(plan, wait=True)
            self.arm_group.clear_pose_targets()
            
            
        else:
           
                
            my_cartesian_speed = CartesianSpeed()
            my_cartesian_speed.translation = 0.1 # m/s
            my_cartesian_speed.orientation = 30  # deg/s
            
            my_constrained_pose = ConstrainedPose()
            my_constrained_pose.constraint.oneof_type.speed.append(my_cartesian_speed)

            my_constrained_pose.target_pose.x = position[0]
            my_constrained_pose.target_pose.y = position[1]
            my_constrained_pose.target_pose.z = position[2]
    
            
            my_constrained_pose.target_pose.theta_x = orientation[0]
            my_constrained_pose.target_pose.theta_y = orientation[1]
            my_constrained_pose.target_pose.theta_z = orientation[2]
        
        
            req = ExecuteActionRequest()
            req.input.oneof_action_parameters.reach_pose.append(my_constrained_pose)
            req.input.name = "pose_movement"
            req.input.handle.action_type = ActionType.REACH_POSE
            req.input.handle.identifier = 1001

            rospy.loginfo("Sending pose...")
            self.last_action_notif_type = None
            try:
                self.execute_action(req)
            except rospy.ServiceException:
                rospy.logerr("Failed to send pose")
                success = False
            else:
                rospy.loginfo("Waiting for pose to finish...")

            return self.wait_for_action_end_or_abort()
            
            
            
    def visualize_pose(self, position, orientation):
        # Pose visualisation topic in rviz
        pose_pub = rospy.Publisher("/pose_viz", PoseArray, queue_size=1)
        
        
        pose = Pose()
        pose.position.x = position[0]
        pose.position.y = position[1]
        pose.position.z = position[2]
        orientions = self.rpy_to_quaternion(orientation)
        pose.orientation.x = orientions[0]
        pose.orientation.y = orientions[1]
        pose.orientation.z = orientions[2]
        pose.orientation.w = orientions[3]

        pose_pub.publish(
            PoseArray(header=Header(frame_id="base_link"), poses=[pose])
        )
        
        return pose

    def rpy_to_quaternion(self, angles):
        """Change roll pitch yaw to quaternion

        Args:
            angles: in r,p,y order in degrees
        Returns:
            an array in x,y,z,w order
        """
        
        quaternion = tf.transformations.quaternion_from_euler(angles[0] * np.pi/180, 
                                                            angles[1] * np.pi/180, 
                                                            angles[2] * np.pi/180)

        return quaternion


        

    def move_joints(self, joints):
        """Use velocity control to move the robot of a simple pose

        Args:
            joints: target joint in radians based on /tool_frame
        """        
        
        gain = np.ones(6) * 10
        
        target_pose = self.fkine(joints)
        
        arrived = False
        while not arrived:
            
            # Work out the base frame manipulator Jacobian using the current robot configuration
            J = self.jacob0(self.get_current_joint_values())
            
            # The /tool_frame pose of the robot 
            Te = self.fkine(self.get_current_joint_values()) * self.ee_to_tool
            
            # Calculate the jacobian of the robot
            J_inv = np.linalg.inv(J)
            
            # Use the robotic toolbox to find the desired joint velocities
            ev, arrived = rtb.p_servo(Te.A, target_pose.A, gain=gain, threshold=0.001, method='angle-axis')
            desired_joint_velocities = J_inv @ ev
            # Create the speed message
            speeds = Base_JointSpeeds()
        
            for i in range(self.degrees_of_freedom):
                joint = JointSpeed()
                joint.joint_identifier = i
                joint.value = desired_joint_velocities[i]
                speeds.joint_speeds.append(joint)

            # publish it to the topic
            self.joint_vel_cmd(SendJointSpeedsCommandRequest(input=speeds))

        self.stop()


    def demo_release(self, left):
        upper_x_limit = 0.46 
        lower_x_limit = 0.2 
        
        upper_y_limit = 0.4 if left else -0.2
        lower_y_limit = 0.2 if left else -0.4
        
        work_space_x = random.uniform(lower_x_limit, upper_x_limit)
        work_space_y = random.uniform(lower_y_limit, upper_y_limit)
        
        orientation = [180, 0, -90]
        
        print(f'The planned pos is:{work_space_x, work_space_y}')
        self.move_pose([work_space_x, work_space_y, 0.1], orientation)


            
    def move_trajectories(self, angles):
        """Move robots based on joint angles (in radians)

        Args:
            angles: six joint angles in radians

        Returns:
            _description_
        """
        if not self.is_simulation:
            self.last_action_notif_type = None

            req = ExecuteActionRequest()

            trajectory = WaypointList()
            waypoint = Waypoint()
            angularWaypoint = AngularWaypoint()

            # Angles to send the arm 
            for i in range(self.degrees_of_freedom):
                angularWaypoint.angles.append(angles[i]*rad2deg)

            # Each AngularWaypoint needs a duration and the global duration (from WaypointList) is disregarded. 
            # If you put something too small (for either global duration or AngularWaypoint duration), the trajectory will be rejected.
            angular_duration = 0
            angularWaypoint.duration = angular_duration
            # Initialize Waypoint and WaypointList
            waypoint.oneof_type_of_waypoint.angular_waypoint.append(angularWaypoint)
            trajectory.duration = 0
            trajectory.use_optimal_blending = False
            trajectory.waypoints.append(waypoint)

            try:
                res = self.validate_waypoint_list(trajectory)
            except rospy.ServiceException:
                rospy.logerr("Failed to call ValidateWaypointList")
                return False

            error_number = len(res.output.trajectory_error_report.trajectory_error_elements)
            MAX_ANGULAR_DURATION = 30

            while (error_number >= 1 and angular_duration != MAX_ANGULAR_DURATION) :
                angular_duration += 1
                trajectory.waypoints[0].oneof_type_of_waypoint.angular_waypoint[0].duration = angular_duration

                try:
                    
                    res = self.validate_waypoint_list(trajectory)
                        
                except rospy.ServiceException:
                    rospy.logerr("Failed to call ValidateWaypointList")
                    return False

                error_number = len(res.output.trajectory_error_report.trajectory_error_elements)

            if (angular_duration == MAX_ANGULAR_DURATION) :
                # It should be possible to reach position within 30s
                # WaypointList is invalid (other error than angularWaypoint duration)
                rospy.loginfo("WaypointList is invalid")
                return False

            req.input.oneof_action_parameters.execute_waypoint_list.append(trajectory)
            
            # Send the angles
            rospy.loginfo("Sending the robot to the trajectories...")
            
            try:
                self.execute_action(req)
            except rospy.ServiceException:
                rospy.logerr("Failed to call ExecuteWaypointjectory")
                return False
            else:
                return self.wait_for_action_end_or_abort()       
            
        else:
            joint_values = self.arm_group.get_current_joint_values()
            joint_values = angles
            self.arm_group.go(joint_values, wait=True)    
            
            
    def move_gripper(self, value):
        """Changes the gripper position

        Args:
            value: 0 means fully open, 1 means fully close

        Returns:
            True if the action is successful
        """
        # Initialize the request
        # Close the gripper
        req = SendGripperCommandRequest()
        finger = Finger()
        finger.finger_identifier = 0
        finger.value = value
        req.input.gripper.finger.append(finger)
        req.input.mode = GripperMode.GRIPPER_POSITION

        rospy.loginfo("Sending the gripper command...")

        # Call the service 
        try:
            self.send_gripper_command(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call SendGripperCommand")
            return False
        else:
            time.sleep(0.5)
            return True       
            
            
    def wait_for_action_end_or_abort(self):
        while not rospy.is_shutdown():
            if (self.last_action_notif_type == ActionEvent.ACTION_END):
                rospy.loginfo("Received ACTION_END notification")
                return True
            elif (self.last_action_notif_type == ActionEvent.ACTION_ABORT):
                rospy.loginfo("Received ACTION_ABORT notification")
                return False
            else:
                time.sleep(0.01)
                
                
                
                
    def subscribe_to_a_robot_notification(self):
        # Activate the publishing of the ActionNotification
        req = OnNotificationActionTopicRequest()
        rospy.loginfo("Activating the action notifications...")
        try:
            self.activate_publishing_of_action_notification(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call OnNotificationActionTopic")
            return False
        else:
            rospy.loginfo("Successfully activated the Action Notifications!")

        rospy.sleep(1.0)
        return True
    
    def clear(self):
        try:
            self.clear_faults()
        except rospy.ServiceException:
            rospy.logerr("Failed to call ClearFaults")
            return False
        else:
            rospy.loginfo("Cleared the faults successfully")
            rospy.sleep(2.5)
            return True

    def push_primitive(self, index):
        """Based on the position and direction to execute the push primitives

        Args:
            length: distance in meters
            direction: from value 0-7, each one is 45 degrees
        """
        distance = 0.2
        direction = np.linspace(0, 2*pi, 9)
        
        
        current_position, _ = self.get_current_pose()
        
        
        
        x_movement = distance*np.cos(direction[index])
        y_movement = distance*np.sin(direction[index])
        
        position = [current_position[0] + x_movement, current_position[1] + y_movement, 0.04]
        tool_orientation = [0, 180, 90]
        
        if position[0] < 0.2:
            position[0] = 0.2
        
        self.move_pose(position, tool_orientation)
        
        
        return
        
    
if __name__ == "__main__":

    rospy.init_node("robot_model")
    kinova_lite = GEN3_LITE()
    kinova_lite.clear()
    # kinova_lite.move_trajectories(kinova_lite.pre_grasp)
    current_pose, current_orientation = kinova_lite.get_current_pose()
    # print(current_pose, current_orientation)
    kinova_lite.move_pose([0.16243746876716614, -0.04241149500012398, 0.0355830192565918], [2.2619452476501465, -177.9755401611328, 84.61754608154297])
    
    # Forward
    current_pose[0] += 0.09
    kinova_lite.move_pose(current_pose, current_orientation)
    # kinova_lite.move_gripper(0)
    current_pose, current_orientation = kinova_lite.get_current_pose()

    # Left
    current_pose[1] += 0.1
    kinova_lite.move_pose(current_pose, current_orientation)
    
    # Forward again
    current_pose, current_orientation = kinova_lite.get_current_pose()
    # print(current_pose, current_orientation)
    current_pose[0] += 0.05
    kinova_lite.move_pose(current_pose, current_orientation)
    
    # Right
    current_pose, current_orientation = kinova_lite.get_current_pose()
    # print(current_pose, current_orientation)
    current_pose[1] -= 0.1
    kinova_lite.move_pose(current_pose, current_orientation)
    
    # kinova_lite.init_pose()
    # kinova_lite.clear()
    # kinova_lite.move_trajectories(kinova_lite.pre_grasp)
    # position = [0.2, 0, 0.0]
    # tool_orientation = [180, 0, -90]
    # # while True:
    # #     kinova_lite.visualize_pose(position, tool_orientation)
    # kinova_lite.move_pose(position, tool_orientation)
    # rospy.spin()
    
    # # Calibration process
    # kinova_lite.move_trajectories(kinova_lite.calibration)
    # position = [0.4, 0, 0.1]
    # tool_orientation = [90, 0, 90]
    
    # kinova_lite.move_pose(position, tool_orientation)
    
    
