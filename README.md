# Demo Instructions

- For the simple original one: run
    - roslaunch single_camera_demo setup_system.launch
    - run simple.py

- Facing an error:
    - run robot_model.py to clear faults directly

- Two user input requirement:
    - Going to the top of the object
    - Then going down
    - b for restarting, q for re-selecting


- To set it up again:
    - Firstly, put the marker into the desired place, make sure the direction is "facing to you", like shown in the pic below:
    ![Marker Direction](pic/Image.jpeg "Correct marker orientation")
    - Then hand recorded the position difference between the center of the marker and the robot, put it into the "static_transform_publisher" inside the steup_system.launch file, for the first three parameters of x,y,z
    - Next, fire up the aruco_ros_single.launch, you can use any files, just make sure that is started, and rostopic echo the information of  "/aruco_single/transform", put that for the second static_transforma publisher (both position and orientation)
    - After that, change values inside the publish_camera.py, to make sure values are consistent (probably only need to do it here, but do it both just to make sure)
    - Then the transformation should be ready