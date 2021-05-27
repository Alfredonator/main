# Potato, radish and lemon picker.
## Authors: students from Aalborg University.

This project is plan for a bachellor project during the 6th semester. Authors of this project are:
 - Demira Demitrova
 - Szymon Gabryel
 - Alicja Haraszczuk
 - Patrick Vibild.

This project has been done in collaboration with Aalborg University IoT Super project and CMI department from Aalborg University.


## Installation of dependencies.
The system is based on Robotic Operating System (ROS) and several developed packages need to be download to make the system run.

Dependecies that have to be installed are.

 - [Ubuntu](https://releases.ubuntu.com/18.04/) 18.04 on bare metal. VM will cuase issues with reading information from the cameras.
 - [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu) Desktop full installation mode.
 - Python2.7
 - Python 3.6 or higher
 - Install RealSense SDK `https://dev.intelrealsense.com/docs/compiling-librealsense-for-linux-ubuntu-guide`
 - Install RealSense ROS package `https://github.com/IntelRealSense/realsense-ros`
 - Install OpenCV 4.* 
 - Install Moveit Simple Controller `apt get install ros-melodic-moveit-simple-controller-manager`
 - Install several Python2 packages `pip install getkey && pip install --upgrade setuptools && pip install numpy &&`
  
 
 Afterwards create a catkin workspace. 
```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
```

Now copy next ROS developed packages into the source of the new catkin workspace `cd ~/catkin_ws/src`

| ROS package | Description |
| ------ | ------ |
| [Main] | package that contains general launch files to execute the application |
| [edo_moveit_gripper] | MoveIt package for the robot. |
| [eDO_description] | URDF, meshes and definition of the robot |
| [edo_gripper] | URDF, meshes and definition of the gripper |
| [eDO_control_v3] | Controller package that interacts with the robotic arm and gripper|
| [Decision Making Pipeline] | "business logic" that picks lemon, potatoes and raidish and put them in the right bucket |
| [Camera Node] | Node that obtains the positions of objects |
| [Moveit_grasps] | Find grasping positions for the robotic system |
| [TensorFlow server] | Server that runs the ML algorithm in Tf2. Connects with [camera Node] over local TCP |
| [moveit_calibration] | hand-eye calibration for the cameras in robotic system |
| [eDO core msgs] | eDO ROS messages |
| [geometry_representation] | ROS geometry messages. System has dependencies on this package. |
| [RVIZ visual tools] | dependency for [moveit_calibration] |

After cloning all repositories, compile and build the catkin workspace. 
```
cd ~/catkin_ws
catkin_make
```

## Setup installation.

The system has to be installed on a computer external to the raspberry pi contained in the eDO robot.

During out development we used a Intel(R) Xeon(R) W-2133 CPU @ 3.60GHz processor and were hitting 80-90% of CPU usage on the system runtime. 

The robot and the server/computer need to be connected with a switch that has 1000MB/s throughput. Also check for cables Cat5e or superior cables.

Server/Computer and the robot raspberry pi need to have their clocks synchronize. NTP server can be installed in the server and make the raspberry pi update its clock and synchronize with the server/computer.

Camera top (camera with serial_no = f0190391) is situated 70CM above the robotic work-space.
Camera side (camera with serial_no = f0191108) is situated at 2m of the robot and captures the workspace.


## Running the System
Before running the system there are different steps to be done.

 - Calibrate the robot
 - Calibrate TOP camera
 - Calirbate SIDE camera

Every time we execute any ROS package or we want to use any ROS command and check the status of our ROS master node we need to declare ROS_MASTER_URI and ROS_IP to point the Robot RaspberryPi. This can be done with the next lines:

```
cd ~/catkin_ws
./src/eDO_control_v3/start.bash
source devel/setup.bash
```


##### Calibrate the robot.
Turn on the robot.

From the server side open a terminal.

```
roslaunch edo_control calibrate.launch
```
This launches a calibration node. After several seconds the robot joints should be able to be moved with the keyboard arrows. Pressing `enter` goes to the next joint and `-` goes to previous.

##### Calibrate TOP or SIDE camera
Launch the single camera node so it publishes the RGB-D feed to ROS.

For camera top: 
```
roslaunch realsense2_camera rs_camera.launch camera:=camera_top serial_no:=f0190391 align_depth:=true filters:=spatial,pointcloud initial_reset:=true
```
For camera side: 
```
roslaunch realsense2_camera rs_camera.launch camera:=camera_side serial_no:=f0191108 filters:=spatial,pointcloud initial_reset:=true
```

Another terminal start-up Moveit:
```
roslaunch edo_moveit_gripper demo.launch
```
Follow [MoveIt Calibration Tutorial](https://ros-planning.github.io/moveit_tutorials/doc/hand_eye_calibration/hand_eye_calibration_tutorial.html) and calibrate the camera. Some notes:
 - Print ChArUco
 - During Context/Object frame, the ChArUco wont be notizable until the board is been seen by the camera.
 - Calibration can be obtained after 5 samplings. But at least 30 samplings are recommended.

Once the calibration is done, click `Save camera pose` and override the corresponding pose in the package [Main]/src/[top/side]_camera_pose.launch

##### Launch Robotic system

Before running any ROS launch, execute the [TensorFlow server] on the server.

```
cd ~/catkin_ws/src/tf_server
python3 communication_handler.py
```
In three different terminals launch the next roslaunch files.

``` 
# Launches MoveIt package, create the environment and table for the robot, and execute the decision making pipeline
roslaunch main robot_system.launch
```

``` 
# Starts the top camera, publishes the TF frame of the camera, and connects to Tensor Flow server so it can detect objects and publishes them to MoveIt planning scene.
roslaunch main top_camera.launch
```

``` 
# Starts side camera, publishes the TF frame of the camera. This enables an octomap server and create a occupancy map in the planning scene. This will enable any collision avoidance during any execution.
roslaunch main side_camera.launch
```

## Known issues/bugs
 - Robot randomly brakes its joints. Source of the issue is unknown.
 - Camera nodes dies after been executed for a while. Real sense node manager have time missmatch with the system. Unable to resolve.
 - System wont always be able to find grasping position and trajectories. Solution could be to add tolerance when finding positions using the OMPL library.
 - `rospy.Times.now()` generate timestamps that are 5s ahead of the system internal clock. This could be the cause of the first and second bug.


[//]: # (These are reference links used in the body of this note and get stripped out when the markdown processor does its job. There is no need to format nicely because it shouldn't be seen. Thanks SO - http://stackoverflow.com/questions/4823468/store-comments-in-markdown-syntax)

   [Main]: <https://github.com/Alfredonator/main>
   [edo_moveit_gripper]: <https://github.com/Alfredonator/edo_moveit_gripper>
   [eDO_description]: <https://github.com/Alfredonator/eDO_description>
   [eDO_control_v3]: <https://github.com/Alfredonator/eDO_control_v3>
   [Decision Making Pipeline]: <https://github.com/Alfredonator/grasper>
   [Camera Node]: <https://github.com/Alfredonator/camera_node_v4>
   [edo_gripper]: <https://github.com/Alfredonator/edo_gripper>
   [moveit_grasps]: <https://github.com/Alfredonator/moveit_grasps>
   [TensorFlow server]: <https://github.com/Alfredonator/tf_server>
   [moveit_calibration]: <https://github.com/Alfredonator/moveit_calibration>
   [geometry_representation]: <https://github.com/Alfredonator/geometry_representation>
   [eDO core msgs]: <https://github.com/Alfredonator/eDO_core_msgs>
   [RVIZ visual tools]: <https://github.com/Alfredonator/rviz_visual_tools>

   

