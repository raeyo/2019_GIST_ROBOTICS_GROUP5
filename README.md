# 2019_GIST_ROBOTICS_GROUP5
2019_Spring_Robotics_Project_Repository_Group5
# Overview
# Methodology
# Results
# How to run the code
- roscore

[remote PC] [Terminal_1] roscore 

- to bring up sensor data from turtlebot 

[turtlebot] [Terminal_1] ROS_NAMESPACE=om_with_tb3 roslaunch turtlebot3_bringup turtlebot3_robot.launch multi_robot_name:=om_with_tb3 set_lidar_frame_id:=om_with_tb3/base_scan

- to bring up camera image from raspicam

[turtlebot] [Terminal_2] ROS_NAMESPACE=om_with_tb3 roslaunch turtlebot3_bringup turtlebot3_rpicamera.launch

- to bring up uwb data from turtlebot

[turtlebot] [Terminal_3] roslaunch healthcare_robotics_uwb healthcare_robotics_uwb_loc.launch

- to bring up sensor data from turtlebot

[remote PC] [Terminal_2] ROS_NAMESPACE=om_with_tb3 roslaunch open_manipulator_with_tb3_tools om_with_tb3_robot.launch

- to transport compressed image to raw image

[remote PC] [Terminal_3] rosrun image_transport republish compressed in:=/om_with_tb3/raspicam_node/image raw out:=/image_raw

- to activate manipulation move group

[remote PC] [Terminal_4] roslaunch open_manipulator_with_tb3_tools manipulation.launch open_rviz:=false
 
- by using uwb sensor initialize the position and orientation of turtlebot and launch navigation tool

[remote PC] [Terminal_5] roslaunch healthcare_robotics_uwb_sub uwb_sub_py.launch

- service server for get nearest trash position from ceiling camera image

[remote PC] [Terminal_6] python getTrashPos.py

- service server for setting turtlebot direction to middle of trash by using raspicam 

[remote PC] [Terminal_7] python cv_test_node.py

- service server for manipulation which execute saved trajectory for each trash

[remote PC] [Terminal_8]rosrun kong_test kong_test_node.py

- main node 

[remote PC] [Terminal_9]rosrun robotics_main robotics_main_node.py



