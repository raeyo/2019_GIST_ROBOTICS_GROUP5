# 2019_GIST_ROBOTICS_GROUP5
2019_Spring_Robotics_Project_Repository_Group5
# How to run the code
[remote PC] [Terminal_1] roscore

[turtlebot] [Terminal_1] ROS_NAMESPACE=om_with_tb3 roslaunch turtlebot3_bringup turtlebot3_robot.launch multi_robot_name:=om_with_tb3 set_lidar_frame_id:=om_with_tb3/base_scan

[turtlebot] [Terminal_2] ROS_NAMESPACE=om_with_tb3 roslaunch turtlebot3_bringup turtlebot3_rpicamera.launch

[turtlebot] [Terminal_3] roslaunch healthcare_robotics_uwb healthcare_robotics_uwb_loc.launch

[remote PC] [Terminal_2] ROS_NAMESPACE=om_with_tb3 roslaunch open_manipulator_with_tb3_tools om_with_tb3_robot.launch

[remote PC] [Terminal_3] rosrun image_transport republish compressed in:=/om_with_tb3/raspicam_node/image raw out:=/image_raw

[remote PC] [Terminal_4] roslaunch open_manipulator_with_tb3_tools manipulation.launch open_rviz:=false

[remote PC] [Terminal_5] roslaunch healthcare_robotics_uwb_sub uwb_sub_py.launch
 
[remote PC] [Terminal_6] python getTrashPos.py

[remote PC] [Terminal_7] python cv_test_node.py

[remote PC] [Terminal_8]rosrun kong_test kong_test_node.py

[remote PC] [Terminal_9]rosrun robotics_main robotics_main_node.py
