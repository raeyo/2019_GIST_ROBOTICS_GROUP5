# 2019_GIST_ROBOTICS_GROUP5
2019_Spring_Robotics_Project_Repository_Group5
# Overview
- Objective : Make autonomous pick trash and place them in trashbin
- Platform : Turtlebot3 with open-manipulator
- Used Sensors : 2D_LIDAR, webcam, raspicam, uwb sensor
- Environment : ROS-kinetic, opencv3
# Methodology
- Initialize position of turtlebot : Using uwb sensor
- Get map of competition room : SLAM tool for turtlebot3
- Navigation : Navigation tool for turtlebot3 
- Get trash position : image processing(color detection) , ceiling camera
- Set the position of turtlebot3 to pickup trash : image processing(color detection), raspicam, extra equipment
- Pick up trash : saved joint value state for pickup,lego,cup etc... and execute them
# Results
- Task1 : How to find initial position and heading of platform 
  - using uwb sensor to get initial position of turtlebot
  - commend to turtlebot going forward until get 1000 position of turtlebot
  - calculate linear regression equation to get slope of turtlebot 
- Task2 : How to Detect 9 trash-objects
  - we only detect 3 of legos
  - get image from ceiling camera
  - find lego by color detect algorithm 
- Task3 : How to Navigate platform to each trash-object
  - use Navigation tool for turtlebot3
  - set goal position : get position of trash from image processing of ceiling camera 
  - set goal angle : get direction from turtlebot to trash
  - set goal pos and angle in SLAM_map coordinate : calculate transform equation by empirical method
- Task4 : How to Pick and drop the trash-object into trash-bin
  - save the trajectory of joint state like PickUp_state, Place_state, init_state etc...
  - only can pick up the trash locate at the right front of turtlebot
  - set the turtlebot direction to locate the trash front of turtlebot : use image processing from raspicam 
  - set the trash(lego) orientation : use extra equipment
# How to Improve Task
- more delicate image processing or deep learning algorithm to detect all trashes and recognize them.
  - we only find lego from ceiling image and raspicam
  - use raspicam to recognize is fail and also there is many errors when detect only lego
- more precise transform equation(uwb_to_map, ceilingImg_to_map) 
  - it's hard to get precise position of trash from image.
- when try each script respectively it works, but try together it's hard to work..., optimize the communication of each program is needded
  - fail to call service error 
  - commend to go forward not active sometimes
  - etc...
# How to run the code
- [remote PC] [Terminal_1] roscore 
  - roscore
- [turtlebot] [Terminal_1] ROS_NAMESPACE=om_with_tb3 roslaunch turtlebot3_bringup turtlebot3_robot.launch multi_robot_name:=om_with_tb3 set_lidar_frame_id:=om_with_tb3/base_scan
  - to bring up sensor data from turtlebot 
- [turtlebot] [Terminal_2] ROS_NAMESPACE=om_with_tb3 roslaunch turtlebot3_bringup turtlebot3_rpicamera.launch
  - to bring up camera image from raspicam
- [turtlebot] [Terminal_3] roslaunch healthcare_robotics_uwb healthcare_robotics_uwb_loc.launch
  - to bring up uwb data from turtlebot
- [remote PC] [Terminal_2] ROS_NAMESPACE=om_with_tb3 roslaunch open_manipulator_with_tb3_tools om_with_tb3_robot.launch
  - to bring up sensor data from turtlebot
- [remote PC] [Terminal_3] rosrun image_transport republish compressed in:=/om_with_tb3/raspicam_node/image raw out:=/image_raw
  - to transport compressed image to raw image
- [remote PC] [Terminal_4] roslaunch open_manipulator_with_tb3_tools manipulation.launch open_rviz:=false
  - to activate manipulation move group
- [remote PC] [Terminal_5] roslaunch healthcare_robotics_uwb_sub uwb_sub_py.launch
  - by using uwb sensor initialize the position and orientation of turtlebot and launch navigation tool
- [remote PC] [Terminal_6] python getTrashPos.py
  - service server for get nearest trash position from ceiling camera image
- [remote PC] [Terminal_7] python cv_test_node.py
  - service server for setting turtlebot direction to middle of trash by using raspicam 
- [remote PC] [Terminal_8]rosrun kong_test kong_test_node.py
  - service server for manipulation which execute saved trajectory for each trash
- [remote PC] [Terminal_9]rosrun robotics_main robotics_main_node.py
  - main node 



