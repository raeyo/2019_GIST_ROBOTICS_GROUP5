cmake_minimum_required(VERSION 2.8.3)

# folder name: which is actually "ros package"
project(healthcare_robotics_uwb_sub) 

# define compiler type
add_compile_options(-std=c++11) 

find_package(catkin REQUIRED COMPONENTS
	roscpp
	std_msgs
	tf
	
	#serial
  #std_msgs
  	geometry_msgs
  #roslib
  #rostime##
  #sensor_msgs
  
  message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  msgAnchorRanges.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs

)

#catkin_package(
#  INCLUDE_DIRS include
#  CATKIN_DEPENDS roscpp std_msgs geometry_msgs #sensor_msgs
#  DEPENDS Boost
#)

find_package(Boost REQUIRED COMPONENTS filesystem)
#find_package(PCL REQUIRED)

include_directories(
	${catkin_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
	#${PCL_INCLUDE_DIRS}
)


# ros node details and execution
add_executable(hc_uwb_subscriber_node 
	src/hc_uwb_sub.cpp 
)

target_link_libraries(hc_uwb_subscriber_node
	${catkin_LIBRARIES}
	${Boost_LIBRARIES}
	#${PCL_LIBRARIES}
)

#add_dependencies(healthcare_robotics_uwb_sub 
	#${healthcare_robotics_uwb_sub_EXPORTED_TARGETS}
#)


#http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29

