
#include <ros/ros.h>
#include <boost/asio.hpp>
#include <boost/asio.hpp>
#include <healthcare_robotics_uwb_sub/msgAnchorRanges.h> // custom ros msg: sub
#include <fstream>  // c++ // fstream
#include <sstream> 


void callback(const healthcare_robotics_uwb_sub::msgAnchorRanges::ConstPtr& msg)
{
  //ROS_INFO("%d", msg->tagid);
  printf("Subscribe{hc_uwb_cam} tag%d, %0.0f, %0.0f, %0.0f, %0.0f, \n", msg->tagid, msg->x1, msg->y1, msg->x2, msg->y2);
	
}

// Main ftn                                             
int main(int argc, char **argv) 
{
  // define ros node name
  ros::init(argc, argv, "hc_uwb_subscriber_node");
  ros::NodeHandle n;
  
  // subscribe message for specific topic and call ftn to decode msg 
  ros::Subscriber sub = n.subscribe("/hc_uwb_cam", 1000, callback);
  //note: The 2nd argument is the queue size, in case we are not able to process messages fast enough. In this case, if the queue reaches 1000 messages, we will start throwing away old messages as new ones arrive. 
  
  //ros::Publisher pub = n.advertise<hc_uwb_tb3_driver::msgAnchorRanges>("/hc_uwb", 1000);
  
  // continue: go next for next iteration or Donot stop
  ros::spin();
  // note: ros::spin() enters a loop, calling message callbacks as fast as possible. Don't worry though, if there's nothing for it to do it won't use much CPU. ros::spin() will exit once ros::ok() returns false, which means ros::shutdown() has been called, either by the default Ctrl-C handler, the master telling us to shutdown, or it being called manually. 
  
	return 0;
}
	
