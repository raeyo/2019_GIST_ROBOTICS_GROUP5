#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import String
from healthcare_robotics_uwb_sub.msg import msgAnchorRanges
from geometry_msgs.msg import Twist
# import geometry_msgs.msg import PoseWithCovarianceStamped
import time
import numpy as np
import subprocess
# dist = [0.0,0.0,0.0,0.0]
uwb_offset = [-0.26,5.33]
x0, y0 = 0, 0
pos_x = np.empty(0)
pos_y = np.empty(0)
count = 0
alpha = 0.01
is_initialize = False
pub_cmd = rospy.Publisher('/om_with_tb3/cmd_vel', Twist, queue_size=10)
#twist = Twist()
#rc100 is useful only when using bluetooth-based controller
def linearRegrssion(x_ar,y_ar):
    m = (len(x_ar)*np.sum(x_ar*y_ar)-np.sum(x_ar)*np.sum(y_ar))/(len(x_ar)*np.sum(x_ar*x_ar)-(np.sum(x_ar))**2)   
    return m
    
def callback(data):
    global x0,y0,count,pub_cmd, is_initialize, pos_x,pos_y
    if is_initialize == True:
        return
    if count == 0:
        print("start")
        x0 = data.x1
        y0 = data.y1
        # command to move forward
        twist = Twist()
        twist.linear.x = 0.06;    twist.linear.y = 0.0;    twist.linear.z = 0.0
        twist.angular.x = 0.0;    twist.angular.y = 0.0;    twist.angular.z = 0.0
        pub_cmd.publish(twist)
        count+=1    
        return
    if count == 1000:
        print("stop")
        is_initialize = True
        slope = linearRegrssion(pos_x,pos_y)
        print(slope)
        # stop command
        twist = Twist()
        twist.linear.x = 0.0;    twist.linear.y = 0.0;    twist.linear.z = 0.0
        twist.angular.x = 0.0;    twist.angular.y = 0.0;    twist.angular.z = 0.0
        pub_cmd.publish(twist)
        # launch navigation
        x1 = data.x1
        y1 = data.y1
        dx = x1-x0
        dy = y1-y0
        # -pi/2<math.atan()<pi/2, we want -pi~pi angle
        angle = math.atan(slope)
        if dx > 0 and dy < 0 :
            angle = angle + math.pi
        elif dx > 0 and dy > 0:
            angle = angle - math.pi
        else:
            pass
        package = 'open_manipulator_with_tb3_tools'
        launch_file = 'navigation.launch'
        set_x = 'initial_pose_x:={}'.format(-x1/100 + uwb_offset[0])
        set_y = 'initial_pose_y:={}'.format(-y1/100 + uwb_offset[1])
        set_angle = 'initial_pose_a:={}'.format(angle)
        print(set_x,set_y,set_angle)
        command = "roslaunch {} {} use_platform:=true map_file:=$HOME/map.yaml {} {} {}".format(package,launch_file,set_x,set_y,set_angle)
        p = subprocess.Popen(command, shell=True)
        state = p.poll()
        if state is None:
            rospy.loginfo("process is running fine")
        elif state < 0:
            rospy.loginfo("Process terminated with error")
        elif state > 0:
            rospy.loginfo("Process terminated without error")
        is_initialize = True
    pos_x = np.append(pos_x,data.x1)
    pos_y = np.append(pos_y,data.y1)
        
    count += 1
    
def uwb_sub_node():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('uwb_sub_node', anonymous=True)

    rospy.Subscriber('hc_uwb_cam', msgAnchorRanges, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    
    
if __name__ == '__main__':
    uwb_sub_node()

