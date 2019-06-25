#!/usr/bin/env python

import rospy
import math
import actionlib
from move_base_msgs.msg import *
from robotics_main.srv import *
from geometry_msgs.msg import Twist
import time

# tag
is_finished = False

# parameter
trash_bin = [0.214,1.096,1.561]
init_pos = [-0.5,0,3.14]
# initialize
rospy.init_node("robotics_main_node",anonymous = True)
get_pos_client = rospy.ServiceProxy('get_image_pos',ImagePos)
pub_cmd = rospy.Publisher('/om_with_tb3/cmd_vel', Twist, queue_size=10)
pickUp_client = rospy.ServiceProxy('pickUp_trash',ObjectType)
sac = actionlib.SimpleActionClient('/om_with_tb3/move_base', MoveBaseAction )
    

def get_trash_pos():
    # 1. from top image get trash postion
    # 2. if no trash return no tresh signal
    # service client
    print("start getting position")
    rospy.wait_for_service('get_image_pos')
    try:
        res = get_pos_client(True)
        print(res.is_trash)
        if res.is_trash ==True:
            return [res.x , res.y,res.dx,res.dy]
        else :
            return None
    except:
        print("Service call failed")

#move to trash
def set_goal(pos_img):
    goal = MoveBaseGoal()
    x = pos_img[0]
    y = pos_img[1]
    angle = pos_img[2]

    # img to map
    x_map = x- 0.25*math.cos(angle) 
    y_map = y- 0.25*math.sin(angle)
    a_map = angle
    print("navigate start goal x:{}, y:{}, a:{}".format(x_map,y_map,a_map))
    
    #set goal
    goal.target_pose.pose.position.x = x_map
    goal.target_pose.pose.position.y = y_map
    goal.target_pose.pose.orientation.z = math.sin(angle/2)
    goal.target_pose.pose.orientation.w = math.cos(angle/2)
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.header.stamp = rospy.Time.now()

    #start listner
    sac.wait_for_server()

    #send goal
    sac.send_goal(goal)

    #finish
    sac.wait_for_result()


    print (sac.get_result())
    return None

def recognize():
    print("start to recognize")
    rospy.wait_for_service('recognize_trash')
    try:
        recognize_client = rospy.ServiceProxy('recognize_trash',GetObjectType)
        res = recognize_client(True)
        return res.type
    except:
        print("fail to call recognize")
    print("fail to recognize")
    return "lego"
        
def pickup_trash(th_type):
    rospy.wait_for_service('pickUp_trash')
    try:
        res = pickUp_client(th_type)
        print(res)
        return res
    except:
        print("fail to call pick up")
    
def place_trash():
    get_closer()
    rospy.wait_for_service('pickUp_trash')
    try:
        res = pickUp_client("place")
        print(res)
        return res
    except:
        print("fail to call place")
    pass

# move forward for 2sec and stop
def get_closer():
    print("get_closer")
    twist = Twist()
    twist.linear.x = 0.1;    twist.linear.y = 0.0;    twist.linear.z = 0.0
    twist.angular.x = 0.0;    twist.angular.y = 0.0;    twist.angular.z = 0.0
    try:
        pub_cmd.publish(twist)
        pub_cmd.publish(twist)
    except:
        time.sleep(1)
        print("get_closer_again")
        pub_cmd.publish(twist)
    time.sleep(2)
    print("stop")
    twist.linear.x = 0.0;    twist.linear.y = 0.0;    twist.linear.z = 0.0
    twist.angular.x = 0.0;    twist.angular.y = 0.0;    twist.angular.z = 0.0
    pub_cmd.publish(twist)
    return None
while is_finished == False:
    # get trash pos or no trash result
    pos_trash = get_trash_pos()
    # set goal if there is trash
    if pos_trash == None:
        is_finished = True
        break
    else :
        dx = pos_trash[2]
        dy = pos_trash[3]
        slope = dy/dx
        angle = math.atan(slope)
        if dx < 0 and dy > 0 :
            angle = angle + math.pi
        elif dx < 0 and dy < 0:
            angle = angle - math.pi
        else:
            pass
        pos_img = [pos_trash[0],pos_trash[1],angle]
        set_goal(pos_img)
    # recognize trash and set trash to middle of turtlebot
    th_type = recognize()
    # pick up the trash
    get_closer()
    pickup_trash(th_type)
    # go to trash bag and place the trash
    set_goal(trash_bin)
    place_trash()
    set_goal(init_pos)
print("there is no trash")
