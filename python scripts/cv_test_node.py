#!/usr/bin/env python

import rospy
import cv2
import math
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from robotics_main.srv import *
from geometry_msgs.msg import Twist
import time
# from cv_test3.srv import GetImageInfo

cv_image = None
bridge = CvBridge()
is_finish = True
mid_length = 10

pub_cmd = rospy.Publisher('/om_with_tb3/cmd_vel', Twist, queue_size=10)
def image_callback(img_msg):
    global is_finish
    if is_finish == False:
        #rospy.loginfo(img_msg.header)

        try:
            cv_image = bridge.imgmsg_to_cv2(img_msg,"bgr8")
            
        except CvBridgeError, e:
            rospy.logerr("CvBridge Error: {0}".format(e))
        cv2.imshow("Image Window",cv_image)
        k = cv2.waitKey(3)
        y_img,x_img,_ = cv_image.shape
        img = cv_image.copy()
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower_blue = np.array([0, 74, 0])
        upper_blue = np.array([179, 255, 255])
        mask = cv2.inRange(hsv, lower_blue, upper_blue)
        cv2.imshow("mas",mask)
        _,contours_blue,_ = cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
        trash_ind = 0
        max_area = 0
        pos_tr = [0.0,0.0]
        for ind,contour in enumerate(contours_blue):
            cx,cy,w,h = cv2.boundingRect(contour)
            if w*h > max_area:
                pos_tr = [cx+w/2,cy+h/2]
                max_area = w*h
                print(cx,cy)
                trash_ind = ind
        cv2.drawContours(img,[contours_blue[trash_ind]],-1,(0,255,0),3)
        cv2.imshow("mask",img)        
        len_x = [x_img/2-mid_length,x_img/2+mid_length]
        pos = None
        if pos_tr[0] < len_x[0]:
            pos = "left"
        elif pos_tr[0] > len_x[1]:
            pos = "right"
        else:
            pos = "mid"
        twist = Twist()
        if pos == "right":
            twist.linear.x = 0.0;    twist.linear.y = 0.0;    twist.linear.z = 0.0
            twist.angular.x = 0.0;    twist.angular.y = 0.0;    twist.angular.z = -0.1
            pub_cmd.publish(twist)
            print("right")
        elif pos == "left":
            twist.linear.x = 0.0;    twist.linear.y = 0.0;    twist.linear.z = 0.0
            twist.angular.x = 0.0;    twist.angular.y = 0.0;    twist.angular.z = 0.1
            pub_cmd.publish(twist)
            print("left")
        elif pos == "mid":
            twist.linear.x = 0.0;    twist.linear.y = 0.0;    twist.linear.z = 0.0
            twist.angular.x = 0.0;    twist.angular.y = 0.0;    twist.angular.z = 0.0
            pub_cmd.publish(twist)
            print("mid")
            is_finish = True
            cv2.waitKey(0)
            cv2.destroyAllWindows()
    else:
        print("waiting for service call...")

    

def get_img_pos(req):
    global is_finish
    print("get image from raspi_cam")
    is_finish = False
    while is_finish == False:
        continue
    print("end to set middle")
    return GetObjectTypeResponse("lego")



if __name__ == "__main__":
    print("start")
    # initialize
    rospy.init_node('opencv_test_node', anonymous=True)
    
    image_sub = rospy.Subscriber("/image_raw", Image, image_callback)
    goal_server = rospy.Service('recognize_trash', GetObjectType, get_img_pos)  

    rospy.spin()

