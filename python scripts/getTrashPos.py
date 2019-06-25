import cv2 
import numpy as np
import rospy
from robotics_main.srv import *
import time
import math
#parameter
scale_x = 0.372
scale_y = 0.366
img_offset = [-3.569,1.853]
#---------------------------------------------------------#
KernelOpen=np.ones((5,5))
KernelClose=np.ones((5,5))
Lego_MIN = np.array([0,133,167])
Lego_MAX = np.array([179,216,255])
BLACK_MIN = np.array([60, 0, 0])
BLACK_MAX = np.array([134, 98, 63])
#---------------------------------------------------------#
# output parameter
is_getPos = False
x_th = 0.0
y_th = 0.0

def img_to_map(pos_img):
    x_img = pos_img[0]
    y_img = pos_img[1]
    x_map = scale_x*x_img
    y_map = scale_y*y_img
    y_map = (-1)*y_map
    x_map = x_map/100 + img_offset[0]
    y_map = y_map/100 + img_offset[1]
    return [x_map, y_map]
def find_turtlebot(img):
    # find_by color_black
    # turtle bot area (100*100)
    turtlebot_area = 10000
    turtlebot_pos = [0.0,0.0]
    dif = 0
    
    
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask_black = cv2.inRange(hsv, BLACK_MIN, BLACK_MAX)
    mask_dilate_black = cv2.dilate(mask_black,KernelClose,iterations = 2)
    mask_final_black=cv2.morphologyEx(mask_dilate_black,cv2.MORPH_OPEN,KernelOpen)
  
    _,contours_black,_ = cv2.findContours(mask_final_black.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
    for ind,contour in enumerate(contours_black):
        x_black,y_black,w_black,h_black = cv2.boundingRect(contour)
        area = w_black*h_black
        pre_dif = (turtlebot_area-area)**2   
        if ind == 0 or pre_dif<dif:
            dif = pre_dif
            turtlebot_pos[0] = x_black+w_black/2
            turtlebot_pos[1] = y_black+h_black/2
    
    # cv2.drawContours(img,contours,-1,(0,255,0),3)
    return turtlebot_pos

def find_trashes(img):
    hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
    mask_trash = cv2.inRange(hsv,Lego_MIN,Lego_MAX)
    mask_dilate_trash = cv2.dilate(mask_trash,KernelClose,iterations=2)
    mask_final_trash = cv2.morphologyEx(mask_dilate_trash,cv2.MORPH_OPEN,KernelOpen)
    return mask_final_trash

def get_nearest_pos(contours, turtlebot_pos):
    shortest_length = 0
    nearest_ind = 0
    nearest_pos = [0.0,0.0]
    for ind,contour in enumerate(contours):
        cx,cy,w,h = cv2.boundingRect(contour)
        length = (turtlebot_pos[0]-cx)**2+(turtlebot_pos[1]-cy)**2
        if ind == 0 or length < shortest_length:
            nearest_ind = ind
            shortest_length = length
            nearest_pos = [cx+w/2,cy+h/2] 
    return [nearest_pos , nearest_ind] 

def find_pos(img):
    turtlebot_pos = find_turtlebot(img)
    mask_trash = find_trashes(img)
    # contours of trashes
    _,contours,_ = cv2.findContours(mask_trash,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
    if len(contours)==0:
        return None    
    # get nearest trash's position
    trash_pos, nearest_ind = get_nearest_pos(contours,turtlebot_pos)
    cv2.drawContours(img,[contours[nearest_ind]],-1,(0,255,0),3)
    cv2.imshow("find trash",img)
    dx = trash_pos[0]-turtlebot_pos[0]
    dy = trash_pos[1]-turtlebot_pos[1]

    pos_img = [trash_pos[0],trash_pos[1],dx,dy] 
    return pos_img

def get_image_pos(req):
    # image processing
    # camera setting camera_index is 0 or 1
    camera_index = 0

    try:
        cap = cv2.VideoCapture(camera_index)
    except:
        print("error")
    cap.set(3,1920)
    cap.set(4,1080)
    time.sleep(2)
    ret, frame = cap.read()
    img = frame[100:950, 450:1400]
    if not ret:
        print("error ret")
    
    cv2.imshow("image",img)
    trash_pos = find_pos(img)
    
        
    if trash_pos ==None:
       # return ImagePosResponse(True,-2.0,-0.1,0.1,0.)
        return ImagePosResponse(False,0.,0.,0.,0.)
    pos_img = trash_pos[:2]
    
    dir_img = trash_pos[2:]
    pos_map = img_to_map(pos_img)
    dir_map = [scale_x*dir_img[0]/100,-scale_y*dir_img[1]/100]
    print(pos_map)
    print(dir_map)
    cv2.destroyAllWindows()
    return ImagePosResponse(True,pos_map[0],pos_map[1],dir_map[0],dir_map[1])
def nothing(req):
    print("start to get the pos of trash")
    return ImagePosResponse(True,-2.0,-0.1,0.1,0.)
# init service 
if __name__=="__main__":
    print("start")
    rospy.init_node('get_imagePos_server')
    s = rospy.Service('get_image_pos',ImagePos,get_image_pos)
    rospy.spin()
