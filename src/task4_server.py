#!/usr/bin/env python3


import rospy
from com2009_msgs.srv import SetBool,SetBoolResponse
from geometry_msgs.msg import Twist

import cv2
from cv_bridge import CvBridge,CvBridgeError
from sensor_msgs.msg import Image

import math

# Variable declaration
service_name = "identify_colour_service"
find_colour = False

pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
vel = Twist()
cvbridge_interface = CvBridge()



def callback_service(service_request):
    global find_colour
    global colour 
    service_response = SetBoolResponse()

    target_degree = math.pi/5
    # check if recive the request signal
    if service_request.request_signal == True:
        # the robot will turn and face to the colour
        print(f"The '{service_name}' Server received a 'true' request") 
        print("--Turn to the cylinder--")
        turning(target_degree,5)

        # and then identify the colour
        print("--Prepare to find the colour--")
        find_colour = True

        # after identify turning back
        print("--Turning back now--")
        turning(-target_degree,5)
        service_response.response_signal = True
        service_response.response_message = colour

        # Start to navigate
         
    else:
        service_response.response_signal = False
        service_response.response_message = "Error"
    return service_response


def turning(turning_degree,wait_time):
        
        startTime = rospy.get_rostime()
            
        vel.angular.z = turning_degree
        pub.publish(vel)

        # wait for 5 secs
        while (rospy.get_rostime().secs - startTime.secs) < wait_time:
            continue
            
        # stop the robot
        print("---Stop the robot now---")
        vel.angular.z = 0.0
        pub.publish(vel)


def callback_camera(img_data):
    global find_colour
    global colour 
    # Thresholds for ["Blue", "Red", "Green", "Turquoise", "Yellow", "Purple"]
    #lower_list = [(115, 224, 100), (0, 185, 100), 
    # (25, 150, 100), (75, 150, 100), (25,225,100), (145,225,100)]
    #upper_list = [(130, 255, 255), (10, 255, 255), 
    # (70, 255, 255), (100, 255, 255), (35,255,255), (155,255,255)]
    
    blue_lower = (115, 224, 100)
    blue_upper = (130, 255, 255)
    red_lower = (0, 185, 100)
    red_upper = (10, 255, 255)
    green_lower = (25, 150, 100)
    green_upper = (70, 255, 255)
    turquoise_lower = (75, 150, 100)
    turquoise_upper = (100, 255, 255)
    yellow_lower = (25, 225, 100)
    yellow_upper = (35, 255, 255)
    purple_lower = (145, 225, 100)
    purple_upper = (155, 255, 255)


    try:
        cv_img = cvbridge_interface.imgmsg_to_cv2(img_data, desired_encoding="bgr8")
    except CvBridgeError as e:
        print(e)

    height, width,_ = cv_img.shape
    crop_width = width - 800
    crop_height = 400
    crop_x = int((width/2) - (crop_width/2))
    crop_y = int((height/2) - (crop_height/2))

    crop_img = cv_img[crop_y:crop_y+crop_height, crop_x:crop_x+crop_width]
    hsv_img = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)    
    # create a single mask to accommodate all six dectection colours:
    blue = cv2.inRange(hsv_img, blue_lower, blue_upper).mean(axis=0).mean(axis=0)
    red = cv2.inRange(hsv_img, red_lower, red_upper).mean(axis=0).mean(axis=0)
    green = cv2.inRange(hsv_img, green_lower, green_upper).mean(axis=0).mean(axis=0)
    turquoise = cv2.inRange(hsv_img, turquoise_lower, turquoise_upper).mean(axis=0).mean(axis=0)
    yellow = cv2.inRange(hsv_img, yellow_lower, yellow_upper).mean(axis=0).mean(axis=0)
    purple = cv2.inRange(hsv_img, purple_lower, purple_upper).mean(axis=0).mean(axis=0)
      
    # Start to find the colour
    while find_colour == True:
        if (int(blue) == 255):
            colour = "Blue"
        elif (int(red) == 255):
            colour = "Red"
        elif (int(green) == 255):
            colour = "Green"
        elif (int(yellow) == 255):
            colour = "Yellow"
        elif (int(turquoise) == 255):
            colour = "Turquoise"
        elif (int(purple) == 255):
            colour = "Purple"
        else:
            colour = "can't read the colour"
            
        # Give a feedback
        print (f"SRARCH INITIATED: The target beacon colour is {colour}")
        find_colour = False

rospy.Subscriber("/camera/rgb/image_raw", Image, callback_camera)
rospy.init_node(f"{service_name}_server")
my_service = rospy.Service(service_name, SetBool, callback_service)

rospy.loginfo(f"the '{service_name}' Server is ready to be called...")
rospy.spin()
