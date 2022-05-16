#!/usr/bin/env python3
 
import rospy
from com2009_msgs.srv import SetBool,SetBoolResponse, SetBoolRequest
from geometry_msgs.msg import Twist
import cv2
from cv_bridge import CvBridge,CvBridgeError
from sensor_msgs.msg import Image,LaserScan
import numpy as np
import time
from nav_msgs.msg import Odometry
import math

class Task4:
    def __init__(self):
        rospy.init_node("identify_colour_service_server")
        rospy.loginfo("Starting Task4.")
        my_service = rospy.Service("identify_colour_service", SetBool, self.callback_service)

        rospy.Subscriber("/odom", Odometry, self.odom_cb)
        rospy.Subscriber("/camera/rgb/image_raw", Image, self.callback_camera)
        rospy.Subscriber('scan', LaserScan, self.callback_lidar)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        self.exploration_trigger = rospy.ServiceProxy('exploration/enable', SetBool)

        self.cvbridge_interface = CvBridge()
        self.find_colour = True
        self.colour = "" 
        
        self.lower = (0,0,0)
        self.upper = (0,0,0)
        self.cy = 0.0
        self.m00 = 0
        self.m00_min = 100000
        self.trigger_detection = False
        self.color_detected = False
        self.img_data = Image
        self.got_frame = False
        self.position_x = 0.0
        self.position_y = 0.0        
        
        self.init_position_x = 0.0
        self.init_position_y = 0.0
        self.width = 0.0
        self.lidar_data = LaserScan()


    def odom_cb(self, msg):
        self.position_x = msg.pose.pose.position.x
        self.position_y = msg.pose.pose.position.y

    def callback_camera(self, msg):
        """
        Create common callback function to subscribe image msgs and store it in global variable self.img_data
        to use for the diffrent membur functions e.g detect_color and beacon_detetction
        """
        self.img_data = msg 
        self.got_frame = True # flag to check if new data received
        
    def detect_color(self):
        if self.got_frame: # process only if new data is received in topic
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
                cv_img = self.cvbridge_interface.imgmsg_to_cv2(self.img_data, desired_encoding="bgr8")
            except CvBridgeError as e:
                print(e)

            height, width,_ = cv_img.shape
            crop_width = width - 800
            crop_height = 400
            self.width = crop_width
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
            # while self.find_colour == True:
            if (int(blue) == 255):
                self.colour = "Blue"
                self.color_detected = True
            elif (int(red) == 255):
                self.color_detected = True
                self.colour = "Red"
            elif (int(green) == 255):
                self.color_detected = True
                self.colour = "Green"
            elif (int(yellow) == 255):
                self.color_detected = True
                self.colour = "Yellow"
            elif (int(turquoise) == 255):
                self.color_detected = True
                self.colour = "Turquoise"
            elif (int(purple) == 255):
                self.color_detected = True
                self.colour = "Purple"
            else:
                self.color_detected = False
                self.colour = "can't read the self.colour"
                
            # Give a feedback
            print (f"SRARCH INITIATED: The target beacon colour is {self.colour}")
            self.got_frame = False #re unset flag

    def beacon_detetction(self):
        if self.got_frame: # process only if new data is received in topic
            print(self.colour, self.cy)

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
                cv_img = self.cvbridge_interface.imgmsg_to_cv2(self.img_data, desired_encoding="bgr8")
            except CvBridgeError as e:
                print(e)

            height, width, _ = cv_img.shape
            print(height, width)
            crop_width = width - 1120
            crop_height = 120
            crop_x = int((width/2) - (crop_width/2))
            crop_y = int((height/2) - (crop_height/2))

            crop_img = cv_img[crop_y:crop_y+crop_height, crop_x:crop_x+crop_width]
            hsv_img = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

            if (self.colour == "Blue"):
                self.lower = blue_lower
                self.upper = blue_upper
            elif (self.colour == "Red"):
                self.lower = red_lower
                self.upper = red_upper
            elif (self.colour == "Green"):
                self.lower = green_lower
                self.upper = green_upper
            elif (self.colour == "Yellow"):
                self.lower = yellow_lower
                self.upper = yellow_upper
            elif (self.colour == "Turquoise"):
                self.lower = turquoise_lower
                self.upper = turquoise_upper 
            elif (self.colour == "Purple"):
                self.lower = purple_lower
                self.upper = purple_upper 
            else:
                print("Can't detect colour")
            mask = cv2.inRange(hsv_img, self.lower, self.upper)
            m = cv2.moments(mask)
                    
            self.m00 = m["m00"]
            self.cy = m["m10"] / (m["m00"] + 1e-5)

            if self.m00 > self.m00_min:
                cv2.circle(crop_img, (int(self.cy), int(crop_height/2)), 10, (0, 0, 255), 2)
                
                
            cv2.imshow("cropped image", crop_img)
            cv2.waitKey(1)
            self.got_frame = False #re unset flag


    def toggle_exploration(self, req):
        trigger = SetBoolRequest()
        trigger.request_signal = req
        # if req == False :
        #     self.pub_cmd_vel(0,0,0.0)
        try:
            self.exploration_trigger.call(req)
        except:
            pass

    def callback_service(self, req):
        self.trigger_detection = req.request_signal
        print("exploration trigger: ", self.trigger_detection)
        

    def pub_cmd_vel(self, vx, vy, vyaw):
            
        vel = Twist()
        vel.linear.x = vx
        vel.linear.y = 0.0
        vel.linear.z = 0.0
        vel.angular.x = 0.0
        vel.angular.y = 0.0
        vel.angular.z = vyaw

        self.pub.publish(vel)

    def target_found(self):

        if self.m00 > self.m00_min:
            print (f"TARGET DETECTED: The target beacon colour is {self.colour}")

            # blob detected
            # print("TARGET FOUND")
            return True
        else:
            # print(f"MOVING FAST: I can't see anything at the moment (blob size = {self.m00:.0f}), scanning the area...")
            # print("TARGET NOT FOUND")
            return False

    def align_target_move(self):
        lower_lim = (self.width/2)-((25/100)*self.width)
        upper_lim = (self.width/2)+((25/100)*self.width)
        print("lower_lim ", lower_lim, " :: " "upper_lim ", upper_lim, " :: ", "self.cy ", self.cy)
        if not (self.cy >= lower_lim and self.cy <= upper_lim):
            
            # self.pub_cmd_vel(0.0, 0.0, 0.3)
            
            if self.cy < lower_lim:
                self.pub_cmd_vel(0.0, 0.0, 0.5)
            elif self.cy > upper_lim:
                self.pub_cmd_vel(0.0, 0.0, -0.5)

            return False
        else:
            return True
    
    def callback_lidar(self, msg):
        self.lidar_data = msg
        # get the front the robot from a +/- 10 degree arc
        
        
    def move_to_target(self):
        front_left_arc = self.lidar_data.ranges[0:5]
        front_right_arc = self.lidar_data.ranges[-5:]
        front_arc = np.array(front_left_arc + front_right_arc)

        # get the left side of the robot
        #left_left_arc = self.lidar_data.ranges[15:38]
        #left_right_arc = self.lidar_data.ranges[38:60]
        left_left_arc = self.lidar_data.ranges[15:30]
        left_right_arc = self.lidar_data.ranges[30:45]
        left_arc = np.array(left_left_arc + left_right_arc)

        # get the right side of the robot
        right_left_arc = self.lidar_data.ranges[300:323]
        right_right_arc = self.lidar_data.ranges[323:345]
        right_arc = np.array(right_left_arc + right_right_arc)

        # get the closest distance from the arcs
        front_min = front_arc.min()
        left_min = left_arc.min()
        right_min = right_arc.min()


        if front_min < 0.5:
            linear_x = 0.0
            angular_z = 0.0
        # else:
        #     # Obstacle detected in front! Turning away
        #     linear_x = 0.0
        #     angular_z = 0.0
            print (f"BEACONING COMPLETE: The robot has now stopped.")


            self.pub_cmd_vel(linear_x,0,angular_z)

    
    def main(self):
        init = False
        exploration_state = True

        while not rospy.is_shutdown():

            while not rospy.is_shutdown() and not init: # loop to initialize robot with color detection
                self.detect_color() # detect start position color
                self.pub_cmd_vel(0,0,1.0) # while turning the robot heading
                self.init_position_x = self.position_x # get initial position of robot
                self.init_position_y = self.position_y
                print("detecting color")
                if self.color_detected: # check if color is detected
                    print("Color Detected: ",self.color_detected, "  Exploratrion Started") 
                    print (f"SRARCH INITIATED: The target beacon colour is {self.colour}")
                    self.toggle_exploration(True) # call service (Task 2) to start exploration
                    init = True # set flag onde initialised
                    break # break loop

            while not rospy.is_shutdown() and exploration_state==True: #Loop to detection and exploration
                dist_current = math.sqrt( (self.init_position_x-self.position_x)**2 + (self.init_position_y-self.position_y)**2  ) # used to filter the start position detection
                self.beacon_detetction() # function to detect beacon
                target_found = self.target_found() # check if target found
                if target_found and dist_current>1.0: # if target found but its not in the initial location area of radius (1.0 m) ie start zone
                    
                    self.toggle_exploration(False) # turn exploration off if target detected
                    if self.align_target_move(): # alighn target in frame and check once done alignment
                        self.toggle_exploration(True) # toggle exploration on to move to target while avoiding obstacles
                        self.move_to_target()
            



if __name__ == "__main__":
    ros_node = Task4()
    ros_node.main()









