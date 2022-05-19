#!/usr/bin/env python3
 
import rospy
from tf.transformations import euler_from_quaternion
from cv_bridge import CvBridge,CvBridgeError
from sensor_msgs.msg import Image,LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import time
import math
import cv2

class Task4:

    def __init__(self):

        node_name = "task4"
        rospy.init_node(node_name, anonymous=True)

        self.odom_subscriber = rospy.Subscriber("/odom", Odometry, self.callback_odom)
        self.camera_subscriber = rospy.Subscriber("/camera/rgb/image_raw", Image, self.callback_camera)
        self.lidar_subscriber = rospy.Subscriber('/scan', LaserScan, self.callback_lidar)
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.cvbridge_interface = CvBridge()
        self.find_colour = True
        self.colour = "" 
        self.startup = True
        
        self.min = 999 # minimum distance of obstacle in-front of robot
        self.argmin = 25 # index of minimum distance in array 
        self.front_min = 999
        self.left_min = 999

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
        self.lidar_data = LaserScan() # ?

        self.rate = rospy.Rate(10)
        self.vel = Twist()
        self.ctrl_c = False

        rospy.on_shutdown(self.shutdownhook)
        rospy.loginfo(f"The {node_name} node has been initialised!")

    def shutdownhook(self):
        self.velocity_publisher.publish(Twist())
        self.ctrl_c = True

    def callback_lidar(self, lidar_data):

        # get the front arc of the robot
        front_left_arc = lidar_data.ranges[0:3]
        front_right_arc = lidar_data.ranges[-3:]
        front_arc = np.array(front_left_arc + front_right_arc)

        # get the right side of the robot
        left_left_arc = lidar_data.ranges[5:35]
        left_right_arc = lidar_data.ranges[35:65]
        left_arc = np.array(left_left_arc + left_right_arc)

        lock_left_arc = lidar_data.ranges[0:5]
        lock_right_arc = lidar_data.ranges[-5:]
        lock_on_arc = np.array(lock_left_arc + lock_right_arc)

        # get the closest distance from the arcs
        self.front_min = front_arc.min()
        self.argmin = front_arc.argmin()
        self.left_min = left_arc.min()
        self.lock_min = lock_on_arc.min()

    def callback_odom(self, odom_data):
        pos_x = odom_data.pose.pose.position.x
        pos_y = odom_data.pose.pose.position.y

        or_x = odom_data.pose.pose.orientation.x
        or_y = odom_data.pose.pose.orientation.y
        or_z = odom_data.pose.pose.orientation.z
        or_w = odom_data.pose.pose.orientation.w

        (_, _, yaw) = euler_from_quaternion([or_x, or_y, or_z, or_w], 'sxyz')
        
        self.x = pos_x
        self.y = pos_y
        self.theta_z = yaw 

        if self.startup:
            self.startup = False
            self.x0 = self.x
            self.y0 = self.y
            self.theta_z0 = self.theta_z

    def callback_camera(self, img_data):
        self.img_data = img_data
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
            print (f"SEARCH INITIATED: The target beacon colour is {self.colour}")
            self.got_frame = False # re unset flag

    def beacon_detetction(self):

        if self.got_frame: # process only if new data is received in topic
            # print(self.colour, self.cy)

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
            # print(height, width)
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

            self.got_frame = False # re unset flag

    def start_exploration(self):

        start_time = time.time()
        execution_time = 0
        right_turning_speed = 0.5
        left_turning_speed = 0.35
        moving_speed = 0.12
        front_threshold = 0.3
        side_threshold = 0.3

        while not self.ctrl_c: # and execution_time < 150:

            # dist_current = math.sqrt(((self.x0 - self.x) ** 2) + ((self.y0 - self.y) ** 2))     
            self.beacon_detetction() 
            target_found = self.target_found()

            if target_found: #and dist_current > 1.0: # not in start zone
                self.align_target_move()
                self.move_to_target()

            else:
                if self.front_min > front_threshold:
                    # Nothing detected in front, move forward

                    if self.left_min < side_threshold:
                        # Too close to right wall, turn away!
                        self.vel.linear.x = moving_speed
                        self.vel.angular.z = -left_turning_speed
                    
                    else:
                        # Too far away from the right wall, move closer!
                        self.vel.linear.x = moving_speed
                        self.vel.angular.z = right_turning_speed

                else:
                    # Obstacle detected in front, stop and turn away!
                    self.vel.linear.x = 0.0
                    self.vel.angular.z = -right_turning_speed

            self.velocity_publisher.publish(self.vel)
            self.rate.sleep()

    def target_found(self):
        if self.m00 > self.m00_min:
            print (f"TARGET DETECTED: The target beacon colour is {self.colour}")
            return True
        else:
            return False

    def align_target_move(self):

        lower_lim = (self.width/2)-((25/100)*self.width)
        upper_lim = (self.width/2)+((25/100)*self.width)

        print("lower_lim ", lower_lim, " :: " "upper_lim ", upper_lim, " :: ", "self.cy ", self.cy)

        self.vel.linear.x = 0.26

        if self.cy < lower_lim:     
            self.vel.angular.z = 0.25
        else: # self.cy > upper_lim
            self.vel.angular.z = -0.25
        
    def move_to_target(self):
        if self.lock_min < 0.45:
            print (f"BEACONING COMPLETE: The robot has now stopped.")
            self.shutdownhook()
            
    def main(self):

        while not self.ctrl_c: 

            #print(f"SEARCH INITIATED: The target beacon colour is {self.colour}")
            self.start_exploration() # call to start exploration

            self.velocity_publisher.publish(self.vel)
            self.rate.sleep()

if __name__ == "__main__":
    ros_node = Task4()
    ros_node.main()









