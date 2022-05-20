#!/usr/bin/env python3
 
import rospy
import roslaunch
from tf.transformations import euler_from_quaternion
from cv_bridge import CvBridge,CvBridgeError
from sensor_msgs.msg import Image,LaserScan
from geometry_msgs.msg import Twist
from pathlib import Path
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import numpy as np
import time
import math
import cv2
import argparse

class Task5:

    def __init__(self):

        node_name = "task5"
        rospy.init_node(node_name, anonymous=True)

        self.odom_subscriber = rospy.Subscriber("/odom", Odometry, self.callback_odom)
        self.camera_subscriber = rospy.Subscriber("/camera/rgb/image_raw", Image, self.callback_camera)
        self.lidar_subscriber = rospy.Subscriber('/scan', LaserScan, self.callback_lidar)
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.cvbridge_interface = CvBridge()
        self.find_colour = True
        self.colour = "" 
        self.startup = True

        self.base_image_path = Path("../snaps")
        self.base_image_path.mkdir(parents=True, exist_ok=True)
        self.waiting_for_image = True

        self.map_path = "$(find team2)/maps/task5_map"
        self.launch = roslaunch.scriptapi.ROSLaunch()
        self.launch.start()

        # Command-Line Interface:
        cli = argparse.ArgumentParser(description=f"Command-line interface for the task5 node.")
        cli.add_argument("-target_colour", metavar="COL", type=String, help="The colour of target beacon.")
       
        # obtain the arguments passed to this node from the command-line:
        self.args = cli.parse_args(rospy.myargv()[1:])

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
        front_left_arc = lidar_data.ranges[0:35]
        front_right_arc = lidar_data.ranges[-10:]
        front_arc = np.array(front_left_arc + front_right_arc)

        # get the right side of the robot
        left_left_arc = lidar_data.ranges[0:35]
        left_right_arc = lidar_data.ranges[35:70]
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
        
    def get_width(self):
        if self.got_frame:
            try:
                cv_img = self.cvbridge_interface.imgmsg_to_cv2(self.img_data, desired_encoding="bgr8")
            except CvBridgeError as e:
                print(e)

            _, width,_ = cv_img.shape
            crop_width = width - 800
            self.width = crop_width

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

            if (self.colour == "blue"):
                self.lower = blue_lower
                self.upper = blue_upper
            elif (self.colour == "red"):
                self.lower = red_lower
                self.upper = red_upper
            elif (self.colour == "green"):
                self.lower = green_lower
                self.upper = green_upper
            elif (self.colour == "yellow"):
                self.lower = yellow_lower
                self.upper = yellow_upper
            elif (self.colour == "turquoise"):
                self.lower = turquoise_lower
                self.upper = turquoise_upper 
            elif (self.colour == "purple"):
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
        right_turning_speed = 0.8
        left_turning_speed = 0.2
        moving_speed = 0.15
        front_threshold = 0.35
        side_threshold = 0.3
        count = 0
        image_taken = False
        boundary_time = 0

        while not self.ctrl_c: # and execution_time < 150:

            execution_time = time.time() - start_time

            # dist_current = math.sqrt(((self.x0 - self.x) ** 2) + ((self.y0 - self.y) ** 2))
            if not image_taken:     
                self.beacon_detetction() 
                target_found = self.target_found()

            # if target_found: #and dist_current > 1.0: # not in start zone
            #     self.align_target_move()
            #     self.take_image()
            if self.ctrl_c:
                pass
            else:
                #print("hit")
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

            if execution_time > boundary_time:
                print(f"Saving map at time: {rospy.get_time()}...")
                node = roslaunch.core.Node(package="map_server", node_type="map_saver", args=f"-f {self.map_path}")
                process = self.launch.launch(node)
                boundary_time += 5

            count += 1

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

        self.vel.linear.x = 0

        if self.cy < lower_lim:     
            self.vel.angular.z = -0.1
        else: # self.cy > upper_lim
            self.vel.angular.z = 0.1
        
    def take_image(self):
        full_image_path = self.base_image_path.joinpath(f"the_beacon.jpg")

        image_taken = True
        target_found = False

        return(image_taken, target_found)

        # cv2.imshow(self, img)
        # cv2.waitKey(0)

        # cv2.imwrite(str(full_image_path), img)
        # print(f"Saved an image to '{full_image_path}'\n"
        # f"image dims = {img.shape[0]}x{img.shape[1]}px\n"
        # f"file size = {full_image_path.stat().st_size} bytes")
        #return
        # if self.cy in the middle:
        #     save image
        #     image_taken = True
        #     target_found = False
            
    def main(self):

        self.get_width()

        self.colour = self.args.target_colour.data

        while not self.ctrl_c: 

            #print(f"SEARCH INITIATED: The target beacon colour is {self.colour}")
            self.start_exploration() # call to start exploration

            self.velocity_publisher.publish(self.vel)
            self.rate.sleep()

if __name__ == "__main__":
    ros_node = Task5()
    ros_node.main()









