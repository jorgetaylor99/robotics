#!/usr/bin/env python3

from logging import shutdown
from termios import VEOL
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from math import pi
import numpy as np
import time

class Task2:

    def __init__(self):
        node_name = "task2"
        rospy.init_node(node_name, anonymous=True)

        self.velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.lidar_subscriber = rospy.Subscriber('scan', LaserScan, self.callback_lidar)

        self.front_min = 999
        self.left_min = 999
        self.right_min = 999

        self.rate = rospy.Rate(10) # hz        
        self.vel = Twist()

        self.ctrl_c = False

        rospy.on_shutdown(self.shutdownhook)
        rospy.loginfo(f"the {node_name} node has been initialised...")

    def callback_lidar(self, lidar_data):

        # get the left side of the robot
        left_left_arc = lidar_data.ranges[0:20]
        left_right_arc = lidar_data.ranges[20:40]
        left_arc = np.array(left_left_arc + left_right_arc)

        # get the right side of the robot
        right_left_arc = lidar_data.ranges[-20:]
        right_right_arc = lidar_data.ranges[320:340]
        right_arc = np.array(right_left_arc + right_right_arc)

        # get the closest distance from the arcs
        self.left_min = left_arc.min()
        self.right_min = right_arc.min()

    def shutdownhook(self):
        self.velocity_publisher.publish(Twist())
        self.ctrl_c = True
    
    def print_lidar(self):
        print(f"front min: {self.front_min}, left min: {self.left_min}, right min: {self.right_min}")

    def main_loop(self):

        threshold = 0.75
        multiplier = 1

        while not self.ctrl_c:
            
            if self.right_min < threshold:
                self.vel.linear.x = 0.2
                self.vel.angular.z = multiplier * (1 / self.right_min)
            elif self.left_min < threshold:
                self.vel.linear.x = 0.2
                self.vel.angular.z = -multiplier * (1 / self.left_min)
            else:
                self.vel.angular.z = 0
                
            self.vel.linear.x = 0.3

            self.velocity_publisher.publish(self.vel)
            self.rate.sleep()

if __name__ == '__main__':
    task2_instance = Task2()
    try:
        task2_instance.main_loop()
    except rospy.ROSInterruptException:
        pass