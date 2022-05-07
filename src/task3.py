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

class Task3:

    def __init__(self):
        node_name = "task3"
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

        # get the front the robot from a +/- 10 degree arc
        front_left_arc = lidar_data.ranges[0:5]
        front_right_arc = lidar_data.ranges[-5:]
        front_arc = np.array(front_left_arc + front_right_arc)

        # get the left side of the robot
        #left_left_arc = lidar_data.ranges[15:38]
        #left_right_arc = lidar_data.ranges[38:60]
        left_left_arc = lidar_data.ranges[15:30]
        left_right_arc = lidar_data.ranges[30:45]
        left_arc = np.array(left_left_arc + left_right_arc)

        # get the right side of the robot
        right_left_arc = lidar_data.ranges[300:323]
        right_right_arc = lidar_data.ranges[323:345]
        right_arc = np.array(right_left_arc + right_right_arc)

        # get the closest distance from the arcs
        self.front_min = front_arc.min()
        self.left_min = left_arc.min()
        self.right_min = right_arc.min()

    def shutdownhook(self):
        self.velocity_publisher.publish(Twist())
        self.ctrl_c = True
    
    def print_lidar(self):
        print(f"front min: {self.front_min}, left min: {self.left_min}, right min: {self.right_min}")

    def main_loop(self):

        start_time = time.time()
        execution_time = 0

        while not self.ctrl_c and execution_time < 150:

            execution_time = time.time() - start_time
            print(f"Minimum distance from right wall: {self.right_min}, time: {execution_time}")

            if self.front_min > 0.4:
                # Nothing is directly in-front!
                if self.right_min < 0.33:
                    # We are too close to the left wall - back up!
                    self.vel.linear.x = 0
                    self.vel.angular.z = 0.8
                
                elif self.right_min > 0.38:
                    # We are to far away from the left wall - move closer!
                    self.vel.linear.x = 0.25
                    self.vel.angular.z = -0.7

                else:
                    self.vel.linear.x = 0.25
                    self.vel.angular.z = 0.7

            else:
                # Obstacle detected in front! Turning away
                self.vel.linear.x = 0.0
                self.vel.angular.z = 0.8

            self.velocity_publisher.publish(self.vel)
            self.rate.sleep()

if __name__ == '__main__':
    task3_instance = Task3()
    try:
        task3_instance.main_loop()
    except rospy.ROSInterruptException:
        pass