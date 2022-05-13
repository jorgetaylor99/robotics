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

        self.min = 999
        self.argmin = 10

        self.rate = rospy.Rate(10) # hz        
        self.vel = Twist()

        self.ctrl_c = False

        rospy.on_shutdown(self.shutdownhook)
        rospy.loginfo(f"the {node_name} node has been initialised...")

    def callback_lidar(self, lidar_data):
        # get the front arc of the robot
        left_arc = lidar_data.ranges[0:30]
        right_arc = lidar_data.ranges[-30:]
        front_arc = np.array(left_arc + right_arc)

        # get the closest distance and the argmin of the closest distance
        self.min = front_arc.min()
        self.argmin = front_arc.argmin()

    def shutdownhook(self):
        self.velocity_publisher.publish(Twist())
        self.ctrl_c = True
    
    def print_lidar(self):
        print(f"Minimum distance in-front: {self.min}")

    def main_loop(self):

        start_time = time.time()
        execution_time = 0
        threshold_distance = 0.5
        moving_speed = 0.26
        turning_speed = 1.8

        while not self.ctrl_c and execution_time < 90:
            execution_time = time.time() - start_time
            
            # if there's an obstacle in-front
            if self.min < threshold_distance:
                if not turning:
                    # grab the min distance argmin to determine which direction to turn in
                    start_argmin = self.argmin
                    turning = True
                else:
                    # stop the robot and turn the robot (until in-front is clear)
                    self.vel.linear.x = 0.0
                    if start_argmin < 30:
                        self.vel.angular.z = -turning_speed
                    else:
                        self.vel.angular.z = turning_speed
            else:
                # if in-front is clear, set turning to false and move forwards again!
                turning = False
                self.vel.linear.x = moving_speed
                self.vel.angular.z = 0

            self.velocity_publisher.publish(self.vel)
            self.rate.sleep()

if __name__ == '__main__':
    task2_instance = Task2()
    try:
        task2_instance.main_loop()
    except rospy.ROSInterruptException:
        pass