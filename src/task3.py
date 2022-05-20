#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np
import time

class Task3:

    def __init__(self):

        node_name = "task3"
        rospy.init_node(node_name, anonymous=True)

        self.velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.lidar_subscriber = rospy.Subscriber('scan', LaserScan, self.callback_lidar)

        self.front_min = 999
        self.right_min = 999

        self.rate = rospy.Rate(10) # hz        
        self.vel = Twist()
        self.ctrl_c = False

        rospy.on_shutdown(self.shutdownhook)
        rospy.loginfo(f"the {node_name} node has been initialised...")

    def callback_lidar(self, lidar_data):
        # get the front arc of the robot
        front_left_arc = lidar_data.ranges[0:10]
        front_right_arc = lidar_data.ranges[-10:]
        front_arc = np.array(front_left_arc + front_right_arc)

        # get the right side of the robot
        right_left_arc = lidar_data.ranges[300:320]
        right_right_arc = lidar_data.ranges[320:340]
        right_arc = np.array(right_left_arc + right_right_arc)

        # get the closest distance from the arcs
        self.front_min = front_arc.min()
        self.right_min = right_arc.min()

    def shutdownhook(self):
        self.velocity_publisher.publish(Twist())
        self.ctrl_c = True

    def main_loop(self):

        start_time = time.time()
        execution_time = 0
        right_turning_speed = 1
        left_turning_speed = 0.25
        moving_speed = 0.26
        front_threshold = 0.45
        side_threshold = 0.35

        while not self.ctrl_c: # and execution_time < 150:

            execution_time = time.time() - start_time

            if self.front_min > front_threshold:
                # Nothing detected in front, move forward

                if self.right_min < side_threshold:
                    # Too close to right wall, turn away!
                    self.vel.linear.x = moving_speed
                    self.vel.angular.z = left_turning_speed
                
                else:
                    # Too far away from the right wall, move closer!
                    self.vel.linear.x = moving_speed
                    self.vel.angular.z = -right_turning_speed

            else:
                # Obstacle detected in front, stop and turn away!
                self.vel.linear.x = 0.0
                self.vel.angular.z = right_turning_speed

            self.velocity_publisher.publish(self.vel)
            self.rate.sleep()

if __name__ == '__main__':
    task3_instance = Task3()
    try:
        task3_instance.main_loop()
    except rospy.ROSInterruptException:
        pass