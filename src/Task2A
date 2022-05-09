#!/usr/bin/env python
# license removed for brevity
from cmath import sqrt
import rospy
from std_msgs.msg import String
from tb3 import *
import time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from math import degrees, radians, sqrt
import numpy as np

class Task2A :
    """Class create to move the bot, avoid obstacles, rotate the bot and explore the area in a given amount of time
    """

    def __init__(self):
        self.initialize_params()
        self.subscriber = rospy.Subscriber('/scan', LaserScan, self.laserscan_cb)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_cb)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.move_bot()

    def initialize_params(self):
        """Initialize the parameters used to tune the movement of the turtlebot
        """
        self.obstacle_distance = 0.0                # Current distance from the obstacle
        self.collision_avoid_distance = 0.7         # Distance before which the obstacle should stop to avoid collision
        self.posx = 0.0                             # x-coordinate of current position
        self.posy = 0.0                             # y-coordinate of current position
        self.yaw = 0.0                              # yaw value at the current state
        self.turn = -90                             # Value in degrees by which the bot should rotate of obstacle is found
        self.forward_velocity = 0.26                # Forward velocity of the bot (along x-axis)
        self.turning_velocity = 0.5                # Turning velocity of the bot (along z-axis)
        self.total_time = 90.0                      # Total time the bot should be moving
        self.radius_of_circle = 0.005                 # Radius of the circle while checking if the bot turned at the current point earlier or not

        self.vel_cmd = Twist()
        self.states_list = list()                   # List in which all the points that the bot turned at earlier are stored

    def odom_cb(self, odom_data):
        """Callback function to get the odometery of the bot

        Args:
            odom_data (Odometry): Odometry data calculated for the bot
        """
        orientation = odom_data.pose.pose.orientation
        position = odom_data.pose.pose.position
        (_, _, yaw) = euler_from_quaternion([orientation.x,
            orientation.y, orientation.z, orientation.w],'sxyz')
        
        self.yaw = round(degrees(yaw), 0)
        self.posx = round(position.x, 4)
        self.posy = round(position.y, 4)

    def laserscan_cb(self, scan_data):
        """Callback function to get the laser scan data from lidar and calculate the distance from the obstacle

        Args:
            scan_data (LaserScan): Laser scan data from the Lidar
        """
        left_arc = scan_data.ranges[0:21]
        right_arc = scan_data.ranges[-20:]
        front_arc = np.array(left_arc[::-1] + right_arc[::-1])
        
        self.obstacle_distance = front_arc.min()

    def set_move_cmd(self, linear = 0.0, angular = 0.0):
        """Set the velocity of the bot in linear and rotational motion

        Args:
            linear (float, optional): Linear velocity in x-axis. Defaults to 0.0.
            angular (float, optional): Rotational velocity along z-axis. Defaults to 0.0.
        """
        self.vel_cmd.linear.x = linear
        self.vel_cmd.angular.z = angular


    def check_point(self):
        """Check if the bot tried to turn from the current point earlier or not

        Returns:
            bool: Return True if bot had turned earlier at this position, otherwise False
        """
        for point in self.states_list:
            x = point.split(',')[0]
            y = point.split(',')[1]
            distance_from_center = sqrt((float(x)-self.posx)**2 + (float(y)-self.posy)**2)
            if distance_from_center <= self.radius_of_circle:
                return True
        return False

    def move_bot(self):
        """Function to move the bot, avoid obstacles and explore the area
        """
        rate = rospy.Rate(10) # 10hz
        start_time = time.time()

        # Keep the loop running unless ros node is killed or timer of 90.0 seconds (in real time) is complete
        while not rospy.is_shutdown() and time.time() - start_time < self.total_time:
            # If the obstacle is farther than collision_avoid_distance then keep moving forward
            if self.obstacle_distance > self.collision_avoid_distance:
                self.set_move_cmd(self.forward_velocity, 0)
                self.pub.publish(self.vel_cmd)
                rate.sleep()
            
            # If the bot is near an obstacle then turn the bot
            else:
                current_yaw = self.yaw
                # If the bot did not turn at the current point earlier then rotate right
                if not self.check_point():
                    # print(self.states_list)
                    self.states_list.append(f"{self.posx}, {self.posy}")
                    # Keep turning the bot unless it has rotated till the required yaw value or the timer of 90.0 seconds is complete
                    turn_angle = (current_yaw - self.turn)
                    if turn_angle > 180 or turn_angle < -180:
                        turn_angle = -90
                    while (turn_angle - 10 > self.yaw or self.yaw > turn_angle + 10) and time.time() - start_time < self.total_time:
                        self.set_move_cmd(0, self.turning_velocity)
                        self.pub.publish(self.vel_cmd)
                        rate.sleep()

                    self.set_move_cmd(0, 0)
                # If the bot did rotate at the current point earlier, then rotate left
                else:
                    turn_angle = (current_yaw - self.turn)
                    if turn_angle > 180 or turn_angle < -180:
                        turn_angle = 90
                    while (turn_angle - 10 > self.yaw or self.yaw > turn_angle + 10) and time.time() - start_time < self.total_time:
                        self.set_move_cmd(0, -self.turning_velocity)
                        self.pub.publish(self.vel_cmd)
                        rate.sleep()        
                    
                    self.set_move_cmd(0, 0)

        # Stop the bot when the timer is complete
        self.set_move_cmd(0, 0)
        self.pub.publish(self.vel_cmd)
        

if __name__ == '__main__':
    rospy.init_node('Task2A', anonymous=True)
    Task2A = Task2A()
