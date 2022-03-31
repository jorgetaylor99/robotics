#!/usr/bin/env python3

from logging import shutdown
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from math import sqrt, pow, pi
import time

class Figure:
    def callback_function(self, odom_data):
        # obtain the orientation and position co-ords:
        or_x = odom_data.pose.pose.orientation.x
        or_y = odom_data.pose.pose.orientation.y
        or_z = odom_data.pose.pose.orientation.z
        or_w = odom_data.pose.pose.orientation.w
        pos_x = odom_data.pose.pose.position.x
        pos_y = odom_data.pose.pose.position.y

        # convert orientation co-ords to roll, pitch & yaw (theta_x, theta_y, theta_z):
        (roll, pitch, yaw) = euler_from_quaternion([or_x, or_y, or_z, or_w], 'sxyz')
        
        self.x = pos_x
        self.y = pos_y
        self.theta_z = yaw 

        if self.startup:
            self.startup = False
            self.x0 = self.x
            self.y0 = self.y
            self.theta_z0 = self.theta_z

    def __init__(self):
        node_name = "move_figure"
        
        self.startup = True

        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('odom', Odometry, self.callback_function)

        rospy.init_node(node_name, anonymous=True)
        self.rate = rospy.Rate(10) # hz

        self.x = 0.0
        self.y = 0.0
        self.theta_z = 0.0
        self.x0 = 0.0
        self.y0 = 0.0
        self.theta_z0 = 0.0
        
        self.vel = Twist()

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)
        rospy.loginfo(f"the {node_name} node has been initialised...")

    def shutdownhook(self):
        self.pub.publish(Twist())
        self.ctrl_c = True
    
    def print_odometry(self):
        print(f"x = {self.x:.2f} [m], y = {self.y:.2f} [m], yaw = {(self.theta_z*180)/pi:.1f} [degrees].")

    def main_loop(self):
        status = "anti_clockwise"
        count = 0
        start_time = time.time()

        while not self.ctrl_c:
            if self.startup:
                self.vel = Twist()
                status = "anti_clockwise"

            elif status == "anti_clockwise":
                self.vel.angular.z = 0.22
                self.vel.linear.x = 0.11

                # use a counter to ensure robot does not get stuck before it starts moving
                if (count > 10) and (-0.01 <= self.x <= 0.01) and (-0.01 <= self.y <= 0.01):
                    status = "clockwise"
                    count = 0

            elif status == "clockwise":
                self.vel.angular.z = -0.22
                self.vel.linear.x = 0.11

                if (count > 10) and (-0.01 <= self.x <= 0.01) and (-0.01 <= self.y <= 0.01):
                    status = "stop"

            else: # status is stop
                self.vel.angular.z = 0
                self.vel.linear.x = 0
                
                # calculate execution time (approx 60s)
                execution_time = time.time() - start_time
                print(f"Execution time = {execution_time:.1f}s")

                # shutdown once sequence is complete          
                self.shutdownhook()

            self.pub.publish(self.vel)

            # Print odometry data every 1Hz (every 1s)
            # Since we're running at 10Hz just run every 10th cycle
            if count % 10 == 0:
                self.print_odometry()
            
            self.rate.sleep()
            count += 1

if __name__ == '__main__':
    figure8_instance = Figure()
    try:
        figure8_instance.main_loop()
    except rospy.ROSInterruptException:
        pass