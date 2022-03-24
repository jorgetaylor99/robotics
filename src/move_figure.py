#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from math import sqrt, pow, pi

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
    
    def print_stuff(self, a_message):
        print(a_message)
        print(f"current velocity: lin.x = {self.vel.linear.x:.1f}, ang.z = {self.vel.angular.z:.1f}")
        print(f"current odometry: x = {self.x:.3f}, y = {self.y:.3f}, theta_z = {self.theta_z:.3f}")

    def main_loop(self):
        status = ""
        wait = 0
        while not self.ctrl_c:
            if self.startup:
                self.vel = Twist()
                status = "init"
            """
            elif self.turn:
                if abs(self.theta_z0 - self.theta_z) >= pi/2 and wait > 5:
                    # If the robot has turned 90 degrees (in radians) then stop turning
                    self.turn = False
                    self.vel = Twist()
                    self.theta_z0 = self.theta_z
                    status = "turn-fwd transition"
                    wait = 0
                else:
                    self.vel = Twist()
                    self.vel.angular.z = 0.2
                    status = "turning"
                    wait += 1
            else:
                if sqrt(pow(self.x0 - self.x, 2) + pow(self.y0 - self.y, 2)) >= 0.5:
                    # if distance travelled is greater than 0.5m then stop, and start turning:
                    self.vel = Twist()
                    self.turn = True
                    self.x0 = self.x
                    self.y0 = self.y
                    status = "fwd-turn transition"
                else:
                    self.vel = Twist()
                    self.vel.linear.x = 0.1
                    status = "moving forwards"
            """
            # if self.turn:
            #     if self.x0 == self.x and self.y0 == self.y:
            #         #swap direction
            #     else:
            #         self.vel = Twist()
            #         self.    

            self.pub.publish(self.vel)
            self.print_stuff(status)
            self.rate.sleep()

if __name__ == '__main__':
    movesquare_instance = Figure()
    try:
        movesquare_instance.main_loop()
    except rospy.ROSInterruptException:
        pass