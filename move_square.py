#!/usr/bin/env python3

from base64 import encode
from typing import List
import rospy
import numpy as np
from collections import deque
import time
# import the Twist message for publishing velocity commands:
from geometry_msgs.msg import Twist

# import the Odometry message for subscribing to the odom topic:
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from sensor_msgs.msg import JointState
from sensor_msgs.msg import LaserScan

# import the function to convert orientation from quaternions to angles:
from tf.transformations import euler_from_quaternion

# import some useful mathematical operations (and pi), which you may find useful:
from math import sqrt, pow, pi

class Square():
    def Jointstates_callback_function(self, topic_data: JointState):
        'Grabs info on encoders'
        encoders = topic_data.position
        #positive motion is clockwise, negative anticlockwise, in radians
        self.encoder_right = encoders[0]
        self.encoder_left = encoders[1]

    def Imu_callback_function(self, topic_data: Imu):
        pass 
        #rostopic echo -c imu
        #https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Imu.html



    def update_readings_history(self, laser_reading_list):
        # Update each deque with the new reading for that degree
        for i in range(len(laser_reading_list)):
            self.readings_history[i].append(laser_reading_list[i])

    def get_degree_average(self, degree):
        # Compute the average for a specific degree
        if self.readings_history[degree]:  # Check if there is data in the deque
            return np.mean([x for x in self.readings_history[degree] if x != 0])
        return 0  # Return 0 or an appropriate default value if no valid data exists

    def interpret_reading(self, degree, reading):
        
        moving_average = self.get_degree_average(degree)

        if reading == 0:
            if moving_average >= self.threshold_max or self.min_max_lidar[degree] == "max":
                self.min_max_lidar[degree] = "max"
                return 'max range'
            elif moving_average <= self.threshold_min or self.min_max_lidar[degree] == "min":
                self.min_max_lidar[degree] = "min"
                return 'min range'
            else:
                return 'ambiguous'
        else:
            self.min_max_lidar[degree] = None  # Reset memory when a valid reading is detected
            return f"{round(reading, 3)} m"

    def Laser_callback_function(self, topic_data: LaserScan):
        laser_reading_list = [0 if x == float('inf') else x for x in topic_data.ranges]  # Process and filter data
        self.update_readings_history(laser_reading_list)  # Update readings history

        
        print('_______________')
        for i in range(self.display_degrees):
            current_left = laser_reading_list[i]
            current_right = laser_reading_list[-(i + 1)]

            interpreted_left = self.interpret_reading(i, current_left)
            interpreted_right = self.interpret_reading(360 - i - 1, current_right)

            print(f'_______{i+1}_______')
            print(f"Left: {interpreted_left}")
            print(f"Right: {interpreted_right}")


    def Odom_callback_function(self, topic_data: Odometry):
        'grabs info on LOCAL SIMULATION BASED odometry - is useless IRL'
        # obtain relevant topic data: pose (position and orientation):
        # obtain the robot's position co-ords:
        pos_x = topic_data.pose.pose.position.x
        pos_y = topic_data.pose.pose.position.y
        orientation = topic_data.pose.pose.orientation
        # convert orientation co-ords to roll, pitch & yaw 
        # (theta_x, theta_y, theta_z):
        (roll, pitch, yaw) = euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w], "sxyz"
        )

        # We're only interested in x, y and theta_z
        # so assign these to class variables (so that we can
        # access them elsewhere within our Square() class):
        self.x = round(pos_x, 3)
        self.y = round(pos_y, 3)
        self.theta_z = round(yaw, 3)

        # If this is the first time that the callback_function has run
        # (e.g. the first time a message has been received), then
        # obtain a "reference position" (used to determine how far
        # the robot has moved during its current operation)

        if self.startup:
            # don't initialise again:

            self.startup = False
            # set the reference position:
            self.x0 = self.x
            self.y0 = self.y
            self.theta_z0 = self.theta_z

            print(f"Started at: {round(self.x0, 3), round(self.y0, 3), round(self.theta_z0, 3)}")
            print(f"Lencoder: {round(self.encoder_left, 3)}  Rencoder: {round(self.encoder_right, 3)}")

    def __init__(self):
        self.node_name = "move_square"
        # a flag if this node has just been launched
        self.startup = True
        self.spam_count = 0
        self.checkpoint = 0

        "Lidar related stuff"
        # Initialize a list of deques, one for each degree
        num_degrees=360 
        history_size=5
        self.threshold_max = 3.0
        self.threshold_min = 0.5
        self.display_degrees = 90  # Number of degrees to display (adjust as needed)

        self.readings_history = [deque(maxlen=history_size) for _ in range(num_degrees)]
        self.min_max_lidar = [None] * num_degrees  # Maintain a state for each degree

        

        # setup a '/cmd_vel' publisher and an '/odom' subscriber:
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.sub = rospy.Subscriber("/odom", Odometry, self.Odom_callback_function)
        self.sub = rospy.Subscriber("/joint_states", JointState, self.Jointstates_callback_function)
        self.sub = rospy.Subscriber("/imu", Imu, self.Imu_callback_function)
        self.sub = rospy.Subscriber("/scan", LaserScan, self.Laser_callback_function)

        rospy.init_node(self.node_name, anonymous=True)
        self.rate = rospy.Rate(10)  # hz

        # define the robot pose variables and initialise them to zero:
        # variables for the robot's "current position":
        self.x = 0.0
        self.y = 0.0
        self.theta_z = 0.0
        # variables for a "reference position":
        self.x0 = 0.0
        self.y0 = 0.0
        self.theta_z0 = 0.0

        # define a Twist message instance, to set robot velocities
        self.vel_cmd = Twist()

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)

        rospy.loginfo(f"the {self.node_name} node has been initialised...")

    def shutdownhook(self):
        # publish an empty twist message to stop the robot
        # (by default all velocities will be zero):
        self.pub.publish(Twist())
        rospy.loginfo("You killed the square maker")
        self.ctrl_c = True

    def move_forward(self):
            if self.checkpoint == 0:
                if self.x - self.x0 < 0.9:
                    self.vel_cmd.linear.x = 0.3 # m/s
                    self.vel_cmd.angular.z = 0.0 # rad/s
                elif self.x - self.x0 < 1 and self.x - self.x0 > 0.9:
                    self.vel_cmd.linear.x = 0.05 # m/s
                elif self.x - self.x0 > 1.1:
                    self.vel_cmd.linear.x = -0.05 # m/s
                else:
                    self.vel_cmd.linear.x = 0
                    self.checkpoint = 1
                    self.rotate()

    def rotate(self):
        if self.theta_z - self.theta_z0 < 80:
            self.vel_cmd.angular.z = 0.3
        elif self.theta_z - self.theta_z0 > 80 and self.theta_z - self.theta_z0 < 90:
            self.vel_cmd.angular.z = 0.05
        elif self.theta_z - self.theta_z0 > 100:
            self.vel_cmd.angular.z = -0.05
        else:
            self.vel_cmd.angular = 0

    def main_loop(self):
        while not self.ctrl_c:
            # here is where your code would go to control the motion of your
            # robot. Add code here to make your robot move in a square of
            # dimensions 1 x 1m...
            #self.move_forward()

            # publish whatever velocity command has been set in your code above:
            self.pub.publish(self.vel_cmd)
            
            
            # maintain the loop rate @ 10 hz
            self.rate.sleep()

if __name__ == "__main__":
    node = Square()
    try:
        node.main_loop()
    except rospy.ROSInterruptException:
        rospy.loginfo(f"Shit node broke")