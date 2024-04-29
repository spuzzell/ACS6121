#!/usr/bin/env python3

from base64 import encode
from typing import List
import rospy
import time
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

    def Laser_callback_function(self, topic_data: LaserScan) -> List:
        laser_reading_list = [0 if x == float('inf') else x for x in topic_data.ranges]  # Replace inf with 0 so sim is accurate

        """ print('_______________')
        for i in range(self.display_degrees):
            current_left = laser_reading_list[i]
            current_right = laser_reading_list[-(i + 1)]
            
            print(f'_______{i+1}_______')
            print(f"Left: {current_left}")
            print(f"Right: {current_right}") """

        self.forwardleft_reading = round(laser_reading_list[44], 2)
        self.forwardright_reading = round(laser_reading_list[-45], 2)
        self.forward_reading = round(laser_reading_list[0], 2)
        self.back_reading = round(laser_reading_list[180], 2)
        self.backleft_reading = round(laser_reading_list[134], 2)
        self.backright_reading = round(laser_reading_list[-135], 2)
        self.right_reading = round(laser_reading_list[-90], 2)
        self.left_reading = round(laser_reading_list[89], 2)
        

    def __init__(self):
        self.node_name = "move_square"
        # a flag if this node has just been launched
        self.startup = True
        self.last_time1 = time.time()
        self.last_time2 = time.time()
        self.last_time3 = time.time()
        self.state = "FORWARD"
        self.timer_depth = 0

        "Lidar related stuff"
        self.display_degrees = 2  # Number of degrees to display (adjust as needed)
        self.forwardleft_reading = 1
        self.forwardright_reading = 1
        self.forward_reading = 1
        self.back_reading = 1
        self.backleft_reading = 1
        self.backright_reading = 1
        self.right_reading = 1
        self.left_reading = 1

         # setup a '/cmd_vel' publisher and an '/odom' subscriber:
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.sub = rospy.Subscriber("/odom", Odometry, self.Odom_callback_function)
        self.sub = rospy.Subscriber("/joint_states", JointState, self.Jointstates_callback_function)
        self.sub = rospy.Subscriber("/imu", Imu, self.Imu_callback_function)
        self.sub = rospy.Subscriber("/scan", LaserScan, self.Laser_callback_function)

        rospy.init_node(self.node_name, anonymous=True)
        self.rate = rospy.Rate(10)  # hz

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

    def move_forward(self, speed: float) -> None:
        self.vel_cmd.linear.x = speed # m/s
        self.vel_cmd.angular.z = 0.0 # rad/s

    def rotate(self, rotate: float) -> None:
        #positive rotates counterclockwise, negative clockwise
        self.vel_cmd.linear.x = 0.0 # m/s
        self.vel_cmd.angular.z = rotate

    def full_stop(self) -> None:
        self.vel_cmd.linear.x = 0.0 # m/s
        self.vel_cmd.angular.z = 0

    def timer_1(self, time_to_wait: float) -> bool:
        #Function to time events without freezing the script
        #Returns true if requested timer passed, false otherwise
        current_time = time.time()
        if current_time - self.last_time1 >= time_to_wait:
            self.last_time1 = current_time
            return True
        else:
            return False
    def timer_2(self, time_to_wait: float) -> bool:
        #Function to time events without freezing the script
        #Returns true if requested timer passed, false otherwise
        current_time = time.time()
        if current_time - self.last_time2 >= time_to_wait:
            self.last_time2 = current_time
            return True
        else:
            return False
    def timer_3(self, time_to_wait: float) -> bool:
        #Function to time events without freezing the script
        #Returns true if requested timer passed, false otherwise
        current_time = time.time()
        if current_time - self.last_time3 >= time_to_wait:
            self.last_time3 = current_time
            return True
        else:
            return False                            

    def main_loop(self):
        while not self.ctrl_c:
            self.current_time = time.time()
            print(self.timer_depth)
            #print(f"Front: {self.forward_reading}     FwdLeft: {self.forwardleft_reading}        FwdRight: {self.forwardright_reading}      BkLeft: {self.backleft_reading}       Bkright: {self.backright_reading}     Left: {self.left_reading}   Right: {self.right_reading}")
            print(f"Current state: {self.state}")
            if self.state == "FORWARD":
                if self.timer_depth == 0:
                    if self.forward_reading > 0.4:
                        self.move_forward(0.3)
                    else:
                        self.timer_depth = 1
                elif self.timer_depth == 1:
                    self.full_stop()
                    if self.timer_1(1):
                        self.timer_depth = 0
                        self.state = "REVERSING"
            
            elif self.state == "REVERSING":
                if self.timer_depth == 0:
                    if self.back_reading > 0.45:
                        self.move_forward(-0.2)
                        if self.timer_1(1.5):
                            self.timer_depth = 1
                    else:
                        self.timer_depth = 1
                elif self.timer_depth == 1:
                    self.full_stop()
                    if self.timer_2(1):
                        self.timer_depth = 2
                elif self.timer_depth == 2:
                    self.state = "TURN ANTICLOCKWISE 90"
                    self.timer_depth = 0
            
            elif self.state == "TURN ANTICLOCKWISE 90":
                if self.timer_depth == 0:
                    self.rotate(1)
                    if self.timer_1(2):
                        self.timer_depth = 1
                      
                elif self.timer_depth == 1:
                    self.full_stop()
                    if self.timer_2(1):
                        self.timer_depth = 2

                elif self.timer_depth == 2:
                    self.timer_depth = 0
                    self.state = "FORWARD"
                    








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