#!/usr/bin/env python3
# A simple ROS publisher node in Python

import rospy 
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import time


class Publisher(): 

    def __init__(self): 
        self.node_name = "Ved_circle_brudah" 
        topic_name = "/cmd_vel" 

        self.pub = rospy.Publisher(topic_name, Twist, queue_size=10) 
        rospy.init_node(self.node_name, anonymous=True) 
        self.rate = rospy.Rate(10) 
        self.vel_cmd = Twist()
        self.vel_cmd.linear.x = 0.3 # m/s
        self.vel_cmd.angular.z = 0.4 # rad/s

        self.ctrl_c = False 
        rospy.on_shutdown(self.shutdownhook) 

        rospy.loginfo(f"The '{self.node_name}' node is active...") 

    def shutdownhook(self): 
        print(f" Killing {self.node_name} node at: {rospy.get_time()}")
        self.vel_cmd.linear.x = 0.0 # m/s
        self.vel_cmd.angular.z = 0.0 # rad/s
        self.pub.publish(self.vel_cmd)
        time.sleep(1)
        self.ctrl_c = True

    def main_loop(self):
        while not self.ctrl_c: 
            self.pub.publish(self.vel_cmd)
            self.rate.sleep()

if __name__ == '__main__': 
    publisher_instance = Publisher() 
    try:
        publisher_instance.main_loop() 
    except rospy.ROSInterruptException:
        pass