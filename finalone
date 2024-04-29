#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import subprocess

class Explore():

    def __init__(self):
        rospy.init_node('Gr_idk', disable_signals=True, anonymous=True)

        self.range_limit = 3.602
        self.step_back_thresh = 0.265
        self.turn_thresh = 0.511
        self.linear_speed = 0.254
        self.angular_speed = 1.392
        
        self.laser_data_full = []

        rospy.Subscriber("/scan", LaserScan, self.callback_laser)

        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10, latch=True)

        rospy.Timer(rospy.Duration(0.02), self.update_controller)

        rospy.on_shutdown(self.shutdown)

    def start_slam(self):
        """Launches the turtlebot3_slam launch file to start SLAM."""
        try:
            self.slam_process = subprocess.Popen(["roslaunch", "turtlebot3_slam", "turtlebot3_slam.launch"])
            rospy.loginfo("TurtleBot3 SLAM has been successfully launched.")
        except Exception as e:
            rospy.logerr("Failed to start TurtleBot3 SLAM: " + str(e))

    def save_map(self):
        map_path = "/home/student/catkin_ws/src/acs6121_realrobot/maps"
        try:
            rospy.loginfo("Saving the map to {}".format(map_path))
            subprocess.check_call(["rosrun", "map_server", "map_saver", "-f", map_path])
            rospy.loginfo("Map successfully saved.")
        except subprocess.CalledProcessError as e:
            rospy.logerr("Failed to save the map: {}".format(e))
        except Exception as e:
            rospy.logerr("An unexpected error occurred: {}".format(e))

    def publish_twist(self, linear_x, angular_z):
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.publisher.publish(twist)
    def MARCH(self):
        self.publish_twist(self.linear_speed, 0)
    def STEP_BACK(self):
        self.publish_twist(-self.linear_speed + 0.014, 0)
    def LEAN_LEFT(self):
        self.publish_twist(self.linear_speed / 2, self.angular_speed)
    def LEAN_RIGHT(self):
        self.publish_twist(self.linear_speed / 2, -self.angular_speed)
    def TURN_LEFT(self):
        self.publish_twist(0.0, self.angular_speed)
    def TURN_RIGHT(self):
        self.publish_twist(0.0, -self.angular_speed)
    def TERMINATED(self):
        self.publish_twist(0, 0)

    def callback_laser(self, msg):
        self.laser_data_full = msg.ranges

        for i, _ in enumerate(self.laser_data_full):
            if self.laser_data_full[i] == 0:
                self.laser_data_full = list(self.laser_data_full)
                self.laser_data_full[i] = self.range_limit
                self.laser_data_full = tuple(self.laser_data_full)

    def avoid_obstacles(self):
        # Define the degree ranges
        right_degrees = self.laser_data_full[0:40]
        left_degrees = self.laser_data_full[320:360]

        # Check for obstacles
        if any((distance < self.step_back_thresh for distance in self.laser_data_full[0:20]) or any(distance < self.step_back_thresh for distance in self.laser_data_full[70:90])):
            self.STEP_BACK()
        elif min(right_degrees, default=self.turn_thresh+1) < self.turn_thresh:
            if min(right_degrees, default=(self.turn_thresh/2)+1) < self.turn_thresh/2:
                self.TURN_RIGHT()
            else:
                self.LEAN_RIGHT()
        elif min(left_degrees, default=self.turn_thresh+1) < self.turn_thresh:
            if min(left_degrees, default=(self.turn_thresh/2)+1) < self.turn_thresh/2:
                self.TURN_LEFT()
            else:
                self.LEAN_LEFT()
        elif min(right_degrees, default=self.turn_thresh+1) < self.turn_thresh <= max(left_degrees, default=0):
            if min(right_degrees, default=(self.turn_thresh/2)+1) < self.turn_thresh/2:
                self.TURN_RIGHT()
            else:
                self.LEAN_RIGHT()
        elif min(left_degrees, default=self.turn_thresh+1) < self.turn_thresh <= max(right_degrees, default=0):
            if min(left_degrees, default=(self.turn_thresh/2)+1) < self.turn_thresh/2:
                self.TURN_LEFT()
            else:
                self.LEAN_LEFT()
        else:
            self.MARCH()

    def update_controller(self, _):
        if self.laser_data_full:
            self.avoid_obstacles()
            pass
        else:
            self.TERMINATED()

    def shutdown(self):
        rospy.logwarn("Shutting down explorer_node.")
        self.save_map()
        if self.gmapping_process:
            self.gmapping_process.terminate()
        # Set the linear and angular velocity to 0
        twist = Twist()
        twist.linear.x = 0
        twist.angular.z = 0
        self.publisher.publish(twist)

def main():
    try:
        Explore()
        rospy.spin()
    except (rospy.exceptions.ROSInitException,
            rospy.exceptions.ROSException, KeyboardInterrupt):
        rospy.signal_shutdown()

if __name__ == '__main__':
    main()
