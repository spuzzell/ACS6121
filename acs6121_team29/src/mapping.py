#!/usr/bin/env python3

# this code is for retrieving the ocupation map. 
# it will first print map info.
import rospy
import rospkg
from nav_msgs.msg import OccupancyGrid
import os

class Mapper:
    def __init__(self):
        # Initialize the ROS package manager
        rospack = rospkg.RosPack()
        # Get the path to the 'week2_navigation' package
        package_path = rospack.get_path('acs6121_team29')
        # Construct the path to the maps directory
        self.maps_dir = os.path.join(package_path, 'maps')
        node_name = "mapper"
        topic_name_sub = "/map"  # Topic to subscribe to
        rospy.init_node(node_name, anonymous=True)
        self.ctrl_c = False

        # Adjust the rate based on message publishing rate
        self.rate = rospy.Rate(0.2)  # Adjust the rate as needed

        self.sub = rospy.Subscriber(topic_name_sub, OccupancyGrid, self.grid_callback)

        rospy.loginfo(f"The '{node_name}' node is active...")

    def grid_callback(self, topic_data: OccupancyGrid):
        # Process the occupancy grid data here
        rospy.loginfo("Received occupancy grid")

        # Execute the commands to save the map
        try:
            os.chdir(self.maps_dir)  # Change directory to where maps are stored
            os.system("rosrun map_server map_saver -f explore_map")  # Save the map
        except Exception as e:
            rospy.logerr(f"Error saving map: {str(e)}")

    def shutdownhook(self):
        rospy.loginfo(f"Stopping the '{self.node_name}' node at: {rospy.get_time()}")
        self.ctrl_c = True

    def main(self):
        while not self.ctrl_c and not rospy.is_shutdown():
            self.rate.sleep()  # Sleep to control the execution rate

if __name__ == '__main__':
    node = Mapper()
    node.main()
