#!/usr/bin/env python
# license removed for brevity
import rospy
import time
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from ros_copter.srv import AddWaypoint, RemoveWaypoint, SetWaypointsFromFile
import csv
import numpy as np

class WaypointManager():

    def __init__(self):

        # get parameters
        # how close does the MAV need to get before going to the next waypoint?
        self.threshold = rospy.get_param('~threshold', 2)
        self.cyclical_path = rospy.get_param('~cycle', True)
        self.waypoint_filename = rospy.get_param('~waypoint_filename', "/home/jarvis/waypoints.csv")

        self.prev_time = rospy.Time.now()

        # set up Services
        self.add_waypoint_service = rospy.Service('add_waypoint', AddWaypoint, self.addWaypointCallback)
        self.remove_waypoint_service = rospy.Service('remove_waypoint', RemoveWaypoint, self.addWaypointCallback)
        self.set_waypoint_from_file_service = rospy.Service('set_waypoints_from_file', SetWaypointsFromFile, self.addWaypointCallback)

        # Set Up Publishers and Subscribers
        self.xhat_sub_ = rospy.Subscriber('shredder/ground_truth/odometry', Odometry, self.odometryCallback, queue_size=5)
        self.waypoint_pub_ = rospy.Publisher('waypoint', Vector3, queue_size=5, latch=True)

        # Start Up Waypoint List
        self.waypoint_list = []
        print(self.waypoint_filename)
        if self.waypoint_filename:
            file = csv.reader(open(self.waypoint_filename, 'r'))
            print("Waypoints file")
            for row in file:
                print(map(float, row))
                self.waypoint_list.append(map(float, row))

        self.current_waypoint_index = 0

        waypoint_msg = Vector3()
        current_waypoint = np.array(self.waypoint_list[0])
        waypoint_msg.x = current_waypoint[0]
        waypoint_msg.y = current_waypoint[1]
        waypoint_msg.z = current_waypoint[2]
        self.waypoint_pub_.publish(waypoint_msg)

        while not rospy.is_shutdown():
            # wait for new messages and call the callback when they arrive
            print("spinning")
            rospy.spin()


    def addWaypointCallback(req):
        print("addwaypoints")

    def removeWaypointCallback(req):
        print("remove Waypoints")

    def setWaypointsFromFile(req):
        print("set Waypoints from File")

    def odometryCallback(self, msg):
        # Get error between waypoint and current state
        current_waypoint = np.array(self.waypoint_list[self.current_waypoint_index])
        current_position = np.array([msg.pose.pose.position.x,
                                     msg.pose.pose.position.y,
                                     msg.pose.pose.position.z])
        error = np.linalg.norm(current_position - current_waypoint)

        if error < self.threshold:
            # Get new waypoint index
            self.current_waypoint_index += 1
            if self.cyclical_path:
                self.current_waypoint_index %= len(self.waypoint_list)
            else:
                if self.current_waypoint_index > len(self.waypoint_list):
                    self.current_waypoint_index -=1
            next_waypoint = np.array(self.waypoint_list[self.current_waypoint_index])
            waypoint_msg = Vector3()
            waypoint_msg.x = next_waypoint[0]
            waypoint_msg.y = next_waypoint[1]
            waypoint_msg.z = next_waypoint[2]
            self.waypoint_pub_.publish(waypoint_msg)

            
if __name__ == '__main__':
    rospy.init_node('waypoint_manager', anonymous=True)
    try:
        wp_manager = WaypointManager()
    except:
        rospy.ROSInterruptException
    pass
