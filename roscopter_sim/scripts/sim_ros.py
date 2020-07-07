#!/usr/bin/env python

import numpy as np
import rospy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from rosflight_msgs.msg import GNSS
from ublox.msg import PosVelEcef


class SimManager():

    def __init__(self):

        self.rover_pos = np.zeros(3)
        self.rover_lla = np.zeros(3)
        self.rover_PosVelEcef = PosVelEcef()

        # Set Up Publishers and Subscribers
        self.rover_virtual_PosVelEcef_pub_ = rospy.Publisher('rover_PosVelEcef', PosVelEcef, queue_size=5, latch=True)
        self.rover_odom_sub_ = rospy.Subscriber('drone_odom', Odometry, self.roverOdomCallback, queue_size=5)
        self.rover_gnss_sub_ = rospy.Subscriber("gnss", GNSS, self.roverGnssCallback, queue_size=5)

        
        while not rospy.is_shutdown():
            # wait for new messages and call the callback when they arrive
            rospy.spin()

    def roverOdomCallback(self, msg):
        # Get error between waypoint and current state
        #convert from gazebo NWU to NED
        self.rover_pos = np.array([msg.pose.pose.position.x,
                                     -msg.pose.pose.position.y,
                                     -msg.pose.pose.position.z])

    def roverGnssCallback(self, msg):
        # Callback is just used to publish all gps data at the same rate
        self.publish_rover_virtual_PosVelEcef(msg)
        self.publish_rover_virtual_relPos(msg.header)
        self.publish_base_virtual_PosVelEcef()


    def publish_rover_virtual_PosVelEcef(self, msg):
        
        self.rover_PosVelEcef.header = msg.header
        self.rover_PosVelEcef.fix = msg.fix
        # self.rover_PosVelEcef.lla = self.rover_lla  #lla is not currently being used
        self.rover_PosVelEcef.position = msg.position
        self.rover_PosVelEcef.horizontal_accuracy = msg.horizontal_accuracy
        self.rover_PosVelEcef.vertical_accuracy = msg.vertical_accuracy
        self.rover_PosVelEcef.velocity = msg.velocity
        self.rover_PosVelEcef.speed_accuracy = msg.speed_accuracy

        self.rover_virtual_PosVelEcef_pub_.publish(self.rover_PosVelEcef)


if __name__ == '__main__':
    rospy.init_node('sim_manager', anonymous=True)
    try:
        sim_manager = SimManager()
    except:
        rospy.ROSInterruptException
    pass
