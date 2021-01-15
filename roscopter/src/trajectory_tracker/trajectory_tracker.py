#!/usr/bin/env python3
import numpy as np
import rospy

from roscopter_msgs.msg import TrajectoryCommand
from rosflight_msgs.msg import Command
from nav_msgs.msg import Odometry

class TrajectoryTracker():

    def __init__(self):
        self.odometry_subscriber = rospy.Subscriber('state', Odometry, self.odometryCallback, queue_size=5)
        self.trajectory_command_subscriber = rospy.Subscriber('trajectory_command', TrajectoryCommand, self.commandCallback, queue_size=5)
        self.rosflight_command_publisher = rospy.Publisher('rosflight_command', Command, queue_size=5, latch = True)

        while not rospy.is_shutdown():
            rospy.spin()

    def odometryCallback(self, msg):
        north_position = msg.pose.pose.position.x
        east_position = msg.pose.pose.position.y
        down_position = msg.pose.pose.position.z
        current_position = np.array([self.n, self.e, self.d])
        # orientation in quaternion form
        qw = msg.pose.pose.orientation.w
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        # yaw from quaternion
        yaw = np.arctan2(2*(qw*qz + qx*qy), 1 - 2*(qy**2 + qz**2))

    def commandCallback(self,msg):
        positions_command = np.array([msg.x_position, msg.y_position, msg.z_position])
        velocities_command = np.array([msg.x_velocity, msg.y_velocity, msg.z_velocity])
        accelerations_command = np.array([msg.x_acceleration, msg.y_acceleration, msg.z_acceleration])
        heading_command = msg.heading
        heading_rate_command = msg.heading_rate

    def publishRosflightCommand(self):
        cmd_msg = Command()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.mode = 1
        cmd_msg.ignore = 0
        cmd_msg.x = 0
        cmd_msg.y = 0
        cmd_msg.z = 2
        cmd_msg.F = .5
        self.rosflight_command_publisher.publish(self.cmd_msg)


if __name__ == '__main__':
    rospy.init_node('trajectory_tracker', anonymous=True)
    try:
        traj_tracker = TrajectoryTracker()
    except:
        rospy.ROSInterruptException
    pass