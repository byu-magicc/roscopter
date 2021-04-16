#!/usr/bin/env python3
import numpy as np
import rospy

from roscopter_msgs.msg import TrajectoryState
from rosflight_msgs.msg import Command
from nav_msgs.msg import Odometry

from lqr_controller import LQRController

class LQRControllerNode():

    def __init__(self):
        self.odometry_subscriber = rospy.Subscriber('state', Odometry, self.odometryCallback, queue_size=5)
        self.trajectory_command_subscriber = rospy.Subscriber('trajectory_command', TrajectoryState, self.commandCallback, queue_size=5)
        self.rosflight_command_publisher = rospy.Publisher('rosflight_command', Command, queue_size=5, latch = True)
        self.trajectory_state_publisher = rospy.Publisher('trajectory_state', TrajectoryState, queue_size=5, latch = True)
        self.lqr_controller = LQRController()
        self.start_time = rospy.get_rostime().to_sec()
        self.current_time = 0
        while not rospy.is_shutdown():
            rospy.spin()

    def odometryCallback(self, msg):

        previous_time = self.current_time
        self.current_time = rospy.get_rostime().to_sec() - self.start_time
        time_step = self.current_time - previous_time
        if time_step <= 0:
            return
        
        north_position = msg.pose.pose.position.x
        east_position = msg.pose.pose.position.y
        down_position = msg.pose.pose.position.z
        position = np.array([[north_position], [east_position], [down_position]])

        north_velocity = msg.twist.twist.linear.x
        east_velocity = msg.twist.twist.linear.y
        down_velocity = msg.twist.twist.linear.z
        velocity = np.array([[north_velocity], [east_velocity], [down_velocity]])

        x_angular_rate = msg.twist.angular.x 
        y_angular_rate = msg.twist.angular.y
        z_angular_rate = msg.twist.angular.z
        body_angular_rates = np.array([[x_angular_rate], [y_angular_rate],[z_angular_rate]])

        qw = msg.pose.pose.orientation.w
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        attitude = np.array([qw,qx,qy,qz])
        self.lqr_controller.updateState(position, velocity, body_angular_rates, attitude, time_step)

        ### implement function below ###
        current_state = self.lqr_controller.getCurrentTrajectoryState()
        traj_state = TrajectoryState()
        traj_state.x_position = current_state.item(0)
        traj_state.y_position = current_state.item(1)
        traj_state.z_position = current_state.item(2)
        traj_state.x_velocity = current_state.item(3)
        traj_state.y_velocity = current_state.item(4)
        traj_state.z_velocity = current_state.item(5)
        traj_state.x_acceleration = current_state.item(6)
        traj_state.y_acceleration = current_state.item(7)
        traj_state.z_acceleration = current_state.item(8)
        traj_state.x_jerk = current_state.item(9)
        traj_state.y_jerk = current_state.item(10)
        traj_state.z_jerk = current_state.item(11)
        traj_state.heading = current_state.item(12)
        traj_state.heading_rate = current_state.item(13)
        self.trajectory_state_publisher.publish(traj_state)


    def commandCallback(self,msg):
        desired_position = np.array([[msg.x_position], [msg.y_position], [msg.z_position]])
        desired_velocity = np.array([[msg.x_velocity], [msg.y_velocity], [msg.z_velocity]])
        desired_acceleration = np.array([[msg.x_acceleration], [msg.y_acceleration], [msg.z_acceleration]])
        desired_jerk = np.array([[msg.x_jerk], [msg.y_jerk], [msg.z_jerk]])
        desired_heading = msg.heading
        desired_heading_rate = msg.heading_rate
        ### implement function below ###
        self.lqr_controller.updateDesiredState(desired_position, desired_velocity, desired_acceleration, desired_jerk, desired_heading, desired_heading_rate)
        rosflight_command = self.lqr_controller.computeRosflightCommand()
        self.publishRosflightCommand(rosflight_command)

    def publishRosflightCommand(self , rosflight_command):
        roll_rate = rosflight_command.item(0)
        pitch_rate = rosflight_command.item(1)
        yaw_rate = rosflight_command.item(2)
        throttle = rosflight_command.item(3)
        cmd_msg = Command()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.mode = Command.MODE_ROLLRATE_PITCHRATE_YAWRATE_THROTTLE
        cmd_msg.ignore = Command.IGNORE_NONE
        cmd_msg.x = roll_rate
        cmd_msg.y = pitch_rate
        cmd_msg.z = yaw_rate
        cmd_msg.F = throttle
        self.rosflight_command_publisher.publish(cmd_msg)


if __name__ == '__main__':
    rospy.init_node('lqr_controller_node', anonymous=True)
    try:
        lqr_controller = LQRControllerNode()
    except:
        rospy.ROSInterruptException
    pass