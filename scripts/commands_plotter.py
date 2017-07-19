#!/usr/bin/env python
import rospy
import time, tf
import numpy as np
import pyqtgraph as pg
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Imu
from rosflight_msgs.msg import Command

# Enable antialiasing for prettier plots
pg.setConfigOptions(antialias=True)

class Plotter:
    """
    Class for plotting methods.
    """
    def __init__(self):
        # get parameters from server
        self.t_win = rospy.get_param('~time_window', 5.0)
        self.time0 = 0
        self.init_time = True

        # setup subsribers
        rospy.Subscriber('estimate', Odometry, self.estimateCallback)
        rospy.Subscriber('estimate/bias', Imu, self.biasCallback)
        rospy.Subscriber('estimate/drag', Float64, self.dragCallback)
        rospy.Subscriber('ground_truth/odometry', Odometry, self.truthCallback)

        # Add Commands
        rospy.Subscriber('high_level_command', Command, self.highLevelCallback)
        rospy.Subscriber('command', Command, self.commandCallback)
        rospy.Subscriber('command/vel', Command, self.vel_commandCallback)
        rospy.Subscriber('command/acc', Command, self.acc_commandCallback)
        rospy.Subscriber('/relative_pose', Pose, self.relativePoseCallback)

        # initialize Qt gui application and window
        self.app = pg.QtGui.QApplication([])
        self.w = pg.GraphicsWindow(title='States vs Time')
        self.w.resize(1000,800)

        # initialize plots in one window
        self.p_pn = self.w.addPlot()
        self.p_pn.addLegend(size=(1,1), offset=(1,1))
        self.p_pe = self.w.addPlot()
        self.p_pd = self.w.addPlot()
        self.w.nextRow()
        self.p_phi = self.w.addPlot()
        self.p_theta = self.w.addPlot()
        self.p_psi = self.w.addPlot()
        self.w.nextRow()
        self.p_u = self.w.addPlot()
        self.p_v = self.w.addPlot()
        self.p_w = self.w.addPlot()
        self.w.nextRow()
        self.p_p = self.w.addPlot()
        self.p_q = self.w.addPlot()
        self.p_r = self.w.addPlot()
        self.w.nextRow()
        self.p_rx = self.w.addPlot()
        self.p_ry = self.w.addPlot()
        self.p_rz = self.w.addPlot()
        self.w.nextRow()
        self.p_rphi = self.w.addPlot()
        self.p_rtheta = self.w.addPlot()
        self.p_rpsi = self.w.addPlot()
        self.w.nextRow()
        self.p_mu = self.w.addPlot()
        self.p_throttle = self.w.addPlot()

        # label the plots
        self.p_pn.setLabel('left', 'pn')
        self.p_pe.setLabel('left', 'pe')
        self.p_pd.setLabel('left', 'pd')
        self.p_phi.setLabel('left', 'phi')
        self.p_theta.setLabel('left', 'theta')
        self.p_psi.setLabel('left', 'psi')
        self.p_u.setLabel('left', 'u')
        self.p_v.setLabel('left', 'v')
        self.p_w.setLabel('left', 'w')
        self.p_p.setLabel('left', 'p')
        self.p_q.setLabel('left', 'q')
        self.p_r.setLabel('left', 'r')
        self.p_rx.setLabel('left', 'rel_x')
        self.p_ry.setLabel('left', 'rel_y')
        self.p_rz.setLabel('left', 'rel_z')
        self.p_rphi.setLabel('left', 'rel_phi')
        self.p_rtheta.setLabel('left', 'rel_theta')
        self.p_rpsi.setLabel('left', 'rel_psi')
        self.p_mu.setLabel('left', 'mu')
        self.p_throttle.setLabel('left', 'throttle')

        # create curves to update later
        self.c_pn_t = self.p_pn.plot(name='truth')
        self.c_pe_t = self.p_pe.plot()
        self.c_pd_t = self.p_pd.plot()
        self.c_phi_t = self.p_phi.plot()
        self.c_theta_t = self.p_theta.plot()
        self.c_psi_t = self.p_psi.plot()
        self.c_u_t = self.p_u.plot()
        self.c_v_t = self.p_v.plot()
        self.c_w_t = self.p_w.plot()
        self.c_p_t = self.p_p.plot()
        self.c_q_t = self.p_q.plot()
        self.c_r_t = self.p_r.plot()
        self.c_rx_t = self.p_rx.plot()
        self.c_ry_t = self.p_ry.plot()
        self.c_rz_t = self.p_rz.plot()
        self.c_rphi_t = self.p_rphi.plot()
        self.c_rtheta_t = self.p_rtheta.plot()
        self.c_rpsi_t = self.p_rpsi.plot()

        self.c_pn_e = self.p_pn.plot(name='estimate')
        self.c_pe_e = self.p_pe.plot()
        self.c_pd_e = self.p_pd.plot()
        self.c_phi_e = self.p_phi.plot()
        self.c_theta_e = self.p_theta.plot()
        self.c_psi_e = self.p_psi.plot()
        self.c_u_e = self.p_u.plot()
        self.c_v_e = self.p_v.plot()
        self.c_w_e = self.p_w.plot()
        self.c_p_e = self.p_p.plot()
        self.c_q_e = self.p_q.plot()
        self.c_r_e = self.p_r.plot()
        # self.c_gx_e = self.p_gx.plot()
        # self.c_gy_e = self.p_gy.plot()
        # self.c_gz_e = self.p_gz.plot()
        # self.c_ax_e = self.p_ax.plot()
        # self.c_ay_e = self.p_ay.plot()
        # self.c_az_e = self.p_az.plot()
        self.c_mu_e = self.p_mu.plot()

        self.c_pn_c = self.p_pn.plot(name='command')
        self.c_pe_c = self.p_pe.plot()
        self.c_pd_c = self.p_pd.plot()
        self.c_phi_c = self.p_phi.plot()
        self.c_theta_c = self.p_theta.plot()
        self.c_psi_c = self.p_psi.plot()
        self.c_u_c = self.p_u.plot()
        self.c_v_c = self.p_v.plot()
        self.c_w_c = self.p_w.plot()
        # self.c_p_c = self.p_p.plot()
        # self.c_q_c = self.p_q.plot()
        self.c_r_c = self.p_r.plot()
        # self.c_gx_c = self.p_gx.plot()
        # self.c_gy_c = self.p_gy.plot()
        # self.c_gz_c = self.p_gz.plot()
        # self.c_ax_c = self.p_ax.plot()
        # self.c_ay_c = self.p_ay.plot()
        # self.c_az_c = self.p_az.plot()
        # self.c_mu_c = self.p_mu.plot()
        self.c_throttle_c = self.p_throttle.plot()

        # initialize state variables
        self.time_t = 0
        self.pn_t = 0
        self.pe_t = 0
        self.pd_t = 0
        self.u_t = 0
        self.v_t = 0
        self.w_t = 0
        self.phi_t = 0
        self.theta_t = 0
        self.psi_t = 0
        self.p_t = 0
        self.q_t = 0
        self.r_t = 0
        self.rx_t = 0
        self.ry_t = 0
        self.rz_t = 0
        self.rphi_t = 0
        self.rtheta_t = 0
        self.rpsi_t = 0

        self.time_e = 0
        self.pn_e = 0
        self.pe_e = 0
        self.pd_e = 0
        self.u_e = 0
        self.v_e = 0
        self.w_e = 0
        self.phi_e = 0
        self.theta_e = 0
        self.psi_e = 0
        self.p_e = 0
        self.q_e = 0
        self.r_e = 0
        # self.gx_e = 0
        # self.gy_e = 0
        # self.gz_e = 0
        # self.ax_e = 0
        # self.ay_e = 0
        # self.az_e = 0
        self.mu_e = 0

        self.time_c = 0
        self.pn_c = 0
        self.pe_c = 0
        self.pd_c = 0
        self.u_c = 0
        self.v_c = 0
        self.w_c = 0
        self.phi_c = 0
        self.theta_c = 0
        self.psi_c = 0
        # self.p_c = 0
        # self.q_c = 0
        self.r_c = 0
        # self.gx_c = 0
        # self.gy_c = 0
        # self.gz_c = 0
        # self.ax_c = 0
        # self.ay_c = 0
        # self.az_c = 0
        # self.mu_c = 0
        self.throttle_c = 0

        # truth/estimate/commands storage lists
        self.estimates = []
        self.truths = []
        self.commands = []

        # plot list
        self.p_list = [self.p_pn, self.p_pe, self.p_pd, self.p_phi, self.p_theta, self.p_psi, self.p_u, self.p_v, self.p_w, self.p_p, self.p_q, self.p_r, self.p_rx, self.p_ry, self.p_rz, self.p_rphi, self.p_rtheta, self.p_rpsi, self.p_mu]

        # curve lists
        self.c_list_t = [self.c_pn_t, self.c_pe_t, self.c_pd_t, self.c_phi_t, self.c_theta_t, self.c_psi_t, self.c_u_t, self.c_v_t, self.c_w_t, self.c_p_t, self.c_q_t, self.c_r_t, self.c_rx_t, self.c_ry_t, self.c_rz_t, self.c_rphi_t, self.c_rtheta_t, self.c_rpsi_t]
        self.c_list_e = [self.c_pn_e, self.c_pe_e, self.c_pd_e, self.c_phi_e, self.c_theta_e, self.c_psi_e, self.c_u_e, self.c_v_e, self.c_w_e, self.c_p_e, self.c_q_e, self.c_r_e, self.c_mu_e]
        # self.c_list_c = [self.c_pn_c, self.c_pe_c, self.c_pd_c, self.c_u_c, self.c_v_c, self.c_w_c, self.c_phi_c, self.c_theta_c, self.c_psi_c, self.c_p_c, self.c_q_c, self.c_r_c, self.c_gx_c, self.c_gy_c, self.c_gz_c, self.c_ax_c, self.c_ay_c, self.c_az_c, self.c_mu_c]
        self.c_list_c = [self.c_pn_c, self.c_pe_c, self.c_pd_c, self.c_phi_c, self.c_theta_c, self.c_psi_c, self.c_u_c, self.c_v_c, self.c_w_c, self.c_r_c, self.c_throttle_c]

    # method for updating each states
    def update(self):
        # pack stored data into lists
        self.truths.append([self.time_t, self.pn_t, self.pe_t, self.pd_t, self.phi_t, self.theta_t, self.psi_t, self.u_t, self.v_t, self.w_t, self.p_t, self.q_t, self.r_t, self.rx_t, self.ry_t, self.rz_t, self.rphi_t, self.rtheta_t, self.rpsi_t])
        self.estimates.append([self.time_e, self.pn_e, self.pe_e, self.pd_e, self.phi_e, self.theta_e, self.psi_e, self.u_e, self.v_e, self.w_e, self.p_e, self.q_e, self.r_e, self.mu_e])
        self.commands.append([self.time_e, self.pn_c, self.pe_c, self.pd_c, self.phi_c, self.theta_c, self.psi_c, self.u_c, self.v_c, self.w_c, self.r_c, self.throttle_c])

        # discard data outside desired plot time window
        for i in range(0,1000):
            if self.truths[0][0] < self.truths[-1][0] - self.t_win:
                self.truths.pop(0)
            if self.estimates[0][0] < self.estimates[-1][0] - self.t_win:
                self.estimates.pop(0)
            if self.commands[0][0] < self.commands[-1][0] - self.t_win:
                self.commands.pop(0)

        # set the window widths
        for i in range(0,len(self.p_list)):
        	self.p_list[i].setLimits(xMin=self.estimates[-1][0] - self.t_win, xMax=self.estimates[-1][0])

        # stack the data lists
        truths_array = np.vstack(self.truths)
        time_t_array = truths_array[:,0]

        estimates_array = np.vstack(self.estimates)
        time_e_array = estimates_array[:,0]

        commands_array = np.vstack(self.commands)
        time_c_array = commands_array[:,0]

        # set the truth states
        for i in range(0,len(self.c_list_t)):
	        self.c_list_t[i].setData(time_t_array, truths_array[:,i+1], pen=(255,0,0))

        # set the estimated states
        for i in range(0,len(self.c_list_e)):
	        self.c_list_e[i].setData(time_e_array, estimates_array[:,i+1], pen=(0,255,0))

        # set the commanded states
        for i in range(0,len(self.c_list_c)):
	        self.c_list_c[i].setData(time_c_array, commands_array[:,i+1], pen=(0,0,255))

        # update the plotted data
        self.app.processEvents()


    def truthCallback(self, msg):
        # unpack positions and linear velocities
        self.pn_t = msg.pose.pose.position.x
        self.pe_t = msg.pose.pose.position.y
        self.pd_t = msg.pose.pose.position.z
        self.u_t = msg.twist.twist.linear.x
        self.v_t = msg.twist.twist.linear.y
        self.w_t= msg.twist.twist.linear.z

        # orientation in quaternion form
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w)

        # Use ROS tf to convert to Euler angles from quaternion
        euler = tf.transformations.euler_from_quaternion(quaternion)

        # unpack angles and angular velocities
        self.phi_t = euler[0]
        self.theta_t = euler[1]
        self.psi_t = euler[2]
        self.p_t = msg.twist.twist.angular.x
        self.q_t = msg.twist.twist.angular.y
        self.r_t = msg.twist.twist.angular.z

        # unpack time
        if self.init_time == True:
        	self.time0 = msg.header.stamp.to_sec()
        	self.init_time = False
        self.time_t = msg.header.stamp.to_sec() - self.time0


    def estimateCallback(self, msg):
        # unpack positions and linear velocities
        self.pn_e = msg.pose.pose.position.x
        self.pe_e = msg.pose.pose.position.y
        self.pd_e = msg.pose.pose.position.z
        self.u_e = msg.twist.twist.linear.x
        self.v_e = msg.twist.twist.linear.y
        self.w_e = msg.twist.twist.linear.z

        # orientation in quaternion form
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w)

        # Use ROS tf to convert to Euler angles from quaternion
        euler = tf.transformations.euler_from_quaternion(quaternion)

        # unpack angles and angular velocities
        self.phi_e = euler[0]
        self.theta_e = euler[1]
        self.psi_e = euler[2]
        self.p_e = msg.twist.twist.angular.x
        self.q_e = msg.twist.twist.angular.y
        self.r_e = msg.twist.twist.angular.z

        # unpack time
        if self.init_time == True:
        	self.time0 = msg.header.stamp.to_sec()
        	self.init_time = False
        self.time_e = msg.header.stamp.to_sec() - self.time0


    def biasCallback(self, msg):
        self.gx_e = msg.angular_velocity.x
        self.gy_e = msg.angular_velocity.y
        self.gz_e = msg.angular_velocity.z
        self.ax_e = msg.linear_acceleration.x
        self.ay_e = msg.linear_acceleration.y
        self.az_e = msg.linear_acceleration.z

    def dragCallback(self, msg):
        self.mu_e = msg.data

    def highLevelCallback(self, msg):
        if msg.mode == Command.MODE_XPOS_YPOS_YAW_ALTITUDE:
            self.pn_c = msg.x
            self.pe_c = msg.y
            self.psi_c = msg.z
            # self.pd_c = msg.F
        if msg.mode == Command.MODE_XVEL_YVEL_YAWRATE_ALTITUDE:
            self.u_c = msg.x
            self.v_c = msg.y
            self.r_c = msg.z
            self.pd_c = msg.F
        self.time_c = msg.header.stamp.to_sec() - self.time0

    def commandCallback(self, msg):
        if msg.mode == Command.MODE_ROLL_PITCH_YAWRATE_THROTTLE:
            self.phi_c = msg.x
            self.theta_c = msg.y
            self.r_c = msg.z
            self.throttle_c = msg.F
        self.time_c = msg.header.stamp.to_sec() - self.time0

    def vel_commandCallback(self, msg):
        if msg.mode == Command.MODE_XVEL_YVEL_YAWRATE_ALTITUDE:
            self.u_c = msg.x
            self.v_c = msg.y
            self.r_c = msg.z
            self.pd_c = msg.F
        self.time_c = msg.header.stamp.to_sec() - self.time0

    def acc_commandCallback(self, msg):
        if msg.mode == Command.MODE_XACC_YACC_YAWRATE_AZ:
            # self.u_c = msg.x
            # self.v_c = msg.y
            self.w_c = msg.z
            # self.pd_c = msg.F
        self.time_c = msg.header.stamp.to_sec() - self.time0

    def relativePoseCallback(self, msg):
        self.rx_t = msg.position.x
        self.ry_t = msg.position.y
        self.rz_t = msg.position.z

        # orientation in quaternion form
        quaternion = (
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w)

        # Use ROS tf to convert to Euler angles from quaternion
        euler = tf.transformations.euler_from_quaternion(quaternion)

        # unpack angles and angular velocities
        self.rphi_t = euler[0]
        self.rtheta_t = euler[1]
        self.rpsi_t = euler[2]


################################################################################
################################################################################
################################################################################


def main():
    # initialize node
    rospy.init_node('state_plotter', anonymous=True)

    # initialize plotter class
    plotter = Plotter()

    # listen for messages and plot
    while not rospy.is_shutdown():
        try:
            # plot the local positions of each vehicle
            plotter.update()

            # let it rest a bit
            time.sleep(0.001)
        except rospy.ROSInterruptException:
            print "exiting...."
            return

if __name__ == '__main__':
    main()
