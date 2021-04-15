from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt 
import numpy as np
import mpl_toolkits.mplot3d.art3d as art3d
from matplotlib.patches import Circle, Arc
from scipy.spatial.transform import Rotation as R
from lqr_controller import LQRController

def arcData(arc_mag , ortho_axis):
    if ortho_axis == "x":
        xdata = np.zeros(100)
        ydata = np.cos(np.linspace(0,np.pi*2,100)*arc_mag)
        zdata = np.sin(np.linspace(0,np.pi*2,100)*arc_mag)

    elif ortho_axis == "y":
        xdata = np.cos(np.linspace(0,np.pi*2,100)*arc_mag)
        ydata = np.zeros(100)
        zdata = -np.sin(np.linspace(0,np.pi*2,100)*arc_mag)

    else:
        xdata = np.cos(np.linspace(0,np.pi*2,100)*arc_mag)
        ydata = np.sin(np.linspace(0,np.pi*2,100)*arc_mag)
        zdata = np.zeros(100)
    return xdata, ydata, zdata

#initialize trajectory tracker
lqr_control = LQRController()

#initialize states
x_pos = 0
y_pos = 0
z_pos = 0
x_vel = 0
y_vel = 0
z_vel = 0
attitude_as_quat = np.array([1,0,0,0])
time_step = 0.01
position = np.array([[x_pos],[y_pos],[z_pos]])
velocity = np.array([[x_vel],[y_vel],[z_vel]])
lqr_control.updateState(position,velocity, attitude_as_quat, time_step)
x_accel = lqr_control.acceleration.item(0)
y_accel = lqr_control.acceleration.item(1)
z_accel = lqr_control.acceleration.item(2)
xdir = np.dot(np.array([1,0,0]),lqr_control.attitude_in_SO3)
ydir = np.dot(np.array([0,1,0]),lqr_control.attitude_in_SO3)
zdir = np.dot(np.array([0,0,1]),lqr_control.attitude_in_SO3)

#initialize desired states
t = 3
radius = 2
height = 4
z_pos_des = -height
x_pos_des = 2 #np.sin(t)*radius
y_pos_des = 2 #np.cos(t)*radius
z_vel_des = 0
x_vel_des = 0#np.cos(t)*radius
y_vel_des = 0#-np.sin(t)*radius
z_accel_des = 0
x_accel_des = 1#-np.sin(t)*radius
y_accel_des = 1#-np.cos(t)*radius
z_jerk_des = 0
x_jerk_des = 0#-np.cos(t)*radius
y_jerk_des = 0#np.sin(t)*radius
desired_heading = np.pi/5
desired_heading_rate = 0
desired_position = np.array([[x_pos_des], [y_pos_des], [z_pos_des]])
desired_velocity = np.array([[x_vel_des], [y_vel_des], [z_vel_des]])
desired_acceleration = np.array([[x_accel_des], [y_accel_des], [z_accel_des]])
desired_jerk = np.array([[x_jerk_des], [y_jerk_des], [z_jerk_des]])
lqr_control.updateDesiredState(desired_position, desired_velocity, desired_acceleration, desired_jerk, desired_heading, desired_heading_rate)

# # calculate commands and desired values
desired_accel_vec = lqr_control.computeDesiredThrustAccelerationVector()
desired_heading_vec = lqr_control.computeDesiredHeadingVector()
desired_rotation = lqr_control.computeDesiredAttitudeSO3(desired_accel_vec, desired_heading_vec)
desired_xdir = np.dot(desired_rotation, np.array([1,0,0]))
desired_ydir = np.dot(desired_rotation, np.array([0,1,0]))
desired_zdir = np.dot(desired_rotation, np.array([0,0,1]))
# desired_thrust = traj_track.computeDesiredThrust(desired_force_vec)
# throttle_mag = traj_track.computeThrottleCommand(desired_thrust)
# thrust_vec = np.dot(traj_track.attitude_in_SO3, np.array([0,0,-1]))*desired_thrust
# rosflight_command = traj_track.computeRosflightCommand()
# roll_rate = rosflight_command.item(0)
# pitch_rate = rosflight_command.item(1)
# yaw_rate = rosflight_command.item(2)
# throttle = rosflight_command.item(3)




fig = plt.figure()
ax = plt.axes(projection='3d')
min_lim = -5
max_lim = 5
ax.set_xlim3d(min_lim, max_lim)
ax.set_ylim3d(min_lim, max_lim)
ax.set_zlim3d(min_lim, max_lim)
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')
ax.legend()

##### position ######
ax.scatter3D(x_pos, y_pos, z_pos,color='black',label='pos')
ax.scatter3D(x_pos_des, y_pos_des, z_pos_des, color='green',label='des_pos')

##### orientation ######
#desired heading
ax.plot3D([x_pos_des, x_pos_des + desired_heading_vec.item(0)], [y_pos_des, y_pos_des + desired_heading_vec.item(1)], [z_pos_des, z_pos_des + desired_heading_vec.item(2)], color='r', linestyle='-',label='des heading')
#desired acceleration due to thrust
ax.plot3D([x_pos_des, x_pos_des + desired_accel_vec.item(0)], [y_pos_des, y_pos_des + desired_accel_vec.item(1)], [z_pos_des, z_pos_des + desired_accel_vec.item(2)], color='r', linestyle='-',label='des acceleration')
#x dir
ax.plot3D([x_pos, x_pos + xdir.item(0)], [y_pos, y_pos + xdir.item(1)], [z_pos, z_pos + xdir.item(2)], color='black', linestyle='-',label='x_dir')
#desired x dir
ax.plot3D([x_pos_des, x_pos_des + desired_xdir.item(0)], [y_pos_des, y_pos_des + desired_xdir.item(1)], [z_pos_des, z_pos_des + desired_xdir.item(2)], color='green', linestyle='-',label='des x_dir')
#z dir
ax.plot3D([x_pos, x_pos + zdir.item(0)], [y_pos, y_pos + zdir.item(1)], [z_pos, z_pos + zdir.item(2)], color='black', linestyle='-',label='z_dir')
#desired z dir
ax.plot3D([x_pos_des, x_pos_des + desired_zdir.item(0)], [y_pos_des, y_pos_des + desired_zdir.item(1)], [z_pos_des, z_pos_des + desired_zdir.item(2)], color='green', linestyle='-',label='des z_dir')

ax.legend()
plt.show()

# #y orientation
# ax.plot3D([x_pos, x_pos + ydir.item(0)], [y_pos, y_pos + ydir.item(1)], [z_pos, z_pos + ydir.item(2)], color='gray', linestyle='-',label='y_dir')
# ax.plot3D([x_pos_des, x_pos_des + desired_ydir.item(0)], [y_pos_des, y_pos_des + desired_ydir.item(1)], [z_pos_des, z_pos_des + desired_ydir.item(2)], color='limegreen', linestyle='-',label='des y_dir')

# #velocity
# ax.plot3D([x_pos, x_pos + x_vel], [y_pos, y_pos + y_vel], [z_pos, z_pos + z_vel], color='gray', linestyle='--',label='vel')
# ax.plot3D([x_pos_des, x_pos_des + x_vel_des], [y_pos_des, y_pos_des + y_vel_des], [z_pos_des, z_pos_des + z_vel_des], color='green', linestyle='--',label='des_vel')

# #acceleration
# ax.plot3D([x_pos, x_pos + x_accel], [y_pos, y_pos + y_accel], [z_pos, z_pos + z_accel], color='gray', linestyle=':',label='accel')
# ax.plot3D([x_pos_des, x_pos_des + x_accel_des], [y_pos_des, y_pos_des + y_accel_des], [z_pos_des, z_pos_des + z_accel_des], color='green', linestyle=':',label='des_accel')

# #desired_force
# ax.plot3D([x_pos, x_pos + desired_force_vec.item(0)], [y_pos, y_pos +  desired_force_vec.item(1)], [z_pos, z_pos +  desired_force_vec.item(2)], color='green', linestyle='-.',label='desired_force')

# #thrust_vec
# ax.plot3D([x_pos, x_pos + thrust_vec.item(0)], [y_pos, y_pos +  thrust_vec.item(1)], [z_pos, z_pos +  thrust_vec.item(2)], color='blue', linestyle='-.',label='thrust')

# #rate commands
# angle_rate = np.sqrt(roll_rate**2 + pitch_rate**2 + yaw_rate**2)
# roll_mag = roll_rate/angle_rate
# pitch_mag = pitch_rate/angle_rate
# yaw_mag = yaw_rate/angle_rate

# #roll_rate
# roll_x, roll_y, roll_z = arcData(roll_mag , "x")
# ax.plot3D(roll_x,roll_y,roll_z,color='red',label='roll_rate_c')
# ax.scatter3D(roll_x[-1],roll_y[-1],roll_z[-1],color='red',marker='>')

# #pitch_rate
# pitch_x, pitch_y, pitch_z = arcData(pitch_mag , "y")
# ax.plot3D(pitch_x,pitch_y,pitch_z,color='purple',label='pitch_rate_c')
# ax.scatter3D(pitch_x[-1],pitch_y[-1],pitch_z[-1],color='purple',marker='^')

# #yaw_rate
# yaw_x, yaw_y, yaw_z = arcData(yaw_mag , "z")
# ax.plot3D(yaw_x,yaw_y,yaw_z,color='gold',label="yaw_rate_C")
# ax.scatter3D(yaw_x[-1],yaw_y[-1],yaw_z[-1],color='gold',marker='<')

# ax.legend()
# plt.show()

