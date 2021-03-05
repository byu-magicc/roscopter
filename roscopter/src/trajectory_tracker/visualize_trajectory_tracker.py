from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt 
import numpy as np
import mpl_toolkits.mplot3d.art3d as art3d
from matplotlib.patches import Circle, Arc
from scipy.spatial.transform import Rotation as R

def arcData(arc_mag , ortho_axis):
    if ortho_axis == "x":
        xdata = np.zeros(100)
        ydata = np.cos(np.linspace(0,np.pi*2,100)*arc_mag)
        zdata = np.sin(np.linspace(0,np.pi*2,100)*arc_mag)

    elif ortho_axis == "y":
        xdata = np.cos(np.linspace(0,np.pi*2,100)*arc_mag)
        ydata = np.zeros(100)
        zdata = np.sin(np.linspace(0,np.pi*2,100)*arc_mag)

    else:
        xdata = np.cos(np.linspace(0,np.pi*2,100)*arc_mag)
        ydata = np.sin(np.linspace(0,np.pi*2,100)*arc_mag)
        zdata = np.zeros(100)
    return xdata, ydata, zdata




z_pos = 0
x_pos = 0
y_pos = 0
z_pos_des = 2
x_pos_des = 1
y_pos_des = 1
z_vel = 1
x_vel = 2
y_vel = 1
z_vel_des = 2
x_vel_des = 2
y_vel_des = -1
z_accel = -2
x_accel = 1
y_accel = 3
z_accel_des = 1.5
x_accel_des = 1.8
y_accel_des = -0.5
roll_rate = -3
pitch_rate = 10
yaw_rate = 5
rotation = R.from_quat([0, 0, np.sin(np.pi/4), np.cos(np.pi/4)])
xdir = np.array([1,0,0])
ydir = np.array([0,1,0])
desired_xdir = np.dot(np.array([1,0,0]),rotation.as_matrix())
desired_ydir = np.dot(np.array([0,1,0]),rotation.as_matrix())
desired_force = np.array([1,1,1])



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

#position
ax.scatter3D(x_pos, y_pos, z_pos,color='black',label='pos')
ax.scatter3D(x_pos_des, y_pos_des, z_pos_des, color='green',label='des_pos')

#x orientation
ax.plot3D([x_pos, x_pos + xdir.item(0)], [y_pos, y_pos + xdir.item(1)], [z_pos, z_pos + xdir.item(2)], color='black', linestyle='-',label='x_dir')
ax.plot3D([x_pos_des, x_pos_des + desired_xdir.item(0)], [y_pos_des, y_pos_des + desired_xdir.item(1)], [z_pos_des, z_pos_des + desired_xdir.item(2)], color='green', linestyle='-',label='des x_dir')

#y orientation
ax.plot3D([x_pos, x_pos + ydir.item(0)], [y_pos, y_pos + ydir.item(1)], [z_pos, z_pos + ydir.item(2)], color='gray', linestyle='-',label='y_dir')
ax.plot3D([x_pos_des, x_pos_des + desired_ydir.item(0)], [y_pos_des, y_pos_des + desired_ydir.item(1)], [z_pos_des, z_pos_des + desired_ydir.item(2)], color='limegreen', linestyle='-',label='des y_dir')

#velocity
ax.plot3D([x_pos, x_pos + x_vel], [y_pos, y_pos + y_vel], [z_pos, z_pos + z_vel], color='gray', linestyle='--',label='vel')
ax.plot3D([x_pos_des, x_pos_des + x_vel_des], [y_pos_des, y_pos_des + y_vel_des], [z_pos_des, z_pos_des + z_vel_des], color='green', linestyle='--',label='des_vel')

#acceleration
ax.plot3D([x_pos, x_pos + x_accel], [y_pos, y_pos + y_accel], [z_pos, z_pos + z_accel], color='gray', linestyle=':',label='accel')
ax.plot3D([x_pos_des, x_pos_des + x_accel_des], [y_pos_des, y_pos_des + y_accel_des], [z_pos_des, z_pos_des + z_accel_des], color='green', linestyle=':',label='des_accel')

#desired_force
ax.plot3D([x_pos, x_pos + desired_force.item(0)], [y_pos, y_pos +  desired_force.item(1)], [z_pos, z_pos +  desired_force.item(2)], color='green', linestyle='-.',label='desired_force')

#rate commands
angle_rate = np.sqrt(roll_rate**2 + pitch_rate**2 + yaw_rate**2)
roll_mag = roll_rate/angle_rate
pitch_mag = pitch_rate/angle_rate
yaw_mag = yaw_rate/angle_rate

#roll_rate
roll_x, roll_y, roll_z = arcData(roll_mag , "x")
ax.plot3D(roll_x,roll_y,roll_z,color='red',label='roll_rate_c')
ax.scatter3D(roll_x[-1],roll_y[-1],roll_z[-1],color='red',marker='>')

#pitch_rate
pitch_x, pitch_y, pitch_z = arcData(pitch_mag , "y")
ax.plot3D(pitch_x,pitch_y,pitch_z,color='purple',label='pitch_rate_c')
ax.scatter3D(pitch_x[-1],pitch_y[-1],pitch_z[-1],color='purple',marker='^')

#yaw_rate
yaw_x, yaw_y, yaw_z = arcData(yaw_mag , "z")
ax.plot3D(yaw_x,yaw_y,yaw_z,color='gold',label="yaw_rate_C")
ax.scatter3D(yaw_x[-1],yaw_y[-1],yaw_z[-1],color='gold',marker='<')

ax.legend()
plt.show()

