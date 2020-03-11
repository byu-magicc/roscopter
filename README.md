ROScopter
=========

This repository contains the ROS packages associated with flying a multirotor on the [ROSflight](http://rosflight.org) autopilot stack. The `roscopter` ROS package contains a high-level multirotor autopilot while the `roscopter_sim` package is used for simulation.

## Running Simulation ##

### Simplified Dynamics Model

Clone this repo, the [ROSflight_plugins](https://github.com/byu-magicc/rosflight_plugins) repo, and the [ROSflight](https://github.com/rosflight/rosflight.git) repo into `catkin_ws/src`, you can run the Gazebo simulator with

```bash
$ catkin_make
$ source devel/setup.bash
$ roslaunch roscopter_sim multirotor.launch
```

This simulation does not simulate the firmware and does not require a joystick. It has the same API as the SIL stack, and can be useful for developing high-level algorithms, however it makes some simplifying assumptions in the dynamics which reduces the fidelity of the simulation.

### Software In The Loop
To run the `rosflight` SIL simulation [information](http://docs.rosflight.org/en/latest/user-guide/gazebo_simulation/) with the full roscopter stack, clone the [ROSflight](github.com/rosflight/rosflight) repo, connect a joystick or transmitter and run

```bash
$ catkin_make
$ source devel/setup.bash
$ roslaunch roscopter copter_sil.launch
```
# Core Autopilot Components #

## EKF ##

The EKF estimates the position of the multirotor given GPS (or MoCap if indoors) and IMU measurements.

### Controller ###

A PID controller is used.

### Waypoint Manager ###

Velocity manager and commander.

# Motion Capture

The [Motion Capture Wiki page](https://magiccvs.byu.edu/wiki/#!sw_guides/mocap_room_tutorial.md#VRPN_Installation) has instructions on how to include motion capture data in your ROS network.


# TODO: #
 1. Improve GPS estimation performance using Ublox GPS
 2. Support additional altitude sensor support (laser/sonar)
 3. Heading calibration routine (magnetometer)
 4. Include wind estimation
 5. Add simpler way to add new waypoints
 6. Get states_plotter.py and commands_plotter.py
 7. Develop procedure for defining quadrotor physical parameters.  (add to wiki)
