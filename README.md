ROScopter
=========

This repository contains the ROS packages associated with flying a multirotor on the [ROSflight](http://rosflight.org) autopilot stack. The `roscopter` ROS package contains a high-level multirotor autopilot while the `roscopter_sim` package is used for simulation.

## Running Simulation ##

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


# TODO: #
 1. Improve GPS estimation performance using Ublox GPS 
 2. Support additional altitude sensor support (laser/sonar)
 3. Heading calibration routine (magnetometer)
 4. Include wind estimation  
