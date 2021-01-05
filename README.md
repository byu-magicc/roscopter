ROScopter
=========

------ Getting Started ------

Install the latest version of ROS on your computer. Next install catkin and create a catkin workspace.

Make sure to install the following packages to your python environment.

$ pip install pyyaml

$ pip install numpy

$ pip install rospkg

$ pip install pycryptodomex

$ pip install gnupg

If you would like to use the keyboard or a joypad to control the multirotor make sure to install pygame.

$ pip install pygame

Clone this repo (ROScopter) and the ROSflight repo into your catkin_ws/src.

To use the gazebo simulation package, clone the ROSflight_plugins repo. If you would like to use the keyboard as a RC controller, clone the rosflight_joy repo as well.

If you would like to use the rosflight_holodeck simulation package, clone the rosflight_holodeck repo and follow installation instructions on the linked page.

Build and source the catkin workspace by navigating to the workspace and running these commands.

$ catkin_make

$ source devel/setup.bash


----- ROScopter Sim - Simplified Dynamics Model -------

This simulation does not simulate the firmware (rosflight) and does not require RC input. It has the same API as the SIL stack, and can be useful for developing high-level algorithms, however it makes some simplifying assumptions in the dynamics which reduces the fidelity of the simulation.

You can run the Gazebo simulator with

$ roslaunch roscopter_sim gazebo.launch

Make sure to push the play button in the Gazebo simulator.


# TODO: #

1. Develop and document process noise std tuning methods.

2. Develop and document a tuning procedure for the multirotor. This includes tuning all the gain parameters for the flight vehicle.

3. Standardize the flight controller board. Board sensor suite needs to include an IMU, barometer and magnetometer. Current board and sensor combination is becoming obsolete.

4. Implement a trajectory follower control node. Should recieve inertial position, velocity, acceleration and heading, and output angular rate and throttle.

5. Develop and document a magnetometer calibration routine and incorporate magnetometer measurements to the estimator. This would enable accurate heading estimates at initialization and during hover flight.

6. Support an additional altitude sensor for landing. Possible options include sonar, laser, or camera.

7. Create a failsafe mode so the quadrotor can have a safe landing. Use cases include when RC connection is lost.

8. Improve flight performance in windy conditions. Most accurate solution is to incorporate wind estimation. This is possible without air sensors if you can estimate parameters of the flight vehicle such as drag and thrust coefficients.

9. Enable Roscopter to fly in reference to the vehicle 1 frame and body fixed frames as well as the inertial frame.
