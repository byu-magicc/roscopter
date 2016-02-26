# ros_copter

This is intended to eventually be a fully-featured multirotor autopilot for ROS.  It will be built according to the method published in [Quadrotor Dynamics and Control](http://scholarsarchive.byu.edu/cgi/viewcontent.cgi?article=2324&context=facpub), so as to allow anyone to easily understand, modify and use the code.  The framework developed in the afore mentioned reference closely resembles the fixed wing framework developed in Small Unmanned Aircraft by Beard and McLain.  This framework is inherently modular and extensively documented so as to aid the user in understanding and extending for personal use.

The package is intended to be used with fcu\_io or fcu\_sim, for hardware with a naze32 or spracingf3 (or derivatives) or simulation, respectively.

It is a single ROS package, with several nodes.

# - EKF 
# - PID Controller
# - Path Planner
# - Path Follower
# - Monocular Visual Odometry
