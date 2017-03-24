#include <ros/ros.h>
#include "mekf/mekf.h"

int main(int argc, char** argv)
{
	// initialize filter
	ros::init(argc, argv, "mekf_node");
	mekf::kalmanFilter Thing;

	// run estimator at 200Hz
	ros::Rate loop_rate(200);
	while (ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
