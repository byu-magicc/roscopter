#include <ros/ros.h>
#include "mekf/mekf.h"

int main(int argc, char** argv)
{
	// initialize filter
	ros::init(argc, argv, "mekf_node");
	mekf::kalmanFilter estimator;

	// allow each sensor to update as each is received
	ros::spin();

	return 0;
}
