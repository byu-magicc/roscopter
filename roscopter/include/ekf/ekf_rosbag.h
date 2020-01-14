#pragma once

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rosbag/view.h>

#include "ekf/ekf_ros.h"

#include "roscopter_utils/yaml.h"
#include "roscopter_utils/logger.h"
#include "roscopter_utils/input_parser.h"
#include "roscopter_utils/progress_bar.h"
#include "roscopter_utils/gnss.h"

namespace roscopter::ekf
{

class ROSbagParser
{
public:
    ROSbagParser(int argc, char** argv);

    void getArgs(int argc, char** argv);
    void loadParams();
    void displayHelp();
    void openBag();
    void parseBag();

private:
    void odomCB(const nav_msgs::OdometryConstPtr& msg);
    void poseCB(const geometry_msgs::PoseStampedConstPtr& msg);
    void gnssCB(const rosflight_msgs::GNSSConstPtr& msg);

    rosbag::Bag bag_;
    rosbag::View* view_;
    std::string bag_filename_;
    std::string param_filename_;
    std::string log_prefix_;
    double start_;
    double duration_;
    double end_;
    bool got_imu_;

    ros::Time bag_start_;
    ros::Time bag_duration_;
    ros::Time bag_end_;

    EKF_ROS ekf_;

    std::string status_topic_;
    std::string imu_topic_;
    std::string baro_topic_;
    std::string range_topic_;
    std::string pose_topic_;
    std::string odom_topic_;
    std::string gnss_topic_;
    std::string ublox_gnss_topic_;
    std::string inertial_sense_gnss_topic_;

    Matrix6d imu_R_;
    Matrix6d mocap_R_;
    Matrix6d gnss_R_;

    Logger lla_log_;
    Logger truth_log_;
    Logger imu_log_;
};

}

