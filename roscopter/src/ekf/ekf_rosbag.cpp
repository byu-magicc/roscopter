#include "ekf/ekf_rosbag.h"

using namespace Eigen;
using namespace std;
namespace roscopter::ekf
{

ROSbagParser::ROSbagParser(int argc, char** argv)
{
  start_ = 0;
  duration_ = 1e3;

  param_filename_ = ROSCOPTER_DIR"/params/ekf.yaml";
  getArgs(argc, argv);

  loadParams();
  openBag();

  ekf_.init(param_filename_);
}

void ROSbagParser::loadParams()
{
  get_yaml_node("bag_name", param_filename_, bag_filename_);
  get_yaml_node("imu_topic", param_filename_, imu_topic_);
  get_yaml_node("start_time", param_filename_, start_);
  get_yaml_node("duration", param_filename_, duration_);
}

void ROSbagParser::displayHelp()
{
    std::cout << "ROS bag parser\n";
    std::cout << "-h            display this message\n";
    std::cout << "-f <filename> configuration yaml file to parse, [" ROSCOPTER_DIR "/params/ekf.yaml]" << std::endl;
    exit(0);
}

void ROSbagParser::getArgs(int argc, char** argv)
{
    InputParser argparse(argc, argv);

    if (argparse.cmdOptionExists("-h"))
        displayHelp();
    argparse.getCmdOption("-f", param_filename_);
}

void ROSbagParser::openBag()
{
    try
    {
        bag_.open(bag_filename_.c_str(), rosbag::bagmode::Read);
        view_ = new rosbag::View(bag_);
    }
    catch(rosbag::BagIOException e)
    {
        fprintf(stderr, "unable to load rosbag %s, %s", bag_filename_.c_str(), e.what());
        throw e;
    }

    bag_start_ = view_->getBeginTime() + ros::Duration(start_);
    bag_end_ = bag_start_ + ros::Duration(duration_);

    if (bag_end_ > view_->getEndTime())
        bag_end_ = view_->getEndTime();

    delete view_;
    view_ = new rosbag::View(bag_, bag_start_, bag_end_);
}

void ROSbagParser::parseBag()
{
    ProgressBar prog(view_->size(), 80);
    int i = 0;
    for(rosbag::MessageInstance const m  : (*view_))
    {
        if (m.getTime() < bag_start_)
            continue;

        if (m.getTime() > bag_end_)
            break;

        prog.print(i++, (m.getTime() - bag_start_).toSec());

        if (m.isType<sensor_msgs::Imu>() && m.getTopic().compare(imu_topic_) == 0)
            ekf_.imuCallback(m.instantiate<sensor_msgs::Imu>());
        else if (m.isType<geometry_msgs::PoseStamped>())
            ekf_.poseCallback(m.instantiate<geometry_msgs::PoseStamped>());
        else if (m.isType<nav_msgs::Odometry>())
            ekf_.odomCallback(m.instantiate<nav_msgs::Odometry>());
        else if (m.isType<rosflight_msgs::GNSS>())
          ekf_.gnssCallback(m.instantiate<rosflight_msgs::GNSS>());
#ifdef INERTIAL_SENSE
        else if (m.isType<inertial_sense::GPS>())
          ekf_.gnssCallbackInertialSense(m.instantiate<inertial_sense::GPS>());
#endif
#ifdef F9P_GNSS
        else if (m.isType<f9p_gnss::GPS>())
          ekf_.gnssCallbackF9P(m.instantiate<f9p_gnss::GPS>());
#endif
    }
    prog.finished();
    cout << endl;
    cout.flush();
}




}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "ekf_rosbag_parser");
  roscopter::ekf::ROSbagParser thing(argc, argv);
  thing.parseBag();
}
