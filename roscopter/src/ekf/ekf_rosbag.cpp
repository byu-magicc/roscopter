#include "ekf/ekf_rosbag.h"

namespace roscopter::ekf
{

ROSbagParser::ROSbagParser(int argc, char** argv)
{
    start_ = 0;
    duration_ = 1e3;

    param_filename_ = SALSA_DIR"/params/salsa.yaml";
    getArgs(argc, argv);

    loadParams();
    openBag();
    //    getMocapOffset();

    imu_count_ = 0;
    salsa_.init(param_filename_);
    truth_log_.open(salsa_.log_prefix_ + "/../Truth.log");
    imu_log_.open(salsa_.log_prefix_ + "/Imu.log");
}



}


int main()
{

}
