#include "ekf/ekf.h"

#include <experimental/filesystem>

namespace fs = std::experimental::filesystem;

namespace roscopter::ekf
{

void EKF::initLog()
{
    fs::create_directories(log_prefix_);
    for (int i = 0; i < log_names_.size(); i++)
    {
        logs_[i]->open(fs::path(log_prefix_) / log_names_[i]);
    }
}


void EKF::logState()
{
    logs_[LOG_STATE]->log(x().t);
    logs_[LOG_STATE]->logVectors(x().arr);
}


void EKF::logCov()
{
    logs_[LOG_STATE]->log(x().t);
    logs_[LOG_STATE]->log(P());
}


}

