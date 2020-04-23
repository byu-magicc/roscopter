#include "ekf/ekf.h"

#define T transpose()

using namespace Eigen;

namespace roscopter
{
namespace ekf
{

const dxMat EKF::I_BIG = dxMat::Identity();

EKF::EKF() :
  xbuf_(100)
{

}

EKF::~EKF()
{
  for (int i = 0; i < NUM_LOGS; i++)
    delete logs_[i];
}

/////// Used to get parameters.  There is also some converting and intializing going on.  ???There is also a partial update step
// called from EKF_ROS_init
// calls EKF::init_log
//gets parameters from ekf.yaml
void EKF::load(const std::string &filename)
{

  // Constant Parameters
  // These can all be found in the ekf.yaml file
  get_yaml_eigen("p_b2g", filename, p_b2g_); //position from body frame to gps frame
  get_yaml_diag("Qx", filename, Qx_); // Additive Process Noise
  get_yaml_diag("P0", filename, P()); // Initial Uncertainty
  P0_yaw_ = P()(ErrorState::DQ + 2, ErrorState::DQ + 2); //??? ErrorState::DQ is in state.cpp, but I am not sure what it is doing.
  get_yaml_diag("R_zero_vel", filename, R_zero_vel_); //looks like the noise parameters at no velocity

  // Partial Update
  //???why do we partially update?
  // ???this is used to create a lambda matrix
  get_yaml_eigen("lambda", filename, lambda_vec_); //an array of partial updates
  const dxVec ones = dxVec::Constant(1.0); //dxVec defined in state.h, size 1 matrix 
  lambda_mat_ = ones * lambda_vec_.transpose() + lambda_vec_ * ones.transpose() -
                lambda_vec_ * lambda_vec_.transpose();

  // Measurement Flags
  //all of these can be seen on the ekf.yaml
  get_yaml_node("enable_partial_update", filename, enable_partial_update_);
  get_yaml_node("enable_out_of_order", filename, enable_out_of_order_);
  get_yaml_node("use_mocap", filename, use_mocap_);
  get_yaml_node("use_gnss", filename, use_gnss_);
  get_yaml_node("use_baro", filename, use_baro_);
  get_yaml_node("use_range", filename, use_range_);
  get_yaml_node("use_zero_vel", filename, use_zero_vel_);

  // Armed Check
  get_yaml_node("enable_arm_check", filename, enable_arm_check_);
  get_yaml_node("is_flying_threshold", filename, is_flying_threshold_);

  // load initial state
  double ref_heading;
  get_yaml_node("ref_heading", filename, ref_heading);
  q_n2I_ = quat::Quatd::from_euler(0, 0, M_PI/180.0 * ref_heading); //???this looks like a quaternion from ned to inertial frame?

  ref_lla_set_ = false; //this variable is also used in EKF_ROS::gnssCallback.  If false on receiving a gnss message, it will initialize lla with the first gnss message.
  bool manual_ref_lla;
  get_yaml_node("manual_ref_lla", filename, manual_ref_lla);
  if (manual_ref_lla) //Use manual ref_lla or use first lla from gps
  {
    Vector3d ref_lla;
    get_yaml_eigen("ref_lla", filename, ref_lla);
    std::cout << "Set ref lla: " << ref_lla.transpose() << std::endl;
    ref_lla.head<2>() *= M_PI/180.0; // convert to rad
    //x_e2n converts ecef to ned
    xform::Xformd x_e2n = x_ecef2ned(lla2ecef(ref_lla));
    x_e2I_.t() = x_e2n.t();
    x_e2I_.q() = x_e2n.q() * q_n2I_; //??? not sure what this does.

    // initialize the estimated ref altitude state
    x().ref = ref_lla(2); //state x is initialized as a state type in state.h
    ref_lat_radians_ = ref_lla(0);
    ref_lon_radians_ = ref_lla(1);

    ref_lla_set_ = true;
  }

  ground_pressure_ = 0.;
  ground_temperature_ = 0.;
  update_baro_ = false;
  get_yaml_node("update_baro_velocity_threshold", filename, update_baro_vel_thresh_);

  get_yaml_eigen("x0", filename, x0_.arr());

  initLog(filename);
}

///// Just creates a log if configured to do so in ekf.yaml
// called from EKF::load
// gets parameters from ekf.yaml
void EKF::initLog(const std::string &filename)
{
  get_yaml_node("enable_log", filename, enable_log_);
  get_yaml_node("log_prefix", filename, log_prefix_);

  std::experimental::filesystem::create_directories(log_prefix_);

  logs_.resize(NUM_LOGS);
  for (int i = 0; i < NUM_LOGS; i++)
    logs_[i] = new Logger(log_prefix_ + "/" + log_names_[i] + ".bin");
}


///// iniitializes certain states if ref_lla has been set.
//called by EKF::propagate
void EKF::initialize(double t)
{
  x().t = t;
  x().x = x0_; //from yaml
  x().v.setZero(); //???
  x().ba.setZero(); //???
  x().bg.setZero(); //???
  x().bb = 0.; // barometer pressure bias
  if (ref_lla_set_)
    x().ref = x().ref; //??? why set a variable to itself?
  else //???if ref_lla has not been set, it is not flying or armed.  A gnss message has not yet been received
    x().ref = 0.;
  x().a = -gravity;
  x().w.setZero();
  is_flying_ = false;
  armed_ = false;
  //???if ref_lla is not set, when are x().t, x, v, ba, bg, bb ever set?
}

/////
//called by EKF::imuCallback, EKF::update
//calls EKF::initialize, EKF::dynamics (in dynmaics.cpp)
void EKF::propagate(const double &t, const Vector6d &imu, const Matrix6d &R)
{
  //if time t is a nan initialize
  if (std::isnan(x().t))
  {
    initialize(t);
    return;
  }

  double dt = t - x().t;
  assert(dt >= 0);
  if (dt < 1e-6)
    return;
  //this function is in dynamics.cpp but it is scopted in the EKF class
  dynamics(x(), imu, dx_, true);


  ///////////PART OF ALGORITHM /////////////
  // do the state propagation
  xbuf_.next().x = x() + dx_ * dt;
  xbuf_.next().x.t = t;
  xbuf_.next().x.imu = imu;

  // discretize jacobians (first order)
  A_ = I_BIG + A_*dt;
  B_ = B_*dt;
  CHECK_NAN(P());
  CHECK_NAN(A_);
  CHECK_NAN(B_);
  CHECK_NAN(Qx_);
  xbuf_.next().P = A_*P()*A_.T + B_*R*B_.T + Qx_*dt*dt; // covariance propagation
  CHECK_NAN(xbuf_.next().P);
  xbuf_.advance();
  Qu_ = R; // copy because we might need it later.

  ///////////END OF ALGORITHM SECTION/////////////

  if (enable_log_)
  {
    logs_[LOG_STATE]->logVectors(x().arr, x().q.euler());
    logs_[LOG_COV]->log(x().t);
    logs_[LOG_COV]->logVectors(P());
    logs_[LOG_IMU]->log(t);
    logs_[LOG_IMU]->logVectors(imu);
  }
}

//dont wory about this for now.  It is not being used in current configuration.  It is only used if multithreaded is enabled
/////It looks like this can be used to run any update function through EKF::update
//called by EKF::imuCallback(can be if configured enable_out_of_order.  This is used to allow multithreaded)
//calls EKF::update, EKF::logState (looks like the function has not been created yet), EKF::getOldestNewMeas, 
void EKF::run()
{
  meas::MeasSet::iterator nmit = getOldestNewMeas();
  if (nmit == meas_.end())
    return;

  // rewind state history
  if (!xbuf_.rewind((*nmit)->t))
    throw std::runtime_error("unable to rewind enough, expand STATE_BUF");

  // re-propagate forward, integrating measurements on the way
  while (nmit != meas_.end())
  {
    update(*nmit);
    nmit++;
  }

  // clear off measurements older than the state history
  while (xbuf_.begin().x.t > (*meas_.begin())->t)
    meas_.erase(meas_.begin());

  logState();
}

/////This is only used if multithreading is turned on.  It decides which kind of update it is and calls that function
//called by EKF::run
//calls EKF::propagate, EKF:: zeroVelUpdate, EKF::gnssUpdate, EKF::mocapUpdate, EKF::cleanUpMeasurmenetsBuffers 
void EKF::update(const meas::Base* m)
{
  if (m->type == meas::Base::IMU)
  {
    const meas::Imu* z = dynamic_cast<const meas::Imu*>(m);
    propagate(z->t, z->z, z->R);

    if (!is_flying_)
      zeroVelUpdate(z->t);
  }
  else if (!std::isnan(x().t))
  {
    propagate(m->t, x().imu, Qu_);
    switch(m->type)
    {
    case meas::Base::GNSS:
      {
        const meas::Gnss* z = dynamic_cast<const meas::Gnss*>(m);
        gnssUpdate(*z);
        break;
      }
    case meas::Base::MOCAP:
      {
        const meas::Mocap* z = dynamic_cast<const meas::Mocap*>(m);
        mocapUpdate(*z);
        break;
      }
    default:
      break;
    }
  }
  cleanUpMeasurementBuffers();
}

/////only used if multithreading is used.
//called by EKF::run
meas::MeasSet::iterator EKF::getOldestNewMeas()
{
  meas::MeasSet::iterator it = meas_.begin();
  while (it != meas_.end() && (*it)->handled)
  {
    it++;
  }
  return it;
}

/////
//called by EKF::zeroVelUpdate, EKF::baroUpdate, EKF::gnssUpdate
bool EKF::measUpdate(const VectorXd &res, const MatrixXd &R, const MatrixXd &H)
{

  ///////////PART OF ALGORITHM /////////////
  int size = res.rows();
  auto K = K_.leftCols(size);

  ///TODO: perform covariance gating
  MatrixXd innov = (H*P()*H.T + R).inverse();

  CHECK_NAN(H); CHECK_NAN(R); CHECK_NAN(P());
  K = P() * H.T * innov;
  CHECK_NAN(K);

  if (enable_partial_update_)
  {
    // Apply Fixed Gain Partial update per
    // "Partial-Update Schmidt-Kalman Filter" by Brink
    // Modified to operate inline and on the manifold
    x() += lambda_vec_.asDiagonal() * K * res;
    dxMat ImKH = I_BIG - K*H;
    P() += lambda_mat_.cwiseProduct(ImKH*P()*ImKH.T + K*R*K.T - P());
  }
  else
  {
    x() += K * res;
    dxMat ImKH = I_BIG - K*H;
    P() = ImKH*P()*ImKH.T + K*R*K.T;
  }

  CHECK_NAN(P());
  return true;

  ///////////END OF ALGORITHM SECTION/////////////

}

///// Sets is flying if flying, calls propagation function, and creates logs
//called from EKF_ROS::imuCallback
//calls EKF::checkisFlying, can call EKF::run, EKF::propagate, EKF::zeroVelUpdate
void EKF::imuCallback(const double &t, const Vector6d &z, const Matrix6d &R)
{
  //On first update (which is triggered by the first imu message) check if it is flying
  if (!is_flying_)
    checkIsFlying();

  ///TODO: make thread-safe (wrap in mutex)
  if (enable_out_of_order_)
  {
    imu_meas_buf_.push_back(meas::Imu(t, z, R));
    meas_.insert(meas_.end(), &imu_meas_buf_.back());
    run(); // For now, run on the IMU heartbeat (could be made multi-threaded)
  }
  else
  {
    propagate(t, z, R);
    //if it is not flying do a zero velocity update
    if (!is_flying_)
      zeroVelUpdate(t);
  }

  if (enable_log_)
  {
    logs_[LOG_IMU]->log(t);
    logs_[LOG_IMU]->logVectors(z);
  }

}

/////determines if out of order (multithread) is enabled.  If not it calls the baroUpdate
//called from EKF_ROS::baroCallback
//calls EKF_baroUpdate, meas::Baro (this is a struct),
void EKF::baroCallback(const double &t, const double &z, const double &R,
                       const double &temp)
{
  //???why does this not get implimented if multithreaded?
  if (enable_out_of_order_)
  {
    std::cout << "ERROR OUT OF ORDER BARO NOT IMPLEMENTED" << std::endl;
  }
  else
    baroUpdate(meas::Baro(t, z, R, temp)); //meas::Baro is a struct
}

//no subscription, but would be called by EKF_ROS::poseCallback
void EKF::rangeCallback(const double& t, const double& z, const double& R)
{
  if (enable_out_of_order_)
  {
    std::cout << "ERROR OUT OF ORDER RANGE NOT IMPLEMENTED" << std::endl;
  }
  else
    rangeUpdate(meas::Range(t, z, R));
}

/////determines if out of order (multithread) is enabled.  If not it calls the gnss_Update
//Called by EKF_ROS::gnssCallback
//calls gnssUpdate, meas::Gnss(struct)
void EKF::gnssCallback(const double &t, const Vector6d &z, const Matrix6d &R)
{
  if (!ref_lla_set_)
    return;

  if (enable_out_of_order_)
  {
    gnss_meas_buf_.push_back(meas::Gnss(t, z, R)); //adds the new measurment?
    meas_.insert(meas_.end(), &gnss_meas_buf_.back()); //object created in ekf.h from meas::MeasSet, see meas.h
  }
  else
    gnssUpdate(meas::Gnss(t, z, R));

  if (enable_log_)
  {
    logs_[LOG_LLA]->log(t);
    logs_[LOG_LLA]->logVectors(ecef2lla((x_e2I_ * x().x).t()));
    logs_[LOG_LLA]->logVectors(ecef2lla(z.head<3>()));
  }
}

/////similar to EKF::gnssCallback
//called from EKF_ROS::mocapCallback (which comes from odom or pose)
//calls mocapUpdate
void EKF::mocapCallback(const double& t, const xform::Xformd& z, const Matrix6d& R)
{
  if (enable_out_of_order_)
  {
    mocap_meas_buf_.push_back(meas::Mocap(t, z, R));
    meas_.insert(meas_.end(), &mocap_meas_buf_.back());
  }
  else
    mocapUpdate(meas::Mocap(t, z, R));


  if (enable_log_)
  {
    logs_[LOG_REF]->log(t);
    logs_[LOG_REF]->logVectors(z.arr(), z.q().euler());
  }
}

///// determines if baro needs to be updated.  Converts to appropriate parameters.  ???does this contain some of the algorithm? calculates measurement jacobian
//called from baroCallback
//calls this->groundTempPressSet, measUpdate
void EKF::baroUpdate(const meas::Baro &z)
{
  //groundTempPressSet returns true if ground temp and pressure are not both 0
  //return if gound temp and pressure are not yet set.
  if (!this->groundTempPressSet())
  {
    return;
  }
  else if (!update_baro_ || !is_flying_) //update_baro is set to false in load function.  It is set true later in this function
  {
    // Take the lowest pressure while I'm not flying as ground pressure
    // This has the effect of hopefully underestimating my altitude instead of
    // over estimating.
    if (z.z(0) < ground_pressure_)
    {
      ground_pressure_ = z.z(0);
      std::cout << "New ground pressure: " << ground_pressure_ << std::endl;
    }

    // check if we should start updating with the baro yet based on
    // velocity estimate
    if (x().v.norm() > update_baro_vel_thresh_)
      update_baro_ = true;

    return;
  }

  using Vector1d = Eigen::Matrix<double, 1, 1>;

  // // From "Small Unmanned Aircraft: Theory and Practice" eq 7.8
  const double g = 9.80665; // m/(s^2) gravity 
  const double R = 8.31432; // universal gas constant
  const double M = 0.0289644; // kg / mol. molar mass of Earth's air

  const double altitude = -x().p(2);
  const double baro_bias = x().bb;

  // From "Small Unmanned Aircraft: Theory and Practice" eq 7.9
  // const double rho = M * ground_pressure_ / R / ground_temperature_;
  const double rho = M * ground_pressure_ / R / z.temp;

  const double press_hat = ground_pressure_ - rho * g * altitude + baro_bias;

  const Vector1d zhat(press_hat); //???not sure what this function does.  zhat is not defined anywhere.  Maybe this is a way to define it?
  Vector1d r = z.z - zhat;

  ///////////PART OF ALGORITHM /////////////
  typedef ErrorState E;

  Matrix<double, 1, E::NDX> H;
  H.setZero();
  H(0, E::DP + 2) = rho * g;
  H(0, E::DBB) = 1.;
  ///////////END OF ALGORITHM SECTION/////////////

  /// TODO: Saturate r
  if (use_baro_)
    measUpdate(r, z.R, H);

  if (enable_log_)
  {
    logs_[LOG_BARO_RES]->log(z.t);
    logs_[LOG_BARO_RES]->logVectors(r, z.z, zhat);
    logs_[LOG_BARO_RES]->log(z.temp);
  }

}

//not called becaues there is no subscription for this.  But all of the functions are set up if subscribed to.
void EKF::rangeUpdate(const meas::Range &z)
{
  // Assume that the earth is flat and that the range sensor is rigidly attached
  // to the UAV, so distance is dependent on the attitude of the UAV.
  // TODO this assumes that the laser is positioned at 0,0,0 in the body frame
  // of the UAV
  // TODO this also only updates if the UAV is pretty close to level and the
  // measurement model jacobian assumes that the measurement is not dependent
  // on roll or pitch at all
  using Vector1d = Eigen::Matrix<double, 1, 1>;

  const double altitude = -x().p(2);
  const double roll = x().q.roll();
  const double pitch = x().q.pitch();

  // const double level_threshold = 2. * M_PI / 180.; // 1 degree

  // Only update if UAV is close to level
  // if ((abs(roll) > level_threshold) || (abs(pitch) > level_threshold))
    // return;

  const Vector1d zhat(altitude / cos(roll) / cos(pitch)); // TODO roll/ pitch of drone
  Vector1d r = z.z - zhat; // residual

  // std::cout << "Laser Update: " << std::endl;
  // std::cout << "Altitude meas: " << z.z(0) << std::endl;
  // std::cout << "Altitude est: " << altitude << std::endl;

  typedef ErrorState E;

  Matrix<double, 1, E::NDX> H;
  H.setZero();
  H(0, E::DP + 2) = -1.;

  // Vector1d r_saturated
  double r_sat = 0.1;
  if (abs(r(0)) > r_sat)
  {
    double r_sign = (r(0) > 0) - (r(0) < 0);
    r(0) = r_sat * r_sign;
  }

  // TODO: Saturate r
  if (use_range_)
    measUpdate(r, z.R, H);

  if (enable_log_)
  {
    logs_[LOG_RANGE_RES]->log(z.t);
    logs_[LOG_RANGE_RES]->logVectors(r, z.z, zhat);
  }

}

///// like baroCallback, but there are a lot of conversions from different frames as well.
//Called by EKF::gnssCallback, EKF::update
//calls measUpdate
void EKF::gnssUpdate(const meas::Gnss &z)
{
  //calcuate gps (inertial frame) velocities and positions
  const Vector3d w = x().w - x().bg; //angluar velocity minus gyro bias
  const Vector3d gps_pos_I = x().p + x().q.rota(p_b2g_); //I is inertial frame, position plus the orientation rotated into the gps frame
  const Vector3d gps_vel_b = x().v + w.cross(p_b2g_); //b is body fixed frame, velocity plus the angular velocity crossed with a conversion from 
  const Vector3d gps_vel_I = x().q.rota(gps_vel_b);

  // Update ref_lla based on current estimate
  // ???not sure why we would update ref_lla and not sure how this is doing that.
  Vector3d ref_lla(ref_lat_radians_, ref_lon_radians_, x().ref);
  xform::Xformd x_e2n = x_ecef2ned(lla2ecef(ref_lla));
  x_e2I_.t() = x_e2n.t();
  x_e2I_.q() = x_e2n.q() * q_n2I_;

  Vector6d zhat;
  zhat << x_e2I_.transforma(gps_pos_I),
          x_e2I_.rota(gps_vel_I);
  const Vector6d r = z.z - zhat; // residual

  //creating rotation matrices from inertial, body fixed, and ecef
  const Matrix3d R_I2e = x_e2I_.q().R().T;
  const Matrix3d R_b2I = x().q.R().T;
  const Matrix3d R_e2b = R_I2e * R_b2I;

  const double sin_lat = sin(ref_lat_radians_);
  const double cos_lat = cos(ref_lat_radians_);
  const double sin_lon = sin(ref_lon_radians_);
  const double cos_lon = cos(ref_lon_radians_);
  const Vector3d dpEdRefAlt(cos_lat * cos_lon, cos_lat * sin_lon, sin_lat); // used in jacobian calculation

  ///////////PART OF ALGORITHM /////////////
  typedef ErrorState E;

  Matrix<double, 6, E::NDX> H;
  H.setZero();
  H.block<3,3>(0, E::DP) = R_I2e; // dpE/dpI
  H.block<3,3>(0, E::DQ) = -R_e2b * skew(p_b2g_);
  H.block<3, 1>(0, E::DREF) = dpEdRefAlt;
  H.block<3,3>(3, E::DQ) = -R_e2b * skew(gps_vel_b); // dvE/dQI
  H.block<3,3>(3, E::DV) = R_e2b;
  H.block<3,3>(3, E::DBG) = R_e2b * skew(p_b2g_);

  ///////////END OF ALGORITHM SECTION/////////////
  /// TODO: Saturate r
  if (use_gnss_)
    measUpdate(r, z.R, H);

  if (enable_log_)
  {
    logs_[LOG_GNSS_RES]->log(z.t);
    logs_[LOG_GNSS_RES]->logVectors(r, z.z, zhat);
  }
}

/////similar to baroUpdate
//called by mocapCallback, EKF::update
//calls EKF::measUpdate
void EKF::mocapUpdate(const meas::Mocap &z)
{
  xform::Xformd zhat = x().x;

  // TODO Do we need to fix "-" operator for Xformd?
  // Right now using piecewise subtraction
  // on position and attitude separately. This may be correct though because
  // our state is represented as R^3 x S^3 (position, quaterion) not SE3.
  // calculate residual
  Vector6d r;
  r.segment<3>(0) = z.z.t_ - zhat.t_;
  r.segment<3>(3) = z.z.q_ - zhat.q_;

  typedef ErrorState E;
  Matrix<double, 6, E::NDX> H;
  H.setZero();
  H.block<3,3>(0, E::DP) = I_3x3;
  H.block<3,3>(3, E::DQ) = I_3x3;

  /// TODO: Saturate r
  if (use_mocap_)
  {
    measUpdate(r, z.R, H);
  }

  if (enable_log_)
  {
    logs_[LOG_MOCAP_RES]->log(z.t);
    logs_[LOG_MOCAP_RES]->logVectors(r, z.z.arr(), zhat.arr());
  }
}

///// calculates jacovians, calls measUpdate, and sets log if enabled.
//called by EFK::imuCallback, EKF::update
//calls EKF::measUpdate
void EKF::zeroVelUpdate(double t)
{
  ///////////PART OF ALGORITHM /////////////
  // Update Zero velocity and zero altitude
  typedef ErrorState E;
  Matrix<double, 4, E::NDX> H;
  H.setZero();
  H.block<3,3>(0, E::DV) = I_3x3;
  H(3, E::DP + 2) = 1.;

  Vector4d r;
  r.head<3>() = -x().v;
  r(3) = -x().p(2);

  if (use_zero_vel_)
    measUpdate(r, R_zero_vel_, H);

  // Reset the uncertainty in yaw
  P().block<ErrorState::SIZE, 1>(0, ErrorState::DQ + 2).setZero();
  P().block<1, ErrorState::SIZE>(ErrorState::DQ + 2, 0).setZero();
  P()(ErrorState::DQ + 2, ErrorState::DQ + 2) = P0_yaw_;
  ///////////END OF ALGORITHM SECTION/////////////

  if (enable_log_)
  {
    logs_[LOG_ZERO_VEL_RES]->log(t);
    logs_[LOG_ZERO_VEL_RES]->logVectors(r);
  }
}

/////sets the refference lla
//called by EKF_ROS::gnssCallback, EKF_ROS::gnssCallbackUblox, EKF_ROS::gnssCallbackInertialSense
void EKF::setRefLla(Vector3d ref_lla)
{
  if (ref_lla_set_)
    return;

  std::cout << "Set ref lla: " << ref_lla.transpose() << std::endl;
  ref_lla.head<2>() *= M_PI/180.0; // convert to rad
  xform::Xformd x_e2n = x_ecef2ned(lla2ecef(ref_lla));
  x_e2I_.t() = x_e2n.t();
  x_e2I_.q() = x_e2n.q() * q_n2I_;

  // initialize the estimated ref altitude state
  x().ref = ref_lla(2);
  ref_lat_radians_ = ref_lla(0);
  ref_lon_radians_ = ref_lla(1);

  ref_lla_set_ = true;

}

/////not used unless multithreading is enabled
//called by EKF::update
void EKF::cleanUpMeasurementBuffers()
{
  // Remove all measurements older than our oldest state in the measurement buffer
  while ((*meas_.begin())->t < xbuf_.begin().x.t)
    meas_.erase(meas_.begin());
  while (imu_meas_buf_.front().t < xbuf_.begin().x.t)
    imu_meas_buf_.pop_front();
  while (mocap_meas_buf_.front().t < xbuf_.begin().x.t)
    mocap_meas_buf_.pop_front();
  while (gnss_meas_buf_.front().t < xbuf_.begin().x.t)
    gnss_meas_buf_.pop_front();
}

////sets ground temperature and ground pressure on first baroCallback
//called by EKF_ROS::baroCallback
void EKF::setGroundTempPressure(const double& temp, const double& press)
{
  ground_temperature_ = temp;
  ground_pressure_ = press;
}

/////Sets is_flying if it is okay to check and accel exceeds threshold
//called by EKF::imuCallback
void EKF::checkIsFlying()
{
  //enable_arm_check_ set in load function and the value is set by the user in ekf.yaml
  // if enable_arm_check is true, then set okay_to_check to armed_.  If it is false set okay_to_check to true
  // set_armed and set_disarmed functions are set up in ekf.h, but would need to be called to set armed_.  By default it is false.
  bool okay_to_check = enable_arm_check_ ? armed_ : true;
  // x().a is -g according to EKF::initialize.  ???is EKF::initialize called?
  // is_flying_threshold is set in ekf.yaml.  If accel measurment excees this magnitude, set is_flying
  if (okay_to_check && x().a.norm() > is_flying_threshold_)
  {
    std::cout << "Now Flying!  Go Go Go!" << std::endl;
    is_flying_ = true;
  }
}


}
}
