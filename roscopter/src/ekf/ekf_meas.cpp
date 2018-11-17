#include "ekf/ekf.h"

namespace roscopter
{

void EKF::handle_measurements(std::vector<int>* gated_feature_ids)
{
  if (gated_feature_ids)
    gated_feature_ids->clear();

  if (zbuf_.size() == 0)
    return;

  // Find the oldest measurement that hasn't been handled yet
  zBuf::iterator z_it = zbuf_.end() -1;
  while (z_it->handled == true && z_it != zbuf_.begin())
    z_it--;

  // There were no messages to be handled - exit
  if (z_it == zbuf_.begin() && z_it->handled)
    return;


  // Select the input which just before the measurement (so the measurement happened in the next interval))
  uBuf::iterator u_it = u_.begin();
  while (u_it != u_.end())
  {
    if (z_it->t > u_it->first)
      break;
    u_it++;
  }
  if (z_it->t <= u_it->first)
  {
    cerr << "not enough history in input buffer to handle measurement: needed "
         << z_it->t - t_[i_] << "s of buffer, you have " << u_it->first - u_.begin()->first << "s" << endl;
    return;
  }

  // Rewind the state to just before the time indicated by the measurement
  int i = LEN_STATE_HIST;
  double t_now = t_[i_];
  while (i > 0)
  {
    if (t_[(i_ + i) % LEN_STATE_HIST] <= z_it->t)
    {
      // rewind state to here (by just setting the position in the circular buffer)
      i_ = (i_ + i) % LEN_STATE_HIST;
      break;
    }
    i--;
  }
  if (i == 0)
  {
    cerr << "not enough history in state buffer to handle measurement: needed " << t_now - z_it->t
         << "s of buffer, you have " << t_now - t_[(i_ + 1) % LEN_STATE_HIST] << "s" << endl;
    zbuf_.erase(z_it);
    return;
  }

  // Make sure everything is lined up
  if (t_[i_] > z_it->t || t_[i_] < u_it->first)
  {
    cerr << "Time history misaligned\n";
  }


  // Process all inputs and measurements to catch back up
  while (u_it != u_.begin())
  {
    u_it--;

    // While the current measurment occurred between the current time step and the next
    while (z_it->t <= u_it->first)
    {
      // Propagate to the point of the measurement
      if (t_[i_] < z_it->t)
        propagate_state(u_it->second, z_it->t, false);
      else if (t_[i_] > z_it->t)
      {
//        cerr << "can't propagate backwards\n";
      }
      else
      {
        // Perform the measurement
        meas_result_t result = update(*z_it);
        if (result == MEAS_GATED)
        {
          cerr << "gating " << measurement_names[z_it->type] << " measurement\n";
        }
      }

      if (z_it != zbuf_.begin())
        z_it--; // Perform the next measurement in this interval
      else
        break;

    }
    // Propagate to the time of the next input
    propagate_state(u_it->second, u_it->first, false);
  }

  // Clear any old measurements in the queue
  while (zbuf_.size() > LEN_MEAS_HIST)
    zbuf_.pop_back();

  // Clear old propagation memory
  while (u_.size() > LEN_STATE_HIST)
    u_.pop_back();

}


EKF::meas_result_t EKF::add_measurement(const double t, const VectorXd& z, const measurement_type_t& meas_type,
                                            const MatrixXd& R, bool active)
{
  if (t < start_t_)
    return MEAS_INVALID;

  if ((z.array() != z.array()).any())
    return MEAS_NAN;

  // Figure out the measurement that goes just before this one
  auto z_it = zbuf_.begin();
  while (z_it != zbuf_.end())
  {
    if (z_it->t < t)
      break;
    z_it ++;
  }

  // add the measurement to the measurement queue just after the one we just found
  measurement_t meas;
  meas.t = t;
  meas.type = meas_type;
  meas.zdim = z.rows();
  meas.rdim = R.cols();
  meas.R.block(0, 0, meas.rdim, meas.rdim) = R;
  meas.z.segment(0, meas.zdim) = z;
  meas.active = active;
  meas.handled = false;
  if (z_it == zbuf_.begin())
  {
    zbuf_.push_front(meas);
    z_it = zbuf_.begin();
  }
  else
    zbuf_.insert(z_it, meas);

    // Ensure that all the measurements are in order
    double t_prev = zbuf_.end()->t;
    if (zbuf_.size() > 1)
    {
      for (auto it = zbuf_.end()-1; it != zbuf_.begin(); it--)
      {
        if (it->t < t_prev)
        {
          cerr << "measurements out of order" << endl;
        }
        t_prev = it->t;
      }
    }

  return MEAS_SUCCESS;

}

EKF::meas_result_t EKF::update(measurement_t& meas)
{
  meas.handled = true;
  NAN_CHECK;
  
  zhat_.setZero();
  H_.setZero();
  K_.setZero();
  
  (this->*(measurement_functions[meas.type]))(x_[i_], zhat_, H_);
  
  NAN_CHECK;
  
  zVector residual;
  if (meas.type == ATT)
  {
    residual.topRows(3) = Quatd(meas.z) - Quatd(zhat_);
  }
  else
  {
    residual.topRows(meas.zdim) = meas.z.topRows(meas.zdim) - zhat_.topRows(meas.zdim);
  }

  auto K = K_.leftCols(meas.rdim);
  auto H = H_.topRows(meas.rdim);
  auto res = residual.topRows(meas.rdim);
  auto R = meas.R.block(0, 0, meas.rdim, meas.rdim);


  //  Perform Covariance Gating Check on Residual
  if (meas.active)
  {
    auto innov =  (H * P_[i_] * H.transpose() + R).inverse();

    double mahal = res.transpose() * innov * res;
    if (mahal > gating_threshold_)
    {
      //      std::cout << "gating " << measurement_names[meas_type] << " measurement: " << mahal << std::endl;
      return MEAS_GATED;
    }

    K = P_[i_] * H.transpose() * innov;
    NAN_CHECK;
    
    //    CHECK_MAT_FOR_NANS(H_);
    //    CHECK_MAT_FOR_NANS(K_);
    
    if (NO_NANS(K_) && NO_NANS(H_))
    {
      if (partial_update_)
      {
        // Apply Fixed Gain Partial update per
        // "Partial-Update Schmidt-Kalman Filter" by Brink
        // Modified to operate inline and on the manifold
        boxplus(x_[i_], lambda_.asDiagonal() * K * residual.topRows(meas.rdim), xp_);
        x_[i_] = xp_;
        A_ = (I_big_ - K * H);
        P_[i_] += (Lambda_).cwiseProduct(A_*P_[i_]*A_.transpose() + K * R * K.transpose() - P_[i_]);

      }
      else
      {
        boxplus(x_[i_], K * residual.topRows(meas.rdim), xp_);
        x_[i_] = xp_;
        A_ = (I_big_ - K * H);
        P_[i_] = A_*P_[i_]*A_.transpose() + K * R * K.transpose();
      }
    }
  }
  
  NAN_CHECK;
  
  log_measurement(meas.type, t_[i_], meas.zdim, meas.z, zhat_, meas.active);
  return MEAS_SUCCESS;
}


void EKF::h_acc(const xVector& x, zVector& h, hMatrix& H) const
{
  H.setZero();
  
  Vector3d b_a = x.block<3,1>((int)xB_A,0);
  
  if (use_drag_term_)
  {
    Vector3d vel = x.block<3,1>((int)xVEL,0);
    double mu = x(xMU,0);
    
    h.topRows(2) = I_2x3 * (-mu * vel + b_a);
    
    H.block<2, 3>(0, (int)dxVEL) = -mu * I_2x3;
    H.block<2, 3>(0, (int)dxB_A) = I_2x3;
    H.block<2, 1>(0, (int)dxMU) = -I_2x3*vel;
  }
  else
  {
    Vector3d gravity_B = Quatd(x.block<4,1>((int)xATT, 0)).rotp(gravity); // R_I^b * vel
    h.topRows(3) = b_a - gravity_B;
    H.block<3,3>(0, (int)dxATT) = -skew(gravity_B);
    H.block<3,3>(0, (int)dxB_A) = I_3x3;
  }
}

void EKF::h_alt(const xVector& x, zVector& h, hMatrix& H) const
{
  h.row(0) = -x.block<1,1>(xPOS+2, 0);
  
  H.setZero();
  H(0, dxPOS+2) = -1.0;
}

void EKF::h_att(const xVector& x, zVector& h, hMatrix& H) const
{
  h = x.block<4,1>((int)xATT, 0);
  
  H.setZero();
  H.block<3,3>(0, dxATT) = I_3x3;
}

void EKF::h_pos(const xVector& x, zVector& h, hMatrix& H) const
{
  h.topRows(3) = x.block<3,1>((int)xPOS,0);
  
  H.setZero();
  H.block<3,3>(0, (int)xPOS) = I_3x3;
}

void EKF::h_vel(const xVector& x, zVector& h, hMatrix& H) const
{
  h.topRows(3) = x.block<3,1>((int)xVEL, 0);
  
  H.setZero();
  H.block<3,3>(0, (int)dxVEL) = I_3x3;
}

}
