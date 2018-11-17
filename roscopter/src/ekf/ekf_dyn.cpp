#include "ekf/ekf.h"

namespace roscopter
{
void EKF::dynamics(const xVector &x, const uVector &u, dxVector &xdot, dxMatrix &dfdx, dxuMatrix &dfdu)
{
  dynamics(x, u);
  xdot = dx_;
  dfdx = A_;
  dfdu = G_;
}


void EKF::dynamics(const xVector& x, const uVector &u, bool state, bool jac)
{
  if (state)
  {
    dx_.setZero();
  }
  
  if (jac)
  {
    A_.setZero();
    G_.setZero();
  }
  
  Vector3d vel = x.block<3, 1>((int)xVEL, 0);
  Quatd q_I_b(x.block<4,1>((int)xATT,0));
  
  Vector3d acc = u.block<3,1>((int)uA, 0) - x.block<3,1>((int)xB_A, 0);
  Vector3d omega = u.block<3,1>((int)uG, 0) - x.block<3,1>((int)xB_G, 0);
  Vector3d acc_z;
  acc_z << 0, 0, acc(2,0);
  double mu = x((int)xMU);
  
  Matrix3d R_I_b = q_I_b.R();
  Vector3d gravity_B = q_I_b.rotp(gravity); // R_I^b * vel
  Vector3d vel_xy;  
  vel_xy << vel(0), vel(1), 0.0;
  
  // Calculate State Dynamics
  if (state)
  {
    dx_.block<3,1>((int)dxPOS,0) = q_I_b.rota(vel); // R_I^b.T * vel
    if (use_drag_term_)
      dx_.block<3,1>((int)dxVEL,0) = acc_z + gravity_B - omega.cross(vel) - mu*vel_xy;
    else
      dx_.block<3,1>((int)dxVEL,0) = acc + gravity_B - omega.cross(vel);
    dx_.block<3,1>((int)dxATT, 0) = omega;
  }
  
  // State Jacobian
  if (jac)
  {
    A_.block<3,3>((int)dxPOS, (int)dxVEL) = R_I_b.transpose();
    A_.block<3,3>((int)dxPOS, (int)dxATT) = -R_I_b.transpose()*skew(vel);
    if (use_drag_term_)
    {
      A_.block<3,3>((int)dxVEL, (int)dxVEL) = -mu * I_2x3.transpose() * I_2x3 - skew(omega);
      A_.block<3,3>((int)dxVEL, (int)dxB_A) << 0, 0, 0, 0, 0, 0, 0, 0, -1;
      A_.block<3,1>((int)dxVEL, (int)dxMU) = -vel_xy;
    }
    else
    {
      A_.block<3,3>((int)dxVEL, (int)dxB_A) = -I_3x3;
      A_.block<3,3>((int)dxVEL, (int)dxVEL) = -skew(omega);
    }
    A_.block<3,3>((int)dxVEL, (int)dxATT) = skew(gravity_B);
    A_.block<3,3>((int)dxVEL, (int)dxB_G) = -skew(vel);
    A_.block<3,3>((int)dxATT, (int)dxB_G) = -I_3x3;
    
    // Input Jacobian
    if (use_drag_term_)
      G_.block<3,3>((int)dxVEL, (int)uA) << 0, 0, 0, 0, 0, 0, 0, 0, -1;
    else
      G_.block<3,3>((int)dxVEL, (int)uA) = -I_3x3;
    G_.block<3,3>((int)dxVEL, (int)uG) = -skew(vel);
    G_.block<3,3>((int)dxATT, (int)uG) = -I_3x3;
  }  
}

}
