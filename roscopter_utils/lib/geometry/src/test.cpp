#include <iostream>

#include "gtest/gtest.h"
#include "geometry/xform.h"
#include "geometry/cam.h"

#include <eigen3/Eigen/Geometry>
#include <eigen3/unsupported/Eigen/MatrixFunctions>

#include <opencv2/opencv.hpp>

using namespace quat;
using namespace xform;
using namespace Eigen;
using namespace std;

#ifdef NDEBUG
#define NUM_ITERS 1000
#else
#define NUM_ITERS 10
#endif

#define MATRIX_CLOSE(m1, m2, tol) {\
    for (int row = 0; row < (m1).rows(); row++ ) \
{ \
    for (int col = 0; col < (m1).cols(); col++) \
{ \
    EXPECT_NEAR((m1)(row, col), (m2)(row, col), tol); \
    } \
    } \
    }

#define QUATERNION_EQUALS(q1, q2) \
    MATRIX_CLOSE((q1).arr_,  (sign((q2).w()) * sign((q1).w())) * (q2).arr_, 1e-8)

#define MATRIX_EQUALS(v1, v2) \
    MATRIX_CLOSE(v1, v2, 1e-8)

#define TRANSFORM_EQUALS(t1, t2) \
    MATRIX_EQUALS((t1).t(), (t2).t()); \
    QUATERNION_EQUALS((t1).q(), (t2).q())

#define TRANSFORM_CLOSE(t1, t2, tol) \
    MATRIX_CLOSE(t1.q_.arr_, (sign(t2.q_.w()) * sign(t1.q_.w())) * t2.q_.arr_, tol); \
    MATRIX_CLOSE(t1.t_, t1.t_, tol)

void known_transform()
{
    Xformd T_known((Vector3d() << 1, 1, 0).finished(),
                   Quatd::from_axis_angle((Vector3d() << 0, 0, 1).finished(), M_PI/4.0));
    Xformd T_known_inv((Vector3d() << -sqrt(2), 0, 0).finished(),
                       Quatd::from_axis_angle((Vector3d() << 0, 0, 1).finished(), -M_PI/4.0));
    TRANSFORM_EQUALS(T_known.inverse(), T_known_inv);
}
TEST(xform, known_transform){known_transform();}

void known_vector_passive_transform()
{
    Vector3d p1; p1 << 1, 0, 0;
    Xformd T_known((Vector3d() << 1, 1, 0).finished(),
                   Quatd::from_axis_angle((Vector3d() << 0, 0, 1).finished(), M_PI/4.0));
    Vector3d p2; p2 << -sqrt(0.5), -sqrt(0.5), 0;
    MATRIX_EQUALS(p2, T_known.transformp(p1));
}
TEST(xform, known_vector_passive_transform){known_vector_passive_transform();}

void known_vector_active_transform()
{
    Vector3d p1; p1 << 1, 0, 0;
    Xformd T_known((Vector3d() << 1, 1, 0).finished(),
                   Quatd::from_axis_angle((Vector3d() << 0, 0, 1).finished(), M_PI/4.0));
    Vector3d p2; p2 << 1+sqrt(0.5), 1+sqrt(0.5), 0;
    MATRIX_EQUALS(p2, T_known.transforma(p1));
}
TEST(xform, known_vector_active_transform){known_vector_active_transform();}

void inverse()
{
    for (int i = 0; i < NUM_ITERS; i++)
    {
        Xformd T1 = Xformd::Random();
        Xformd T2 = T1.inverse();
        Xformd T3 = T1 * T2;
        QUATERNION_EQUALS(T3.q(), Quatd::Identity());
        EXPECT_NEAR(T3.t().norm(), 0, 1e-8);
    }
}
TEST(xform, inverse){inverse();}

void transform_vector()
{
    for (int i = 0; i < NUM_ITERS; i++)
    {
        Xformd T1 = Xformd::Random();
        Vector3d p;
        p.setRandom();
        MATRIX_EQUALS(T1.transformp(T1.inverse().transformp(p)), p);
        MATRIX_EQUALS(T1.inverse().transformp(T1.transformp(p)), p);
        MATRIX_EQUALS(T1.transforma(T1.inverse().transforma(p)), p);
        MATRIX_EQUALS(T1.inverse().transforma(T1.transforma(p)), p);
    }
}
TEST(xform, transform_vector){transform_vector();}

void exp()
{
    Vector3d v;
    Vector3d omega;
    for (int i = 0; i < NUM_ITERS; i++)
    {
        v.setRandom();
        omega.setRandom();

        Xformd x = Xformd::Identity();
        double dt = 0.0001;
        for (double t = 0; t < 1.0; t += dt)
        {
            x.t_ += x.q_.rotp(v * dt);
            x.q_ += omega*dt;
        }

        Vector6d delta; delta << v, omega;
        Xformd xexp = Xformd::exp(delta);
        TRANSFORM_CLOSE(x, xexp, 1e-4);
    }
}
TEST(xform, exp){exp();}

void log_exp()
{
    for (int i = 0; i < NUM_ITERS; i++)
    {
        Vector6d xi;
        xi.setRandom();
        MATRIX_EQUALS(Xformd::log(Xformd::exp(xi)), xi);
    }
}
TEST(xform, log_exp){log_exp();}

void boxplus_boxminus()
{
    for (int i = 0; i < NUM_ITERS; i++)
    {
        Xformd T = Xformd::Random();
        Xformd T2 = Xformd::Random();
        Vector6d zeros, dT;
        zeros.setZero();
        dT.setRandom();
        TRANSFORM_EQUALS(T + zeros, T);
        TRANSFORM_EQUALS(T + (T2 - T), T2);
        MATRIX_EQUALS((T + dT) - T, dT);
    }
}


void quat_rotation_direction()
{
    // Compare against a known active and passive rotation
    Vector3d v, beta, v_active_rotated, v_passive_rotated;
    v << 0, 0, 1;
    v_active_rotated << 0, -1.0*std::pow(0.5,0.5), std::pow(0.5,0.5);
    beta << 1, 0, 0;
    Quatd q_x_45 = Quatd::from_axis_angle(beta, 45.0*M_PI/180.0);
    Eigen::Vector3d v_x_45 = q_x_45.rota(v);

    Matrix3d R_true;
    R_true << 1.0000000,  0.0000000,  0.0000000,
            0.0000000,  0.70710678118654757,  0.70710678118654757,
            0.0000000,  -0.70710678118654757, 0.70710678118654757;
    Matrix3d qR = q_x_45.R();
    MATRIX_EQUALS(qR, R_true);
    MATRIX_EQUALS(qR.transpose() * v, v_active_rotated);
    MATRIX_EQUALS(R_true.transpose() * v, v_active_rotated);

    MATRIX_EQUALS(v_x_45, v_active_rotated);

    v_passive_rotated << 0, std::pow(0.5, 0.5), std::pow(0.5, 0.5);
    Vector3d v_x_45_T = q_x_45.rotp(v);
    MATRIX_EQUALS(v_x_45_T, v_passive_rotated);
    MATRIX_EQUALS(qR * v, v_passive_rotated);
    MATRIX_EQUALS(R_true * v, v_passive_rotated);
}
TEST(Quat, quat_rotation_direction) { quat_rotation_direction(); }

void quat_rot_invrot_R()
{
    Vector3d v;
    Quatd q1 = Quatd::Random();
    for (int i = 0; i < NUM_ITERS; i++)
    {
        v.setRandom();
        q1 = Quatd::Random();

        // Check that rotations are inverses of each other
        MATRIX_EQUALS(q1.rota(v), q1.R().transpose() * v);
        MATRIX_EQUALS(q1.rotp(v), q1.R() * v);
    }
}
TEST(Quat, quat_rot_invrot_R){quat_rot_invrot_R();}

void quat_from_two_unit_vectors()
{
    Vector3d v1, v2;
    for (int i = 0; i < NUM_ITERS; i++)
    {
        v1.setRandom();
        v2.setRandom();
        v1 /= v1.norm();
        v2 /= v2.norm();

        MATRIX_EQUALS(Quatd::from_two_unit_vectors(v1, v2).rota(v1), v2);
        MATRIX_EQUALS(Quatd::from_two_unit_vectors(v2, v1).rotp(v1), v2);
    }
}
TEST(Quat, quat_from_two_unit_vectors){quat_from_two_unit_vectors();}

void quat_from_R()
{
    Vector3d v;
    for (int i = 0; i < NUM_ITERS; i++)
    {
        Quatd q1 = Quatd::Random();
        Matrix3d R = q1.R();
        Quatd qR = Quatd::from_R(R);
        v.setRandom();
        MATRIX_EQUALS(qR.rota(v), R.transpose() * v);
        MATRIX_EQUALS(q1.rota(v), R.transpose() * v);
        MATRIX_EQUALS(qR.rotp(v), R * v);
        MATRIX_EQUALS(q1.rotp(v), R * v);
        MATRIX_EQUALS(R, qR.R());
        QUATERNION_EQUALS(qR, q1);
    }
}
TEST(Quat, from_R){quat_from_R();}

void quat_otimes()
{
    Quatd q1 = Quatd::Random();
    Quatd qI = Quatd::Identity();
    QUATERNION_EQUALS(q1 * q1.inverse(), qI);
}
TEST(Quat, otimes){quat_otimes();}

TEST(Quat, exp_log_axis_angle)
{
    // Check that qexp is right by comparing with matrix exp and axis-angle
    for (int i = 0; i < NUM_ITERS; i++)
    {
        Vector3d omega;
        omega.setRandom();
        Matrix3d R_omega_exp_T = Quatd::skew(omega).exp();  // Why is this needed?
        Quatd q_R_omega_exp = Quatd::from_R(R_omega_exp_T.transpose());
        Quatd q_omega = Quatd::from_axis_angle(omega/omega.norm(), omega.norm());
        Quatd q_omega_exp = Quatd::exp(omega);
        QUATERNION_EQUALS(q_R_omega_exp, q_omega);
        QUATERNION_EQUALS(q_omega_exp, q_omega);

        // Check that exp and log are inverses of each otherprint_error
        MATRIX_EQUALS(Quatd::log(Quatd::exp(omega)), omega);
        QUATERNION_EQUALS(Quatd::exp(Quatd::log(q_omega)), q_omega);
    }
}


TEST(Quat, boxplus_and_boxminus)
{
    Vector3d delta1, delta2, zeros;
    zeros.setZero();
    for (int i = 0; i < NUM_ITERS; i++)
    {
        Quatd q = Quatd::Random();
        Quatd q2 = Quatd::Random();
        delta1.setRandom();
        delta2.setRandom();

        QUATERNION_EQUALS(q + zeros, q);
        QUATERNION_EQUALS(q + (q2 - q), q2);
        MATRIX_EQUALS((q + delta1) - q, delta1);
        ASSERT_LE(((q+delta1)-(q+delta2)).norm(), (delta1-delta2).norm());
    }
}

void quat_inplace_add_and_mul()
{
    Vector3d delta1, delta2, zeros;
    zeros.setZero();
    for (int i = 0; i < NUM_ITERS; i++)
    {
        Quatd q = Quatd::Random();
        Quatd q2 = Quatd::Random();
        Quatd q_copy = q.copy();
        QUATERNION_EQUALS(q_copy, q);
        delta1.setRandom();
        delta2.setRandom();

        q_copy += delta1;
        QUATERNION_EQUALS(q_copy, q+delta1);

        q_copy = q.copy();
        q_copy *= q2;
        QUATERNION_EQUALS(q_copy, q*q2);
    }
}
TEST(Quat, inplace_add_and_mul){quat_inplace_add_and_mul();}

void quat_euler()
{
    for (int i =0; i < NUM_ITERS; i++)
    {
        double roll = random(-M_PI, M_PI);
        double pitch = random(-M_PI/2.0, M_PI/2.0);
        double yaw = random(-M_PI, M_PI);
        Quatd q = Quatd::from_euler(roll, pitch, yaw);
        ASSERT_NEAR(roll, q.roll(), 1e-8);
        ASSERT_NEAR(pitch, q.pitch(), 1e-8);
        ASSERT_NEAR(yaw, q.yaw(), 1e-8);
        Quatd q2 = Quatd::from_euler(q.roll(), q.pitch(), q.yaw());
        QUATERNION_EQUALS(q, q2);
    }
}
TEST(Quat, euler){quat_euler();}

void quat_passive_derivative()
{
    Quatd q0 = Quatd::Random();
    Vector3d v;
    v.setRandom();

    Matrix3d a;
    Matrix3d fd;
    Matrix3d I = Matrix3d::Identity();
    double epsilon = 1e-8;

    a = skew(q0.rotp(v));  // [R(q)v]

    for (int i = 0; i < 3; i++)
    {
        Quatd qi = q0 + (epsilon * (I.col(i)));
        fd.col(i) = (qi.rotp(v) - q0.rotp(v))/epsilon;
    }
    if ((fd - a).array().abs().sum() > 1e-6)
    {
        std::cout << "ERROR IN LINE " << __LINE__ << "\nA:\n" << a << "\nD:\nfd" << fd << std::endl;
    }
    ASSERT_LE((fd - a).array().abs().sum(), 1e-6);
}
TEST(Quat, passive_rotation_derivative){quat_passive_derivative();}

void quat_active_derivative()
{
    Quatd q0 = Quatd::Random();
    Vector3d v;
    v.setRandom();

    Matrix3d a;
    Matrix3d fd;
    Matrix3d I = Matrix3d::Identity();
    double epsilon = 1e-8;

    a = -q0.R().transpose() * skew(v);  // -R(q).T * [v]

    for (int i = 0; i < 3; i++)
    {
        Quatd qi = q0 + (epsilon * (I.col(i)));
        fd.col(i) = (qi.rota(v) - q0.rota(v))/epsilon;
    }
    if ((fd - a).array().abs().sum() > 1e-6)
    {
        std::cout << "ERROR IN LINE " << __LINE__ << "\nA:\n" << a << "\nD:\nfd" << fd << std::endl;
    }
    ASSERT_LE((fd - a).array().abs().sum(), 1e-6);
}
TEST(Quat, active_rotation_derivative){quat_active_derivative();}


void exp_approx()
{
    for (int j = 0; j < NUM_ITERS; j++)
    {
        Quatd q = Quatd::Random();
        if (j == 0)
            q = Quatd::Identity();
        Vector3d delta;
        Matrix3d I = Matrix3d::Identity();
        delta.setRandom();
        delta *= 0.1;

        Quatd qp = q + delta;

        Matrix3d actual = qp.R();
        Matrix3d approx = (I - skew(delta))*q.R();
        ASSERT_LE((actual - approx).array().abs().sum(), 0.1);
    }
}
TEST(Quat, exp_approx){exp_approx();}

TEST(Camera, Proj_InvProj)
{
    Vector2d focal_len{250.0, 250.0};
    Vector2d cam_center{320.0, 240.0};
    Vector2d img_size{640, 480};
    double s = 0.0;
    Camera<double> cam(focal_len, cam_center, s, img_size);

    for (int i = 0; i < 640; i+=30)
    {
        for(int j = 0; j < 480; j+=30)
        {
            Vector2d pix{i, j};
            Vector2d pix_out;
            Vector3d pt;
            cam.invProj(pix, 1.0, pt);
            cam.proj(pt, pix_out);

            EXPECT_NEAR(pix.x(), pix_out.x(), 2e-3);
            EXPECT_NEAR(pix.y(), pix_out.y(), 2e-3);
            EXPECT_NEAR(pt.norm(), 1.0, 1e-8);
        }
    }
}

TEST(Camera, Proj_InvProj_Distortion)
{
    Vector2d focal_len{250.0, 250.0};
    Vector2d cam_center{320.0, 240.0};
    Vector2d img_size{640, 480};
    Vector5d distortion = (Vector5d() << -0.28340811, 0.07395907, 0.00019359, 1.76187114e-05, 0.0).finished();
    double s = 0.0;
    UncalibratedCamera<double> cam(focal_len, cam_center, s, img_size, distortion);

    for (int i = 0; i < 640; i+=30)
    {
        for(int j = 0; j < 480; j+=30)
        {
            Vector2d pix{i, j};
            Vector2d pix_out;
            Vector3d pt;
            cam.invProj(pix, 1.0, pt);
            cam.proj(pt, pix_out);

            EXPECT_NEAR(pix.x(), pix_out.x(), 1.5);
            EXPECT_NEAR(pix.y(), pix_out.y(), 1.5);
            EXPECT_NEAR(pt.norm(), 1.0, 1e-8);
        }
    }
}

TEST(Camera, UnDistort_Distort)
{
    Vector2d focal_len{250.0, 250.0};
    Vector2d cam_center{320.0, 240.0};
    Vector2d img_size{640, 480};
    Vector5d distortion = (Vector5d() << -0.28340811, 0.07395907, 0.00019359, 1.76187114e-05, 0.0).finished();
    double s = 0.0;
    UncalibratedCamera<double> cam(focal_len, cam_center, s, img_size, distortion);

    for (int i = 0; i < 640; i+=30)
    {
        for(int j = 0; j < 480; j+=30)
        {
            Vector2d pix_d{i, j};
            Vector2d pix_d2;
            Vector2d pi_u, pi_d, pi_d2;
            cam.pix2intrinsic(pix_d, pi_d);
            cam.unDistort(pi_d, pi_u);
            cam.Distort(pi_u, pi_d2);
            cam.intrinsic2pix(pi_d2, pix_d2);

            EXPECT_NEAR(pix_d.x(), pix_d2.x(), 1.5);
            EXPECT_NEAR(pix_d.y(), pix_d2.y(), 1.5);
        }
    }
}

TEST(Camera, Distort_UnDistort_Calibrated)
{
    Vector2d focal_len{250.0, 250.0};
    Vector2d cam_center{320.0, 240.0};
    Vector2d img_size{640, 480};
    Vector5d distortion = Vector5d::Zero();
    double s = 0.0;
    UncalibratedCamera<double> cam(focal_len, cam_center, s, img_size, distortion);

    Vector2d pix_d, pix_u;
    Vector2d pix_d2, pi_d2;
    Vector3d pt{1.0, 0.5, 2.0};

    cam.proj(pt, pix_d);
    Vector2d pi_d, pi_u;

    cam.pix2intrinsic(pix_d, pi_d);
    cam.Distort(pi_d, pi_u);
    cam.unDistort(pi_u, pi_d2);
    cam.intrinsic2pix(pi_d2, pix_d2);

    EXPECT_NEAR(pix_d2.x(), pix_d.x(), 1e-3);
    EXPECT_NEAR(pix_d2.y(), pix_d.y(), 1e-3);
}


TEST (Camera, DistortJac)
{
    Vector2d focal_len{189.288014941235, 188.89300027962554};
    Vector2d cam_center{189.57352223555952, 114.25998149460739};
    Vector2d img_size{376, 240};
    Vector5d distortion;
    double s = 0;
    distortion << -0.28839265926889596, 0.06052865219979488, 0.003175682270413171,  0.0014326472335837869, 0;
    UncalibratedCamera<double> cam(focal_len, cam_center, s, img_size, distortion);

    Vector2d pix_d = img_size.cwiseProduct(Vector2d::Random());
    Vector2d pi_d;
    cam.pix2intrinsic(pix_d, pi_d);

    Matrix2d J_fd, J_A;
    double eps = 1e-8;
    for (int i = 0; i < 2; i++)
    {
        Vector2d pi_d_plus, pi_d_minus;
        pi_d_plus = pi_d + Vector2d::Unit(i)*eps;
        pi_d_minus = pi_d - Vector2d::Unit(i)*eps;
        Vector2d pi_u_plus, pi_u_minus;
        cam.unDistort(pi_d_plus, pi_u_plus, 1);
        cam.unDistort(pi_d_minus, pi_u_minus, 1);

        J_fd.col(i) = (pi_u_plus - pi_u_minus)/(2.0*eps);
    }

    cam.distortJac(pi_d, J_A);
    //std::cout << "J_FD\n" << J_fd << std::endl;
    //std::cout << "J_A\n" << J_A << std::endl;

    for (int i = 0; i < 2; i++)
        for (int j = 0; j < 2; j++)
            EXPECT_NEAR(J_A(i,j), J_fd(i,j), 2e-2);

}

//TEST(Camera, UnDistort_Distort)
//{
//    Vector2d focal_len{250.0, 250.0};
//    Vector2d cam_center{320.0, 240.0};
//    Vector2d img_size{640, 480};
//    Vector5d distortion = (Vector5d() << -0.28340811, 0.07395907, 0.00019359, 1.76187114e-05, 0.0).finished();
//    double s = 0.0;
//    UncalibratedCamera<double> cam(focal_len, cam_center, s, img_size, distortion);

//    Vector2d pix_d, pix_u;
//    Vector2d pix_u2, pi_u2;
//    Vector3d pt{1.0, 0.5, 2.0};

//    cam.proj(pt, pix_d);
//    Vector2d pi_d, pi_u;

//    cam.pix2intrinsic(pix_u, pi_u);
//    cam.unDistort(pi_u, pi_d);
//    cam.Distort(pi_d, pi_u2);
//    cam.intrinsic2pix(pi_u2, pix_u2);

//    EXPECT_NEAR(pix_u2.x(), pix_u.x(), 1e-3);
//    EXPECT_NEAR(pix_u2.y(), pix_u.y(), 1e-3);
//    EXPECT_NEAR(pi_u2.x(), pi_u.x(), 1e-3);
//    EXPECT_NEAR(pi_u2.y(), pi_u.y(), 1e-3);
//}

TEST(Camera, UnDistort_Distort_Calibrated)
{
    Vector2d focal_len{250.0, 250.0};
    Vector2d cam_center{320.0, 240.0};
    Vector2d img_size{640, 480};
    Vector5d distortion = Vector5d::Zero();
    double s = 0.0;
    UncalibratedCamera<double> cam(focal_len, cam_center, s, img_size, distortion);

    Vector2d pix_d, pix_u;
    Vector2d pix_u2, pi_u2;
    Vector3d pt{1.0, 0.5, 2.0};

    cam.proj(pt, pix_d);
    Vector2d pi_d, pi_u;

    cam.pix2intrinsic(pix_u, pi_u);
    cam.unDistort(pi_u, pi_d);
    cam.Distort(pi_d, pi_u2);
    cam.intrinsic2pix(pi_u2, pix_u2);

    EXPECT_NEAR(pix_u2.x(), pix_u.x(), 1e-3);
    EXPECT_NEAR(pix_u2.y(), pix_u.y(), 1e-3);
}

TEST (Camera, CalibratedVsUncalibrated)
{
    Vector2d focal_len{250.0, 250.0};
    Vector2d cam_center{320.0, 240.0};
    Vector2d img_size{640, 480};
    double s = 0.0;
    Camera<double> cam(focal_len, cam_center, s, img_size);
    UncalibratedCamera<double> ucam(cam);

    Vector2d pix, pix2;
    Vector3d pt{1.0, 0.5, 2.0};

    cam.proj(pt, pix);
    ucam.proj(pt, pix2);
    EXPECT_NEAR(pix.x(), pix.x(), 1e-5);
    EXPECT_NEAR(pix.y(), pix.y(), 1e-5);


    Vector3d pt1, pt2;
    cam.invProj(pix, pt.norm(), pt1);
    ucam.invProj(pix2, pt.norm(), pt2);
    EXPECT_NEAR(pt1.x(), pt2.x(), 1e-5);
    EXPECT_NEAR(pt1.y(), pt2.y(), 1e-5);
    EXPECT_NEAR(pt1.z(), pt2.z(), 1e-5);
}

TEST (Camera, VsOpenCV)
{
    Vector2d focal_len{189.288014941235, 188.89300027962554};
    Vector2d cam_center{189.57352223555952, 114.25998149460739};
    Vector2d img_size{376, 240};
    Vector5d distortion;
    double s = 0;
    distortion << -0.28839265926889596, 0.06052865219979488, 0.003175682270413171,  0.0014326472335837869, 0;
    UncalibratedCamera<double> cam(focal_len, cam_center, s, img_size, distortion);

    cv::Mat K = (cv::Mat_<double>(3,3) << focal_len(0), s, cam_center(0),
                                          0, focal_len(1), cam_center(1),
                                          0, 0, 1);
    cv::Mat D(5, 1, CV_64F, distortion.data());

    std::vector<cv::Point2d> cv_pts_u, cv_pts_d;
    std::vector<Vector2d, aligned_allocator<Vector2d>> e_pts_u, e_pts_d;

    for (int i = 0; i < 376; i+=10)
    {
        for(int j = 0; j < 240; j+=10)
        {
            cv_pts_d.push_back(cv::Point2d(i, j));
            e_pts_d.push_back(Vector2d(i, j));
        }
    }

    // undistort points with opencv
    cv::undistortPoints(cv_pts_d, cv_pts_u, K, D);

    // undistort with Uncalibrated Camera class
    e_pts_u.reserve(e_pts_d.size());
    for (auto& pix_d : e_pts_d)
    {
        Vector2d pi_d;
        cam.pix2intrinsic(pix_d, pi_d);
        Vector2d pi_u;
        cam.unDistort(pi_d, pi_u);
        e_pts_u.push_back(pi_u);
    }

    // check if they are right
    for (int i = 0; i < cv_pts_u.size(); i++)
    {
        EXPECT_NEAR(cv_pts_u[i].x, e_pts_u[i].x(), 1e-5);
        EXPECT_NEAR(cv_pts_u[i].y, e_pts_u[i].y(), 1e-5);
    }
}



