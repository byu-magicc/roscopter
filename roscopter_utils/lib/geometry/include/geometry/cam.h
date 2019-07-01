#pragma once

#include <vector>
#include <Eigen/Core>

template <typename T>
class Camera
{

public:
    typedef Eigen::Matrix<T,2,1> Vec2;
    typedef Eigen::Matrix<T,2,2> Mat2;
    typedef Eigen::Matrix<T,3,1> Vec3;
    typedef Eigen::Matrix<T,5,1> Vec5;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Camera() :
        focal_len_(buf_),
        cam_center_(buf_+CX),
        image_size_(buf_+RX),
        s_(buf_[S])
    {
      focal_len_.setZero();
      cam_center_.setZero();
      image_size_.setZero();
      s_ = T(0);
    }

    Camera(const Vec2& f, const Vec2& c, const T& s) :
        focal_len_(const_cast<T*>(f.data())),
        cam_center_(const_cast<T*>(c.data())),
        image_size_(buf_+RX),
        s_(*const_cast<T*>(&s))
    { }


    Camera(const T* f, const T* c, const T* s) :
        focal_len_ (const_cast<T*>(f)),
        cam_center_(const_cast<T*>(c)),
        image_size_(buf_+RX),
        s_(*const_cast<T*>(s))
    { }

    Camera(const Vec2& f, const Vec2& c, const T& s, const Vec2& size) :
        focal_len_(const_cast<T*>(f.data())),
        cam_center_(const_cast<T*>(c.data())),
        image_size_(const_cast<T*>(size.data())),
        s_(*const_cast<T*>(&s))
    { }

    Camera(const T* f, const T* c,const T* s, const T* size) :
        focal_len_ (const_cast<T*>(f)),
        cam_center_(const_cast<T*>(c)),
        image_size_(const_cast<T*>(size)),
        s_(*const_cast<T*>(s))
    { }

    Camera& operator=(const Camera& cam)
    {
      focal_len_ = cam.focal_len_;
      cam_center_ = cam.cam_center_;
      s_ = cam.s_;
      image_size_ = cam.image_size_;
    }

    template<typename T2>
    const Camera<T2> cast() const
    {
      Camera<T2> cam;
      cam.focal_len_ = focal_len_.template cast<T2>();
      cam.cam_center_ = cam_center_.template cast<T2>();
      cam.image_size_ = image_size_.template cast<T2>();
      cam.s_ = T2(s_);
      return cam;
    }

    template <typename T2>
    void pix2intrinsic(const Eigen::Matrix<T2,2,1>& pix, Eigen::Matrix<T2,2,1>& pi) const
    {
        const T& fx(focal_len_.x());
        const T& fy(focal_len_.y());
        const T& cx(cam_center_.x());
        const T& cy(cam_center_.y());
        pi << (1.0/fx) * (pix.x() - cx - (s_/fy) * (pix.y() - cy)),
              (1.0/fy) * (pix.y() - cy);
    }

    template <typename T2>
    void intrinsic2pix(const Eigen::Matrix<T2,2,1>& pi, Eigen::Matrix<T2,2,1>& pix) const
    {
        const T fx = focal_len_.x();
        const T fy = focal_len_.y();
        const T cx = cam_center_.x();
        const T cy = cam_center_.y();
        pix << fx*pi.x() + s_*pi.y() + cx,
               fy*pi.y() + cy;
    }

    template <typename T2>
    void proj(const Eigen::Matrix<T2,3,1>& pt, Eigen::Matrix<T2,2,1>& pix) const
    {
        Eigen::Matrix<T2,2,1> pi = pt.template topRows<2>() / pt.z();
        intrinsic2pix(pi, pix);
    }

    template <typename T2>
    Eigen::Matrix<T2,2,1> proj(const Eigen::Matrix<T2,3,1>& pt) const
    {
        Eigen::Matrix<T2,2,1> pix;
        proj(pt, pix);
        return pix;
    }

    template <typename T2>
    void invProj(const Eigen::Matrix<T2,2,1>& pix, const T2& depth, Eigen::Matrix<T2,3,1>& pt) const
    {
        Eigen::Matrix<T2,2,1> pi;
        pix2intrinsic(pix, pi);
        pt.template segment<2>(0) = pi;
        pt(2) = (T2)1.0;
        pt *= depth / pt.norm();
    }

    template <typename T2>
    Eigen::Matrix<T2,3,1> invProj(const Eigen::Matrix<T2,2,1>& pix, const T2& depth) const
    {
        Eigen::Matrix<T2,3,1> zeta;
        invProj(pix, depth, zeta);
        return zeta;
    }

    inline bool check(const Eigen::Vector2d& pix) const
    {
        return !((pix.array() > image_size_.array()).any()|| (pix.array() < 0).any());
    }

    Eigen::Matrix<T, 3, 3> K() const
    {
        Eigen::Matrix<T, 3, 3> out;
        out << focal_len_(0), s_, cam_center_(0),
               0, focal_len_(1), cam_center_(1),
               0, 0, 1;
        return out;
    }

    Eigen::Map<Vec2> focal_len_;
    Eigen::Map<Vec2> cam_center_;
    Eigen::Map<Vec2> image_size_;
    T& s_;

    enum {FX, FY, CX, CY, RX, RY, S, BUF_SIZE};
    T buf_[BUF_SIZE]; // [fx, fy, cx, cy, size_x, size_y, s]
};


template <typename T>
class UncalibratedCamera : public Camera<T>
{
public:
    typedef Eigen::Matrix<T,2,1> Vec2;
    typedef Eigen::Matrix<T,2,2> Mat2;
    typedef Eigen::Matrix<T,3,1> Vec3;
    typedef Eigen::Matrix<T,5,1> Vec5;

    T dist_buf[5];
    Eigen::Map<Vec5> distortion_;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    UncalibratedCamera() :
        distortion_(dist_buf)
    {
      distortion_.setZero();
    }

    UncalibratedCamera(const Vec2& f, const Vec2& c, const T& s, const Vec5& d) :
        Camera<T>(f, c, s),
        distortion_(const_cast<T*>(d.data()))
    { }


    UncalibratedCamera(const T* f, const T* c, const T* s, const T* d) :
        Camera<T>(f,c,s),
        distortion_(const_cast<T*>(d))
    { }

    UncalibratedCamera(const Vec2& f, const Vec2& c, const T& s, const Vec2& size, const Vec5& d) :
        Camera<T>(f,c,s,size),
        distortion_(const_cast<T*>(d.data()))
    { }

    UncalibratedCamera(const T* f, const T* c,const T* s, const T* size, const T* d) :
        Camera<T>(f, c, s, size),
        distortion_(const_cast<T*>(d))
    { }

    UncalibratedCamera(const Camera<T>& other) :
        Camera<T>(other),
        distortion_(dist_buf)
    {
        distortion_.setZero();
    }

    UncalibratedCamera& operator=(const Camera<T>& other)
    {
        Camera<T>::operator=(other);
        distortion_.setZero();
        return *this;
    }

    void unDistort(const Vec2& pi_d, Vec2& pi_u, int iter=5) const
    {
        if (distortion_(0) == (T)0)
        {
            pi_u = pi_d;
            return;
        }

        const T k1 = distortion_(0);
        const T k2 = distortion_(1);
        const T p1 = distortion_(2);
        const T p2 = distortion_(3);
        const T k3 = distortion_(4);
        T x = pi_d.x();
        T y = pi_d.y();

        const T x0 = x;
        const T y0 = y;

        // https://docs.opencv.org/2.4/doc/tutorials/calib3d/camera_calibration/camera_calibration.html
        // https://github.com/opencv/opencv/blob/master/modules/calib3d/src/undistort.cpp
        for (int i = 0; i < iter; i++)
        {
            const T xy = x*y;
            const T yy = y*y;
            const T xx = x*x;
            const T rr = xx + yy;
            const T g  = (T)1.0/((T)1.0 + ((k3*rr + k2)*rr + k1)*rr);
            const T dx = (T)2.0*p1*xy + p2*(rr + (T)2.0*xx);
            const T dy = p1*(rr + (T)2.0*yy) + (T)2.0*p2*xy;

            x = g * (x0 - dx);
            y = g * (y0 - dy);
        }

        pi_u << x, y;
    }

    void Distort(const Vec2& pi_u, Vec2& pi_d, double tol=1e-6) const
    {
        pi_d = pi_u;

        if (distortion_(0) == (T)0)
            return;

        Vec2 pihat_u;
        Mat2 J;
        Vec2 e;
        T prev_e = (T)1000.0;
        T enorm = (T)0.0;

        static const int max_iter = 50;
        int i = 0;
        while (i < max_iter)
        {
            double alpha = 1.0;
            do
            {
                distortJac(pi_d, J);
                unDistort(pi_d, pihat_u);
                e = pihat_u - pi_u;
                enorm = e.norm();
                pi_d = pi_d - J*(alpha * e);
                alpha *= 0.75;
            }
            while (enorm >= prev_e && alpha > 0.1);

            if (enorm <= tol)
                break;
            prev_e = enorm;
            i++;
        }
    }

    void distortJac(const Vec2& pi_u, Mat2& J) const
    {
        J.setOnes();
        const T k1 = distortion_(0);
        const T k2 = distortion_(1);
        const T p1 = distortion_(2);
        const T p2 = distortion_(3);
        const T k3 = distortion_(4);

        const T x = pi_u.x();
        const T y = pi_u.y();
        const T xy = x*y;
        const T xx = x*x;
        const T yy = y*y;
        const T r2 = xx+yy;
        const T r4 = r2*r2;
        const T r6 = r2*r4;
        const T r = sqrt(r2);
        const T g  = (T)1/((T)1 + ((k3*r2 + k2)*r2 + k1)*r2);
        const T dx = (T)2*p1*xy + p2*(r2 + (T)2*xx);
        const T dy = p1*(r2 + (T)2*yy) + (T)2*p2*xy;

        const T drdx = x / r;
        const T drdy = y / r;
        const T dgdr_denom = (1+k3*r6+k2*r4+k1*r2);
        const T dgdr = -2*r*(3*k3*r4+2*k2*r2+k1)/(dgdr_denom*dgdr_denom);
        const T ddxdx = 2*p1*y + 6*p2*x;
        const T ddxdy = 2*p1*x + 2*p2*y;
        const T ddydx = ddxdy; // 2*p1*x + 2*p2*y;
        const T ddydy = 6*p1*y + 2*p2*x;

        Mat2 J1, J2;
        J1 << drdx*(x-dx), drdy*(x-dx),
              drdx*(y-dy), drdy*(y-dy);

        J2 << (1-ddxdx), ddxdy,
              ddydx, (1-ddydy);

        J = dgdr * J1 + g * J2;
        if ((J.array() != J.array()).any())
        {
            int debug = 1;
        }
    }

    template <typename T2>
    void proj(const Eigen::Matrix<T2,3,1>& pt, Eigen::Matrix<T2,2,1>& pix) const
    {
        Eigen::Matrix<T2,2,1> pi_d;
        Eigen::Matrix<T2,2,1> pi_u = pt.template topRows<2>() / pt.z();
        Distort(pi_u, pi_d);
        Camera<T>::intrinsic2pix(pi_d, pix);
    }

    template <typename T2>
    Eigen::Matrix<T2,2,1> proj(const Eigen::Matrix<T2,3,1>& pt) const
    {
        Eigen::Matrix<T2,2,1> pix;
        proj(pt, pix);
        return pix;
    }

    template <typename T2>
    void invProj(const Eigen::Matrix<T2,2,1>& pix, const T& depth, Eigen::Matrix<T2,3,1>& pt) const
    {
        Eigen::Matrix<T2,2,1> pi_d, pi_u;
        Camera<T>::pix2intrinsic(pix, pi_d);
        unDistort(pi_d, pi_u);
        pt.template segment<2>(0) = pi_u;
        pt(2) = (T)1.0;
        pt *= depth / pt.norm();
    }

    template <typename T2>
    Eigen::Matrix<T2,3,1> invProj(const Eigen::Matrix<T2,2,1>& pix, const T2& depth) const
    {
        Eigen::Matrix<T2,3,1> zeta;
        invProj(pix, depth, zeta);
        return zeta;
    }
};

typedef UncalibratedCamera<double> UCam;
typedef Camera<double> Cam;
