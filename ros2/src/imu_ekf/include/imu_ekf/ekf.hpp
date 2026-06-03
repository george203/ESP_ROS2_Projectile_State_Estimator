#pragma once
#include <Eigen/Dense>

// State layout (16-D):
//   x[0:3]   position      (world frame, m)
//   x[3:6]   velocity      (world frame, m/s)
//   x[6:10]  quaternion    q = [qw, qx, qy, qz], world <- body, Hamilton convention
//   x[10:13] accel bias    (body frame, m/s²)
//   x[13:16] gyro bias     (body frame, rad/s)

class EKF {
public:
    EKF();

    void predict(const Eigen::Vector3d& accel_m, const Eigen::Vector3d& gyro_m, double dt);

    Eigen::Matrix<double, 16, 1>  x_;  // state vector
    Eigen::Matrix<double, 16, 16> P_;  // state covariance

private:
    Eigen::Matrix<double, 16, 16> Q_;  // continuous-time process noise PSD (scaled by dt in predict)

    static Eigen::Matrix3d rotFromQuat(const Eigen::Vector4d& q);
    static Eigen::Vector4d quatMul(const Eigen::Vector4d& p, const Eigen::Vector4d& r);
    static Eigen::Matrix4d omegaMat(const Eigen::Vector3d& w);
    static Eigen::Matrix<double, 4, 3> xiMat(const Eigen::Vector4d& q);
    static Eigen::Matrix<double, 3, 4> dRotVec_dq(const Eigen::Vector4d& q, const Eigen::Vector3d& v);
};
