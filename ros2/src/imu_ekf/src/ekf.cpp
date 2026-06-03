#include "imu_ekf/ekf.hpp"
#include <cmath>

using namespace Eigen;

// Noise parameters — all from MPU-6050 datasheet §6.1 except bias RW which is tunable
static constexpr double ACCEL_NOISE_DENSITY = 3.923e-3;  // m/s²/√Hz
static constexpr double GYRO_NOISE_DENSITY  = 8.727e-5;  // rad/s/√Hz
static constexpr double ACCEL_BIAS_RW       = 1.0e-4;    // m/s²/√s  (random walk, tunable)
static constexpr double GYRO_BIAS_RW        = 1.0e-5;    // rad/s/√s (random walk, tunable)

static const Vector3d G_W(0.0, 0.0, -9.80665);  // gravity, world frame (z-up)

EKF::EKF()
{
    x_.setZero();
    x_(6) = 1.0;  // identity quaternion: qw = 1

    P_.setZero();
    P_.block<3,3>(0,0)   = Matrix3d::Identity() * 1e-6;  // position: starts at known origin
    P_.block<3,3>(3,3)   = Matrix3d::Identity() * 1e-4;  // velocity: near-zero but uncertain
    P_.block<4,4>(6,6)   = Matrix4d::Identity() * 1e-6;  // orientation: set by init phase
    P_.block<3,3>(10,10) = Matrix3d::Identity() * 1e-4;  // accel bias: unknown
    P_.block<3,3>(13,13) = Matrix3d::Identity() * 1e-4;  // gyro bias: unknown

    // Q stores per-unit-time PSD; multiplied by dt each predict() call
    Q_.setZero();
    Q_.block<3,3>(3,3)   = Matrix3d::Identity() * (ACCEL_NOISE_DENSITY * ACCEL_NOISE_DENSITY);
    Q_.block<4,4>(6,6)   = Matrix4d::Identity() * 0.25 * (GYRO_NOISE_DENSITY * GYRO_NOISE_DENSITY);
    Q_.block<3,3>(10,10) = Matrix3d::Identity() * (ACCEL_BIAS_RW * ACCEL_BIAS_RW);
    Q_.block<3,3>(13,13) = Matrix3d::Identity() * (GYRO_BIAS_RW * GYRO_BIAS_RW);
}

void EKF::predict(const Vector3d& accel_m, const Vector3d& gyro_m, double dt)
{
    const Vector3d p  = x_.segment<3>(0);
    const Vector3d v  = x_.segment<3>(3);
    const Vector4d q  = x_.segment<4>(6);
    const Vector3d ba = x_.segment<3>(10);
    const Vector3d bg = x_.segment<3>(13);

    const Vector3d a = accel_m - ba;  // bias-corrected accel (body frame)
    const Vector3d w = gyro_m  - bg;  // bias-corrected gyro  (body frame)

    const Matrix3d R = rotFromQuat(q);

    // State propagation
    x_.segment<3>(0) = p + v * dt;
    x_.segment<3>(3) = v + (R * a + G_W) * dt;

    // Quaternion integration: exact small-angle rotation in body frame
    const double angle = w.norm() * dt;
    Vector4d dq;
    if (angle > 1e-10) {
        const double s = std::sin(0.5 * angle) / w.norm();
        dq << std::cos(0.5 * angle), s * w;
    } else {
        dq(0) = 1.0;
        dq.tail<3>() = 0.5 * dt * w;
    }
    Vector4d q_new = quatMul(q, dq);
    q_new.normalize();
    x_.segment<4>(6) = q_new;
    // biases: no change (modelled as random walk, driven by Q)

    // Covariance propagation: P = F*P*F' + Q*dt
    // F = I + Fc*dt, first-order linearization about current state
    Matrix<double, 16, 16> F = Matrix<double, 16, 16>::Identity();
    F.block<3,3>(0,  3)  = Matrix3d::Identity() * dt;                       // dp/dv
    F.block<3,4>(3,  6)  = dRotVec_dq(q, a) * dt;                           // dv/dq
    F.block<3,3>(3,  10) = -R * dt;                                          // dv/dba
    F.block<4,4>(6,  6)  = Matrix4d::Identity() + 0.5 * omegaMat(w) * dt;   // dq/dq
    F.block<4,3>(6,  13) = -0.5 * xiMat(q) * dt;                            // dq/dbg

    P_ = F * P_ * F.transpose() + Q_ * dt;
}

// Rotation matrix from body to world given quaternion q = [qw, qx, qy, qz]
Matrix3d EKF::rotFromQuat(const Vector4d& q)
{
    const double qw = q(0), qx = q(1), qy = q(2), qz = q(3);
    Matrix3d R;
    R << 1-2*(qy*qy+qz*qz),  2*(qx*qy-qw*qz),    2*(qx*qz+qw*qy),
         2*(qx*qy+qw*qz),    1-2*(qx*qx+qz*qz),  2*(qy*qz-qw*qx),
         2*(qx*qz-qw*qy),    2*(qy*qz+qw*qx),    1-2*(qx*qx+qy*qy);
    return R;
}

// Hamilton quaternion product p ⊗ r
Vector4d EKF::quatMul(const Vector4d& p, const Vector4d& r)
{
    Vector4d out;
    out(0) = p(0)*r(0) - p(1)*r(1) - p(2)*r(2) - p(3)*r(3);
    out(1) = p(0)*r(1) + p(1)*r(0) + p(2)*r(3) - p(3)*r(2);
    out(2) = p(0)*r(2) - p(1)*r(3) + p(2)*r(0) + p(3)*r(1);
    out(3) = p(0)*r(3) + p(1)*r(2) - p(2)*r(1) + p(3)*r(0);
    return out;
}

// 4x4 matrix Ω(w) such that q̇ = 0.5 * Ω(w) * q
Matrix4d EKF::omegaMat(const Vector3d& w)
{
    Matrix4d O;
    O <<     0, -w(0), -w(1), -w(2),
          w(0),     0,  w(2), -w(1),
          w(1), -w(2),     0,  w(0),
          w(2),  w(1), -w(0),     0;
    return O;
}

// 4x3 matrix Ξ(q) such that q̇ = 0.5 * Ξ(q) * ω (used for dq/dbg Jacobian)
Matrix<double, 4, 3> EKF::xiMat(const Vector4d& q)
{
    const double qw = q(0), qx = q(1), qy = q(2), qz = q(3);
    Matrix<double, 4, 3> Xi;
    Xi << -qx, -qy, -qz,
           qw, -qz,  qy,
           qz,  qw, -qx,
          -qy,  qx,  qw;
    return Xi;
}

// 3x4 Jacobian of R(q)*v with respect to q
Matrix<double, 3, 4> EKF::dRotVec_dq(const Vector4d& q, const Vector3d& v)
{
    const double qw = q(0), qx = q(1), qy = q(2), qz = q(3);
    const double vx = v(0), vy = v(1), vz = v(2);
    Matrix<double, 3, 4> J;
    J << -2*qz*vy+2*qy*vz,   2*qy*vy+2*qz*vz,          -4*qy*vx+2*qx*vy+2*qw*vz,  -4*qz*vx-2*qw*vy+2*qx*vz,
         2*qz*vx-2*qx*vz,    2*qy*vx-4*qx*vy-2*qw*vz,   2*qx*vx+2*qz*vz,           2*qw*vx-4*qz*vy+2*qy*vz,
        -2*qy*vx+2*qx*vy,    2*qz*vx+2*qw*vy-4*qx*vz,  -2*qw*vx+2*qz*vy-4*qy*vz,   2*qx*vx+2*qy*vy;
    return J;
}
