#include "aquila/estimator.hpp"
#include <cmath>
#include <iostream>

namespace aquila {

namespace {
    constexpr double PI = 3.14159265358979323846;
    constexpr double EPS_VEL = 1e-3;
} // namespace

// =========================================================
// SimpleEstimator Implementation 
// =========================================================

double SimpleEstimator::wrap_pi(double angle) {
    while (angle > PI) {
        angle -= 2.0 * PI;
    }
    while (angle < -PI) {
        angle += 2.0 * PI;
    }
    return angle;
}

// Construct a yaw-only quaternion (w, x, y, z) from yaw [rad]
std::array<double, 4> SimpleEstimator::quat_from_yaw(double yaw_rad) {
    const double half_yaw = 0.5 * yaw_rad;
    const double c = std::cos(half_yaw);
    const double s = std::sin(half_yaw);
    // Yaw about z-axis (NED convention)
    return {c, 0.0, 0.0, s};
}

SimpleEstimator::SimpleEstimator() {
    // Start with zero state and identity quaternion.
    state_.position_ned = {0.0, 0.0, 0.0};
    state_.velocity_ned = {0.0, 0.0, 0.0};
    state_.quat_nb      = {1.0, 0.0, 0.0, 0.0};
    state_.omega_body   = {0.0, 0.0, 0.0};
    yaw_rad_            = 0.0;
    initialized_        = false;
}

void SimpleEstimator::predict(double dt_s, const ImuMeasurement& imu) {
    if (!initialized_) {
        // We need an initial GPS fix before we can propagate meaningfully.
        return;
    }

    if (dt_s <= 0.0) {
        return;
    }

    // --- Attitude / Yaw propagation using gyro yaw rate ---

    // For now, we assume roll/pitch are small and only integrate yaw.
    yaw_rad_ += imu.gyro_rads[2] * dt_s;
    yaw_rad_ = wrap_pi(yaw_rad_);

    state_.quat_nb = quat_from_yaw(yaw_rad_);
    state_.omega_body = imu.gyro_rads;

    // --- Velocity / Position propagation using accelerometer ---

    // In this early version, we assume accel_x in body is roughly aligned with
    // forward direction in NED (x: North, y: East). We ignore accel_y, accel_z
    // for now. This will be upgraded for 6-DOF later.
    const double accel_body_x = imu.accel_mps2[0];

    // Project body forward accel into NED horizontal frame using yaw.
    const double cos_yaw = std::cos(yaw_rad_);
    const double sin_yaw = std::sin(yaw_rad_);

    const double ax_n = accel_body_x * cos_yaw;
    const double ay_n = accel_body_x * sin_yaw;
    const double az_n = 0.0;  // keep altitude dynamics trivial for now

    // Integrate velocity
    state_.velocity_ned[0] += ax_n * dt_s;
    state_.velocity_ned[1] += ay_n * dt_s;
    state_.velocity_ned[2] += az_n * dt_s;

    // Integrate position
    state_.position_ned[0] += state_.velocity_ned[0] * dt_s;
    state_.position_ned[1] += state_.velocity_ned[1] * dt_s;
    state_.position_ned[2] += state_.velocity_ned[2] * dt_s;
}

void SimpleEstimator::update_gps(const GpsMeasurement& gps) {
    if (!gps.valid) {
        return;
    }

    // On first valid GPS, initialize state directly from measurements.
    if (!initialized_) {
        state_.position_ned = gps.position_ned;
        state_.velocity_ned = gps.velocity_ned;

        // Initialize yaw from GPS velocity, if we're moving.
        const double vx = gps.velocity_ned[0];
        const double vy = gps.velocity_ned[1];

        if (std::abs(vx) > EPS_VEL || std::abs(vy) > EPS_VEL) {
            yaw_rad_ = std::atan2(vy, vx); // NED: x=North, y=East
        } else {
            yaw_rad_ = 0.0;
        }
        yaw_rad_ = wrap_pi(yaw_rad_);
        state_.quat_nb = quat_from_yaw(yaw_rad_);

        state_.omega_body = {0.0, 0.0, 0.0};
        initialized_ = true;
        return;
    }

    // --- Position / velocity correction ---

    // GPS - estimate
    std::array<double, 3> pos_err{};
    std::array<double, 3> vel_err{};

    for (int i = 0; i < 3; ++i) {
        pos_err[i] = gps.position_ned[i] - state_.position_ned[i];
        vel_err[i] = gps.velocity_ned[i] - state_.velocity_ned[i];
    }

    for (int i = 0; i < 3; ++i) {
        state_.position_ned[i] += gains_.k_pos * pos_err[i];
        state_.velocity_ned[i] += gains_.k_vel * vel_err[i];
    }

    // --- Yaw correction from GPS track ---

    const double vx = gps.velocity_ned[0];
    const double vy = gps.velocity_ned[1];

    if (std::abs(vx) > EPS_VEL || std::abs(vy) > EPS_VEL) {
        const double yaw_gps = wrap_pi(std::atan2(vy, vx));
        double yaw_err = wrap_pi(yaw_gps - yaw_rad_);

        yaw_rad_ += gains_.k_yaw * yaw_err;
        yaw_rad_ = wrap_pi(yaw_rad_);
        state_.quat_nb = quat_from_yaw(yaw_rad_);
    }
}

void SimpleEstimator::update_baro(const BaroMeasurement& baro) {
    if (!baro.valid) {
        return;
    }

    // Barometer altitude is positive up; NED z is down.
    const double z_meas = -baro.altitude_m;

    // Complementary filter on altitude, short-term agile from sensor measurement and long-term stable from state estimation
    const double z_est = state_.position_ned[2];
    const double z_new = (1.0 - gains_.k_baro) * z_est + gains_.k_baro * z_meas;
    state_.position_ned[2] = z_new;
}

// =========================================================
// EkfEstimator Implementation
// =========================================================

EkfEstimator::EkfEstimator() 
    : accel_noise_std_mps2_(0.5),
      yaw_rate_noise_std_radps_(0.01),
      gps_pos_noise_std_m_(2.0),
      gps_vel_noise_std_mps_(0.5),
      baro_noise_std_m_(1.0) 
{
    x_.setZero();
    x_(2) = -100.0; // Default initialization
    
    P_.setIdentity();
    P_ *= 10.0; // Large initial uncertainty

    // Build constant measurement covariances
    R_gps_.setZero();
    double sp2 = gps_pos_noise_std_m_ * gps_pos_noise_std_m_;
    double sv2 = gps_vel_noise_std_mps_ * gps_vel_noise_std_mps_;
    for (int i = 0; i < 3; ++i) {
        R_gps_(i, i)     = sp2;
        R_gps_(i+3, i+3) = sv2;
    }
    R_baro_(0,0) = baro_noise_std_m_ * baro_noise_std_m_;
}

Eigen::Matrix3d EkfEstimator::R_nb(double yaw_rad) {
    double c = std::cos(yaw_rad);
    double s = std::sin(yaw_rad);
    Eigen::Matrix3d R;
    R <<  c, -s, 0.0,
          s,  c, 0.0,
        0.0, 0.0, 1.0;
    return R;
}

double EkfEstimator::wrap_pi(double angle) {
    while (angle > PI) angle -= 2.0 * PI;
    while (angle < -PI) angle += 2.0 * PI;
    return angle;
}

void EkfEstimator::build_process_noise(double dt_s, Mat7& Q) const {
    Q.setZero();
    double dt2 = dt_s * dt_s;
    double sa2 = accel_noise_std_mps2_ * accel_noise_std_mps2_;
    double so2 = yaw_rate_noise_std_radps_ * yaw_rate_noise_std_radps_;
    
    // Position/Velocity driven by accelerometer noise (Horizontal)
    for (int i = 0; i < 2; ++i) { 
        Q(i, i)       = 0.5 * sa2 * dt2;
        Q(i + 3, i+3) = sa2 * dt2; 
    }
    
    // Vertical velocity modeled as random walk
    double svd2 = 1.0; 
    Q(5, 5) = svd2 * dt_s;
    Q(2, 2) = 0.25 * svd2 * dt2; 

    // Yaw noise
    Q(6, 6) = so2 * dt2;
}

void EkfEstimator::predict(double dt_s, const ImuMeasurement& imu) {
    if (dt_s <= 0.0) return;

    // 1. Unpack Linear State
    double pn = x_(0), pe = x_(1), pd = x_(2);
    double vn = x_(3), ve = x_(4), vd = x_(5);
    // Note: We stop using x_(6) (psi) for rotation, using quat_est_ instead.

    // 2. Propagate Attitude (Quaternion Integration)
    Eigen::Vector3d omega_b(imu.gyro_rads[0], imu.gyro_rads[1], imu.gyro_rads[2]);
    omega_body_radps_ = imu.gyro_rads; // Store for controller

    // dq/dt = 0.5 * q * omega
    // Eigen quaternion multiplication is p * q (Hamilton product)
    Eigen::Quaterniond q_omega(0.0, omega_b.x(), omega_b.y(), omega_b.z());
    Eigen::Quaterniond dq = quat_est_ * q_omega;
    
    // Euler integration for attitude
    quat_est_.w() += 0.5 * dq.w() * dt_s;
    quat_est_.x() += 0.5 * dq.x() * dt_s;
    quat_est_.y() += 0.5 * dq.y() * dt_s;
    quat_est_.z() += 0.5 * dq.z() * dt_s;
    quat_est_.normalize(); // Vital to prevent drift

    // 3. Propagate Velocity (rotate specific force body->NED)
    Eigen::Vector3d f_b(imu.accel_mps2[0], imu.accel_mps2[1], imu.accel_mps2[2]);
    Eigen::Vector3d g_n(0.0, 0.0, 9.81);
    
    // Rotate f_b to NED using the new attitude
    Eigen::Vector3d a_n = quat_est_ * f_b + g_n;

    double vn_next = vn + a_n.x() * dt_s;
    double ve_next = ve + a_n.y() * dt_s;
    double vd_next = vd + a_n.z() * dt_s; // Now we use Z-accel because we have Pitch!

    // 4. Propagate Position
    double pn_next = pn + vn_next * dt_s;
    double pe_next = pe + ve_next * dt_s;
    double pd_next = pd + vd_next * dt_s;

    // 5. Update State Vector
    // We keep x_(6) as "Yaw" for the EKF linear model, but we sync it from the quaternion
    // to keep the linearized Jacobian logic somewhat valid for GPS updates.
    // (A full Error-State Kalman Filter is better, but this is the simplest fix).
    Eigen::Vector3d euler = quat_est_.toRotationMatrix().eulerAngles(2, 1, 0); // ZYX order -> psi, theta, phi
    double current_yaw = euler[0]; 

    x_ << pn_next, pe_next, pd_next, vn_next, ve_next, vd_next, current_yaw;

    // 6. Propagate Covariance (Simplified)
    // We stick to the generic kinematic propagation. 
    // Since we now use full accel rotation, the simple Jacobian is an approximation,
    // but sufficient for Position/Velocity filtering.
    Mat7 F = Mat7::Identity();
    F(0, 3) = dt_s; F(1, 4) = dt_s; F(2, 5) = dt_s;
    
    Mat7 Q;
    build_process_noise(dt_s, Q);
    P_ = F * P_ * F.transpose() + Q;
}

void EkfEstimator::update_gps(const GpsMeasurement& gps) {
    if (!gps.valid) return;

    Eigen::Matrix<double, 6, 1> z;
    z << gps.position_ned[0], gps.position_ned[1], gps.position_ned[2],
         gps.velocity_ned[0], gps.velocity_ned[1], gps.velocity_ned[2];

    Eigen::Matrix<double, 6, 7> H;
    H.setZero();
    H.block<3,3>(0,0) = Eigen::Matrix3d::Identity(); // Pos
    H.block<3,3>(3,3) = Eigen::Matrix3d::Identity(); // Vel

    // Innovation
    Eigen::Matrix<double, 6, 1> y = z - H * x_;

    // Innovation Covariance
    Eigen::Matrix<double, 6, 6> S = H * P_ * H.transpose() + R_gps_;
    
    // Kalman Gain
    Eigen::Matrix<double, 7, 6> K = P_ * H.transpose() * S.inverse();

    // Update
    x_ = x_ + K * y;
    P_ = (Mat7::Identity() - K * H) * P_;
}

void EkfEstimator::update_baro(const BaroMeasurement& baro) {
    if (!baro.valid) return;

    double z = baro.altitude_m; // "Altitude" is typically positive Up
    // State pd is positive Down. So Altitude = -pd.
    // Measurement model: z = -pd
    
    double hx = -x_(2);
    double y = z - hx;

    Eigen::Matrix<double, 1, 7> H;
    H.setZero();
    H(0, 2) = -1.0;

    double S = (H * P_ * H.transpose())(0,0) + R_baro_(0,0);
    Eigen::Matrix<double, 7, 1> K = P_ * H.transpose() / S;

    x_ = x_ + K * y;
    P_ = (Mat7::Identity() - K * H) * P_;
}

NavState EkfEstimator::get_state() const {
    NavState s{};
    s.position_ned = {x_(0), x_(1), x_(2)};
    s.velocity_ned = {x_(3), x_(4), x_(5)};
    
    // CRITICAL FIX: Return the integrated full quaternion, not just Yaw.
    s.quat_nb = {quat_est_.w(), quat_est_.x(), quat_est_.y(), quat_est_.z()};
    
    s.omega_body = omega_body_radps_;
    return s;
}

} // namespace aquila