#include "aquila/estimator.hpp"
#include <cmath>

namespace aquila {

namespace {

constexpr double PI = 3.14159265358979323846;

// Convenience for tiny thresholds
constexpr double EPS_VEL = 1e-3;

} // namespace

// --- Simple Estimator Helper Functions ---

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

// --- EKF Helper Functions ---

Eigen::Matrix3d EkfEstimator::R_nb(double yaw_rad) {
    const double c = std::cos(yaw_rad);
    const double s = std::sin(yaw_rad);
    Eigen::Matrix3d R;
    R <<  c, -s, 0.0,
          s,  c, 0.0,
          0.0, 0.0, 1.0;
    return R;
}

double EkfEstimator::wrap_pi(double angle) {
    while (angle > PI) {
        angle -= 2.0 * PI;
    }
    while (angle < -PI) {
        angle += 2.0 * PI;
    }
    return angle;
}

void EkfEstimator::build_process_noise(double dt_s, Mat7& Q) const {
    Q.setZero();

    const double sa2 = accel_noise_std_mps2_ * accel_noise_std_mps2_;
    const double so2 = yaw_rate_noise_std_radps_ * yaw_rate_noise_std_radps_;

    const double dt2 = dt_s * dt_s;

    // Simple diagonal model: position and velocity affected by accel noise,
    // yaw affected by yaw-rate noise.
    for (int i = 0; i < 3; ++i) {
        Q(i, i)       = 0.5 * sa2 * dt2; // position
        Q(i + 3, i+3) = sa2 * dt2;       // velocity
    }
    Q(6, 6) = so2 * dt2;
}

// --- Constructor ---

EkfEstimator::EkfEstimator()
    : accel_noise_std_mps2_(0.5),
      yaw_rate_noise_std_radps_(0.01),
      gps_pos_noise_std_m_(2.0),
      gps_vel_noise_std_mps_(0.5),
      baro_noise_std_m_(1.0) {

    x_.setZero();
    // Initial position NED = (0,0,-100), velocity zero, yaw 0
    x_(2) = -100.0;

    P_.setIdentity();
    P_ *= 10.0; // fairly large initial uncertainty

    // Build measurement covariances
    R_gps_.setZero();
    const double sp2 = gps_pos_noise_std_m_ * gps_pos_noise_std_m_;
    const double sv2 = gps_vel_noise_std_mps_ * gps_vel_noise_std_mps_;
    for (int i = 0; i < 3; ++i) {
        R_gps_(i, i)     = sp2;
        R_gps_(i+3, i+3) = sv2;
    }

    R_baro_(0,0) = baro_noise_std_m_ * baro_noise_std_m_;
}

// --- Predict ---

void EkfEstimator::predict(double dt_s, const ImuMeasurement& imu) {
    if (dt_s <= 0.0) {
        return;
    }

    // Unpack state
    double pn  = x_(0);
    double pe  = x_(1);
    double pd  = x_(2);
    double vn  = x_(3);
    double ve  = x_(4);
    double vd  = x_(5);
    double psi = x_(6);

    // IMU inputs (body-frame specific force and yaw-rate)
    const double fb_x = imu.accel_mps2[0];
    const double fb_y = imu.accel_mps2[1];
    const double fb_z = imu.accel_mps2[2];
    const double wz   = imu.gyro_rads[2]; 

    // Convert specific force to NED acceleration
    Eigen::Vector3d f_b(fb_x, fb_y, fb_z);
    const Eigen::Matrix3d Rnb = R_nb(psi);
    const Eigen::Vector3d g_n(0.0, 0.0, 9.81);

    Eigen::Vector3d a_n = Rnb * f_b + g_n;
    const double an = a_n(0);
    const double ae = a_n(1);
    const double ad = a_n(2);

    // Integrate velocity
    const double vn_next = vn + an * dt_s;
    const double ve_next = ve + ae * dt_s;
    const double vd_next = vd + ad * dt_s;

    // Integrate position (using updated velocity)
    const double pn_next = pn + vn_next * dt_s;
    const double pe_next = pe + ve_next * dt_s;
    const double pd_next = pd + vd_next * dt_s;

    // Integrate yaw
    double psi_next = wrap_pi(psi + wz * dt_s);

    // Write back predicted state
    x_(0) = pn_next;
    x_(1) = pe_next;
    x_(2) = pd_next;
    x_(3) = vn_next;
    x_(4) = ve_next;
    x_(5) = vd_next;
    x_(6) = psi_next;

    // Build state transition Jacobian F
    Mat7 F = Mat7::Identity();

    // Position derivatives wrt velocity
    F(0, 3) = dt_s;
    F(1, 4) = dt_s;
    F(2, 5) = dt_s;

    // Velocity derivatives wrt yaw (from a_n(psi))
    const double c = std::cos(psi);
    const double s = std::sin(psi);

    const double da_n_dpsi = -s * fb_x - c * fb_y;
    const double da_e_dpsi =  c * fb_x - s * fb_y;

    F(3, 6) = da_n_dpsi * dt_s;
    F(4, 6) = da_e_dpsi * dt_s;
    // F(5, 6) remains 0 (ad doesn't depend on yaw)

    // Process noise
    Mat7 Q;
    build_process_noise(dt_s, Q);

    // Covariance prediction
    P_ = F * P_ * F.transpose() + Q;
}

// --- GPS update ---

void EkfEstimator::update_gps(const GpsMeasurement& gps) {
    // Measurement z = [pn, pe, pd, vn, ve, vd]^T
    Eigen::Matrix<double, 6, 1> z;
    z << gps.position_ned[0],
         gps.position_ned[1],
         gps.position_ned[2],
         gps.velocity_ned[0],
         gps.velocity_ned[1],
         gps.velocity_ned[2];

    // h(x) for GPS is linear: H * x
    Eigen::Matrix<double, 6, 7> H;
    H.setZero();
    H.block<3,3>(0,0) = Eigen::Matrix3d::Identity();    // position
    H.block<3,3>(3,3) = Eigen::Matrix3d::Identity();    // velocity

    Eigen::Matrix<double, 6, 1> hx = H * x_;
    Eigen::Matrix<double, 6, 1> y  = z - hx;            // innovation

    Eigen::Matrix<double, 6, 6> S = H * P_ * H.transpose() + R_gps_;
    Eigen::Matrix<double, 7, 6> K = P_ * H.transpose() * S.inverse();

    x_ = x_ + K * y;
    P_ = (Mat7::Identity() - K * H) * P_;
}

// --- Baro update ---

void EkfEstimator::update_baro(const BaroMeasurement& baro) {
    // z = altitude [m], positive up
    const double z = baro.altitude_m;

    // h(x) = -pd
    const double hx = -x_(2);

    const double y = z - hx; // innovation

    Eigen::Matrix<double, 1, 7> H;
    H.setZero();
    H(0, 2) = -1.0;

    const double S = (H * P_ * H.transpose())(0,0) + R_baro_(0,0);
    Eigen::Matrix<double, 7, 1> K = P_ * H.transpose() / S;

    x_ = x_ + K * y;
    P_ = (Mat7::Identity() - K * H) * P_;
}

// --- NavState output ---

NavState EkfEstimator::get_state() const {
    NavState s{};

    s.position_ned[0] = x_(0);
    s.position_ned[1] = x_(1);
    s.position_ned[2] = x_(2);

    s.velocity_ned[0] = x_(3);
    s.velocity_ned[1] = x_(4);
    s.velocity_ned[2] = x_(5);

    const double psi = x_(6);

    // Build quaternion with yaw-only attitude (roll=pitch=0)
    const double half_yaw = 0.5 * psi;
    const double cy = std::cos(half_yaw);
    const double sy = std::sin(half_yaw);

    // q = [w, x, y, z], yaw-only
    s.quat_nb[0] = cy;
    s.quat_nb[1] = 0.0;
    s.quat_nb[2] = 0.0;
    s.quat_nb[3] = sy;

    return s;
}

} // namespace aquila