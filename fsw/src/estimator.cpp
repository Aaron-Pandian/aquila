#include "aquila/estimator.hpp"

#include <cmath>

namespace aquila {

namespace {

constexpr double PI = 3.14159265358979323846;

// Convenience for tiny thresholds
constexpr double EPS_VEL = 1e-3;

} // namespace

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

void SimpleEstimator::predict(double dt_s, const ImuData& imu) {
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

void SimpleEstimator::update_gps(const GpsData& gps) {
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

void SimpleEstimator::update_baro(const BaroData& baro) {
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

} // namespace aquila