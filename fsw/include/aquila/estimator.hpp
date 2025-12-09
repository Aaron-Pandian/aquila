#pragma once

#include <Eigen/Dense>
#include "state.hpp"
#include "sensors.hpp"

namespace aquila {

class Estimator {
public:
    virtual ~Estimator() = default;

    virtual void predict(double dt_s, const ImuMeasurement& imu) = 0;
    virtual void update_gps(const GpsMeasurement& gps) = 0;
    virtual void update_baro(const BaroMeasurement& baro) = 0;

    virtual NavState get_state() const = 0;
};

// Lightweight IMU + GPS + Barometer fusion.
class SimpleEstimator : public Estimator {
public:
    SimpleEstimator();

    void predict(double dt_s, const ImuMeasurement& imu) override;
    void update_gps(const GpsMeasurement& gps) override;
    void update_baro(const BaroMeasurement& baro) override;

    NavState get_state() const override { return state_; }

private:
    struct Gains {
        double k_pos   = 0.2;  // position correction gain
        double k_vel   = 0.3;  // velocity correction gain
        double k_yaw   = 0.1;  // yaw correction gain
        double k_baro  = 0.2;  // altitude correction gain
    };

    NavState state_{};
    Gains gains_{};

    double yaw_rad_{0.0};   // current yaw estimate [rad]
    bool initialized_{false};

    static double wrap_pi(double angle);
    static std::array<double, 4> quat_from_yaw(double yaw_rad);
};

// Extended Kalman Filter for GPS + Barometer fusion.
class EkfEstimator : public Estimator {
public:
    EkfEstimator();

    void predict(double dt_s, const ImuMeasurement& imu) override;
    void update_gps(const GpsMeasurement& gps) override;
    void update_baro(const BaroMeasurement& baro) override;
    NavState get_state() const override;

private:
    using Vec7 = Eigen::Matrix<double, 7, 1>;
    using Mat7 = Eigen::Matrix<double, 7, 7>;

    Vec7 x_;   // [pn, pe, pd, vn, ve, vd, psi]
    Mat7 P_;   // 7x7 covariance

    // Process noise params
    double accel_noise_std_mps2_;
    double yaw_rate_noise_std_radps_;

    // Measurement noise params
    double gps_pos_noise_std_m_;
    double gps_vel_noise_std_mps_;
    double baro_noise_std_m_;

    // Cached measurement covariances
    Eigen::Matrix<double, 6, 6> R_gps_;
    Eigen::Matrix<double, 1, 1> R_baro_;

    // Helpers
    static Eigen::Matrix3d R_nb(double yaw_rad);
    static double wrap_pi(double angle);

    void build_process_noise(double dt_s, Mat7& Q) const;
};

} // namespace aquila