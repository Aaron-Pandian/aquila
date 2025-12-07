#pragma once

#include "state.hpp"
#include "sensors.hpp"

namespace aquila {

class Estimator {
public:
    virtual ~Estimator() = default;

    virtual void predict(double dt_s, const ImuData& imu) = 0;
    virtual void update_gps(const GpsData& gps) = 0;
    virtual void update_baro(const BaroData& baro) = 0;

    virtual NavState get_state() const = 0;
};

// Lightweight IMU + GPS + Barometer fusion.
// This is intentionally kept simple (no matrices, O(1) per update) but
class SimpleEstimator : public Estimator {
public:
    SimpleEstimator();

    void predict(double dt_s, const ImuData& imu) override;
    void update_gps(const GpsData& gps) override;
    void update_baro(const BaroData& baro) override;

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

} // namespace aquila