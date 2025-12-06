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

// Simple placeholder implementation
class SimpleEstimator : public Estimator {
public:
    void predict(double dt_s, const ImuData& imu) override;
    void update_gps(const GpsData& gps) override;
    void update_baro(const BaroData& baro) override;

    NavState get_state() const override { return state_; }

private:
    NavState state_;
};

} // namespace aquila