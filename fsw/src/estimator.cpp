#include "aquila/estimator.hpp"

namespace aquila {

void SimpleEstimator::predict(double /*dt_s*/, const ImuData& /*imu*/) {
    // TODO: implement prediction step
}

void SimpleEstimator::update_gps(const GpsData& /*gps*/) {
    // TODO: implement GPS update
}

void SimpleEstimator::update_baro(const BaroData& /*baro*/) {
    // TODO: implement baro update
}

} // namespace aquila