#pragma once

#include <array>
#include <cstdint>

namespace aquila {

struct ImuData {
    double timestamp_s{};
    std::array<double, 3> accel_mps2{};
    std::array<double, 3> gyro_rads{};
    bool valid{true};
};

struct GpsData {
    double timestamp_s{};
    std::array<double, 3> position_ned{};
    std::array<double, 3> velocity_ned{};
    bool valid{true};
};

struct BaroData {
    double timestamp_s{};
    double altitude_m{};
    bool valid{true};
};

} // namespace aquila