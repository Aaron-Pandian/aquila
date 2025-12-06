#pragma once

#include <array>

namespace aquila {

struct NavState {
    // Position and velocity in NED [m], [m/s]
    std::array<double, 3> position_ned{};
    std::array<double, 3> velocity_ned{};

    // Attitude quaternion (w, x, y, z) body relative to NED
    std::array<double, 4> quat_nb{1.0, 0.0, 0.0, 0.0};

    // Body angular rates [rad/s]
    std::array<double, 3> omega_body{};
};

} // namespace aquila