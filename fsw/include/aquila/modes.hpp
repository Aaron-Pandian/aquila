#pragma once

#include "state.hpp"

namespace aquila {

enum class FlightMode {
    STANDBY,
    TAKEOFF,
    CRUISE,
    RTL,
    FAILSAFE
};

class ModeManager {
public:
    FlightMode current_mode() const { return mode_; }

    void update(const NavState& state);

private:
    FlightMode mode_{FlightMode::STANDBY};
};

} // namespace aquila