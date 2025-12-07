#pragma once

#include "state.hpp"

namespace aquila {

enum class FlightMode {
    STANDBY = 0,
    CRUISE  = 1,
    FAILSAFE = 2,
};

class ModeManager {
public:
    ModeManager();

    // Update mode based on current time and navigation state.
    // This is intentionally simple for v1 and can be extended later.
    void update(double t_s, const NavState& state);

    FlightMode mode() const { return mode_; }

private:
    FlightMode mode_{FlightMode::STANDBY};

    // Helper hooks for future logic
    bool should_enter_cruise(double t_s, const NavState& state) const;
    bool should_enter_failsafe(double t_s, const NavState& state) const;
};

} // namespace aquila