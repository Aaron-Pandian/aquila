#include "aquila/modes.hpp"

#include <cmath>

namespace aquila {

ModeManager::ModeManager() : mode_(FlightMode::STANDBY) {}

// For now, we define a very simple policy:
// - Start in STANDBY
// - After a short time and some forward speed, go to CRUISE
// - FAILSAFE hook is present but not used yet
void ModeManager::update(double t_s, const NavState& state) {
    switch (mode_) {
    case FlightMode::STANDBY:
        if (should_enter_cruise(t_s, state)) {
            mode_ = FlightMode::CRUISE;
        }
        break;
    case FlightMode::CRUISE:
        if (should_enter_failsafe(t_s, state)) {
            mode_ = FlightMode::FAILSAFE;
        }
        break;
    case FlightMode::FAILSAFE:
        // Stay in FAILSAFE for now; future versions may support recovery.
        break;
    }
}

bool ModeManager::should_enter_cruise(double t_s, const NavState& state) const {
    // Example policy:
    // - Wait a few seconds (simulate takeoff/arming)
    // - Require some forward speed
    if (t_s < 2.0) {
        return false;
    }

    const double vx = state.velocity_ned[0];
    const double vy = state.velocity_ned[1];
    const double speed = std::sqrt(vx * vx + vy * vy);

    return speed > 5.0; // m/s threshold for "flying"
}

bool ModeManager::should_enter_failsafe(double /*t_s*/, const NavState& state) const {
    // Placeholder for future logic (e.g., low altitude, loss of nav, etc.)
    // For now, always return false.
    (void)state;
    return false;
}

} // namespace aquila