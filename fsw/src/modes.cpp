#include "aquila/modes.hpp"

#include <cmath>

namespace aquila {

ModeManager::ModeManager(const ModeManagerConfig& cfg)
    : cfg_(cfg),
      mode_(FlightMode::STANDBY),
      target_altitude_m_(cfg.target_cruise_alt_m),
      target_speed_mps_(cfg.target_speed_mps),
      has_exited_standby_(false) {}

// Main mode update entry point. Call once per FSW step.
void ModeManager::update(double t_s, const NavState& nav) {
    // Basic health check: if things look bad, drop into FAILSAFE.
    const double vx = nav.velocity_ned[0];
    const double vy = nav.velocity_ned[1];
    const double speed = std::sqrt(vx * vx + vy * vy);
    const double alt_m = -nav.position_ned[2]; // NED z (down) -> altitude up

    const bool low_speed   = (speed < cfg_.failsafe_min_speed_mps);
    const bool low_alt     = (alt_m < cfg_.failsafe_min_altitude_m);
    const bool enable_failsafe = false; // flip to true if you want to exercise FAILSAFE

    if (enable_failsafe && (low_speed || low_alt)) {
        mode_ = FlightMode::FAILSAFE;
        handle_failsafe();
        return;
    }

    // Normal mode progression
    switch (mode_) {
    case FlightMode::STANDBY:
        handle_standby(t_s);
        break;
    case FlightMode::CLIMB:
        handle_climb(t_s, nav);
        break;
    case FlightMode::CRUISE:
        handle_cruise(t_s);
        break;
    case FlightMode::RTL:
        handle_rtl(t_s, nav);
        break;
    case FlightMode::FAILSAFE:
        // Remain in FAILSAFE for now
        handle_failsafe();
        break;
    }
}

void ModeManager::handle_standby(double t_s) {
    // We simply wait a bit to simulate arming / takeoff.
    if (t_s >= cfg_.standby_duration_s && !has_exited_standby_) {
        // Transition to CLIMB
        mode_ = FlightMode::CLIMB;
        has_exited_standby_ = true;

        // Climb to a slightly higher altitude than cruise, then level off.
        target_altitude_m_ = cfg_.climb_altitude_m;
        target_speed_mps_  = cfg_.target_speed_mps;
    }
}

void ModeManager::handle_climb(double t_s, const NavState& nav) {
    const double alt_m = -nav.position_ned[2];

    // While in CLIMB, aim for climb_altitude_m
    target_altitude_m_ = cfg_.climb_altitude_m;
    target_speed_mps_  = cfg_.target_speed_mps;

    const double alt_reached_thresh_m = 1.0;
    const bool reached_alt =
        (alt_m >= cfg_.climb_altitude_m - alt_reached_thresh_m);

    // In offline simulation with no feedback, switch climb mode after a timeout
    const double climb_start_s = cfg_.standby_duration_s;
    const bool timed_out =
        (t_s >= climb_start_s + cfg_.max_climb_time_s);

    if (reached_alt || timed_out) {
        mode_ = FlightMode::CRUISE;
        target_altitude_m_ = cfg_.target_cruise_alt_m;
        target_speed_mps_  = cfg_.target_speed_mps;
    }
}

void ModeManager::handle_cruise(double t_s) {
    // CRUISE: hold nominal cruise altitude and speed
    target_altitude_m_ = cfg_.target_cruise_alt_m;
    target_speed_mps_  = cfg_.target_speed_mps;

    // Simple time-based trigger to go into RTL near the end of the scenario.
    if (t_s >= cfg_.rtl_trigger_time_s) {
        mode_ = FlightMode::RTL;
    }
}

void ModeManager::handle_rtl(double /*t_s*/, const NavState& nav) {
    (void)nav;
    // For now, RTL uses the same altitude/speed targets as CRUISE.
    target_altitude_m_ = cfg_.target_cruise_alt_m;
    target_speed_mps_  = cfg_.target_speed_mps;
}

void ModeManager::handle_failsafe() {
    // Minimal FAILSAFE behavior for now:
    // - Keep altitude target at current value and reduce speed target
    target_speed_mps_ = 0.0; 
}

} // namespace aquila