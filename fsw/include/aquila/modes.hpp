#pragma once

#include "state.hpp"

namespace aquila {

enum class FlightMode {
    STANDBY,   // surfaces neutral, throttle idle
    CLIMB,     // climb to target altitude
    CRUISE,    // steady missions (square pattern)
    RTL,       // return to home (for now just "head to origin")
    FAILSAFE   // safe configuration on loss of health
};

struct ModeManagerConfig {
    double standby_duration_s{3.0};
    double target_cruise_alt_m{100.0};
    double climb_altitude_m{110.0};    // step up during CLIMB
    double target_speed_mps{15.0};
    double rtl_trigger_time_s{55.0};   // near end of 60s scenario
    double failsafe_min_speed_mps{3.0};
    double failsafe_min_altitude_m{20.0};
    double max_climb_time_s{10.0};     // avoid climbing forever in offline simulation, modes.hpp and modes.cpp hold changes
};

class ModeManager {
public:
    explicit ModeManager(const ModeManagerConfig& cfg = {});

    void update(double t_s, const NavState& nav);
    FlightMode mode() const { return mode_; }

    // “Guidance-level” targets for controller
    double target_altitude_m() const { return target_altitude_m_; }
    double target_speed_mps() const { return target_speed_mps_; }

private:
    ModeManagerConfig cfg_;
    FlightMode mode_{FlightMode::STANDBY};

    double target_altitude_m_{100.0};
    double target_speed_mps_{15.0};

    bool has_exited_standby_{false};

    void handle_standby(double t_s);
    void handle_climb(double t_s, const NavState& nav);
    void handle_cruise(double t_s);
    void handle_rtl(double t_s, const NavState& nav);
    void handle_failsafe();
};

// For logging mode states
inline int flight_mode_to_int(FlightMode m) {
    switch (m) {
    case FlightMode::STANDBY: return 0;
    case FlightMode::CLIMB:   return 1;
    case FlightMode::CRUISE:  return 2;
    case FlightMode::RTL:     return 3;
    case FlightMode::FAILSAFE:return 4;
    }
    return -1;
}

} // namespace aquila