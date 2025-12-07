#pragma once

#include <array>
#include <vector>

#include "state.hpp"

namespace aquila {

struct ActuatorCommands {
    double aileron{};
    double elevator{};
    double rudder{};
    double throttle{};
};

class Controller {
public:
    virtual ~Controller() = default;

    virtual ActuatorCommands compute_commands(const NavState& state) = 0;
};

// Simple waypoint-following controller.
// - Holds altitude near a target value
// - Steers towards a sequence of waypoints in NED (x, y)
// - Regulates airspeed via throttle
class SimpleController : public Controller {
public:
    SimpleController();

    ActuatorCommands compute_commands(const NavState& state) override;

private:
    struct Waypoint {
        double x_m{};
        double y_m{};
    };

    std::vector<Waypoint> waypoints_;
    std::size_t wp_index_{0};

    // Mission / guidance parameters
    double wp_reached_thresh_m_{10.0};
    double target_altitude_m_{100.0}; // positive up
    double desired_speed_mps_{15.0};

    // Control gains and limits (tunable, later loadable from config)
    double k_heading_{0.4};    // heading P gain
    double max_bank_cmd_{0.7}; // max |aileron| command
    double k_alt_{0.02};       // altitude P gain
    double k_speed_{0.10};     // speed P gain

    static double wrap_pi(double angle);
};

} // namespace aquila