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

    double target_altitude_m_{100.0}; // positive up in body reference frame
    double desired_speed_mps_{15.0};

    double wp_reached_thresh_m_{10.0};

    static double wrap_pi(double angle);
};

} // namespace aquila