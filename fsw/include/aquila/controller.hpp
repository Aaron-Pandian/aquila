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

// Waypoint-following controller with simple cascaded structure:
// - Guidance computes desired heading, altitude, and speed
// - Inner P loops generate aileron, elevator, throttle commands
class SimpleController : public Controller {
public:
    SimpleController();

    ActuatorCommands compute_commands(const NavState& state) override;

    void set_target_altitude(double alt_m) { target_altitude_m_ = alt_m; }
    void set_desired_speed(double speed_mps) { desired_speed_mps_ = speed_mps; }

private:
    struct Waypoint {
        double x_m{};
        double y_m{};
    };

    struct GuidanceCommand {
        double psi_cmd_rad;     // desired heading
        double alt_cmd_m;       // desired altitude (positive up)
        double speed_cmd_mps;   // desired airspeed
    };

    std::vector<Waypoint> waypoints_;
    std::size_t wp_index_{0};

    // Guidance parameters
    double wp_reached_thresh_m_{10.0};
    double target_altitude_m_{100.0};
    double desired_speed_mps_{15.0};

    // Inner-loop gains and limits
    double k_heading_p_{0.4};
    double max_aileron_cmd_{0.7};

    double k_alt_p_{0.02};
    double max_elevator_cmd_{1.0};

    double k_speed_p_{0.10};
    double throttle_trim_{0.6};
    double min_throttle_{0.0};
    double max_throttle_{1.0};

    // --- Helpers ---

    static double wrap_pi(double angle);

    GuidanceCommand compute_guidance(const NavState& state);

    double heading_control(double psi_cmd_rad, double psi_rad) const;
    double altitude_control(double alt_cmd_m, double alt_m) const;
    double speed_control(double speed_cmd_mps, double speed_mps) const;
};

} // namespace aquila