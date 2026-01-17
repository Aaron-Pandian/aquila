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

struct ControllerDebug {
    double theta_rad{NAN};
    double theta_cmd_rad{NAN};
    double theta_err_rad{NAN};

    double alt_err_m{NAN};
    double climb_cmd_mps{NAN};

    double speed_mps{NAN};
    double throttle_cmd{NAN};
    double q_radps{NAN};
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
    ActuatorCommands compute_commands(const NavState& state, ControllerDebug* dbg);

    void set_target_altitude(double alt_m) { target_altitude_m_ = alt_m; }
    void set_desired_speed(double speed_mps) { desired_speed_mps_ = speed_mps; }
    void set_elevator_trim(double elev_trim) { elevator_trim_ = elev_trim; }

    double elevator_trim() const { return elevator_trim_; }
    double throttle_trim() const { return throttle_trim_; }
    
    double k_alt_p() const { return k_alt_p_; }
    double max_elevator_cmd() const { return max_elevator_cmd_; }

    double elevator_rate() const { return max_elevator_change_per_sec_; }
    double throttle_rate() const { return max_throttle_change_per_sec_; }

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
    double k_heading_p_{1.2};
    double max_aileron_cmd_{0.6};

    double k_alt_p_{0.02};
    double k_pitch_rate_d_{0.12};   // elevator damping using body pitch rate q [rad/s]
    double max_elevator_cmd_{0.34906585}; // 20 degrees in radians
    double k_alt_to_vz_ {0.25};     // m/s per m of alt error
    double max_climb_mps_{1.5};     
    // double k_vz_p_ {-0.05};    // elevator per climb-rate error
    double k_vz_to_theta_{0.05};  // rad of pitch per (m/s) climb rate

    double k_speed_p_{0.05};
    double k_theta_p_{0.2};
    double throttle_trim_{0.6};
    double min_throttle_{0.1};
    double max_throttle_{1.0};
    double theta_max_rad_{0.15}; 

    double elevator_trim_{0.0};

    // To be tunned
    double max_elevator_change_per_sec_{1.5};   // rad/s
    double max_throttle_change_per_sec_{1.0};   // 1/s (full range per second)

    // --- Helpers ---

    static double wrap_pi(double angle);

    GuidanceCommand compute_guidance(const NavState& state);

    double heading_control(double psi_cmd_rad, double psi_rad) const;
    double altitude_control(double alt_cmd_m, double alt_m) const;
    double speed_control(double speed_cmd_mps, double speed_mps) const;
};

} // namespace aquila