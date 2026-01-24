#include "aquila/controller.hpp"

#include <cmath>
#include <algorithm>

namespace aquila {

namespace {

constexpr double PI = 3.14159265358979323846;

double distance_2d(double x1, double y1, double x2, double y2) {
    const double dx = x2 - x1;
    const double dy = y2 - y1;
    return std::sqrt(dx * dx + dy * dy);
}

double heading_to_waypoint(double x, double y, double wp_x, double wp_y) {
    const double dx = wp_x - x;
    const double dy = wp_y - y;
    return std::atan2(dy, dx);
}

} // namespace

double SimpleController::wrap_pi(double angle) {
    while (angle > PI) {
        angle -= 2.0 * PI;
    }
    while (angle < -PI) {
        angle += 2.0 * PI;
    }
    return angle;
}

SimpleController::SimpleController() {
  // Waypoints
  waypoints_.clear();
  waypoints_.push_back({0.0, 0.0});
  waypoints_.push_back({1000.0, 0.0}); // 10 km ahead -> essentially constant heading

  // ===== REQUIRED TRIM =====
  throttle_trim_ = 0.4942496918512492; // standby throttle
  elevator_trim_ = -0.0793214; // standby elevator 
  desired_speed_mps_ = 15.0;
  target_altitude_m_ = 100.0;
}

SimpleController::GuidanceCommand
SimpleController::compute_guidance(const NavState& state) {
    GuidanceCommand g{};

    const double x = state.position_ned[0];
    const double y = state.position_ned[1];

    // Waypoint selection
    if (!waypoints_.empty()) {
        const auto& current_wp = waypoints_[wp_index_];
        if (distance_2d(x, y, current_wp.x_m, current_wp.y_m) < wp_reached_thresh_m_) {
            wp_index_ = (wp_index_ + 1U) % waypoints_.size();
        }
        const auto& wp = waypoints_[wp_index_];
        g.psi_cmd_rad = heading_to_waypoint(x, y, wp.x_m, wp.y_m);
    } else {
        g.psi_cmd_rad = 0.0;
    }

    // For now, altitude and speed setpoints are constant; later this can be made segment-dependent or mission-dependent.
    g.alt_cmd_m = target_altitude_m_;
    g.speed_cmd_mps = desired_speed_mps_;

    return g;
}

double SimpleController::heading_control(double psi_cmd_rad, double psi_rad) const {
    double heading_err = wrap_pi(psi_cmd_rad - psi_rad);
    double aileron_cmd = k_heading_p_ * heading_err;

    if (aileron_cmd > max_aileron_cmd_)  aileron_cmd = max_aileron_cmd_;
    if (aileron_cmd < -max_aileron_cmd_) aileron_cmd = -max_aileron_cmd_;

    return aileron_cmd;
}

// Tier 1: Outer Loop - Converts Altitude Error to Vertical Speed Command
double SimpleController::run_altitude_loop(double alt_cmd_m, double alt_m, double dt) {
    double alt_err = alt_cmd_m - alt_m;
    
    // Only integrate if we are NOT saturated, or if the error helps unwind the saturation
    bool saturation_high = (max_elevator_cmd_ - std::abs(elevator_trim_) < 0.01); // simplistic check
    // Ideally, pass in the actual saturation flag from the inner loop
    
    // For now, simpler anti-windup: stricter clamp
    alt_int_ += alt_err * dt;
    alt_int_ = std::clamp(alt_int_, -3.0, 3.0); // Reduced from 10.0 to prevent deep saturation

    double vz_cmd = (k_alt_to_vz_ * alt_err) + (k_alt_i_ * alt_int_);
    return std::clamp(vz_cmd, -max_climb_mps_, max_climb_mps_);
}

// Tier 2 & 3: Middle/Inner Loop - Converts Pitch Command to Elevator
double SimpleController::run_pitch_loop(double theta_cmd_rad, double theta_rad, double q_radps) {
    double theta_err = theta_cmd_rad - theta_rad;

    // Standard Aerospace Convention:
    // To PITCH DOWN (correct +err): Need POSITIVE elevator (Negative Moment)
    // To DAMP PITCH UP (+q): Need POSITIVE elevator (Negative Moment)
    double elevator_cmd = -(k_theta_p_ * theta_err) + (k_pitch_rate_d_ * q_radps);
    
    elevator_cmd += elevator_trim_;
    return std::clamp(elevator_cmd, -max_elevator_cmd_, max_elevator_cmd_);
}

// Energy Loop: Throttle maintains airspeed
double SimpleController::run_speed_loop(double speed_cmd_mps, double speed_mps) {
    double speed_err = speed_cmd_mps - speed_mps;
    double throttle_cmd = throttle_trim_ + (k_speed_p_ * speed_err);
    return std::clamp(throttle_cmd, min_throttle_, max_throttle_);
}

ActuatorCommands SimpleController::compute_commands(const NavState& state, ControllerDebug* dbg) {
    ActuatorCommands cmd{};
    if (waypoints_.empty()) return cmd;

    // 1. Guidance Logic
    auto g = compute_guidance(state);

    // 2. Extract State
    double alt_m = -state.position_ned[2];
    double vn = state.velocity_ned[0], ve = state.velocity_ned[1], vd = state.velocity_ned[2];
    double speed_mps = std::sqrt(vn*vn + ve*ve + vd*vd);
    double q_radps = state.omega_body[1];
    
    // Pitch Extraction
    double qw = state.quat_nb[0], qy = state.quat_nb[2];
    double qx = state.quat_nb[1], qz = state.quat_nb[3];
    double theta = std::asin(std::clamp(2.0 * (qw*qy - qz*qx), -1.0, 1.0));

    // 3. Cascade Execution
    double dt = 0.05; // Fixed time step for stability
    
    double vz_cmd = run_altitude_loop(g.alt_cmd_m, alt_m, dt);
    
    // Vertical Speed Error -> theta_cmd
    double vz_err = vz_cmd - (-vd); // -vd is vertical speed (+up)
    double theta_cmd = vz_err * k_vz_to_theta_;

    // --- UPDATED STALL PROTECTION ---
    // Instead of forcing a hard nose-down (-0.10), which causes a dive,
    // we simply restrict the "climb" capability.
    
    if (speed_mps < 9.0) {
        // Critical speed: Gentle nose down to regain energy
        theta_cmd = -0.05; 
    } 
    else if (speed_mps < 13.0) {
        // Low speed: Cap pitch at 0 (Level Flight). 
        // This allows the engine to accelerate the plane without losing altitude.
        theta_cmd = std::min(theta_cmd, 0.0); 
    }
    // (If speed > 13.0, allow full climb authority)

    theta_cmd = std::clamp(theta_cmd, -theta_max_rad_, theta_max_rad_);

    // 4. Final Actuation
    cmd.elevator = run_pitch_loop(theta_cmd, theta, q_radps);
    cmd.throttle = run_speed_loop(g.speed_cmd_mps, speed_mps);
    
    // Bank-to-Turn (Aileron)
    double psi = std::atan2(2.0*(qw*qz+qx*qy), 1.0-2.0*(qy*qy+qz*qz));
    cmd.aileron = heading_control(g.psi_cmd_rad, wrap_pi(psi));
    cmd.rudder = 0.0;

    if (dbg) {
        dbg->theta_rad = theta;
        dbg->theta_cmd_rad = theta_cmd;
        dbg->theta_err_rad = theta_cmd - theta;
        dbg->climb_cmd_mps = vz_cmd;
        dbg->alt_err_m = g.alt_cmd_m - alt_m;
        dbg->speed_mps = speed_mps;
        dbg->throttle_cmd = cmd.throttle;
        dbg->q_radps = q_radps;
    }
    return cmd;
}

ActuatorCommands SimpleController::compute_commands(const NavState& state) {
    return compute_commands(state, nullptr);
}

} // namespace aquila