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
  waypoints_.push_back({10000.0, 0.0}); // 10 km ahead -> essentially constant heading

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

double SimpleController::altitude_control(double alt_cmd_m, double alt_m) const {
    const double alt_err = alt_cmd_m - alt_m;

    // P on altitude error
    double elevator_cmd = k_alt_p_ * alt_err;

    // Add trim to elevator command before clamping, clamp and damping added in main_loop.cpp
    elevator_cmd += elevator_trim_;

    if (elevator_cmd >  max_elevator_cmd_) elevator_cmd =  max_elevator_cmd_;
    if (elevator_cmd < -max_elevator_cmd_) elevator_cmd = -max_elevator_cmd_;

    return elevator_cmd;
}

double SimpleController::speed_control(double speed_cmd_mps, double speed_mps) const {
    const double speed_err = speed_cmd_mps - speed_mps;
    double throttle_cmd = throttle_trim_ + k_speed_p_ * speed_err;

    if (throttle_cmd > max_throttle_) throttle_cmd = max_throttle_;
    if (throttle_cmd < min_throttle_) throttle_cmd = min_throttle_;

    return throttle_cmd;
}

ActuatorCommands SimpleController::compute_commands(const NavState& state) {
    return compute_commands(state, nullptr);
}

ActuatorCommands SimpleController::compute_commands(const NavState& state, ControllerDebug* dbg) {
    ActuatorCommands cmd{};

    if (waypoints_.empty()) {
        cmd.throttle = throttle_trim_;
        return cmd;
    }

    // 1) Guidance
    auto g = compute_guidance(state);

    // 2) Extract attitude
    const auto& q = state.quat_nb;    // q = [qw,qx,qy,qz]
    const double qw = q[0], qx = q[1], qy = q[2], qz = q[3];

    const double siny_cosp = 2.0 * (qw*qz + qx*qy);
    const double cosy_cosp = 1.0 - 2.0 * (qy*qy + qz*qz);
    const double yaw_current = wrap_pi(std::atan2(siny_cosp, cosy_cosp));

    // pitch theta from quat (NED, standard aerospace)
    double sinp = 2.0 * (qw*qy - qz*qx);
    sinp = std::clamp(sinp, -1.0, 1.0);
    const double theta = std::asin(sinp);

    // Altitude
    const double alt_m = -state.position_ned[2]; // NED z down, alt positive up

    // Velocity (use HORIZONTAL speed for control + stall logic)
    const double vn = state.velocity_ned[0];
    const double ve = state.velocity_ned[1];
    const double vd = state.velocity_ned[2];
    const double speed_mps = std::sqrt(vn*vn + ve*ve + vd*vd);

    // Body pitch rate q
    const double q_radps = state.omega_body[1];

    // 3) Heading control -> aileron
    const double aileron_cmd = heading_control(g.psi_cmd_rad, yaw_current);

    // 4) Altitude -> climb -> theta_cmd
    const double alt_err = g.alt_cmd_m - alt_m;
    double climb_cmd = k_alt_to_vz_ * alt_err;
    climb_cmd = std::clamp(climb_cmd, -max_climb_mps_, max_climb_mps_);

    double theta_cmd = k_vz_to_theta_ * climb_cmd;
    theta_cmd = std::clamp(theta_cmd, -theta_max_rad_, theta_max_rad_);

    // 5) Speed -> throttle
    double v_horiz = sqrt(vn*vn + ve*ve);
    const double throttle_cmd = speed_control(g.speed_cmd_mps, v_horiz);

    // Stall/energy protection
    if (speed_mps < 12.0) {
        theta_cmd = std::min(theta_cmd, 0.10); // cap nose-up demand
    }
    if (speed_mps < 10.0) {
        theta_cmd = std::min(theta_cmd, 0.03); // nearly level
        // optionally: cmd.throttle = 1.0;
    }

    // 6) Pitch loop -> elevator
    const double theta_err = theta_cmd - theta;
    double elevator_cmd = elevator_trim_ + k_theta_p_ * theta_err - k_pitch_rate_d_ * q_radps; // pitch loop: NEGATIVE feedback on q
    elevator_cmd = std::clamp(elevator_cmd, -max_elevator_cmd_, max_elevator_cmd_);

    // Populate outputs
    cmd.aileron  = aileron_cmd;
    cmd.elevator = elevator_cmd;
    cmd.rudder   = 0.0;
    cmd.throttle = throttle_cmd;

    // Populate debug
    if (dbg) {
        dbg->theta_rad      = theta;
        dbg->theta_cmd_rad  = theta_cmd;
        dbg->theta_err_rad  = theta_err;
        dbg->alt_err_m      = alt_err;
        dbg->climb_cmd_mps  = climb_cmd;
        dbg->speed_mps      = speed_mps;
        dbg->throttle_cmd   = throttle_cmd;
        dbg->q_radps        = q_radps;
    }

    return cmd;
}

} // namespace aquila