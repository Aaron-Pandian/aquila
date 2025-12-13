#include "aquila/controller.hpp"

#include <cmath>

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
    // Square pattern, aligned with sim.scenarios (x: North, y: East)
    waypoints_.push_back({0.0, 0.0});
    waypoints_.push_back({200.0, 0.0});
    waypoints_.push_back({200.0, 200.0});
    waypoints_.push_back({0.0, 200.0});
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
    // Both are altitude [m], positive UP                                        
    const double alt_err = alt_cmd_m - alt_m; 
    double elevator_cmd = k_alt_p_ * alt_err;

    if (elevator_cmd > max_elevator_cmd_)  elevator_cmd = max_elevator_cmd_;
    if (elevator_cmd < -max_elevator_cmd_) elevator_cmd = -max_elevator_cmd_;

    return elevator_cmd;
}

double SimpleController::speed_control(double speed_cmd_mps,
                                       double speed_mps) const {
    const double speed_err = speed_cmd_mps - speed_mps;
    double throttle_cmd = throttle_trim_ + k_speed_p_ * speed_err;

    if (throttle_cmd > max_throttle_) throttle_cmd = max_throttle_;
    if (throttle_cmd < min_throttle_) throttle_cmd = min_throttle_;

    return throttle_cmd;
}

ActuatorCommands SimpleController::compute_commands(const NavState& state) {
    ActuatorCommands cmd{};

    if (waypoints_.empty()) {
        cmd.throttle = throttle_trim_;
        return cmd;
    }

    // 1) Guidance: what heading / altitude / speed do we want?
    auto g = compute_guidance(state);

    // 2) Extract current yaw, altitude, and speed from NavState
    const auto& q = state.quat_nb;
    const double yaw_current = wrap_pi(2.0 * std::atan2(q[3], q[0]));

    const double alt_m = -state.position_ned[2]; // NED z down, alt is positive up

    const double vx = state.velocity_ned[0];
    const double vy = state.velocity_ned[1];
    const double speed_mps = std::sqrt(vx * vx + vy * vy);

    // 3) Inner loops
    const double aileron_cmd  = heading_control(g.psi_cmd_rad, yaw_current);
    const double elevator_cmd = altitude_control(g.alt_cmd_m, alt_m);
    const double throttle_cmd = speed_control(g.speed_cmd_mps, speed_mps);

    cmd.aileron  = aileron_cmd;
    cmd.elevator = elevator_cmd;
    cmd.rudder   = 0.0;          // still unused
    cmd.throttle = throttle_cmd;

    return cmd;
}

} // namespace aquila