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
    // Align waypoints with the Python scenario (x: North, y: East)
    waypoints_.push_back({0.0, 0.0});
    waypoints_.push_back({200.0, 0.0});
    waypoints_.push_back({200.0, 200.0});
    waypoints_.push_back({0.0, 200.0});
}

ActuatorCommands SimpleController::compute_commands(const NavState& state) {
    ActuatorCommands cmd{};

    if (waypoints_.empty()) {
        // Safe default: neutral controls, moderate throttle
        cmd.throttle = 0.6;
        return cmd;
    }

    const double x = state.position_ned[0];
    const double y = state.position_ned[1];
    const double z = state.position_ned[2];

    // Altitude (positive up) from NED z
    const double alt_m = -z;

    // ---- Waypoint Logic ----
    const auto& current_wp = waypoints_[wp_index_];
    const double dist_to_wp = distance_2d(x, y, current_wp.x_m, current_wp.y_m);
    if (dist_to_wp < wp_reached_thresh_m_) {
        wp_index_ = (wp_index_ + 1U) % waypoints_.size();
    }
    const auto& wp = waypoints_[wp_index_];

    // ---- Heading / Aileron Control ----

    const double psi_des = heading_to_waypoint(x, y, wp.x_m, wp.y_m);

    // Extract current yaw from quaternion (assumes yaw-dominant attitude)
    const auto& q = state.quat_nb;
    const double yaw_current = wrap_pi(2.0 * std::atan2(q[3], q[0]));

    double heading_err = wrap_pi(psi_des - yaw_current);

    // P controller on heading error
    double aileron_cmd = k_heading_ * heading_err;

    // Saturate to keep turns within reasonable limits
    if (aileron_cmd > max_bank_cmd_)  aileron_cmd = max_bank_cmd_;
    if (aileron_cmd < -max_bank_cmd_) aileron_cmd = -max_bank_cmd_;

    // ---- Altitude / Elevator Control ----

    const double alt_err = target_altitude_m_ - alt_m;
    double elevator_cmd = k_alt_ * alt_err;

    if (elevator_cmd > 1.0)  elevator_cmd = 1.0;
    if (elevator_cmd < -1.0) elevator_cmd = -1.0;

    // ---- Speed / Throttle Control ----

    const double vx = state.velocity_ned[0];
    const double vy = state.velocity_ned[1];
    const double speed = std::sqrt(vx * vx + vy * vy);

    const double speed_err = desired_speed_mps_ - speed;
    double throttle_cmd = 0.6 + k_speed_ * speed_err;

    if (throttle_cmd > 1.0) throttle_cmd = 1.0;
    if (throttle_cmd < 0.0) throttle_cmd = 0.0;

    // ---- Rudder ----
    double rudder_cmd = 0.0;

    cmd.aileron  = aileron_cmd;
    cmd.elevator = elevator_cmd;
    cmd.rudder   = rudder_cmd;
    cmd.throttle = throttle_cmd;

    return cmd;
}

} // namespace aquila