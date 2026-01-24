#include "aquila/logger.hpp"

#include <iomanip>

namespace aquila {

CsvLogger::CsvLogger(std::ostream& os)
    : os_(os) {}

void CsvLogger::write_header() {
    os_ << "t,"
        << "est_pn,est_pe,est_pd,"
        << "est_vn,est_ve,est_vd,"
        << "est_qw,est_qx,est_qy,est_qz,"
        << "est_wx,est_wy,est_wz,"
        << "imu_ax,imu_ay,imu_az,"
        << "imu_gx,imu_gy,imu_gz,"
        << "gps_pn,gps_pe,gps_pd,"
        << "gps_vn,gps_ve,gps_vd,"
        << "baro_alt,"
        << "cmd_aileron,cmd_elevator,cmd_rudder,cmd_throttle,"
        << "dbg_alt_cmd_m,dbg_alt_m,dbg_alt_err_m,"
        << "dbg_est_phi_rad,dbg_est_theta_rad,dbg_est_psi_rad,"
        << "dbg_alt_dot_mps,dbg_vd_mps,dbg_q_radps,dbg_elev_times_q,"
        << "dbg_theta_rad,dbg_theta_cmd_rad,dbg_theta_err_rad,dbg_climb_cmd_mps,"
        << "mode\n";
}

void CsvLogger::log(double timestamp_s,
                    const NavState& state,
                    const ImuMeasurement& imu,
                    const GpsMeasurement& gps,
                    const BaroMeasurement& baro,
                    const ActuatorCommands& cmd,
                    int mode_index,
                    double dbg_alt_cmd_m,
                    double dbg_alt_m,
                    double dbg_alt_err_m,
                    double dbg_est_phi_rad,
                    double dbg_est_theta_rad,
                    double dbg_est_psi_rad,
                    double dbg_alt_dot_mps,
                    double dbg_vd_mps,
                    double dbg_q_radps,
                    double dbg_elev_times_q,
                    double dbg_theta_rad,
                    double dbg_theta_cmd_rad,
                    double dbg_theta_err_rad,
                    double dbg_climb_cmd_mps) {

    if (!header_written_) {
        write_header();
        header_written_ = true;
    }

    os_ << std::setprecision(6) << std::fixed;

    os_ << timestamp_s << ",";

    // NavState
    os_ << state.position_ned[0] << ","
        << state.position_ned[1] << ","
        << state.position_ned[2] << ","
        << state.velocity_ned[0] << ","
        << state.velocity_ned[1] << ","
        << state.velocity_ned[2] << ","
        << state.quat_nb[0] << ","
        << state.quat_nb[1] << ","
        << state.quat_nb[2] << ","
        << state.quat_nb[3] << ","
        << state.omega_body[0] << ","
        << state.omega_body[1] << ","
        << state.omega_body[2] << ",";

    // IMU
    os_ << imu.accel_mps2[0] << ","
        << imu.accel_mps2[1] << ","
        << imu.accel_mps2[2] << ","
        << imu.gyro_rads[0] << ","
        << imu.gyro_rads[1] << ","
        << imu.gyro_rads[2] << ",";

    // GPS
    os_ << gps.position_ned[0] << ","
        << gps.position_ned[1] << ","
        << gps.position_ned[2] << ","
        << gps.velocity_ned[0] << ","
        << gps.velocity_ned[1] << ","
        << gps.velocity_ned[2] << ",";

    // Baro
    os_ << baro.altitude_m << ",";

    // Commands
    os_ << cmd.aileron << ","
        << cmd.elevator << ","
        << cmd.rudder << ","
        << cmd.throttle << ",";

    // Debug telemetry (altitude loop)
    os_ << dbg_alt_cmd_m << ","
        << dbg_alt_m << ","
        << dbg_alt_err_m << ",";

    // --- Debug telemetry (sign/frame sanity) ---
    os_ << dbg_est_phi_rad << ","
        << dbg_est_theta_rad << ","
        << dbg_est_psi_rad << ","
        << dbg_alt_dot_mps << ","
        << dbg_vd_mps << ","
        << dbg_q_radps << ","
        << dbg_elev_times_q << ",";

    // --- Debug telemetry (controller) ---
    os_ << dbg_theta_rad << ","
        << dbg_theta_cmd_rad << ","
        << dbg_theta_err_rad << ","
        << dbg_climb_cmd_mps << ",";

    // Flight Mode
    os_ << mode_index << "\n";
}

} // namespace aquila