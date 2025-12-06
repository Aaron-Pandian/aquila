#include "aquila/logger.hpp"

#include <iomanip>

namespace aquila {

// Output stream constructor
CsvLogger::CsvLogger(std::ostream& os)
    : os_(os) {}

void CsvLogger::write_header() {
    os_ << "t,"
        << "pn,pe,pd,"
        << "vn,ve,vd,"
        << "qw,qx,qy,qz,"
        << "wx,wy,wz,"
        << "imu_ax,imu_ay,imu_az,"
        << "imu_gx,imu_gy,imu_gz,"
        << "gps_pn,gps_pe,gps_pd,"
        << "gps_vn,gps_ve,gps_vd,"
        << "baro_alt,"
        << "aileron,elevator,rudder,throttle\n";
}

void CsvLogger::log(double timestamp_s,
                    const NavState& state,
                    const ImuData& imu,
                    const GpsData& gps,
                    const BaroData& baro,
                    const ActuatorCommands& cmd) {
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
        << cmd.throttle << "\n";
}

} // namespace aquila