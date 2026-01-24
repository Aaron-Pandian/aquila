#pragma once

#include <ostream>

#include "state.hpp"
#include "sensors.hpp"
#include "controller.hpp"

namespace aquila {

class Logger {
public:
    virtual ~Logger() = default;
    virtual void log(double timestamp_s,
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
                     double dbg_climb_cmd_mps) = 0;
};

class CsvLogger : public Logger {
public:
    explicit CsvLogger(std::ostream& os);

    void log(double timestamp_s,
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
             double dbg_climb_cmd_mps) override;

private:
    std::ostream& os_;
    bool header_written_{false};
    void write_header();
};

} // namespace aquila