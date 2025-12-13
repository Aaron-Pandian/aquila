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
                     double dbg_elevator_unsat) = 0;
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
         double dbg_elevator_unsat) override;


private:
    std::ostream& os_;
    bool header_written_{false};
    void write_header();
};

} // namespace aquila