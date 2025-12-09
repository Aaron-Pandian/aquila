#include <chrono>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "aquila/config.hpp"
#include "aquila/state.hpp"
#include "aquila/sensors.hpp"
#include "aquila/estimator.hpp"
#include "aquila/controller.hpp"
#include "aquila/modes.hpp"
#include "aquila/logger.hpp"

namespace {

struct CsvRow {
    double t_s{};
    double x_m{};
    double y_m{};
    double z_m{};
    double psi_rad{};
    double v_mps{};
    double imu_ax{};
    double imu_ay{};
    double imu_az{};
    double imu_gx{};
    double imu_gy{};
    double imu_gz{};
    double gps_x{};
    double gps_y{};
    double gps_z{};
    double gps_vx{};
    double gps_vy{};
    double gps_vz{};
    double baro_alt{};
};

// Simple CSV parsing for the known log format.
// Returns true if a row was parsed successfully.
bool parse_log_line(const std::string& line, CsvRow& row) {
    if (line.empty()) {
        return false;
    }

    std::stringstream ss(line);
    std::string field;
    std::vector<double> values;
    values.reserve(19);

    while (std::getline(ss, field, ',')) {
        try {
            values.push_back(std::stod(field));
        } catch (const std::exception&) {
            return false;
        }
    }

    if (values.size() != 19U) {
        return false;
    }

    row.t_s      = values[0];
    row.x_m      = values[1];
    row.y_m      = values[2];
    row.z_m      = values[3];
    row.psi_rad  = values[4];
    row.v_mps    = values[5];
    row.imu_ax   = values[6];
    row.imu_ay   = values[7];
    row.imu_az   = values[8];
    row.imu_gx   = values[9];
    row.imu_gy   = values[10];
    row.imu_gz   = values[11];
    row.gps_x    = values[12];
    row.gps_y    = values[13];
    row.gps_z    = values[14];
    row.gps_vx   = values[15];
    row.gps_vy   = values[16];
    row.gps_vz   = values[17];
    row.baro_alt = values[18];

    return true;
}

} // namespace

int main() {
    using namespace aquila;

    std::cout << "Aquila FSW starting..." << std::endl;

    // Paths (relative to repo root when executed from there)
    const std::string input_log_path  = "logs/sim_sensors.csv";
    const std::string output_log_path = "logs/fsw_output.csv";

    std::ifstream input_file(input_log_path);
    if (!input_file.is_open()) {
        std::cerr << "Failed to open input log: " << input_log_path << std::endl;
        return 1;
    }

    std::ofstream output_file(output_log_path);
    if (!output_file.is_open()) {
        std::cerr << "Failed to open output log: " << output_log_path << std::endl;
        return 1;
    }

    CsvLogger logger(output_file);

    #define USE_EKF 1   // 1 = True

    #if USE_EKF
    EkfEstimator estimator;
    #else
    SimpleEstimator estimator;
    #endif
    
    SimpleController controller;
    ModeManager mode_manager;

    NavState state{};
    ImuMeasurement imu{};
    GpsMeasurement gps{};
    BaroMeasurement baro{};
    ActuatorCommands cmd{};

    // Read and discard header line
    std::string line;
    if (!std::getline(input_file, line)) {
        std::cerr << "Input log is empty: " << input_log_path << std::endl;
        return 1;
    }

    double last_t = 0.0;
    bool first_row = true;

    while (std::getline(input_file, line)) {
        CsvRow row{};
        if (!parse_log_line(line, row)) {
            std::cerr << "Skipping malformed line: " << line << std::endl;
            continue;
        }

        double dt_s = 0.0;
        if (first_row) {
            dt_s = 0.0;
            last_t = row.t_s;
            first_row = false;
        } else {
            dt_s = row.t_s - last_t;
            if (dt_s < 0.0) {
                dt_s = 0.0;
            }
            last_t = row.t_s;
        }

        // Fill sensor data structures
        imu.timestamp_s = row.t_s;
        imu.accel_mps2  = {row.imu_ax, row.imu_ay, row.imu_az};
        imu.gyro_rads   = {row.imu_gx, row.imu_gy, row.imu_gz};
        imu.valid       = true;

        gps.timestamp_s   = row.t_s;
        gps.position_ned  = {row.gps_x, row.gps_y, row.gps_z};
        gps.velocity_ned  = {row.gps_vx, row.gps_vy, row.gps_vz};
        gps.valid         = true;

        baro.timestamp_s = row.t_s;
        baro.altitude_m  = row.baro_alt;
        baro.valid       = true;

        // Estimation
        estimator.predict(dt_s, imu);
        estimator.update_gps(gps);
        estimator.update_baro(baro);
        state = estimator.get_state();

        // Mode management
        mode_manager.update(row.t_s, state);
        const FlightMode mode = mode_manager.mode();

        // Control depends on mode
        switch (mode) {
        case FlightMode::STANDBY:
            // Surfaces neutral, throttle idle
            cmd.aileron  = 0.0;
            cmd.elevator = 0.0;
            cmd.rudder   = 0.0;
            cmd.throttle = 0.0;
            break;
        case FlightMode::CRUISE:
            cmd = controller.compute_commands(state);
            break;
        case FlightMode::FAILSAFE:
            // Simple FAILSAFE placeholder: cut throttle, neutral surfaces.
            // Later you can replace this with glide-home or loiter.
            cmd.aileron  = 0.0;
            cmd.elevator = 0.0;
            cmd.rudder   = 0.0;
            cmd.throttle = 0.0;
            break;
        }

        // Log everything
        logger.log(row.t_s, state, imu, gps, baro, cmd);
    }

    std::cout << "Aquila FSW exiting. Wrote log to " << output_log_path << std::endl;
    return 0;
}