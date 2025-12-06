#include <iostream>
#include <chrono>

#include "aquila/config.hpp"
#include "aquila/state.hpp"
#include "aquila/sensors.hpp"
#include "aquila/estimator.hpp"
#include "aquila/controller.hpp"
#include "aquila/modes.hpp"
#include "aquila/logger.hpp"

int main() {
    using namespace aquila;

    std::cout << "Aquila FSW starting..." << std::endl;

    ConfigPaths cfg = default_config_paths();
    (void)cfg; // suppress unused for now

    SimpleEstimator estimator;
    SimpleController controller;
    ModeManager mode_manager;

    NavState state{};
    ImuData imu{};
    GpsData gps{};
    BaroData baro{};
    ActuatorCommands cmd{};

    CsvLogger logger(std::cout);

    // Very simple demo loop for now
    const double dt_s = 0.01;
    double t = 0.0;
    const double sim_duration_s = 0.1; // keep tiny for now

    while (t < sim_duration_s) {
        estimator.predict(dt_s, imu);
        estimator.update_gps(gps);
        estimator.update_baro(baro);
        state = estimator.get_state();

        mode_manager.update(state);
        cmd = controller.compute_commands(state);

        logger.log(t, state, imu, gps, baro, cmd);

        t += dt_s;
    }

    std::cout << "Aquila FSW exiting." << std::endl;
    return 0;
}