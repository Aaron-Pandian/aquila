#pragma once

#include <string>

namespace aquila {

struct ConfigPaths {
    std::string vehicle_params;
    std::string controller_gains;
    std::string sensor_noise;
};

ConfigPaths default_config_paths();

// TODO: add parsed config structs later (VehicleParams, ControllerGains, etc.)

} // namespace aquila