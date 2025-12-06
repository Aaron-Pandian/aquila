#include "aquila/config.hpp"

namespace aquila {

ConfigPaths default_config_paths() {
    ConfigPaths paths;
    paths.vehicle_params   = "configs/vehicle_params.yaml";
    paths.controller_gains = "configs/controller_gains.yaml";
    paths.sensor_noise     = "configs/sensor_noise.yaml";
    return paths;
}

} // namespace aquila