#pragma once

#include <array>

#include "state.hpp"

namespace aquila {

struct ActuatorCommands {
    double aileron{};
    double elevator{};
    double rudder{};
    double throttle{};
};

class Controller {
public:
    virtual ~Controller() = default;

    virtual ActuatorCommands compute_commands(const NavState& state) = 0;
};

// Placeholder implementation
class SimpleController : public Controller {
public:
    ActuatorCommands compute_commands(const NavState& state) override;
};

} // namespace aquila