#include "aquila/controller.hpp"

namespace aquila {

ActuatorCommands SimpleController::compute_commands(const NavState& /*state*/) {
    ActuatorCommands cmd{};
    // TODO: implement guidance & control logic
    return cmd;
}

} // namespace aquila