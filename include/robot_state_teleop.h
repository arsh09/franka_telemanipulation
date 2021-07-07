#include <franka/robot_state.h>

namespace franka_teleop
{

struct RobotStateTeleop {
    const franka::RobotState& _state;   
};

std::ostream& operator<<(std::ostream, const franka_teleop::RobotStateTeleop& robot_state);

} // namespace name



