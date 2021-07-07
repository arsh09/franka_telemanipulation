
#include "robot_state_teleop.h"

#include <algorithm>
#include <cstring>
#include <iterator>

namespace franka_teleop{

    std::ostream& operator<<(std::ostream& ostream, const franka_teleop::RobotStateTeleop& robot_state)
    {
        ostream << robot_state._state;
        return ostream;
    }

}

