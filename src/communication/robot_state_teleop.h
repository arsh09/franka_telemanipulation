#pragma once

#include <algorithm>
#include <cstring>
#include <iterator>
#include <franka/robot_state.h>

namespace franka_teleop{

    struct RobotStateTeleop
    {
        franka::RobotState _robot_state;
    };

    std::ostream& operator<<(std::ostream& ostream, const franka_teleop::RobotStateTeleop);
    std::istream& operator>>(std::istream& ostream, const franka_teleop::RobotStateTeleop);
}

