#include <iostream>
#include "follower.h"

franka::Torques control_loop( franka::RobotState _fstate, franka::RobotState _lstate, franka::Duration _period, bool is_state)
{
    
    // you can control state in this loop (if is_state is true) 

    franka::Torques zero_torques{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    return zero_torques;
}


bool read_loop( franka::RobotState _fstate, franka::RobotState _lstate, franka::Duration _period, bool is_state)
{

    if (is_state)
    {
        std::cout << "Follower and leader states are available" << std::endl;
    }
    // you can read state in this loop (if is_state is true) 
    return true;
}


int main(int argc, char** argv) 
{
    if (argc != 3)
    {
        std::cerr << "Usage: "  << argv[0] << "  <server-port>  <follower-robot-hostname> \n";
        return 1;
    }

    teleop::Follower follower(argv[1], argv[2]) ;
    follower.Read(read_loop);

    std::cout << "Done" << std::endl;
    return 0;
}
