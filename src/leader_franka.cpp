/** 
 * Muhammad Arshad 
 * 26-July-2021
 * See license file
**/

#include <iostream>
#include "leader.h"

franka::Torques control_loop( franka::RobotState _fstate, franka::RobotState _lstate, franka::Duration _period, bool is_state)
{
    
    // you can control the leader in this loop (if is_state is true) 

    franka::Torques zero_torques{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    return zero_torques;
}


bool read_loop( franka::RobotState _fstate, franka::RobotState _lstate, franka::Duration _period, bool is_state)
{
    
    if (is_state)
    {
        std::cout << "Follower states are available" << std::endl;
    }
    // you can read state in this loop (if is_state is true) 
    return true;
}


int main(int argc, char** argv) 
{
    if (argc != 4)
    {
        std::cerr << "Usage: "  << argv[0] << "  <server-port>  <server-pc-host> <leader-robot-hostname> \n";
        return 1;
    }

    teleop::Leader leader(argv[1], argv[2], argv[3]) ;
    std::cout << "WARNING: The robot will go to ready pose! "
        << "Please make sure to have the user stop button at hand!" << std::endl
        << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    
    leader.GoHome();

    std::cout << "WARNING: The robot will go in zero torque mode (zero gravity)! "
        << "Please make sure to have the user stop button at hand!" << std::endl
        << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    
    // leader.Read(read_loop);
    leader.Control(control_loop);

    std::cout << "Done" << std::endl;
    return 0;
}
