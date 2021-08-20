/** 
 * Muhammad Arshad 
 * 26-July-2021
 * See license file
**/

#include "follower.h" 

extern "C"
{
    namespace teleop
    {

    Follower::Follower(char* server_port, char* follower_ip)
        : server(io_service, std::atoi(server_port)),
          robot(follower_ip)
    {
        InitializeRobot();
    }

    Follower::~Follower()
    {

    }


    void Follower::InitializeRobot()
    {

        try 
        {
            setDefaultBehavior(robot);
            // Set additional parameters always before the control loop, NEVER in the control loop!
            // Set collision behavior.
            robot.setCollisionBehavior(
                {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
                {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
                {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
                {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});

        } 
        catch (franka::Exception const& e) 
        {
            std::cout << e.what() << std::endl;
        }
    }


    void Follower::GoHome()
    {
        std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
        MotionGenerator motion_generator(0.5, q_goal);
        robot.control(motion_generator);
    }

    void Follower::Control( std::function<franka::Torques( 
        const franka::RobotState& _fstate,  const franka::RobotState& _lstate, 
        franka::Duration period,  bool _is_leader_state_received )> control_loop)
    {

        robot.control(
            [this, &control_loop]( const franka::RobotState& robot_state, franka::Duration period )
            -> franka::Torques
            {
                // send state the other end 
                server.DoSend(robot_state);
                _slave_state = robot_state;
                _master_state = server._master_state;
                is_state_received = server.is_state_received;
                
                // call user callback
                franka::Torques _torques = control_loop(_slave_state, _master_state, period, is_state_received);

                franka::Torques zero_torques{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
                return _torques;
            }
        );

    }

    void Follower::Read( std::function<bool( 
        const franka::RobotState& _fstate,  const franka::RobotState& _lstate, 
        franka::Duration period,  bool _is_leader_state_received )> read_loop)
    {
        robot.read(
            [this, &read_loop]( const franka::RobotState& robot_state)
            {
                // send state the other end 
                server.DoSend(robot_state);
                _slave_state = robot_state;
                _master_state = server._master_state;
                franka::Duration _duration(0);
                is_state_received = server.is_state_received;
                // call user callback
                bool read_state = read_loop(_slave_state, _master_state, _duration, is_state_received);
                return read_state;
            }
        );
    }

    }

}

// extern "C"
// {
//     // Check the constructor initialization
//     Follower* Follower_new(char* server_port, char* follower_ip)    {retrun new Follower() ;}

//     void Follower_InitializeRobot(Follower *follower)   {return follower-> InitializeRobot();}

//     void Follower_GoHome(Follower *follower)    {return follower->GoHome();}

//     void Follower_control(Follower *follower, std::function<franka::Torques( const franka::RobotState& _fstate,  const franka::RobotState& _lstate, 
//     franka::Duration period,  bool _is_leader_state_received )> control_loop)   {return follower->Control();}   

//     void Follower_read(Follower * follower, std::function<bool( const franka::RobotState& _fstate,  const franka::RobotState& _lstate, 
//     franka::Duration period,  bool _is_leader_state_received )> read_loop)    {return follower->Read();}

//     void Follower_delete(Follower *follower){
//         if (follower){
//             delete follower;
//             follower = nullptr;
//         }
//     }
// }
