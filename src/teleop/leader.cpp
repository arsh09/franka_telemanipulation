#include "leader.h" 

namespace teleop
{

Leader::Leader(char* server_port, char* server_ip, char* leader_ip)
    : client(io_service, server_port, server_ip),
      robot(leader_ip)
{
    InitializeRobot();
}

Leader::~Leader()
{

}


void Leader::InitializeRobot()
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

void Leader::Control( std::function<franka::Torques( 
    const franka::RobotState& _fstate,  const franka::RobotState& _lstate, 
    franka::Duration period,  bool _is_leader_state_received )> control_loop)
{

    robot.control(
        [this, &control_loop]( const franka::RobotState& robot_state, franka::Duration period )
        -> franka::Torques
        {
            // send state the other end 
            client.DoSend(robot_state);
            _master_state = robot_state;
            _slave_state = client._slave_state;
            is_state_received = client.is_state_received;
            // call user callback
            franka::Torques _torques = control_loop(_slave_state, _master_state, period, is_state_received);

            franka::Torques zero_torques{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
            return zero_torques;
        }
    );

}

void Leader::Read( std::function<bool( 
    const franka::RobotState& _fstate,  const franka::RobotState& _lstate, 
    franka::Duration period,  bool _is_leader_state_received )> read_loop)
{
    robot.read(
        [this, &read_loop]( const franka::RobotState& robot_state)
        {
            // send state the other end 
            // send state the other end 
            client.DoSend(robot_state);
            _master_state = robot_state;
            _slave_state = client._slave_state;
            is_state_received = client.is_state_received;
            franka::Duration _duration(0);

            // call user callback
            bool read_state = read_loop(_slave_state, _master_state, _duration, is_state_received);
            return read_state;
        }
    );
}

}