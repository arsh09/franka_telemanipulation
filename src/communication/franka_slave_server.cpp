                
/** 
 * UDP Server -> Slave Robot
 * UDP Client -> Master Robot
 * 
 * 
 * Muhammad Arshad 
 * 07-July-2021
**/

#include <cstdlib>
#include <cstring>
#include <iostream>
#include <stdlib.h>   


// #include <Eigen/Dense>
#include <eigen3/Eigen/Dense>

#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/duration.h>
#include <franka/model.h>
#include "examples_common.h"

#include "udp_messages.h"

#include <boost/asio.hpp>
#include <boost/thread.hpp>

using boost::asio::ip::udp;

enum { max_length = 8192 };

class server 
{
public:

    server(boost::asio::io_service& io_service, short port, char* slave_ip)
       : socket_( io_service, udp::endpoint(udp::v4(), port))
    {
        memset( receive_data_, 0, sizeof(receive_data_) );
        std::cout << "Tis is a UDP server" << std::endl;
        do_receive();
        boost::thread(boost::bind(&boost::asio::io_service::run, &io_service));
        initialize_robot(slave_ip);
    }

    bool initialize_robot(char* slave_ip)
    {
        try 
        {
            franka::Robot robot(slave_ip);
            // setDefaultBehavior(robot);
            // setup_initial_pose(robot);
            // setup_position_control(robot);

            setup_state_read_loop(robot);
        } 
        catch (franka::Exception const& e) 
        {
            std::cout << e.what() << std::endl;
            return false;
        }
    }

    void setup_initial_pose(franka::Robot& robot)
    {
        std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
        MotionGenerator motion_generator(0.5, q_goal);
        std::cout << "WARNING: This example will move the robot to a pre-configured pose! "
                << "Please make sure to have the user stop button at hand!" << std::endl
                << "Press Enter to continue..." << std::endl;
        std::cin.ignore();
        robot.control(motion_generator);
        std::cout << "Finished moving to initial joint configuration." << std::endl;   
    }

    void setup_state_read_loop(franka::Robot& robot)
    {
        robot.read(  [this] (const franka::RobotState& robot_state) 
        {   
            _slave_state = robot_state;
            return true;                
        });
    }

    void setup_position_control(franka::Robot& robot)
    {
        // Set additional parameters always before the control loop, NEVER in the control loop!
        // Set collision behavior.
        robot.setCollisionBehavior(
            {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
            {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
            {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
            {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});

        std::array<double, 7> initial_position;
        double time = 0.0;

        robot.control([this, &initial_position, &time](const franka::RobotState& robot_state, franka::Duration period) -> franka::JointPositions 
        {
            time += period.toSec();
            if (time == 0.0 || !is_receive_state) 
            { 
                initial_position = robot_state.q_d;
            }
            else
            {
                initial_position = _master_state.q;
            }

            franka::JointPositions output = {{
                initial_position[0], initial_position[1], initial_position[2], 
                initial_position[3], initial_position[4],  initial_position[5],
                initial_position[6] 
            }};

            if (time >= 50.0) {
                std::cout << std::endl << "Finished motion, shutting down example" << std::endl;
                return franka::MotionFinished(output);
            }
            return output;
        });
    }

    void do_send(std::stringstream& _stream)
    {
        socket_.async_send_to( boost::asio::buffer(_stream.str()), slave_endpoint, [this](boost::system::error_code ec, std::size_t bytes_sent)
        {   
            if (bytes_sent > 0)
            {               
                // std::cout << "Sent from slave to master:  " << (int) bytes_sent << std::endl;
            }
            do_receive();                
        });
    }

    void do_receive() 
    {
        socket_.async_receive_from(
        boost::asio::buffer(receive_data_, max_length), slave_endpoint,
        [this](boost::system::error_code ec, std::size_t bytes_recvd)
        {
            if (bytes_recvd > 50)
            {
                is_receive_state = true;
                memset( receive_data_, 0, sizeof(receive_data_) );
                std::stringstream ss;
                ss << _slave_state;
                do_send(ss);
            }            
            else
            {
                do_receive();
            }
        });
    }

    void print_array(std::array<double, 7> &arr, std::string &name)
    {   
        std::cout << name.c_str() ;
        for ( int i = 0; i < arr.size(); i++ )
        {
            std::cout << arr[i] << " , " ;
        }
        std::cout << std::endl;
    }

    // order matters 
    void deserialize(  franka::RobotState& robot_state , teleop::message<CustomType>& msg)
    {
        // msg    >> robot_state.time.toMSec();
        // msg    >> robot_state.robot_mode;
        msg    >> robot_state.control_command_success_rate;
        // msg    >> robot_state.last_motion_errors;
        // msg    >> robot_state.current_errors;
        msg    >> robot_state.dtheta;
        msg    >> robot_state.theta;
        msg    >> robot_state.O_ddP_EE_c ;
        msg    >> robot_state.O_dP_EE_c;
        msg    >> robot_state.O_T_EE_c;
        msg    >> robot_state.O_dP_EE_d;
        msg    >> robot_state.K_F_ext_hat_K;
        msg    >> robot_state.O_F_ext_hat_K;
        msg    >> robot_state.tau_ext_hat_filtered;
        msg    >> robot_state.cartesian_collision;
        msg    >> robot_state.joint_collision;
        msg    >> robot_state.cartesian_contact;
        msg    >> robot_state.joint_contact;
        msg    >> robot_state.ddq_d;
        msg    >> robot_state.dq_d;
        msg    >> robot_state.q_d ;
        msg    >> robot_state.dq;
        msg    >> robot_state.q ;
        msg    >> robot_state.dtau_J; 
        msg    >> robot_state.tau_J_d;   
        msg    >> robot_state.tau_J;
        msg    >> robot_state.ddelbow_c; 
        msg    >> robot_state.delbow_c ;
        msg    >> robot_state.elbow_c;
        msg    >> robot_state.elbow_d ;
        msg    >> robot_state.elbow ;
        msg    >> robot_state.I_total; 
        msg    >> robot_state.F_x_Ctotal ; 
        msg    >> robot_state.m_total;
        msg    >> robot_state.I_load;
        msg    >> robot_state.F_x_Cload; 
        msg    >> robot_state.m_load ;
        msg    >> robot_state.I_ee ;
        msg    >> robot_state.F_x_Cee;
        msg    >> robot_state.m_ee  ;
        msg    >> robot_state.EE_T_K ;
        msg    >> robot_state.F_T_EE  ;
        msg    >> robot_state.NE_T_EE;
        msg    >> robot_state.F_T_NE ;
        msg    >> robot_state.O_T_EE_d;
        msg    >> robot_state.O_T_EE  ;
    }

    // order matters 
    void serialize( const franka::RobotState& robot_state , teleop::message<CustomType>& msg)
    {
    
        msg    << robot_state.O_T_EE  ;
        msg    << robot_state.O_T_EE_d;
        msg    << robot_state.F_T_NE ;
        msg    << robot_state.NE_T_EE;
        msg    << robot_state.F_T_EE  ;
        msg    << robot_state.EE_T_K;
        msg    << robot_state.m_ee   ;
        msg    << robot_state.F_x_Cee;
        msg    << robot_state.I_ee ;
        msg    << robot_state.m_load;
        msg    << robot_state.F_x_Cload;  
        msg    << robot_state.I_load;
        msg    << robot_state.m_total;
        msg    << robot_state.F_x_Ctotal;
        msg    << robot_state.I_total   ;
        msg    << robot_state.elbow;
        msg    << robot_state.elbow_d;  
        msg    << robot_state.elbow_c;
        msg    << robot_state.delbow_c;
        msg    << robot_state.ddelbow_c;   
        msg    << robot_state.tau_J;
        msg    << robot_state.tau_J_d;  
        msg    << robot_state.dtau_J;
        msg    << robot_state.q  ;
        msg    << robot_state.dq;
        msg    << robot_state.q_d; 
        msg    << robot_state.dq_d;
        msg    << robot_state.ddq_d;
        msg    << robot_state.joint_contact;
        msg    << robot_state.cartesian_contact;
        msg    << robot_state.joint_collision;
        msg    << robot_state.cartesian_collision;
        msg    << robot_state.tau_ext_hat_filtered;
        msg    << robot_state.O_F_ext_hat_K;
        msg    << robot_state.K_F_ext_hat_K;
        msg    << robot_state.O_dP_EE_d;
        msg    << robot_state.O_T_EE_c;
        msg    << robot_state.O_dP_EE_c;
        msg    << robot_state.O_ddP_EE_c; 
        msg    << robot_state.theta;
        msg    << robot_state.dtheta;
        // msg    << robot_state.current_errors;
        // msg    << robot_state.last_motion_errors;
        msg    << robot_state.control_command_success_rate;
        // msg    << robot_state.robot_mode;
        // msg    << robot_state.time.toMSec();
    }

private:
    udp::socket socket_;
    udp::endpoint slave_endpoint;
    int max_length = 8192;
    char send_data_[8192];
    char receive_data_[8192];
    franka::RobotState _master_state; 
    franka::RobotState _slave_state;
    bool is_receive_state = false;
    
}; // end of client


int main(int argc , char* argv[])
{
    if (argc != 3)
    {
        std::cerr << "Usage: "  << argv[0] << "  <server-port>  <slave-robot-hostname> \n";
        return 1;
    }

    short port = std::atoi(argv[1]);
    boost::asio::io_service io_service;
    server s(io_service, port, argv[2] );
    std::cout << "bye bye server! " << std::endl;
    return 0;
} 