                
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

#include <nlohmann/json.hpp>
// for convenience
using json = nlohmann::json;

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
            setDefaultBehavior(robot);
            setup_initial_pose(robot);
            setup_position_control(robot);

            // setup_state_read_loop(robot);
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

    void do_send( std::array<double, 7>& q)
    {
        socket_.async_send_to( boost::asio::buffer( q ), slave_endpoint, [this](boost::system::error_code ec, std::size_t bytes_sent)
            {   
                if (bytes_sent > 0)
                {
                    // std::cout << "Sent from master to slave: " << (int) bytes_sent << std::endl;
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
                // std::cout << "Received master joints values: " << (int) bytes_recvd << std::endl;
                // state_parser_json(receive_data_ , _master_state);
                // std::string _s = "Master joints: ";
                // print_array(_master_state.q, _s);
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

    bool state_parser_json(std::string s, franka::RobotState& robot_state)
    {
        try
        {
            auto state = json::parse(s);
            // robot_state.EE_T_K = state["EE_T_K"];
            // robot_state.F_T_EE = state["F_T_EE"];
            // robot_state.F_x_Cee = state["F_x_Cee"];
            // robot_state.F_x_Cload = state["F_x_Cload"];
            // robot_state.F_x_Ctotal = state["F_x_Ctotal"];
            // robot_state.I_ee = state["I_ee"];
            // robot_state.I_load = state["I_load"];
            // robot_state.I_total = state["I_total"];
            // robot_state.K_F_ext_hat_K = state["K_F_ext_hat_K"];
            // robot_state.O_F_ext_hat_K = state["O_F_ext_hat_K"];
            // robot_state.O_T_EE = state["O_T_EE"];
            // robot_state.O_T_EE_c = state["O_T_EE_c"];
            // robot_state.O_T_EE_d = state["O_T_EE_d"];
            // robot_state.O_dP_EE_c = state["O_dP_EE_c"];
            // robot_state.O_dP_EE_d = state["O_dP_EE_d"];
            // robot_state.O_ddP_EE_c = state["O_ddP_EE_c"];
            // robot_state.cartesian_collision = state["cartesian_collision"];
            // robot_state.cartesian_contact = state["cartesian_contact"];
            // robot_state.control_command_success_rate = state["control_command_success_rate"];
            // robot_state.ddelbow_c = state["ddelbow_c"];
            // robot_state.ddq_d = state["ddq_d"];
            // robot_state.delbow_c = state["delbow_c"];
            // robot_state.dq = state["dq"];
            // robot_state.dq_d = state["dq_d"];
            // robot_state.dtau_J = state["dtau_J"];
            // robot_state.dtheta = state["dtheta"];
            // robot_state.elbow = state["elbow"];
            // robot_state.elbow_c = state["elbow_c"];
            // robot_state.elbow_d = state["elbow_d"];
            // robot_state.joint_collision = state["joint_collision"];
            // robot_state.joint_contact = state["joint_contact"];
            // robot_state.m_ee = state["m_ee"];
            // robot_state.m_load = state["m_load"];
            // robot_state.m_total = state["m_total"];
            robot_state.q = state["q"];
            // robot_state.q_d = state["q_d"];
            // robot_state.robot_mode = state["robot_mode"];
            // robot_state.tau_J = state["tau_J"];
            // robot_state.tau_ext_hat_filtered = state["tau_ext_hat_filtered"];
            // robot_state.theta = state["theta"];
            // robot_state.last_motion_errors = state["last_motion_errors"];
            // robot_state.last_motion_errors = state["last_motion_errors"];
            // robot_state.time = state["time"];
            return true;
        }
        catch(json::exception& e)
        {
            std::cout << e.what() << std::endl;
            return false;
        }
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