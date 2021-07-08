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

#include <franka/exception.h>
#include <franka/robot.h>

#include <nlohmann/json.hpp>
// for convenience
using json = nlohmann::json;

#include <boost/asio.hpp>
using boost::asio::ip::udp;

enum { max_length = 8192 };

class client 
{
public:
    client(boost::asio::io_service& io_service, char* server_port, char* server_ip, char* master_ip)
       : socket_( io_service, udp::endpoint(udp::v4(), 0))
    {
        // udp client
        udp::resolver resolver(io_service);
        udp::resolver::query query( server_ip , server_port );
        slave_endpoint = *resolver.resolve( query );
        std::cout << "I am a UDP client" << std::endl;
        intiialize_robot(master_ip);
        // std::cout << "events are hooked" << std::endl;
    }

    bool intiialize_robot(char* master_ip)
    {
        try 
        {
            franka::Robot robot(master_ip);
            size_t count = 0;
            robot.read(  [this] (const franka::RobotState& robot_state) 
                {   
                    _master_state = robot_state;
                    std::stringstream ss;
                    ss << _master_state;

                    // std::size_t sentBytes = socket_.send_to(boost::asio::buffer(ss.str()), slave_endpoint);
                    // size_t reply_length = socket_.receive_from( boost::asio::buffer(receive_data_, max_length), master_endpoint);
                    // std::cout << "Sent (from master-client): " << (int) sentBytes << "\tReceived (from slave-server)" << (int) reply_length << std::endl;

                    do_send(ss);
                    
                    return true;
                });
        } 
        catch (franka::Exception const& e) 
        {
            std::cout << e.what() << std::endl;
            return false;
        }
    }

    void do_send(std::stringstream& _stream)
    {
        socket_.async_send_to( boost::asio::buffer(_stream.str()), slave_endpoint, [this](boost::system::error_code ec, std::size_t bytes_sent)
            {   
                std::cout << "Sent bytes: " << bytes_sent << std::endl;
                if (bytes_sent > 0)
                {
                    std::cout << "Sent from master to slave: " << (int) bytes_sent << std::endl;
                }

                do_receive();                
            });
    }

    void do_receive() 
    { 
        socket_.async_receive_from(
        boost::asio::buffer(receive_data_, max_length), master_endpoint,
        [this](boost::system::error_code ec, std::size_t bytes_recvd)
        {
            if (!ec && bytes_recvd > 0)
            {
                std::cout << "Received from slave to master: " << (int) bytes_recvd << std::endl;
            }
            else
            {
                do_receive();
            } 
        });
    }

    bool state_parser_json(std::string s, franka::RobotState& robot_state)
    {
        try
        {
            auto state = json::parse(s);
            robot_state.EE_T_K = state["EE_T_K"];
            robot_state.F_T_EE = state["F_T_EE"];
            robot_state.F_x_Cee = state["F_x_Cee"];
            robot_state.F_x_Cload = state["F_x_Cload"];
            robot_state.F_x_Ctotal = state["F_x_Ctotal"];
            robot_state.I_ee = state["I_ee"];
            robot_state.I_load = state["I_load"];
            robot_state.I_total = state["I_total"];
            robot_state.K_F_ext_hat_K = state["K_F_ext_hat_K"];
            robot_state.O_F_ext_hat_K = state["O_F_ext_hat_K"];
            robot_state.O_T_EE = state["O_T_EE"];
            robot_state.O_T_EE_c = state["O_T_EE_c"];
            robot_state.O_T_EE_d = state["O_T_EE_d"];
            robot_state.O_dP_EE_c = state["O_dP_EE_c"];
            robot_state.O_dP_EE_d = state["O_dP_EE_d"];
            robot_state.O_ddP_EE_c = state["O_ddP_EE_c"];
            robot_state.cartesian_collision = state["cartesian_collision"];
            robot_state.cartesian_contact = state["cartesian_contact"];
            robot_state.control_command_success_rate = state["control_command_success_rate"];
            robot_state.ddelbow_c = state["ddelbow_c"];
            robot_state.ddq_d = state["ddq_d"];
            robot_state.delbow_c = state["delbow_c"];
            robot_state.dq = state["dq"];
            robot_state.dq_d = state["dq_d"];
            robot_state.dtau_J = state["dtau_J"];
            robot_state.dtheta = state["dtheta"];
            robot_state.elbow = state["elbow"];
            robot_state.elbow_c = state["elbow_c"];
            robot_state.elbow_d = state["elbow_d"];
            robot_state.joint_collision = state["joint_collision"];
            robot_state.joint_contact = state["joint_contact"];
            robot_state.m_ee = state["m_ee"];
            robot_state.m_load = state["m_load"];
            robot_state.m_total = state["m_total"];
            robot_state.q = state["q"];
            robot_state.q_d = state["q_d"];
            robot_state.robot_mode = state["robot_mode"];
            robot_state.tau_J = state["tau_J"];
            robot_state.tau_ext_hat_filtered = state["tau_ext_hat_filtered"];
            robot_state.theta = state["theta"];
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
    udp::endpoint master_endpoint;
    udp::endpoint slave_endpoint;
    int max_length = 8192;
    char send_data_[8192];
    char receive_data_[8192];
    bool shouldReceive = true;
    franka::RobotState _master_state; 
    franka::RobotState _slave_state;

}; // end of client


int main(int argc , char* argv[])
{
    if (argc != 4)
    {
        std::cerr << "Usage: "  << argv[0] << "  <server-port>  <server-pc-host> <master-robot-hostname> \n";
        return 1;
    }

    boost::asio::io_service io_service;
    client c(io_service, argv[1], argv[2], argv[3]) ;
    io_service.run();
}








 