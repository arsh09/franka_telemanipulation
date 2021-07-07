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

#include <franka/exception.h>
#include <franka/robot.h>

#include <nlohmann/json.hpp>
// for convenience
using json = nlohmann::json;

#include <boost/asio.hpp>
using boost::asio::ip::udp;

enum { max_length = 8192 };

void state_parser_string(std::string s)
{
    std::string delimiter = ":";
    size_t pos = 0;
    size_t last_pos = 0;
    std::string token;
    while ((pos = s.find(delimiter)) != std::string::npos) {
        token = s.substr(0, pos);
        std::cout << token << std::endl;
        s.erase(0, pos + delimiter.length());
    }
    std::cout << s << std::endl;
    return ;
}

void state_parser_json(std::string s, franka::RobotState& robot_state)
{
    try{
        auto j3 = json::parse(s);
        std::cout << j3 << std::endl;
    }
    catch(json::exception& e)
    {
        std::cout << e.what() << std::endl;
    }

}

int main(int argc, char* argv[])
{

    if (argc != 4)
    {
        std::cerr << "Usage: blocking_udp_echo_client <host> <port> <master-robot-hostname>\n";
        return 1;
    }

    try
    {
        boost::asio::io_service io_service;
        udp::socket sock(io_service, udp::endpoint(udp::v4(), 0));
        udp::resolver resolver(io_service);
        udp::endpoint endpoint = *resolver.resolve({udp::v4(), argv[1], argv[2]});

        try 
        {
            franka::Robot robot(argv[3]);
            size_t count = 0;
            robot.read([&count, &endpoint, &sock](const franka::RobotState& robot_state) 
            {
                udp::endpoint sender_endpoint;
                
                std::stringstream ss;
                ss << robot_state;
                
                std::size_t sentBytes = sock.send_to(boost::asio::buffer(ss.str()), endpoint);

                char reply[max_length];
                memset( reply , 0, sizeof(reply) );
                size_t reply_length = sock.receive_from( boost::asio::buffer(reply, max_length), sender_endpoint);
                
                franka::RobotState _slave_state;
                state_parser_json( reply, _slave_state);
                return true;
            });

            std::cout << "Done." << std::endl;
        } 
        
        catch (franka::Exception const& e) 
        {
            std::cout << e.what() << std::endl;
            return -1;
        }

    }
    
    catch (std::exception& e)
    {
        std::cerr << "Exception: " << e.what() << "\n";
    }

    return 0;
}















 