
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <boost/asio.hpp>

#include <franka/exception.h>
#include <franka/robot.h>

#include <nlohmann/json.hpp>
// for convenience
using json = nlohmann::json;

using boost::asio::ip::udp;

enum { max_length = 8192 };

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
                char request[] = "Msg from client robot callback";      
                
                std::stringstream ss;
                ss << robot_state;
                
                std::size_t sentBytes = sock.send_to(boost::asio::buffer(ss.str()), endpoint);

                char reply[max_length];
                size_t reply_length = sock.receive_from( boost::asio::buffer(reply, max_length), sender_endpoint);
                
                std::cout << "Reply is: ";
                std::cout.write(reply, reply_length);
                std::cout << std::endl << std::endl;

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















 