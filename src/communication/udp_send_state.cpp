// DatagramSocket send example

#include "Poco/Net/DatagramSocket.h"
#include "Poco/Net/SocketAddress.h"
#include "Poco/Timestamp.h"
#include "Poco/DateTimeFormatter.h"

#include <chrono>
#include <iostream>
#include <thread>

#include <franka/exception.h>
#include <franka/robot.h>

int main(int argc, char** argv)
{

    if (argc != 3)
    {
        std::cerr << "Please run as follow: " << argv[0] << "  <robot-hostname>  <remote-pc-hostname> " << std::endl;
        return -1;
    }

    try
    {
        const std::string _address = argv[2]; 
        Poco::Net::SocketAddress sa(Poco::Net::IPAddress(_address), 12346);
        Poco::Net::DatagramSocket dgs;
        dgs.connect(sa);
       
        // Poco::Timestamp now;
        // std::string msg = Poco::DateTimeFormatter::format(now, "<14>%w %f %H:%M:%S I am from other PC, world!");
        // for ( int i = 0; i < 100; i++)
        // {
        //     dgs.sendBytes(msg.data(), msg.size());
        //     std::this_thread::sleep_for( std::chrono::milliseconds(100) ) ;
        // }

        try {
            franka::Robot robot(argv[1]);

                size_t count = 0;
                robot.read([&dgs, &count](const franka::RobotState& robot_state) {
                std::cout << robot_state << std::endl;
                if ( count < 100 )
                {
                    dgs.close();
                    return 0;
                }
            });
            std::cout << "Done." << std::endl;

        } catch (franka::Exception const& e) {
            std::cout << e.what() << std::endl;
            return -1;
        }


    }
    catch(Poco::IOException& e)
    {
        std::cerr << e.what() << '\n';
    }
    
    return 0;

}