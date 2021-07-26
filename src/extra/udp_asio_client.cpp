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

#include <boost/asio.hpp>
using boost::asio::ip::udp;

enum { max_length = 8192 };

class client 
{
public:
    client(boost::asio::io_service& io_service, short port, char* server_port, char* server_ip, char* slave_ip, int is_master)
       : socket_( io_service, udp::endpoint(udp::v4(), port)),
       is_master(is_master)
    {
        if (is_master == 0) // master (192.168.4.5 -> Other PC hostname)
        {
            // udp client
            udp::resolver resolver(io_service);
            udp::resolver::query query( server_ip , server_port );
            server_endpoint = *resolver.resolve( query );
            std::cout << "Tis is a UDP client" << std::endl;
            intiialize_robot(slave_ip);
            do_receive();
        }
        else if (is_master == 1) // slave
        {
            // udp::resolver resolver(io_service);
            // udp::resolver::query query( server_ip , server_port );
            // client_endpoint = *resolver.resolve( query );
            std::cout << "Tis is a UDP server" << std::endl;
            intiialize_robot(slave_ip);
            do_receive();
        }
    }

    bool intiialize_robot(char* slave_ip)
    {
        try 
        {
            franka::Robot robot(slave_ip);
            size_t count = 0;
            robot.read(  [this] (const franka::RobotState& robot_state) 
                {   
                    if (is_master == 0)
                    {
                        _master_state = robot_state;
                        std::stringstream ss;
                        ss << _master_state;
                        do_send(ss);
                        return true;
                    }
                    else
                    {
                        _slave_state = robot_state;
                        std::stringstream ss;
                        ss << _slave_state;
                        do_send(ss);
                        return true;
                    }
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
        udp::endpoint _endpoint;
        if (is_master == 0)
        {
            _endpoint = server_endpoint;
            std::cout << "Client send starting... " << std::endl;
        }
        else
        {
            _endpoint = client_endpoint;
            std::cout << "Server receiving starting... " << std::endl;
        }
        socket_.async_send_to( boost::asio::buffer(_stream.str()), _endpoint, [this](boost::system::error_code ec, std::size_t bytes_sent)
            {   
                std::cout << "Sent bytes: " << bytes_sent << std::endl;
                if (bytes_sent > 0)
                {
                    if (is_master == 0)
                    {
                        std::cout << "Sent from master to slave: " << (int) bytes_sent << std::endl;
                    }
                    else
                    {
                        std::cout << "Sent from slave to master:  " << (int) bytes_sent << std::endl;
                    }
                }
                
                do_receive();                
                
            });
    }

    void do_receive() 
    {
        udp::endpoint _endpoint;
        if (is_master == 0)
        {
            _endpoint = client_endpoint;
            std::cout << "Client receiving starting... " << std::endl;
        }
        else
        {
            _endpoint = client_endpoint;
            std::cout << "Server receiving starting... " << std::endl;
        }
        socket_.async_receive_from(
        boost::asio::buffer(receive_data_, max_length), client_endpoint,
        [this](boost::system::error_code ec, std::size_t bytes_recvd)
        {
            std::cout << "Bytes received on socket: " << bytes_recvd << std::endl;
            if (bytes_recvd > 0)
            {
                if (is_master == 0)
                {
                    std::cout << "Received a msg from a slave to master: " << (int) bytes_recvd << std::endl;
                }
                else
                {
                    std::cout << "Received a msg from a master to slave: " << (int) bytes_recvd << std::endl;
                    // std::stringstream ss;
                    // ss << _slave_state;
                    // do_send(ss);
                }                
                // state_parser_json(receive_data_, _slave_state);
                memset( receive_data_, 0, sizeof(receive_data_) );
            }
            else
            {
                do_receive();
            }
        });
    }

    bool state_parser_json(std::string s, franka::RobotState& robot_state)
    {
        // serialize here.
    }

private:
    udp::socket socket_;
    udp::endpoint client_endpoint;
    udp::endpoint server_endpoint;
    // char* slave_ip;
    int is_master ;
    int max_length = 8192;
    char send_data_[8192];
    char receive_data_[8192];
    franka::RobotState _master_state; 
    franka::RobotState _slave_state;

}; // end of client


int main(int argc , char* argv[])
{
    if (argc != 5)
    {
        std::cerr << "Usage: "  << argv[0] << "  <server-port>  <server-pc-host> <master/slave-robot-hostname> <master/slave-identifier = 0/1>\n";
        return 1;
    }

    int is_master = strtol(argv[4], NULL, 10);
    short port = 0;
    if (is_master != 0) { port = std::atoi(argv[1]); }
    boost::asio::io_service io_service;
    client c(io_service, port, argv[1], argv[2], argv[3], is_master) ;
    io_service.run();
}
 






 