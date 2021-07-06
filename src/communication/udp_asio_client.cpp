
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <boost/asio.hpp>

using boost::asio::ip::udp;

enum { max_length = 1024 };

int main(int argc, char* argv[])
{
    try
    {
        if (argc != 3)
        {
            std::cerr << "Usage: blocking_udp_echo_client <host> <port>\n";
            return 1;
        }

        boost::asio::io_service io_service;
        udp::socket s(io_service, udp::endpoint(udp::v4(), 0));
        udp::resolver resolver(io_service);
        udp::endpoint endpoint = *resolver.resolve({udp::v4(), argv[1], argv[2]});

        for (;;)
        {
            std::cout << "Enter message: ";
            char request[max_length];
            std::cin.getline(request, max_length);
            size_t request_length = std::strlen(request);
            s.send_to(boost::asio::buffer(request, request_length), endpoint);

            char reply[max_length];
            udp::endpoint sender_endpoint;
            size_t reply_length = s.receive_from( boost::asio::buffer(reply, max_length), sender_endpoint);
            std::cout << "Reply is: ";
            std::cout.write(reply, reply_length);
            std::cout << "\n";
        }

    }
    
    catch (std::exception& e)
    {
        std::cerr << "Exception: " << e.what() << "\n";
    }

    return 0;
}
















// #include <cstdlib>
// #include <iostream>
// #include <boost/asio.hpp>

// using boost::asio::ip::udp;

// class server
// {
// public:
//     server(boost::asio::io_service& io_service, short port)
//     : socket_(io_service, udp::endpoint(udp::v4(), port))
//     {
//         do_receive();
//     }

//     void do_receive()
//     {
//         socket_.async_receive_from(
//             boost::asio::buffer(data_, max_length), sender_endpoint_,
//             [this](boost::system::error_code ec, std::size_t bytes_recvd)
//             {
//                 if (!ec && bytes_recvd > 0)
//                 {
//                     std::cout << "Received: " << (int) bytes_recvd << std::endl;
//                     do_send(bytes_recvd);
//                 }
//                 else
//                 {
//                     do_receive();
//                 }
//             });
//     }

//   void do_send(std::size_t length)
//   {
//     socket_.async_send_to(
//         boost::asio::buffer(data_, length), sender_endpoint_,
//         [this](boost::system::error_code /*ec*/, std::size_t /*bytes_sent*/)
//         {
//             do_receive();
//         });
//   }

// private:
//     udp::socket socket_;
//     udp::endpoint sender_endpoint_;
//     enum { max_length = 1024 };
//     char data_[max_length];
// };

// int main(int argc, char* argv[])
// {
//     try
//     {
//         if (argc != 2)
//         {
//             std::cerr << "Usage: async_udp_echo_server <port>\n";
//             return 1;
//         }

//         boost::asio::io_service io_service;
//         server s(io_service, std::atoi(argv[1]));
//         io_service.run();
//     }

//     catch (std::exception& e)
//     {
//         std::cerr << "Exception: " << e.what() << "\n";
//     }

//     return 0;
// }