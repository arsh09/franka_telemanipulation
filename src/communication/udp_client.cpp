// DatagramSocket receive example

#include "Poco/Net/DatagramSocket.h"
#include "Poco/Net/SocketAddress.h"
#include <iostream>


int main(int argc, char** argv)
{

    try 
    {
        const std::string _address = "127.0.0.1"; 
        Poco::Net::SocketAddress sa(Poco::Net::IPAddress(_address), 12346);
        Poco::Net::DatagramSocket dgs(sa);
        // dgs.bind(sa);

        char buffer[1024];
        
        while (true)
        {
            Poco::Net::SocketAddress sender;
            int n = dgs.receiveFrom(buffer, sizeof(buffer)-1, sender);
            buffer[n] = '\0';
            std::cout << sender.toString() << ": " << buffer << std::endl;
        }
    }

    catch (Poco::IOException& e)
    {
        std::cerr << e.what() << '\n';
    }
    
    return 0;
}