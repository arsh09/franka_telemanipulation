// DatagramSocket receive example

#include "Poco/Net/DatagramSocket.h"
#include "Poco/Net/SocketAddress.h"
#include <iostream>


int main(int argc, char** argv)
{
    if (argc != 2)
    {
        std::cerr << "Usage: " << argv[0] << " <remote-pc-hostname>" << std::endl;
        return -1;
    }

    try 
    {
        const std::string _address = argv[1]; 
        Poco::Net::SocketAddress sa(Poco::Net::IPAddress(_address), 12345);
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

        dgs.close();
    }

    catch (Poco::IOException& e)
    {
        std::cerr << e.what() << '\n';
    }
    
    return 0;
}