// DatagramSocket send example

#include "Poco/Net/DatagramSocket.h"
#include "Poco/Net/SocketAddress.h"
#include "Poco/Timestamp.h"
#include "Poco/DateTimeFormatter.h"

#include <chrono>
#include <iostream>
#include <thread>

int main(int argc, char** argv)
{
    try
    {
        const std::string _address = "192.168.4.5"; 
        Poco::Net::SocketAddress sa(Poco::Net::IPAddress(_address), 12346);
        Poco::Net::DatagramSocket dgs;
        dgs.connect(sa);
        
        Poco::Timestamp now;
        std::string msg = Poco::DateTimeFormatter::format(now, "<14>%w %f %H:%M:%S I am from other PC, world!");

        for ( int i = 0; i < 100; i++)
        {
            dgs.sendBytes(msg.data(), msg.size());
            std::this_thread::sleep_for( std::chrono::milliseconds(100) ) ;
        }
    }
    catch(Poco::IOException& e)
    {
        std::cerr << e.what() << '\n';
    }
    
    return 0;

}