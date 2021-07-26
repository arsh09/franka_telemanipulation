 
#pragma once

#include "commons.h"
#include "messages.h"


namespace teleop
{
    class Follower {
        public:
            Follower(boost::asio::io_service& io_service, char* server_port, char* server_ip);
            ~Follower();

            franka::RobotState _master_state;
            franka::RobotState _slave_state;
            bool is_state_received = false;

        private: 

    };
}
