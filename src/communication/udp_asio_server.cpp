
/** 
 * UDP Server -> Slave Robot
 * UDP Client -> Master Robot
 * 
 * 
 * Muhammad Arshad 
 * 07-July-2021
**/


#include <cstdlib>
#include <iostream>
#include <franka/robot_state.h>

#include <nlohmann/json.hpp>
// for convenience
using json = nlohmann::json;

#include <boost/asio.hpp>
using boost::asio::ip::udp;


enum { max_length = 8192 };

class server
{
public:
  server(boost::asio::io_service& io_service, short port)
    : socket_(io_service, udp::endpoint(udp::v4(), port))
  {
    do_receive();
  }

  void do_receive()
  {
    socket_.async_receive_from(
        boost::asio::buffer(data_, max_length), sender_endpoint_,
        [this](boost::system::error_code ec, std::size_t bytes_recvd)
        {
          if (!ec && bytes_recvd > 0)
          {
            franka::RobotState _master_state;
            state_parser_json(data_, _master_state);
            do_send(bytes_recvd);
          }
          else
          {
            do_receive();
          }
        });
  }

  void do_send(std::size_t length)
  {
    socket_.async_send_to(
        boost::asio::buffer(data_, length), sender_endpoint_,
        [this](boost::system::error_code ec, std::size_t bytes_sent)
        {
          memset( data_, 0, sizeof(data_));
          do_receive();
        });
  }

  void state_parser_json(std::string s, franka::RobotState& robot_state)
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
          // robot_state.current_errors = state["current_errors"];
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
          // robot_state.last_motion_errors = state["last_motion_errors"];
          robot_state.m_ee = state["m_ee"];
          robot_state.m_load = state["m_load"];
          robot_state.m_total = state["m_total"];
          robot_state.q = state["q"];
          robot_state.q_d = state["q_d"];
          robot_state.robot_mode = state["robot_mode"];
          robot_state.tau_J = state["tau_J"];
          robot_state.tau_ext_hat_filtered = state["tau_ext_hat_filtered"];
          robot_state.theta = state["theta"];
          // robot_state.time = state["time"];
          
          std::cout << state["last_motion_errors"] << "\t" <<  std::endl;
          
      }
      catch(json::exception& e)
      {
          std::cout << e.what() << std::endl;
      }
  }

private:
  udp::socket socket_;
  udp::endpoint sender_endpoint_;
  enum { max_length = 8192 };
  char data_[max_length];
};


int main(int argc, char* argv[])
{
  try
  {
    if (argc != 2)
    {
      std::cerr << "Usage: blocking_udp_echo_server <port>\n";
      return 1;
    }

    boost::asio::io_service io_service;
    server s(io_service, std::atoi(argv[1]));
    io_service.run();
   
  }

  catch (std::exception& e)
  {
    std::cerr << "Exception: " << e.what() << "\n";
  }

  return 0;
}