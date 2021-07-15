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

// #include <Eigen/Dense>
#include <eigen3/Eigen/Dense>

#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/duration.h>
#include <franka/model.h>
#include "examples_common.h"


#include "udp_messages.h"

#include <boost/thread.hpp>
#include <boost/asio.hpp>
using boost::asio::ip::udp;



enum class CustomType : uint32_t
{
    RobotStateFull,
    MasterJointsPosition,
    SlaveJointsPosition,
    MasterJointsVelociy,
    SlaveJointsVelociy,
};


class client 
{
public:

    client(boost::asio::io_service& io_service, char* server_port, char* server_ip, char* master_ip)
       : socket_( io_service, udp::endpoint(udp::v4(), 0))
    {
        memset( receive_data_, 0, sizeof(receive_data_) );

        // udp client
        udp::resolver resolver(io_service);
        udp::resolver::query query( server_ip , server_port );
        slave_endpoint = *resolver.resolve( query );
        

        // give some work to asio
        do_receive_header();

        boost::thread(boost::bind(&boost::asio::io_service::run, &io_service));        
        initialize_robot(master_ip);
    }

    bool initialize_robot(char* master_ip)
    {
        try 
        {
            franka::Robot robot(master_ip);
            setup_state_read_loop(robot);

            // setDefaultBehavior(robot);
            // setup_initial_pose(robot);
            // setup_compliance();
            // setup_impendance_control(robot);

        } 
        catch (franka::Exception const& e) 
        {
            std::cout << e.what() << std::endl;
            return false;
        }
    }

    void setup_initial_pose(franka::Robot& robot)
    {
        std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
        MotionGenerator motion_generator(0.5, q_goal);
        std::cout << "WARNING: This example will move the robot to a pre-configured pose! "
                << "Please make sure to have the user stop button at hand!" << std::endl
                << "Press Enter to continue..." << std::endl;
        std::cin.ignore();
        robot.control(motion_generator);
        std::cout << "Finished moving to initial joint configuration." << std::endl;   
    }

    void setup_state_read_loop(franka::Robot& robot)
    {
        robot.read(  [this] (const franka::RobotState& robot_state) 
        {   
            // _master_state = robot_state;
            do_send_header( robot_state );
            return true;
        });
    }

    void setup_compliance()
    {
        stiffness = Eigen::MatrixXd(6,6);
        damping = Eigen::MatrixXd(6,6);
        stiffness.setZero();
        stiffness.topLeftCorner(3, 3) << translational_stiffness * Eigen::MatrixXd::Identity(3, 3);
        stiffness.bottomRightCorner(3, 3) << rotational_stiffness * Eigen::MatrixXd::Identity(3, 3);
        damping.setZero();
        damping.topLeftCorner(3, 3) << 2.0 * sqrt(translational_stiffness) *
                                            Eigen::MatrixXd::Identity(3, 3);
        damping.bottomRightCorner(3, 3) << 2.0 * sqrt(rotational_stiffness) *
                                                Eigen::MatrixXd::Identity(3, 3);
    }

    void setup_impendance_control(franka::Robot& robot)
    {
        setup_compliance();
        franka::Model model = robot.loadModel();
        franka::RobotState initial_state = robot.readOnce();
        // equilibrium point is the initial position
        initial_transform = Eigen::Affine3d (Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
        position_d = Eigen::Vector3d (initial_transform.translation());
        orientation_d = Eigen::Quaterniond (initial_transform.linear());

        // set collision behavior
        robot.setCollisionBehavior({{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                                {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                                {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                                {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});


        // define callback for the torque control loop
        std::function<franka::Torques(const franka::RobotState&, franka::Duration)>
            impedance_control_callback = [this, &model](const franka::RobotState& robot_state,
                                            franka::Duration /*duration*/) -> franka::Torques 
        {
            _master_state = robot_state;
            
            // get state variables
            std::array<double, 7> coriolis_array = model.coriolis(robot_state);
            std::array<double, 42> jacobian_array =
                model.zeroJacobian(franka::Frame::kEndEffector, robot_state);

            // convert to Eigen
            Eigen::Map<const Eigen::Matrix<double, 7, 1> > coriolis(coriolis_array.data());
            Eigen::Map<const Eigen::Matrix<double, 6, 7> > jacobian(jacobian_array.data());
            Eigen::Map<const Eigen::Matrix<double, 7, 1> > q(robot_state.q.data());
            Eigen::Map<const Eigen::Matrix<double, 7, 1> > dq(robot_state.dq.data());
            Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
            Eigen::Vector3d position(transform.translation());
            Eigen::Quaterniond orientation(transform.linear());

            // compute error to desired equilibrium pose
            // position error
            Eigen::Matrix<double, 6, 1> error;
            error.head(3) << position - position_d;

            // orientation error
            // "difference" quaternion
            if (orientation_d.coeffs().dot(orientation.coeffs()) < 0.0) {
            orientation.coeffs() << -orientation.coeffs();
            }
            Eigen::Quaterniond error_quaternion(orientation * orientation_d.inverse());
            // convert to axis angle
            Eigen::AngleAxisd error_quaternion_angle_axis(error_quaternion);
            // compute "orientation error"
            error.tail(3) << error_quaternion_angle_axis.axis() * error_quaternion_angle_axis.angle();

            // compute control
            Eigen::VectorXd tau_task(7), tau_d(7);

            // Spring damper system with damping ratio=1
            tau_task << jacobian.transpose() * (-stiffness * error - damping * (jacobian * dq));
            tau_d << tau_task + coriolis;

            std::array<double, 7> tau_d_array{};
            Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_d;
            return tau_d_array;
        };

        // start real-time control loop
        std::cout << "WARNING: Collision thresholds are set to high values. "
                << "Make sure you have the user stop at hand!" << std::endl
                << "After starting try to push the robot and see how it reacts." << std::endl
                << "Press Enter to continue..." << std::endl;
        std::cin.ignore();
        robot.control(impedance_control_callback);
    }

    void do_receive_header() 
    { 
        socket_.async_receive_from(
        boost::asio::buffer(&msgIn.header, sizeof(teleop::message_header<CustomType>)), master_endpoint,
        [this](boost::system::error_code ec, std::size_t bytes_recvd)
        {
            if (!ec && bytes_recvd > 0)
            {
                if ( &msgIn.header.size > 0 )
                {
                    msgIn.body.resize(msgIn.header.size);
                    do_receive_body();
                }
            }
            else
            {
                do_receive_header();
            } 
        });
    }

    void do_receive_body() 
    { 
        socket_.async_receive_from(
        boost::asio::buffer( msgIn.body.data(), msgIn.header.size ), master_endpoint,
        [this](boost::system::error_code ec, std::size_t bytes_recvd)
        {
            if (!ec && bytes_recvd > 0)
            {
                if ( &msgIn.header.size > 0 )
                {
                    msgIn.body.resize(msgIn.header.size);
                    deserialize( _slave_state, msgIn);
                }
            }

            do_receive_header();
        });
    }

    void do_send_header(const franka::RobotState& robot_state) 
    { 
        msgOut.header.size = 0;
        msgOut.body.clear();
        
        serialize(robot_state, msgOut);
        socket_.async_send_to( boost::asio::buffer( &msgOut.header, sizeof(teleop::message_header<CustomType>)), slave_endpoint, [this](boost::system::error_code ec, std::size_t bytes_sent)
            {   
                if ( !ec && bytes_sent > 0 )
                {
                    if ( msgOut.header.size > 0 )
                    {
                        do_send_body();
                    }
                }             
            });
    }

    void do_send_body() 
    { 
        socket_.async_send_to( boost::asio::buffer( msgOut.body.data(), msgOut.header.size ), slave_endpoint, [this](boost::system::error_code ec, std::size_t bytes_sent)
            {   
                if ( !ec && bytes_sent > 0 )
                {
                    if ( msgOut.header.size > 0 )
                    {

                    }
                }             
            });
    }

    void print_array(std::array<double, 7> &arr, std::string &name)
    {   
        std::cout << name.c_str() << ":  " ;
        for ( int i = 0; i < arr.size(); i++ )
        {
            std::cout << arr[i] << " , " ;
        }
        std::cout << std::endl;
    }

    // order matters 
    void deserialize(  franka::RobotState& robot_state , teleop::message<CustomType>& msg)
    {
        // msg    >> robot_state.time.toMSec();
        // msg    >> robot_state.robot_mode;
        msg    >> robot_state.control_command_success_rate;
        // msg    >> robot_state.last_motion_errors;
        // msg    >> robot_state.current_errors;
        msg    >> robot_state.dtheta;
        msg    >> robot_state.theta;
        msg    >> robot_state.O_ddP_EE_c ;
        msg    >> robot_state.O_dP_EE_c;
        msg    >> robot_state.O_T_EE_c;
        msg    >> robot_state.O_dP_EE_d;
        msg    >> robot_state.K_F_ext_hat_K;
        msg    >> robot_state.O_F_ext_hat_K;
        msg    >> robot_state.tau_ext_hat_filtered;
        msg    >> robot_state.cartesian_collision;
        msg    >> robot_state.joint_collision;
        msg    >> robot_state.cartesian_contact;
        msg    >> robot_state.joint_contact;
        msg    >> robot_state.ddq_d;
        msg    >> robot_state.dq_d;
        msg    >> robot_state.q_d ;
        msg    >> robot_state.dq;
        msg    >> robot_state.q ;
        msg    >> robot_state.dtau_J; 
        msg    >> robot_state.tau_J_d;   
        msg    >> robot_state.tau_J;
        msg    >> robot_state.ddelbow_c; 
        msg    >> robot_state.delbow_c ;
        msg    >> robot_state.elbow_c;
        msg    >> robot_state.elbow_d ;
        msg    >> robot_state.elbow ;
        msg    >> robot_state.I_total; 
        msg    >> robot_state.F_x_Ctotal ; 
        msg    >> robot_state.m_total;
        msg    >> robot_state.I_load;
        msg    >> robot_state.F_x_Cload; 
        msg    >> robot_state.m_load ;
        msg    >> robot_state.I_ee ;
        msg    >> robot_state.F_x_Cee;
        msg    >> robot_state.m_ee  ;
        msg    >> robot_state.EE_T_K ;
        msg    >> robot_state.F_T_EE  ;
        msg    >> robot_state.NE_T_EE;
        msg    >> robot_state.F_T_NE ;
        msg    >> robot_state.O_T_EE_d;
        msg    >> robot_state.O_T_EE  ;
    }

    // order matters 
    void serialize( const franka::RobotState& robot_state , teleop::message<CustomType>& msg)
    {
    
        msg    << robot_state.O_T_EE  ;
        msg    << robot_state.O_T_EE_d;
        msg    << robot_state.F_T_NE ;
        msg    << robot_state.NE_T_EE;
        msg    << robot_state.F_T_EE  ;
        msg    << robot_state.EE_T_K;
        msg    << robot_state.m_ee   ;
        msg    << robot_state.F_x_Cee;
        msg    << robot_state.I_ee ;
        msg    << robot_state.m_load;
        msg    << robot_state.F_x_Cload;  
        msg    << robot_state.I_load;
        msg    << robot_state.m_total;
        msg    << robot_state.F_x_Ctotal;
        msg    << robot_state.I_total   ;
        msg    << robot_state.elbow;
        msg    << robot_state.elbow_d;  
        msg    << robot_state.elbow_c;
        msg    << robot_state.delbow_c;
        msg    << robot_state.ddelbow_c;   
        msg    << robot_state.tau_J;
        msg    << robot_state.tau_J_d;  
        msg    << robot_state.dtau_J;
        msg    << robot_state.q  ;
        msg    << robot_state.dq;
        msg    << robot_state.q_d; 
        msg    << robot_state.dq_d;
        msg    << robot_state.ddq_d;
        msg    << robot_state.joint_contact;
        msg    << robot_state.cartesian_contact;
        msg    << robot_state.joint_collision;
        msg    << robot_state.cartesian_collision;
        msg    << robot_state.tau_ext_hat_filtered;
        msg    << robot_state.O_F_ext_hat_K;
        msg    << robot_state.K_F_ext_hat_K;
        msg    << robot_state.O_dP_EE_d;
        msg    << robot_state.O_T_EE_c;
        msg    << robot_state.O_dP_EE_c;
        msg    << robot_state.O_ddP_EE_c; 
        msg    << robot_state.theta;
        msg    << robot_state.dtheta;
        // msg    << robot_state.current_errors;
        // msg    << robot_state.last_motion_errors;
        msg    << robot_state.control_command_success_rate;
        // msg    << robot_state.robot_mode;
        // msg    << robot_state.time.toMSec();
    }

private:
    // networking stuff
    udp::socket socket_;
    udp::endpoint master_endpoint;
    udp::endpoint slave_endpoint;
    int max_length = 8192;
    char send_data_[8192];
    char receive_data_[8192];
    franka::RobotState _master_state; 
    franka::RobotState _slave_state;

    teleop::message<CustomType> msgIn;
    teleop::message<CustomType> msgOut;


    // impedance control stuff
    const double translational_stiffness{150.0};
    const double rotational_stiffness{10.0};
    Eigen::MatrixXd stiffness;
    Eigen::MatrixXd damping;

    // equilibrium point is the initial position
    Eigen::Affine3d initial_transform;
    Eigen::Vector3d position_d;
    Eigen::Quaterniond orientation_d;


}; // end of client


int main(int argc , char* argv[])
{
    if (argc != 4)
    {
        std::cerr << "Usage: "  << argv[0] << "  <server-port>  <server-pc-host> <master-robot-hostname> \n";
        return 1;
    }

    boost::asio::io_service io_service;
    client c(io_service, argv[1], argv[2], argv[3]) ;
    // io_service.run();

    std::cout << "bye bye client! " << std::endl;
    return 0;
}








 