#include "udp_messages.h"

#include <franka/robot_state.h>


enum class CustomType : uint32_t
{
    RobotStateFull,
    MasterJointsPosition,
    SlaveJointsPosition,
    MasterJointsVelociy,
    SlaveJointsVelociy,
};

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

int main(int argc, char* argv[])
{

    franka::RobotState robot_state ;
    robot_state.q = {1,2,3,4,5,6,7};
    robot_state.q_d = {0.0, -1.0, 0.0, -1.0, 0.0, -1.0, 0.0};

    teleop::message<CustomType> msg;
    msg.header.id = CustomType::RobotStateFull;

    serialize( robot_state, msg );
    deserialize( robot_state, msg) ;
    

    return 0;
}