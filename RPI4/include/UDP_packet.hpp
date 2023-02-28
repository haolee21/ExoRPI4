#ifndef UDP_PACKET_HPP
#define UDP_PACKET_HPP
#include <array>
#include "TeensyCommon.h"
#include "SensorHub.hpp"
#include "Valves_hub.hpp"

#define UDP_CMD_PORT 25000
#define UDP_DATA_PORT 25001

struct UDP_DataPacket
{
    std::array<u_int8_t,PWM_VAL_NUM> pwm_duty;
    std::array<double,SensorHub::NUMENC> enc_data;
    std::array<double,SensorHub::NUMPRE> pre_data1;
    std::array<bool,(unsigned)Valves_hub::KneeAnkPair::kTotal> con_status;
    bool recorder;
    int fsm_state;
};
struct UDP_CmdPacket{
    std::array<u_int8_t,PWM_VAL_NUM> pwm_duty;
    std::array<double,(unsigned)JointCon::PreCon::kTotal*(unsigned)Valves_hub::KneeAnkPair::kTotal> des_pre;
    std::array<double,(unsigned)JointCon::ForceCon::kTotal*(unsigned)Valves_hub::KneeAnkPair::kTotal> des_imp;
    std::array<double,(unsigned)JointCon::ForceCon::kTotal*(unsigned)Valves_hub::KneeAnkPair::kTotal> des_force;
    // std::array<bool,(unsigned)JointCon::ForceCon::kTotal*(unsigned)Valves_hub::KneeAnkPair::kTotal> force_red_rec; //true if recycle, 
    double epoch_time;
    bool recorder;
    std::array<bool,(unsigned)Valves_hub::KneeAnkPair::kTotal> con_on_off;

    //impact absorbing 
    std::array<double,(unsigned)JointCon::ForceCon::kTotal*(unsigned)Valves_hub::KneeAnkPair::kTotal> init_force;
    std::array<double,(unsigned)JointCon::ForceCon::kTotal*(unsigned)Valves_hub::KneeAnkPair::kTotal> init_impact_imp;
    std::array<double,(unsigned)JointCon::ForceCon::kTotal*(unsigned)Valves_hub::KneeAnkPair::kTotal> restore_imp;

    //fsm
    bool fsm_left_start,fsm_right_start;

    double left_swing_left_load_ratio;
    double right_swing_right_load_ratio;
    double fsm_left_knee_init_f;
    double fsm_right_knee_init_f;
    double fsm_left_knee_imp;
    double fsm_right_knee_imp;
    


    //change flags
    std::array<bool,PWM_VAL_NUM> pwm_duty_flag{false};
    std::array<bool,SensorHub::NUMENC> reset_enc_flag{false};
    std::array<bool,(unsigned)JointCon::PreCon::kTotal*(unsigned)Valves_hub::KneeAnkPair::kTotal> des_pre_flag{false};
    std::array<bool,(unsigned)JointCon::ForceCon::kTotal*(unsigned)Valves_hub::KneeAnkPair::kTotal> des_imp_flag{false};
    std::array<bool,(unsigned)JointCon::ForceCon::kTotal*(unsigned)Valves_hub::KneeAnkPair::kTotal> des_force_flag{false};
    bool epoch_time_flag = false;
    bool recorder_flag = false;
    std::array<bool,(unsigned)Valves_hub::KneeAnkPair::kTotal> con_on_off_flag{false}; //TODO: use it or remove it

    std::array<bool,(unsigned)JointCon::ForceCon::kTotal*(unsigned)Valves_hub::KneeAnkPair::kTotal> set_joint_pos{false}; //TODO: remove it
    
    std::array<bool,(unsigned)JointCon::ForceCon::kTotal*(unsigned)Valves_hub::KneeAnkPair::kTotal> impact_absorb_flag{false}; //TODO: remove it

    //FSM
    bool set_neutral_pos_flag;
    bool fsm_start_flag;
    bool fsm_param_change_flag;

};
// struct UDP_CmdFlag{
//     std::array<bool,PWM_VAL_NUM> pwm_duty;
//     std::array<bool,SensorHub::NUMENC> reset_enc;
//     std::array<bool,(unsigned)Valves_hub::Chamber::kTotal> des_pre;
//     std::array<bool,(unsigned)Valves_hub::Joint::kTotal> des_imp;
//     std::array<bool,(unsigned)Valves_hub::Joint::kTotal> des_force;
//     bool epoch_time;
//     bool recorder;
//     std::array<bool,(unsigned)Valves_hub::Joint::kTotal> con_on_off;

// };
// struct UDP_CmdPacket
// {
//     UDP_CmdData data;
//     UDP_CmdFlag flags;


// };
#endif