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
    std::array<bool,(unsigned)Valves_hub::Joint::kTotal> con_status;
    bool recorder;
};
struct UDP_CmdPacket{
    std::array<u_int8_t,PWM_VAL_NUM> pwm_duty;
    std::array<double,(unsigned)Valves_hub::Chamber::kTotal> des_pre;
    std::array<double,(unsigned)Valves_hub::Joint::kTotal> des_imp;
    std::array<double,(unsigned)Valves_hub::Joint::kTotal> des_force;
    double epoch_time;
    bool recorder;
    std::array<bool,(unsigned)Valves_hub::Joint::kTotal> con_on_off;

    //change flags
    std::array<bool,PWM_VAL_NUM> pwm_duty_flag;
    std::array<bool,SensorHub::NUMENC> reset_enc_flag;
    std::array<bool,(unsigned)Valves_hub::Chamber::kTotal> des_pre_flag;
    std::array<bool,(unsigned)Valves_hub::Joint::kTotal> des_imp_flag;
    std::array<bool,(unsigned)Valves_hub::Joint::kTotal> des_force_flag;
    bool epoch_time_flag;
    bool recorder_flag;
    std::array<bool,(unsigned)Valves_hub::Joint::kTotal> con_on_off_flag;

    

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