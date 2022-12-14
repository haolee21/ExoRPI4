#ifndef UDP_PACKET_HPP
#define UDP_PACKET_HPP
#include <array>


#define UDP_CMD_PORT 35000
#define UDP_DATA_PORT 35001

struct UDP_DataPacket
{
    std::array<uint8_t,16> pwm_duty;
    std::array<double,6> enc_data;
    std::array<double,8> pre_data1;
    std::array<uint8_t,4> con_status;
};
struct UDP_CmdData{
    std::array<uint8_t,16> pwm_duty;
    std::array<double,10> des_pre;
    std::array<double,4> des_imp;
    std::array<double,4> des_force;

};
struct UDP_CmdFlag{
    std::array<bool,16> pwm_duty;
    std::array<bool,6> reset_enc;
    std::array<bool,10> des_pre;
    std::array<bool,4> des_imp;
    std::array<bool,4> des_force;

};
struct UDP_CmdPacket
{
    UDP_CmdData data;
    UDP_CmdFlag flags;


};
#endif