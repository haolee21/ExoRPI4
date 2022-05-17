#ifndef VALVES_HUB_HPP
#define VALVES_HUB_HPP

#include "SensorHub.hpp"
#include "Teensy.hpp"
#include "Timer.hpp"
#include "SensorHub.hpp"
#include "Recorder.hpp"
#include "MPC.hpp"


class Valves_hub
{
public:
    static Valves_hub& GetInstance();
    // static Valves_hub& GetInstance(std::array<u_int16_t,SensorHub::NUMENC>&,std::array<u_int16_t,SensorHub::NUMPRE>&); // ideally this initializer should be called first

    Valves_hub(const Valves_hub&)=delete; //no copy
    
    static void UpdateValve();
    ~Valves_hub();
    enum SW_ID{
        LKNEBAL,LANKBAL,RKNEBAL,RANKBAL,
        FIRST_SW=LKNEBAL,LAST_SW=RANKBAL
    };
    enum PWM_ID{
        LTANKPRE,LKNEPRE,
        LANKPRE,RKNEPRE,RANKPRE,RTANKPRE,
        FIRST_PWM=LTANKPRE,LAST_PWM=RTANKPRE
    };


    static void SetDuty(uint8_t duty,Valves_hub::PWM_ID id);
    static void SetDuty(const std::array<uint8_t,PWM_VAL_NUM> duty);
    static void SetSW(bool cond,Valves_hub::SW_ID id);
    static void SetSW(const std::array<bool,SW_VAL_NUM> cond);

    //TCP_server read valve condition
    const static std::array<uint8_t,PWM_VAL_NUM>& GetDuty() ;
    const static std::array<bool,SW_VAL_NUM>& GetSWValCond();

    //MPC control
    static void StartMPC(Valves_hub::PWM_ID pwm_valve,bool enable);
    static void SetDesiredPre(Valves_hub::PWM_ID pwm_valve,u_int16_t des_pre);
private:
    
    Valves_hub();
    
    

    std::array<uint8_t,PWM_VAL_NUM> PWM_Duty{0};
    std::array<bool,SW_VAL_NUM> SW_ValCond{0}; //using value initializer to set all values to zero
    void On(Valves_hub::SW_ID sw_valve);//turn on or off the valve
    void Off(Valves_hub::SW_ID sw_valve);
    
    bool valChanged_flag;

    Recorder<uint8_t,PWM_VAL_NUM> pwmRecorder;
    Recorder<bool,SW_VAL_NUM> swRecorder;


    
    TeensyI2C teensyValveCon;

    //MPC Pressure control
    MPC LTankCon,RTankCon;
    bool l_tank_enable,r_tank_enable; //when these flags are true, we will calculate the duty of the pwm during update valve conditions
    std::array<u_int16_t,PWM_VAL_NUM> desired_pre{0};



};


#endif