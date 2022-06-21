#ifndef VALVES_HUB_HPP
#define VALVES_HUB_HPP

#include "SensorHub.hpp"
#include "Teensy.hpp"
#include "Timer.hpp"
#include "SensorHub.hpp"
#include "Recorder.hpp"
#include "MPC.hpp"

#define NUM_OF_MPC 6

class Valves_hub
{
public:
    static Valves_hub& GetInstance();
    // static Valves_hub& GetInstance(std::array<double,SensorHub::NUMENC>&,std::array<double,SensorHub::NUMPRE>&); // ideally this initializer should be called first

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
    static void SetDesiredPre(Valves_hub::PWM_ID pwm_valve,double des_pre);
    const static std::array<bool,NUM_OF_MPC>& GetMpcCond();
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
    MPC LTankCon,LKneCon;
    std::array<bool,NUM_OF_MPC> mpc_enable;
    enum MPC_Enable{
        kLTank,kLKne,kLAnk,kRTank,kRKne,kRAnk
    };
    // bool l_tank_enable,r_tank_enable; //when these flags are true, we will calculate the duty of the pwm during update valve conditions
    std::array<double,PWM_VAL_NUM> desired_pre{0};

    Recorder<float,10> mpc_ltank_rec; //record p_tank, p_set(target), p_val, q_val
    Recorder<float,10> mpc_lkne_rec;


};


#endif