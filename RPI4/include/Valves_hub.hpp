#ifndef VALVES_HUB_HPP
#define VALVES_HUB_HPP
#include "PWM.hpp"
#include "SensorHub.hpp"
#include <pthread.h>
#include "SW_Valve.hpp"
#include "Teensy.hpp"
#include "Timer.hpp"
#include "SensorHub.hpp"

class Valves_hub
{
public:
    static Valves_hub& GetInstance();
    // static Valves_hub& GetInstance(std::array<u_int16_t,SensorHub::NUMENC>&,std::array<u_int16_t,SensorHub::NUMPRE>&); // ideally this initializer should be called first

    Valves_hub(const Valves_hub&)=delete; //no copy
    static void SetBaseTimer(std::shared_ptr<Timer>); //this singleton class shared a timer with SensorHub, really hope I can find a better way to write this
    static void UpdateValve();
    ~Valves_hub();
    enum SW_ID{
        LKNEBAL,LANKBAL,RKNEBAL,RANKBAL
    };
    enum PWM_ID{
        LKNEPRE,LANKPRE,RKNEPRE,RANKPRE,LTANKPRE,RTANKPRE
    };
private:
    pthread_t control_loop;
    Valves_hub();
    
    std::shared_ptr<Timer> baseTimer;

    std::array<uint8_t,PWM_VAL_NUM> PWM_ValCond{};
    std::array<bool,SW_VAL_NUM> SW_ValCond{}; //using value initializer to set all values to zero
    void On(Valves_hub::SW_ID sw_valve);//turn on or off the valve
    void Off(Valves_hub::SW_ID sw_valve);
    void SetDuty(uint8_t duty);
    void SendValveCmd();


    PWM LKnePre,LAnkPre,RKnePre,RAnkPre,LTankPre,RTankPre;
    SW_Valve LKneBal,LAnkBal,RKneBal,RAnkBal;

    TeensyI2C teensyValveCon;
    
    


};


#endif