#ifndef VALVES_HUB_HPP
#define VALVES_HUB_HPP
#include "SensorHub.hpp"
#include <pthread.h>
#include "Teensy.hpp"
#include "Timer.hpp"
#include "SensorHub.hpp"

class Valves_hub
{
public:
    static Valves_hub& GetInstance();
    // static Valves_hub& GetInstance(std::array<u_int16_t,SensorHub::NUMENC>&,std::array<u_int16_t,SensorHub::NUMPRE>&); // ideally this initializer should be called first

    Valves_hub(const Valves_hub&)=delete; //no copy
    
    static void UpdateValve();
    ~Valves_hub();
    enum SW_ID{
        LKNEBAL,LANKBAL,RKNEBAL,RANKBAL,LTANKPRE,RTANKPRE
    };
    enum PWM_ID{
        LKNEPRE,LANKPRE,RKNEPRE,RANKPRE,TANKPRE
    };

    static void SetDuty(uint8_t duty,Valves_hub::PWM_ID id);
    static void SetDuty(const std::array<uint8_t,PWM_VAL_NUM> duty);
    static void SetSW(bool cond,Valves_hub::SW_ID id);
    static void SetSW(const std::array<bool,SW_VAL_NUM> cond);

    //TCP_server read valve condition
    const static std::array<uint8_t,PWM_VAL_NUM>& GetDuty() ;
    const static std::array<bool,SW_VAL_NUM>& GetSWValCond();
private:
    
    Valves_hub();
    
    

    std::array<uint8_t,PWM_VAL_NUM> PWM_Duty{};
    std::array<bool,SW_VAL_NUM> SW_ValCond{}; //using value initializer to set all values to zero
    void On(Valves_hub::SW_ID sw_valve);//turn on or off the valve
    void Off(Valves_hub::SW_ID sw_valve);
    


    


    

    TeensyI2C teensyValveCon;
    
    


};


#endif