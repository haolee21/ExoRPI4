#include "SensorHub.hpp"
#include <iostream>
#include "Timer.hpp"
#include <memory>
#include <array>
#include "Valves_hub.hpp"


int main()
{   
    

    struct timespec t;
    long int interval = 1*SEC;

    
   
    
    Timer::Add_senCallback(SensorHub::UpdateLEnc);
    Timer::Add_senCallback(SensorHub::UpdateREnc);
    Timer::StartRT();



    clock_gettime(CLOCK_MONOTONIC, &t);
    
    for(int i=0;i<50;i++){
        // std::array<u_int16_t,SensorHub::NUMENC> curEnc=SensorHub::GetEncData();
        // std::cout<<i<<"cur enc mea: ";
        // for(int i2=0;i2<SensorHub::NUMENC;i2++){
        //     std::cout<<curEnc[i2]<<',';
        // }
        // std::cout<<"\n";
        // std::array<u_int16_t,SensorHub::NUMPRE> curPre=SensorHub::GetPreData();
        // std::cout<<i<<"cur pre mea: ";
        // for(int i3=0;i3<SensorHub::NUMPRE;i3++){
        //     std::cout<<curPre[i3]<<',';
        // }
        // std::cout<<std::endl;
        
        t.tv_nsec+=interval;
        Timer::Sleep(&t);
    }

    Timer::StopRT();
    
    return 0;
}