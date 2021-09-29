#include "SensorHub.hpp"
#include <iostream>
#include "Timer.hpp"
#include <memory>
#include <array>
#include "Valves_hub.hpp"


int main()
{   
    

    struct timespec t;
    long int interval = 100000* USEC;

    std::function<void()> tickCallBack = [](){Valves_hub::UpdateValve();};
    std::shared_ptr<Timer> baseTimer(new Timer(tickCallBack)); //both sensorHub and ValvesHub share the same timer, yet this method is far from ideal.....
    SensorHub::Start(baseTimer);
    Valves_hub::SetBaseTimer(baseTimer);



    clock_gettime(CLOCK_MONOTONIC, &t);
    
    for(int i=0;i<500;i++){
        std::array<u_int16_t,SensorHub::NUMENC> curEnc=SensorHub::GetEncData();
        std::cout<<i<<"cur enc mea: ";
        for(int i2=0;i2<SensorHub::NUMENC;i2++){
            std::cout<<curEnc[i2]<<',';
        }
        std::cout<<"\n";
        std::array<u_int16_t,SensorHub::NUMPRE> curPre=SensorHub::GetPreData();
        std::cout<<i<<"cur pre mea: ";
        for(int i3=0;i3<SensorHub::NUMPRE;i3++){
            std::cout<<curPre[i3]<<',';
        }
        std::cout<<std::endl;
        
        t.tv_nsec+=interval;
        Timer::Sleep(&t);
    }

    
    SensorHub::Stop();
    return 0;
}