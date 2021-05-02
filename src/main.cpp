#include "SensorHub.hpp"
#include <iostream>
#include "Timer.hpp"
#include <array>
int main()
{

    struct timespec t;
    long int interval = 100000* USEC;
    SensorHub::Start();
    clock_gettime(CLOCK_MONOTONIC, &t);
    
    for(int i=0;i<500;i++){
        std::array<short,SensorHub::NUMENC> curEnc=SensorHub::GetEncData();
        std::cout<<i<<"cur mea: ";
        for(int i2=0;i2<SensorHub::NUMENC;i2++){
            std::cout<<curEnc[i2]<<',';
        }
        std::cout<<"\n";


        t.tv_nsec+=interval;
        Timer::Sleep(&t);
    }

    
    SensorHub::Stop();
    return 0;
}