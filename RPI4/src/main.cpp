#include "Timer.hpp"
#include "SensorHub.hpp"
#include <iostream>
#include <memory>
#include <array>
#include "Valves_hub.hpp"
#include "TCP_server.hpp"
#include "UdpServer.hpp"
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <iomanip>


int main()
{   
    

    struct timespec t;
    long int interval = 1*SEC;


    TCP_server tcp_server;//it needs to connect to a client in order to sync the time when boot
    //Whenever you want to use it, please sync the time of the system first
    //for here I assume the time is already set by the user

    
    UdpServer udp_server;

    
    Timer::Add_senCallback(SensorHub::UpdateLEnc);
    Timer::Add_senCallback(SensorHub::UpdateREnc);
    Timer::Add_senCallback(SensorHub::UpdatePre);
    Timer::Add_conCallback(Valves_hub::UpdateValve);
    
    Timer::StartRT();
    
    

    clock_gettime(CLOCK_MONOTONIC, &t);
    
    // for(int i=0;i<TOT_RUN_TIME;i++){
    //     // std::array<double,SensorHub::NUMENC> curEnc=SensorHub::GetEncData();
    //     // std::cout<<i<<"cur enc mea: ";
    //     // for(int i2=0;i2<SensorHub::NUMENC;i2++){
    //     //     std::cout<<curEnc[i2]<<',';
    //     // }
    //     // std::cout<<"\n";
    //     // std::array<double,SensorHub::NUMPRE> curPre=SensorHub::GetPreData();
    //     // std::cout<<i<<"cur pre mea: ";
    //     // for(int i3=0;i3<SensorHub::NUMPRE;i3++){
    //     //     std::cout<<curPre[i3]<<',';
    //     // }
    //     // std::cout<<std::endl;
        
    //     t.tv_nsec+=interval;
    //     Timer::Sleep(&t);
    // }
    while(true)
    {
        t.tv_nsec+=interval;
        Timer::Sleep(&t);

    }

    Timer::StopRT();
    
    return 0;
}