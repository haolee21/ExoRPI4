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


    // TCP_server tcp_server;//it needs to connect to a client in order to sync the time when boot
    //Whenever you want to use it, please sync the time of the system first
    //for here I assume the time is already set by the user

    
    UdpServer udp_server;

    
    Timer::Add_senCallback(SensorHub::UpdateLEnc);
    Timer::Add_senCallback(SensorHub::UpdateREnc);
    Timer::Add_senCallback(SensorHub::UpdatePre);
    Timer::Add_conCallback(Valves_hub::UpdateValve);
    
    Timer::StartRT();
    
    

    clock_gettime(CLOCK_MONOTONIC, &t);
    
   

    while(true)
    {
        t.tv_nsec+=interval;
        Timer::Sleep(&t);
        // if(run_count>TOT_RUN_TIME)
        //     break;
        // run_count++;

    }

    Timer::StopRT();
    
    return 0;
}