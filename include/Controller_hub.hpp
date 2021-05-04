#ifndef CONTROLLER_HUB_HPP
#define CONTROLLER_HUB_HPP
#include "PWM.hpp"
#include "SensorHub.hpp"
#include <pthread.h>
#include "SW_Valve.hpp"

class Controller_hub
{
private:
    pthread_t control_loop;
    Controller_hub();
    static Controller_hub& GetInstance();
public:
    Controller_hub(const Controller_hub&)=delete; //no copy
    ~Controller_hub();

};

Controller_hub::Controller_hub(/* args */)
{
}

Controller_hub::~Controller_hub()
{
}
Controller_hub& Controller_hub::GetInstance(){
    static Controller_hub instance;
    return instance;
}
#endif