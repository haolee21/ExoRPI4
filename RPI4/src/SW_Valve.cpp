#include "SW_Valve.hpp"
#include <functional>
SW_Valve::SW_Valve(std::string name)
:name(name)
{
    
}

SW_Valve::~SW_Valve()
{
}
void SW_Valve::On(){
    this->condition=true;
}
void SW_Valve::Off(){
    this->condition=false;
}
const bool& SW_Valve::GetValCond(){
    return std::cref(this->condition);
}