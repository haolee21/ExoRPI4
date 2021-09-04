#include "PWM.hpp"
#include <functional>
PWM::PWM(std::string name)
    : name(name)
{

}

PWM::~PWM()
{
}
void PWM::SetDuty(int _duty){
    this->duty = _duty;
}
const int& PWM::GetDuty(){
    return std::cref(this->duty);
}