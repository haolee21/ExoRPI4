#include "Valves_hub.hpp"
Valves_hub::Valves_hub()
:teensyValveCon(TeensyI2C(1))
{

}

Valves_hub::~Valves_hub()
{
}
Valves_hub& Valves_hub::GetInstance(){
    static Valves_hub instance;
    return instance;
}


void Valves_hub::UpdateValve(){

    
}

void Valves_hub::On(Valves_hub::SW_ID valve){
    switch (valve)
    {
    case Valves_hub::SW_ID::LANKBAL:
        this->SW_ValCond[(unsigned)Valves_hub::SW_ID::LANKBAL]=true;
        break;
    case Valves_hub::SW_ID::LKNEBAL:
        this->SW_ValCond[(unsigned)Valves_hub::SW_ID::LKNEBAL]=true;
        break;
    case Valves_hub::SW_ID::RANKBAL:
        this->SW_ValCond[(unsigned)Valves_hub::SW_ID::RANKBAL]=true;
        break;
    case Valves_hub::SW_ID::RKNEBAL:
        this->SW_ValCond[(unsigned)Valves_hub::SW_ID::RKNEBAL]=true;
        break;
    default:
        throw std::invalid_argument( "cannot find this sw valve to turn on" );
        
    }
}
void Valves_hub::Off(Valves_hub::SW_ID valve){
    switch (valve)
    {
    case Valves_hub::SW_ID::LANKBAL:
        this->SW_ValCond[(unsigned)Valves_hub::SW_ID::LANKBAL]=false;
        break;
    case Valves_hub::SW_ID::LKNEBAL:
        this->SW_ValCond[(unsigned)Valves_hub::SW_ID::LKNEBAL]=false;
        break;
    case Valves_hub::SW_ID::RANKBAL:
        this->SW_ValCond[(unsigned)Valves_hub::SW_ID::RANKBAL]=false;
        break;
    case Valves_hub::SW_ID::RKNEBAL:
        this->SW_ValCond[(unsigned)Valves_hub::SW_ID::RKNEBAL]=false;
        break;
    default:
        throw std::invalid_argument( "cannot find this sw valve to turn off" );
        
    }
}
void Valves_hub::SendValveCmd(){
    std::array<char,TeensyI2C::CMDLEN> cmd;

    std::memcpy(cmd.begin(),this->PWM_Duty.begin(),sizeof(uint8_t)*PWM_VAL_NUM);
    std::memcpy(cmd.begin()+PWM_VAL_NUM*sizeof(uint8_t),this->SW_ValCond.begin(),sizeof(bool)*SW_VAL_NUM);
    
    this->teensyValveCon.WriteCmd(cmd);
    
}

void Valves_hub::SetDuty(u_int8_t duty, Valves_hub::PWM_ID id){
    Valves_hub& hub = Valves_hub::GetInstance();
    hub.PWM_Duty[id]=duty;

}
void Valves_hub::SetDuty(const std::array<u_int8_t,PWM_VAL_NUM> duty){
    Valves_hub& hub = Valves_hub::GetInstance();
    std::memcpy(hub.PWM_Duty.begin(),duty.begin(),sizeof(u_int8_t)*PWM_VAL_NUM);
}
void Valves_hub::SetSW(bool cond,Valves_hub::SW_ID id){
    Valves_hub::GetInstance().SW_ValCond[id]=cond;
}
void Valves_hub::SetSW(const std::array<bool,SW_VAL_NUM> cond){
    std::memcpy(Valves_hub::GetInstance().SW_ValCond.begin(),cond.begin(),sizeof(bool)*SW_VAL_NUM);
}
const std::array<uint8_t,PWM_VAL_NUM>& Valves_hub::GetDuty(){
    return std::ref(Valves_hub::GetInstance().PWM_Duty);
}
const std::array<bool,SW_VAL_NUM>& Valves_hub::GetSWValCond(){
    return std::ref(Valves_hub::GetInstance().SW_ValCond);
}