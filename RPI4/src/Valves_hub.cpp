#include "Valves_hub.hpp"
Valves_hub::Valves_hub()
:LKnePre(PWM("LKnePre")),LAnkPre(PWM("LAnkPre")),RKnePre(PWM("RKnePre"))
,RAnkPre(PWM("RAnkPre")),LTankPre(PWM("LTankPre")),RTankPre(PWM("RTankPre"))
,LKneBal(SW_Valve("LKneBal")),LAnkBal(SW_Valve("LAnkBal")),RKneBal(SW_Valve("RKneBal")),RAnkBal(SW_Valve("RAnkBal"))
,teensyValveCon(TeensyI2C(1))
{

}

Valves_hub::~Valves_hub()
{
}
Valves_hub& Valves_hub::GetInstance(){
    static Valves_hub instance;
    return instance;
}

void Valves_hub::SetBaseTimer(std::shared_ptr<Timer> _baseTimer){
    Valves_hub::GetInstance().baseTimer = _baseTimer;
}
void Valves_hub::UpdateValve(){

    
}

void Valves_hub::On(Valves_hub::SW_ID valve){
    switch (valve)
    {
    case Valves_hub::SW_ID::LANKBAL:
        this->SW_ValCond[(unsigned)Valves_hub::SW_ID::LANKBAL]=true;
        this->LAnkBal.On();
        break;
    case Valves_hub::SW_ID::LKNEBAL:
        this->SW_ValCond[(unsigned)Valves_hub::SW_ID::LKNEBAL]=true;
        this->LKneBal.On();
        break;
    case Valves_hub::SW_ID::RANKBAL:
        this->SW_ValCond[(unsigned)Valves_hub::SW_ID::RANKBAL]=true;
        this->RAnkBal.On();
        break;
    case Valves_hub::SW_ID::RKNEBAL:
        this->SW_ValCond[(unsigned)Valves_hub::SW_ID::RKNEBAL]=true;
        this->RKneBal.On();
        break;
    default:
        throw std::invalid_argument( "cannot find this sw valve to turn on" );
        break;
    }
}
void Valves_hub::Off(Valves_hub::SW_ID valve){
    switch (valve)
    {
    case Valves_hub::SW_ID::LANKBAL:
        this->SW_ValCond[(unsigned)Valves_hub::SW_ID::LANKBAL]=false;
        this->LAnkBal.Off();
        break;
    case Valves_hub::SW_ID::LKNEBAL:
        this->SW_ValCond[(unsigned)Valves_hub::SW_ID::LKNEBAL]=false;
        this->LKneBal.Off();
        break;
    case Valves_hub::SW_ID::RANKBAL:
        this->SW_ValCond[(unsigned)Valves_hub::SW_ID::RANKBAL]=false;
        this->RAnkBal.Off();
        break;
    case Valves_hub::SW_ID::RKNEBAL:
        this->SW_ValCond[(unsigned)Valves_hub::SW_ID::RKNEBAL]=false;
        this->RKneBal.Off();
        break;
    default:
        throw std::invalid_argument( "cannot find this sw valve to trun off" );
        break;
    }
}
void Valves_hub::SendValveCmd(){
    std::array<char,TeensyI2C::CMDLEN> cmd;

    std::memcpy(cmd.begin(),this->PWM_ValCond.begin(),sizeof(uint8_t)*PWM_VAL_NUM);
    std::memcpy(cmd.begin()+PWM_VAL_NUM,this->SW_ValCond.begin(),sizeof(bool)*SW_VAL_NUM);
    
    this->teensyValveCon.WriteCmd(cmd);
    
}