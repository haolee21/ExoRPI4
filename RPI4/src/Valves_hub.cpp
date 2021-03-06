#include "Valves_hub.hpp"
Valves_hub::Valves_hub()
:
pwmRecorder(Recorder<uint8_t,PWM_VAL_NUM>("PWM",PWM_HEADER))//TODO: use correct valve names, perhaps adding it in shared file with Teensy
,swRecorder(Recorder<bool,SW_VAL_NUM>("SW",SW_HEADER))
,teensyValveCon(TeensyI2C(1))
{
    //Do not set any valve condition here, it will crash
    //I believe the reason is because TeensyI2C is not created yet
    //I guess the behavior of initialization list is different from I thought

}

Valves_hub::~Valves_hub()
{
    
    //TODO: add air release sequence
    //right now I will just turn off all valves
    std::cout<<"Turn off all valves\n";
    Valves_hub::SetSW(std::array<bool,SW_VAL_NUM>{0});
    Valves_hub::SetDuty(std::array<uint8_t,PWM_VAL_NUM>{0});
    Valves_hub::UpdateValve(); //SetSW or SetDuty only change the flags in Valves_hub
                               //It is UpdateValve that sends them to Teensy
                               //However, when destructor is called, the original callback (RT Timer) has already stopped
                               //Thus, we will have to call it ourself. 


}
Valves_hub& Valves_hub::GetInstance(){
    static Valves_hub instance;
    return instance;
}


void Valves_hub::On(Valves_hub::SW_ID valve){
    switch (valve)
    {
    case Valves_hub::SW_ID::LANKBAL:
        this->SW_ValCond[(unsigned)Valves_hub::SW_ID::LANKBAL]=true;
        this->valChanged_flag=true;
        break;
    case Valves_hub::SW_ID::LKNEBAL:
        this->SW_ValCond[(unsigned)Valves_hub::SW_ID::LKNEBAL]=true;
        this->valChanged_flag=true;
        break;
    case Valves_hub::SW_ID::RANKBAL:
        this->SW_ValCond[(unsigned)Valves_hub::SW_ID::RANKBAL]=true;
        this->valChanged_flag=true;
        break;
    case Valves_hub::SW_ID::RKNEBAL:
        this->SW_ValCond[(unsigned)Valves_hub::SW_ID::RKNEBAL]=true;
        this->valChanged_flag=true;
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
        this->valChanged_flag=true;
        break;
    case Valves_hub::SW_ID::LKNEBAL:
        this->SW_ValCond[(unsigned)Valves_hub::SW_ID::LKNEBAL]=false;
        this->valChanged_flag=true;
        break;
    case Valves_hub::SW_ID::RANKBAL:
        this->SW_ValCond[(unsigned)Valves_hub::SW_ID::RANKBAL]=false;
        this->valChanged_flag=true;
        break;
    case Valves_hub::SW_ID::RKNEBAL:
        this->SW_ValCond[(unsigned)Valves_hub::SW_ID::RKNEBAL]=false;
        this->valChanged_flag=true;
        break;
    default:
        throw std::invalid_argument( "cannot find this sw valve to turn off" );
        
    }
}
void Valves_hub::UpdateValve(){
    
    Valves_hub& hub = Valves_hub::GetInstance();
    
    if(hub.valChanged_flag){
        std::array<char,TeensyI2C::CMDLEN> cmd;

        std::memcpy(cmd.begin(),hub.PWM_Duty.begin(),sizeof(uint8_t)*PWM_VAL_NUM);
        std::memcpy(cmd.begin()+PWM_VAL_NUM*sizeof(uint8_t),hub.SW_ValCond.begin(),sizeof(bool)*SW_VAL_NUM);
    
        hub.teensyValveCon.WriteCmd(cmd);
        hub.valChanged_flag=false;
        
        
    }
    hub.pwmRecorder.PushData(hub.GetDuty());
    hub.swRecorder.PushData(hub.GetSWValCond());

    
}

void Valves_hub::SetDuty(u_int8_t duty, Valves_hub::PWM_ID id){
    Valves_hub& hub = Valves_hub::GetInstance();
    hub.PWM_Duty[id]=duty;
    hub.valChanged_flag=true;
    

}
void Valves_hub::SetDuty(const std::array<u_int8_t,PWM_VAL_NUM> duty){
    Valves_hub& hub = Valves_hub::GetInstance();
    std::memcpy(hub.PWM_Duty.begin(),duty.begin(),sizeof(u_int8_t)*PWM_VAL_NUM);
    hub.valChanged_flag=true;
}
void Valves_hub::SetSW(bool cond,Valves_hub::SW_ID id){
    Valves_hub& hub = Valves_hub::GetInstance();
    hub.SW_ValCond[id]=cond;
    hub.valChanged_flag=true;
}
void Valves_hub::SetSW(const std::array<bool,SW_VAL_NUM> cond){
    Valves_hub& hub = Valves_hub::GetInstance();
    std::memcpy(hub.SW_ValCond.begin(),cond.begin(),sizeof(bool)*SW_VAL_NUM);
    hub.valChanged_flag=true;
}
const std::array<uint8_t,PWM_VAL_NUM>& Valves_hub::GetDuty(){
    return std::ref(Valves_hub::GetInstance().PWM_Duty);
}
const std::array<bool,SW_VAL_NUM>& Valves_hub::GetSWValCond(){
    return std::ref(Valves_hub::GetInstance().SW_ValCond);
}

