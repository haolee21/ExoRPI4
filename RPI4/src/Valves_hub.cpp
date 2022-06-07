#include "Valves_hub.hpp"
#include "MPC_param.hpp"
Valves_hub::Valves_hub()
:
pwmRecorder(Recorder<uint8_t,PWM_VAL_NUM>("PWM",PWM_HEADER))//TODO: use correct valve names, perhaps adding it in shared file with Teensy
,swRecorder(Recorder<bool,SW_VAL_NUM>("SW",SW_HEADER))
,teensyValveCon(TeensyI2C(1))
,LTankCon(MpcInitParam::kLTankCl,MpcInitParam::kLTankCh)
,LKneCon(MpcInitParam::kLKneCl,MpcInitParam::kLKneCh)
,mpc_ltank_rec(Recorder<float,2>("LTank_mpc","Tank,LTank"))
,mpc_lkne_rec(Recorder<float,2>("LKne_mpc","LTank,LKne"))
{
    //Do not set any valve condition here, it will crash
    //I believe the reason is because TeensyI2C is not created yet
    //I guess the behavior of initialization list is different from I thought
    this->mpc_enable.fill(false);
  
    

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


    //MPC check
    const std::array<u_int16_t,SensorHub::NUMPRE>& pre_data = SensorHub::GetPreData(); //use ref to avoid copy
    if(hub.mpc_enable[Valves_hub::MPC_Enable::kLTank]){
        
        int res_duty = hub.LTankCon.GetControl((int)hub.desired_pre[PWM_ID::LTANKPRE],(int)pre_data[SensorHub::PreName::Tank],pre_data[SensorHub::PreName::LTank],hub.PWM_Duty[PWM_ID::LTANKPRE]);
        
        hub.SetDuty(res_duty,Valves_hub::PWM_ID::LTANKPRE); 
        hub.mpc_ltank_rec.PushData(hub.LTankCon.GetPhi());
    }
    if(hub.mpc_enable[Valves_hub::MPC_Enable::kLKne]){
    
        int res_duty = hub.LKneCon.GetControl((int)hub.desired_pre[PWM_ID::LKNEPRE],(int)pre_data[SensorHub::PreName::LTank],pre_data[SensorHub::PreName::LKne],hub.PWM_Duty[PWM_ID::LKNEPRE]);
        hub.SetDuty(res_duty,Valves_hub::PWM_ID::LKNEPRE);
        hub.mpc_lkne_rec.PushData(hub.LKneCon.GetPhi());
    }
    
    
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

const std::array<bool,NUM_OF_MPC>& Valves_hub::GetMpcCond(){
    return std::ref(Valves_hub::GetInstance().mpc_enable);
}

void Valves_hub::StartMPC(Valves_hub::PWM_ID pwm_valve,bool enable){
    Valves_hub& hub = Valves_hub::GetInstance();
    
    switch (pwm_valve)
    {
    case Valves_hub::PWM_ID::LTANKPRE:
        
        hub.mpc_enable[Valves_hub::MPC_Enable::kLTank] = enable;
        break;
    case Valves_hub::PWM_ID::RTANKPRE:
        hub.mpc_enable[Valves_hub::MPC_Enable::kRTank] = enable;
        break;
    case Valves_hub::PWM_ID::LKNEPRE:
        hub.mpc_enable[Valves_hub::MPC_Enable::kLKne] = enable;
        break;
    default:
        break;
    }
}

void Valves_hub::SetDesiredPre(Valves_hub::PWM_ID pwm_valve,u_int16_t des_pre){
    Valves_hub& hub = Valves_hub::GetInstance();
    hub.desired_pre[pwm_valve] = des_pre;

}

