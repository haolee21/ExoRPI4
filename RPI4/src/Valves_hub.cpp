#include "Valves_hub.hpp"
#include "MPC_param.hpp"
Valves_hub::Valves_hub()
:
pwmRecorder("PWM",PWM_HEADER)//TODO: use correct valve names, perhaps adding it in shared file with Teensy
,swRecorder("SW",SW_HEADER)
,teensyValveCon(1)
,LTankCon(CylinderParam::kLTank)
,LKneCon(CylinderParam::kLkne)
,mpc_ltank_rec("LTank_mpc","Time,dTank,dLTank,LTank_pval,LTank_qval,dPhi_du0,dPhi_du1,dPhi_dx00,dPhi_dx01,dPhi_dx10,dPhi_dx11,mpc_force")
,mpc_lkne_rec("LKne_mpc","Time,dLTank,dLKne,LKne_pval,LKne_qval,dPhi_du0,dPhi_du1,dPhi_dx00,dPhi_dx01,dPhi_dx10,dPhi_dx11,mpc_force")
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
    const std::array<double,SensorHub::NUMPRE>& pre_data = SensorHub::GetPreData(); //use ref to avoid copy

    //put measurements in mpc controller, we must do this even the mpc controller are not enabled since it relies on the history of the measurements
    hub.LTankCon.PushMeas(pre_data[SensorHub::PreName::Tank],pre_data[SensorHub::PreName::LTank],hub.PWM_Duty[PWM_ID::LTANKPRE],0.0);
    hub.LKneCon.PushMeas(pre_data[SensorHub::PreName::LTank],pre_data[SensorHub::PreName::LKne],hub.PWM_Duty[PWM_ID::LKNEPRE],pre_data[SensorHub::PreName::Pos]);

    if(hub.mpc_enable[static_cast<int>(Valves_hub::MPC_Enable::kLTank)]){
        
        int res_duty = hub.LTankCon.GetPreControl(hub.desired_pre[Valves_hub::PWM_ID::LTANKPRE],pre_data[SensorHub::PreName::LTank],pre_data[SensorHub::PreName::Tank],1.0);
        
        hub.SetDuty(res_duty,Valves_hub::PWM_ID::LTANKPRE); 
        hub.mpc_ltank_rec.PushData(hub.LTankCon.GetMpcRec());
    }
    if(hub.mpc_enable[static_cast<int>(Valves_hub::MPC_Enable::kLKne)]){
        
        int res_duty = hub.LKneCon.GetPreControl(hub.desired_pre[Valves_hub::PWM_ID::LKNEPRE],pre_data[SensorHub::PreName::LKne],pre_data[SensorHub::PreName::LTank],hub.LKneCon.GetCylinderScale(pre_data[SensorHub::PreName::LKne],pre_data[SensorHub::PreName::Pos]));
        hub.SetDuty(res_duty,Valves_hub::PWM_ID::LKNEPRE);
        hub.mpc_lkne_rec.PushData(hub.LKneCon.GetMpcRec());
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
        
        hub.mpc_enable[static_cast<unsigned>(Valves_hub::MPC_Enable::kLTank)] = enable;
        break;
    case Valves_hub::PWM_ID::RTANKPRE:
        hub.mpc_enable[static_cast<unsigned>(Valves_hub::MPC_Enable::kRTank)] = enable;
        break;
    case Valves_hub::PWM_ID::LKNEPRE:
        hub.mpc_enable[static_cast<unsigned>(Valves_hub::MPC_Enable::kLKne)] = enable;
        break;
    default:
        break;
    }
}

void Valves_hub::SetDesiredPre(Valves_hub::PWM_ID pwm_valve,double des_pre){
    Valves_hub& hub = Valves_hub::GetInstance();
    hub.desired_pre[pwm_valve] = des_pre;

}
void Valves_hub::SetDesiredImp(Valves_hub::Joint imp,double imp_val){ //TODO: finish it

}



void Valves_hub::SetCylnMaxPos(Valves_hub::Joint joint){
    switch (joint)
    {
    case Valves_hub::Joint::kLKne:
        Valves_hub::GetInstance().LKneCon.SetCylinderMaxPos();
        break;
    //TODO: finish the rest of the joints
    default:
        break;
    }
    

}
