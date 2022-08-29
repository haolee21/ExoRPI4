#include "Valves_hub.hpp"
#include "MPC_param.hpp"
Valves_hub::Valves_hub()
:
pwmRecorder("PWM",PWM_HEADER)//TODO: use correct valve names, perhaps adding it in shared file with Teensy
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




void Valves_hub::UpdateValve(){
    
    Valves_hub& hub = Valves_hub::GetInstance();


    //MPC check
    const std::array<double,SensorHub::NUMPRE>& pre_data = SensorHub::GetPreData(); //use ref to avoid copy

    //put measurements in mpc controller, we must do this even the mpc controller are not enabled since it relies on the history of the measurements
    hub.LTankCon.PushMeas(pre_data[SensorHub::PreName::Tank],pre_data[SensorHub::PreName::LTank],hub.PWM_Duty[PWM_ID::LTANKPRE],0.0);
    hub.LKneCon.PushMeas(pre_data[SensorHub::PreName::LTank],pre_data[SensorHub::PreName::LKne],hub.PWM_Duty[PWM_ID::LKNEPRE],pre_data[SensorHub::PreName::Pos]);

    if(hub.mpc_enable[static_cast<int>(Valves_hub::MPC_Enable::kLTank)]){
        int res_duty = hub.LTankCon.GetPreControl(hub.desired_pre[PWM_ID::LTANKPRE],pre_data[SensorHub::PreName::LTank],pre_data[SensorHub::PreName::Tank],1.0);
        
        hub.SetDuty(res_duty,PWM_ID::LTANKPRE); 
        hub.mpc_ltank_rec.PushData(hub.LTankCon.GetMpcRec());
    }
    if(hub.mpc_enable[static_cast<int>(Valves_hub::MPC_Enable::kLKne)]){
        int res_duty=0;
        bool need_bal=false;
        int ank_duty=0;
        if(hub.imp_enable[static_cast<int>(Valves_hub::Joint::kLKne)]){
            res_duty = hub.LKneCon.GetImpControl(hub.desired_imp[static_cast<unsigned>(Valves_hub::Joint::kLKne)],pre_data[SensorHub::PreName::LKne],pre_data[SensorHub::PreName::LTank],pre_data[SensorHub::PreName::Pos],hub.LKneCon.GetCylinderScale(pre_data[SensorHub::PreName::LKne],pre_data[SensorHub::PreName::Pos]),need_bal);
            if(need_bal){
                ank_duty=100;
            }
        }
        else{
            res_duty = hub.LKneCon.GetPreControl(hub.desired_pre[PWM_ID::LKNEPRE],pre_data[SensorHub::PreName::LKne],pre_data[SensorHub::PreName::LTank],hub.LKneCon.GetCylinderScale(pre_data[SensorHub::PreName::LKne],pre_data[SensorHub::PreName::Pos]));
        }
        hub.SetDuty(res_duty,PWM_ID::LKNEPRE);
        hub.SetDuty(ank_duty,PWM_ID::LANKPRE);

        hub.mpc_lkne_rec.PushData(hub.LKneCon.GetMpcRec());
    }
    
    
    if(hub.valChanged_flag){
        std::array<char,TeensyI2C::CMDLEN> cmd;

        std::memcpy(cmd.begin(),hub.PWM_Duty.begin(),sizeof(uint8_t)*PWM_VAL_NUM);
    
        hub.teensyValveCon.WriteCmd(cmd);
        hub.valChanged_flag=false;
        
        
    }
    hub.pwmRecorder.PushData(hub.GetDuty());

    
}

void Valves_hub::SetDuty(u_int8_t duty, PWM_ID id){
    Valves_hub& hub = Valves_hub::GetInstance();
    hub.PWM_Duty[id]=duty;
    hub.valChanged_flag=true;
    

}
void Valves_hub::SetDuty(const std::array<u_int8_t,PWM_VAL_NUM> duty){
    Valves_hub& hub = Valves_hub::GetInstance();
    std::memcpy(hub.PWM_Duty.begin(),duty.begin(),sizeof(u_int8_t)*PWM_VAL_NUM);
    hub.valChanged_flag=true;
}


const std::array<uint8_t,PWM_VAL_NUM>& Valves_hub::GetDuty(){
    return std::ref(Valves_hub::GetInstance().PWM_Duty);
}


const std::array<bool,NUM_OF_MPC>& Valves_hub::GetMpcCond(){
    return std::ref(Valves_hub::GetInstance().mpc_enable);
}

void Valves_hub::StartMPC(PWM_ID pwm_valve,bool enable){
    Valves_hub& hub = Valves_hub::GetInstance();
    
    switch (pwm_valve)
    {
    case PWM_ID::LTANKPRE:
        
        hub.mpc_enable[static_cast<unsigned>(Valves_hub::MPC_Enable::kLTank)] = enable;
        break;
    case PWM_ID::RTANKPRE:
        hub.mpc_enable[static_cast<unsigned>(Valves_hub::MPC_Enable::kRTank)] = enable;
        break;
    case PWM_ID::LKNEPRE:
        hub.mpc_enable[static_cast<unsigned>(Valves_hub::MPC_Enable::kLKne)] = enable;
        break;
    default:
        break;
    }
}

void Valves_hub::SetDesiredPre(PWM_ID pwm_valve,double des_pre){
    Valves_hub& hub = Valves_hub::GetInstance();
    hub.desired_pre[pwm_valve] = des_pre;

}
void Valves_hub::SetDesiredImp(Valves_hub::Joint imp,double imp_val){ //TODO: finish it
    Valves_hub& hub = Valves_hub::GetInstance();
    hub.desired_imp[static_cast<unsigned>(imp)]=imp_val;
    
}
void Valves_hub::EnableImpCon(Valves_hub::Joint imp,bool enable){
    Valves_hub::GetInstance().imp_enable[static_cast<unsigned>(imp)]=enable;

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
