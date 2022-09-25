#include "Valves_hub.hpp"
#include "MPC_param.hpp"
Valves_hub::Valves_hub()
:
left_knee_con(PneumaticParam::kLKne,PneumaticParam::kLTank,"left_knee")
,right_knee_con(PneumaticParam::kRKne,PneumaticParam::kRTank,"right_knee")
,left_ankle_con(PneumaticParam::kLAnk,PneumaticParam::kLTank,"left_ankle")
,right_ankle_con(PneumaticParam::kRAnk,PneumaticParam::kRTank,"right_ankle")
,pwmRecorder("PWM",PWM_HEADER)//TODO: use correct valve names, perhaps adding it in shared file with Teensy
,teensyValveCon(1)
{
    //Do not set any valve condition here, it will crash
    //I believe the reason is because TeensyI2C is not created yet
    //I guess the behavior of initialization list is different from I thought
    // this->mpc_enable.fill(false);
  
    

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
    
    hub.left_knee_con.PushMeas(pre_data[(unsigned)SensorHub::PreName::LKneExt],pre_data[(unsigned)SensorHub::PreName::LKneFlex],
                               pre_data[(unsigned)SensorHub::PreName::LTank],pre_data[(unsigned)SensorHub::PreName::Tank],
                               hub.PWM_Duty[(unsigned)PWM_ID::kLKneExt],hub.PWM_Duty[(unsigned)PWM_ID::kLKneFlex],hub.PWM_Duty[(unsigned)PWM_ID::kLTank],
                               pre_data[(unsigned)SensorHub::PreName::Pos]);
    
    
    if(hub.left_knee_con.GetControlMode()==JointCon::ControlMode::kPreConExt){
        hub.left_knee_con.GetPreCon(hub.desired_pre[(unsigned)Valves_hub::Chamber::kLKneExt],hub.PWM_Duty[(unsigned)PWM_ID::kLKneExt],JointCon::Chamber::kExt);
        hub.valChanged_flag=true;
    }
    else if(hub.left_knee_con.GetControlMode()==JointCon::ControlMode::kPreConFlex){
        hub.left_knee_con.GetPreCon(hub.desired_pre[(unsigned)Valves_hub::Chamber::kLKneFlex],hub.PWM_Duty[(unsigned)PWM_ID::kLKneFlex],JointCon::Chamber::kFlex);
        hub.valChanged_flag=true;
    }
    else if(hub.left_knee_con.GetControlMode()==JointCon::ControlMode::kPreConTank){
        // std::cout<<"ValveLoop: Tank pressure control\n";
        hub.left_knee_con.GetPreCon(hub.desired_pre[(unsigned)Valves_hub::Chamber::kLTank],hub.PWM_Duty[(unsigned)PWM_ID::kLTank],JointCon::Chamber::kTank);
        hub.valChanged_flag=true;
    }
    else if(hub.left_knee_con.GetControlMode()==JointCon::ControlMode::kForceCon){
        hub.left_knee_con.GetForceCon(hub.desired_force[(unsigned)Valves_hub::Joint::kLKne],hub.PWM_Duty[(unsigned)PWM_ID::kLKneExt],hub.PWM_Duty[(unsigned)PWM_ID::kLKneFlex], hub.PWM_Duty[(unsigned)PWM_ID::kLTank]);
        hub.valChanged_flag=true;
    }
    else if(hub.left_knee_con.GetControlMode()==JointCon::ControlMode::kImpCon){
        // std::cout<<"do imp control\n";
        hub.left_knee_con.GetImpCon(hub.desired_imp[(unsigned)Valves_hub::Joint::kLKne],hub.PWM_Duty[(unsigned)PWM_ID::kLKneExt],hub.PWM_Duty[(unsigned)PWM_ID::kLKneFlex],hub.PWM_Duty[(unsigned)PWM_ID::kLTank]);
        hub.valChanged_flag=true;
    }
    
    
    
    if(hub.valChanged_flag){
        
        //check all pwm duty are below 100
        for(int i=0;i<TeensyI2C::CMDLEN;i++){
            if(hub.PWM_Duty[i]>100)
                hub.PWM_Duty[i]=100;
            else if (hub.PWM_Duty[i]<0)
                hub.PWM_Duty[i]=0;
        }

        std::array<char,TeensyI2C::CMDLEN> cmd;

        std::memcpy(cmd.begin(),hub.PWM_Duty.begin(),sizeof(uint8_t)*PWM_VAL_NUM);
    
        hub.teensyValveCon.WriteCmd(cmd);
        hub.valChanged_flag=false;
        
        
    }
    //put measurements in mpc controller, we must do this even the controller are not enabled since it relies on the history of the measurements
    
    hub.pwmRecorder.PushData(hub.GetDuty());

    
}

void Valves_hub::SetDuty(u_int8_t duty, PWM_ID id){
    Valves_hub& hub = Valves_hub::GetInstance();
    hub.PWM_Duty[(unsigned)id]=duty;
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




void Valves_hub::EnableCon(Joint joint,JointCon::ControlMode mode){
    Valves_hub& hub = Valves_hub::GetInstance();

    if(joint == Valves_hub::Joint::kLKne){
        hub.left_knee_con.SetControlMode(mode);
    }
    else if(joint ==Valves_hub::Joint::kLAnk){
        hub.left_ankle_con.SetControlMode(mode);
    }
    else if(joint == Valves_hub::Joint::kRKne){
        hub.right_knee_con.SetControlMode(mode);
    }
    else if(joint == Valves_hub::Joint::kRAnk){
        hub.right_ankle_con.SetControlMode(mode);
    }

}

void Valves_hub::SetDesiredPre(Chamber chamber,double des_pre){
    Valves_hub& hub = Valves_hub::GetInstance();
    hub.desired_pre[(unsigned)chamber] = des_pre;

}
void Valves_hub::SetDesiredImp(Valves_hub::Joint imp,double imp_val){ //TODO: finish it
    Valves_hub& hub = Valves_hub::GetInstance();
    hub.desired_imp[static_cast<unsigned>(imp)]=imp_val;
    
}
void Valves_hub::SetDesiredForce(Valves_hub::Joint joint, double des_force){
    Valves_hub& hub = Valves_hub::GetInstance();
    hub.desired_force[(unsigned)joint]=des_force;
}




void Valves_hub::SetCylnMaxPos(Valves_hub::Joint joint){
    // switch (joint)
    // {
    // case Valves_hub::Joint::kLKne:
    //     Valves_hub::GetInstance().LKneCon.SetCylinderMaxPos();
    //     break;
    // //TODO: finish the rest of the joints
    // default:
    //     break;
    // }
    

}
std::array<bool,(unsigned)Valves_hub::Joint::kTotal>Valves_hub::GetControlCond(){
    Valves_hub& hub = Valves_hub::GetInstance();
    std::array<bool,(unsigned)Valves_hub::Joint::kTotal> cur_cond{0};
    if(hub.left_knee_con.GetControlMode()==JointCon::ControlMode::kForceCon || hub.left_knee_con.GetControlMode()==JointCon::ControlMode::kImpCon){
        cur_cond[0]=true;
    }
    if(hub.left_ankle_con.GetControlMode()==JointCon::ControlMode::kForceCon||hub.left_ankle_con.GetControlMode()==JointCon::ControlMode::kImpCon){
        cur_cond[1]=true;
    }
    return cur_cond;
}