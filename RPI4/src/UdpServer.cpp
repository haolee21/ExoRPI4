#include "UdpServer.hpp"
UdpServer::UdpServer(/* args */)
{
    //rx init
    if((this->sockfd_rx=socket(AF_INET,SOCK_DGRAM,IPPROTO_UDP))==-1){
        perror("socket create failed");
        exit(EXIT_FAILURE);

    }
    memset((char*)&this->addr_rx,0,sizeof this->addr_rx);
    this->addr_rx.sin_family=AF_INET;
    this->addr_rx.sin_port=htons(UDP_CMD_PORT);
    this->addr_rx.sin_addr.s_addr=htonl(INADDR_ANY);
    if(bind(this->sockfd_rx,(struct sockaddr*)&this->addr_rx,sizeof this->addr_rx)==-1){
        perror("bind failed");
        exit(EXIT_FAILURE);
    }
    //tx init
    if((this->sockfd_tx=socket(AF_INET,SOCK_DGRAM,IPPROTO_UDP))==-1){

    }
    memset((char*)&this->addr_tx,0,sizeof this->addr_tx);
    this->addr_tx.sin_family=AF_INET;
    this->addr_tx.sin_port=htons(UDP_DATA_PORT);

    this->sockfd_rx_len = sizeof this->addr_rx;
    this->server_thread.reset(new std::thread(&UdpServer::ServerProcess,this));

}
bool UdpServer::Recv(char* buf, int buf_len){
    int recv_len = recvfrom(this->sockfd_rx,buf,buf_len,0,(struct sockaddr*)&this->addr_rx,(socklen_t*)&this->sockfd_rx_len);
    if(recv_len!=buf_len){
        std::cout<<"recv msg length is wrong\n";
        return false;
    }
    else{
        inet_aton(inet_ntoa(this->addr_rx.sin_addr),&this->addr_tx.sin_addr);
        return true;
    }
}
void UdpServer::Send(char* buf, int buf_len){
    if(sendto(this->sockfd_tx,buf,buf_len,0,(struct sockaddr*)&this->addr_tx,sizeof this->addr_tx)==-1){
        perror("UDP Send Failed");
    }
}
void UdpServer::GetDataPacket(UDP_DataPacket &return_packet){
    const std::array<double,SensorHub::NUMENC>& enc_data = SensorHub::GetEncData();
    std::copy(enc_data.begin(),enc_data.end(),return_packet.enc_data.begin());
    const std::array<double,SensorHub::NUMPRE>& pre_data = SensorHub::GetPreData();
    std::copy(pre_data.begin(),pre_data.end(),return_packet.pre_data1.begin());
    
    const std::array<u_int8_t,PWM_VAL_NUM>& pwm_duty = Valves_hub::GetDuty();
    std::copy(pwm_duty.begin(),pwm_duty.end(),return_packet.pwm_duty.begin());

    const std::array<bool,(unsigned)Valves_hub::Joint::kTotal> controller_cond = Valves_hub::GetControlCond();
    std::copy(controller_cond.begin(),controller_cond.end(),return_packet.con_status.begin());
}   

UdpServer::~UdpServer()
{
}

bool UdpServer::CheckCmdSet(bool* array, int array_len){
    int sum=0;
    for(int i=0;i<array_len;i++){
        sum+=(int)array[i];
    }
    if(sum==0)return false;
    return true;

}
void UdpServer::ServerProcess(){
    while(true){
        UDP_CmdPacket cmd_packet;

        if(this->Recv((char*)&cmd_packet,sizeof cmd_packet)){
            UDP_DataPacket data_packet;
            this->GetDataPacket(data_packet);
            this->Send((char*)&data_packet,sizeof data_packet);

            //handle the commands
            
            
                if(cmd_packet.flags.des_imp[(unsigned)Valves_hub::Joint::kLKne]){
                    Valves_hub::EnableCon(Valves_hub::Joint::kLKne,JointCon::ControlMode::kImpCon);
                    Valves_hub::SetDesiredImp(Valves_hub::Joint::kLKne,cmd_packet.data.des_imp[(unsigned)Valves_hub::Joint::kLKne]);
                }
                else if(cmd_packet.flags.des_imp[(unsigned)Valves_hub::Joint::kLAnk]){
                    Valves_hub::EnableCon(Valves_hub::Joint::kLAnk,JointCon::ControlMode::kImpCon);
                    Valves_hub::SetDesiredImp(Valves_hub::Joint::kLAnk,cmd_packet.data.des_imp[(unsigned)Valves_hub::Joint::kLAnk]);
                }
                else if(cmd_packet.flags.des_imp[(unsigned)Valves_hub::Joint::kRKne]){
                    Valves_hub::EnableCon(Valves_hub::Joint::kRKne,JointCon::ControlMode::kImpCon);
                    Valves_hub::SetDesiredImp(Valves_hub::Joint::kRKne,cmd_packet.data.des_imp[(unsigned)Valves_hub::Joint::kRKne]);
                }
                else if(cmd_packet.flags.des_imp[(unsigned)Valves_hub::Joint::kRAnk]){
                    Valves_hub::EnableCon(Valves_hub::Joint::kRAnk,JointCon::ControlMode::kImpCon);
                    Valves_hub::SetDesiredImp(Valves_hub::Joint::kRAnk,cmd_packet.data.des_imp[(unsigned)Valves_hub::Joint::kRAnk]);
                }

            }

        }


}


void UdpServer::ProcessCmd(UDP_CmdPacket cmd_packet){
    //we check if there is any commands by summing the bool arrays, >0 means there is some commands
    //reset encoder
    if(this->CheckCmdSet(cmd_packet.flags.reset_enc.begin(),cmd_packet.flags.reset_enc.size())){
        if(cmd_packet.flags.reset_enc[SensorHub::EncName::LHipS]){
            SensorHub::ResetEnc(SensorHub::EncName::LHipS);
        }
        if(cmd_packet.flags.reset_enc[SensorHub::EncName::LKneS]){
            SensorHub::ResetEnc(SensorHub::EncName::LKneS);
        }
        if(cmd_packet.flags.reset_enc[SensorHub::EncName::LAnkS]){
            SensorHub::ResetEnc(SensorHub::EncName::LAnkS);
        }
        if(cmd_packet.flags.reset_enc[SensorHub::EncName::RHipS]){
            SensorHub::ResetEnc(SensorHub::EncName::RHipS);
        }
        if(cmd_packet.flags.reset_enc[SensorHub::EncName::RKneS]){
            SensorHub::ResetEnc(SensorHub::EncName::RKneS);
        }
        if(cmd_packet.flags.reset_enc[SensorHub::EncName::RAnkS]){
            SensorHub::ResetEnc(SensorHub::EncName::RAnkS);
        }
    }
    if(this->CheckCmdSet(cmd_packet.flags.pwm_duty.begin(),cmd_packet.flags.pwm_duty.size())){
        if(cmd_packet.flags.pwm_duty[(unsigned)PWM_ID::kLTank]){
            Valves_hub::EnableCon(Valves_hub::Joint::kLKne,JointCon::ControlMode::kPreConTank);
            Valves_hub::SetDuty(cmd_packet.data.pwm_duty[(unsigned)PWM_ID::kLTank],PWM_ID::kLTank);
        }
        if(cmd_packet.flags.pwm_duty[(unsigned)PWM_ID::kLKneExt]){
            Valves_hub::EnableCon(Valves_hub::Joint::kLKne,JointCon::ControlMode::kPreConExt);
            Valves_hub::SetDuty(cmd_packet.data.pwm_duty[(unsigned)PWM_ID::kLKneExt],PWM_ID::kLKneExt);
        }
        if(cmd_packet.flags.pwm_duty[(unsigned)PWM_ID::kLKneFlex]){
            Valves_hub::EnableCon(Valves_hub::Joint::kLKne,JointCon::ControlMode::kPreConFlex);
            Valves_hub::SetDuty(cmd_packet.data.pwm_duty[(unsigned)PWM_ID::kLKneFlex],PWM_ID::kLKneFlex);
        }
        if(cmd_packet.flags.pwm_duty[(unsigned)PWM_ID::kLAnkExt]){
            Valves_hub::EnableCon(Valves_hub::Joint::kLAnk,JointCon::ControlMode::kPreConExt);
            Valves_hub::SetDuty(cmd_packet.data.pwm_duty[(unsigned)PWM_ID::kLAnkExt],PWM_ID::kLAnkExt);
        }
        if(cmd_packet.flags.pwm_duty[(unsigned)PWM_ID::kLAnkFlex]){
            Valves_hub::EnableCon(Valves_hub::Joint::kLAnk,JointCon::ControlMode::kPreConFlex);
            Valves_hub::SetDuty(cmd_packet.data.pwm_duty[(unsigned)PWM_ID::kLAnkFlex],PWM_ID::kLAnkFlex);
        }
        if(cmd_packet.flags.pwm_duty[(unsigned)PWM_ID::kRTank]){
            Valves_hub::EnableCon(Valves_hub::Joint::kRKne,JointCon::ControlMode::kPreConTank);
            Valves_hub::SetDuty(cmd_packet.data.pwm_duty[(unsigned)PWM_ID::kRTank],PWM_ID::kRTank);
        }
        if(cmd_packet.flags.pwm_duty[(unsigned)PWM_ID::kRKneExt]){
            Valves_hub::EnableCon(Valves_hub::Joint::kRKne,JointCon::ControlMode::kPreConExt);
            Valves_hub::SetDuty(cmd_packet.data.pwm_duty[(unsigned)PWM_ID::kRKneExt],PWM_ID::kRKneExt);
        }
        if(cmd_packet.flags.pwm_duty[(unsigned)PWM_ID::kRKneFlex]){
            Valves_hub::EnableCon(Valves_hub::Joint::kRKne,JointCon::ControlMode::kPreConFlex);
            Valves_hub::SetDuty(cmd_packet.data.pwm_duty[(unsigned)PWM_ID::kRKneFlex],PWM_ID::kRKneFlex);
        }
        if(cmd_packet.flags.pwm_duty[(unsigned)PWM_ID::kRAnkExt]){
            Valves_hub::EnableCon(Valves_hub::Joint::kRAnk,JointCon::ControlMode::kPreConExt);
            Valves_hub::SetDuty(cmd_packet.data.pwm_duty[(unsigned)PWM_ID::kRAnkExt],PWM_ID::kRAnkExt);
        }
        if(cmd_packet.flags.pwm_duty[(unsigned)PWM_ID::kRAnkFlex]){
            Valves_hub::EnableCon(Valves_hub::Joint::kRAnk,JointCon::ControlMode::kPreConFlex);
            Valves_hub::SetDuty(cmd_packet.data.pwm_duty[(unsigned)PWM_ID::kRAnkFlex],PWM_ID::kRAnkFlex);
        }
    }
    //force control
    if(this->CheckCmdSet(cmd_packet.flags.des_force.begin(),cmd_packet.flags.des_force.size())){
        if(cmd_packet.flags.des_force[(unsigned)Valves_hub::Joint::kLKne]){
            Valves_hub::EnableCon(Valves_hub::Joint::kLKne,JointCon::ControlMode::kForceCon);
            Valves_hub::SetDesiredForce(Valves_hub::Joint::kLKne,cmd_packet.data.des_force[(unsigned)Valves_hub::Joint::kLKne]);
        }
        if(cmd_packet.flags.des_force[(unsigned)Valves_hub::Joint::kLAnk]){
            Valves_hub::EnableCon(Valves_hub::Joint::kLAnk,JointCon::ControlMode::kForceCon);
            Valves_hub::SetDesiredForce(Valves_hub::Joint::kLAnk,cmd_packet.data.des_force[(unsigned)Valves_hub::Joint::kLAnk]);
        }
        if(cmd_packet.flags.des_force[(unsigned)Valves_hub::Joint::kRKne]){
            Valves_hub::EnableCon(Valves_hub::Joint::kRKne,JointCon::ControlMode::kForceCon);
            Valves_hub::SetDesiredForce(Valves_hub::Joint::kRKne,cmd_packet.data.des_force[(unsigned)Valves_hub::Joint::kRKne]);
        }
        if(cmd_packet.flags.des_force[(unsigned)Valves_hub::Joint::kRAnk]){
            Valves_hub::EnableCon(Valves_hub::Joint::kRAnk,JointCon::ControlMode::kForceCon);
            Valves_hub::SetDesiredForce(Valves_hub::Joint::kRAnk,cmd_packet.data.des_force[(unsigned)Valves_hub::Joint::kRAnk]);
        }
    }
    //impedance control
    if(this->CheckCmdSet(cmd_packet.flags.des_imp.begin(),cmd_packet.flags.des_imp.size())){
        if(cmd_packet.flags.des_imp[(unsigned)Valves_hub::Joint::kLKne]){
            Valves_hub::EnableCon(Valves_hub::Joint::kLKne,JointCon::ControlMode::kImpCon);
            Valves_hub::SetDesiredImp(Valves_hub::Joint::kLKne,cmd_packet.data.des_imp[(unsigned)Valves_hub::Joint::kLKne]);
        }
        if(cmd_packet.flags.des_imp[(unsigned)Valves_hub::Joint::kLAnk]){
            Valves_hub::EnableCon(Valves_hub::Joint::kLAnk,JointCon::ControlMode::kImpCon);
            Valves_hub::SetDesiredImp(Valves_hub::Joint::kLAnk,cmd_packet.data.des_imp[(unsigned)Valves_hub::Joint::kLAnk]);
        }
        if(cmd_packet.flags.des_imp[(unsigned)Valves_hub::Joint::kRKne]){
            Valves_hub::EnableCon(Valves_hub::Joint::kRKne,JointCon::ControlMode::kImpCon);
            Valves_hub::SetDesiredImp(Valves_hub::Joint::kRKne,cmd_packet.data.des_imp[(unsigned)Valves_hub::Joint::kRKne]);
        }
        if(cmd_packet.flags.des_imp[(unsigned)Valves_hub::Joint::kRAnk]){
            Valves_hub::EnableCon(Valves_hub::Joint::kRAnk,JointCon::ControlMode::kImpCon);
            Valves_hub::SetDesiredImp(Valves_hub::Joint::kRAnk,cmd_packet.data.des_imp[(unsigned)Valves_hub::Joint::kRAnk]);
        }
    }

    //update robot's time
    if(cmd_packet.flags.epoch_time){
        timeval time;
        int input_usec = cmd_packet.data.epoch_time-floor(cmd_packet.data.epoch_time)*1000000;
        time.tv_sec = cmd_packet.data.epoch_time;
        time.tv_usec = input_usec;
        settimeofday(&time,NULL);
    }

    //enable recorder, disable recorder
    if(cmd_packet.flags.recorder){
        if(cmd_packet.data.recorder){
            //enable recorder
            Timer::StartRec();
            std::cout<<"start recording\n";
        }
        else{
            //disable recorder
            Timer::EndRec();
            std::cout<<"stop recording\n";
        }
    }
    //

}