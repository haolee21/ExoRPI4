#ifndef UDP_SERVER_HPP
#define UDP_SERVER_HPP
#include <arpa/inet.h>
#include <sys/socket.h>
#include <cstring>
#include <memory>
#include <mutex>
#include <thread>
#include <iostream>

#include <unistd.h>
#include "UDP_packet.hpp"

// #include "Timer.hpp"
// #include "Valves_hub.hpp"
class UdpServer
{
private:
    int sockfd_tx,sockfd_rx,sockfd_rx_len;
    struct sockaddr_in addr_rx,addr_tx;
    void ServerProcess();
    std::unique_ptr<std::thread> server_thread;

    void GetDataPacket(UDP_DataPacket &return_packet);
    bool CheckCmdSet(bool* array, int array_len);

    bool Recv(char* buf, int buf_len);
    void Send(char* buf, int buf_len);
    
public:
    UdpServer(/* args */);
    ~UdpServer();
};

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
    if(bind(this->sockfd_rx,(const struct sockaddr*)&this->addr_rx,sizeof(this->addr_rx))==-1){
        
        perror("bind failed");
        exit(EXIT_FAILURE);
    }
    //tx init
    if((this->sockfd_tx=socket(AF_INET,SOCK_DGRAM,IPPROTO_UDP))==-1){
        std::cout<<"tx bind failed\n";
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
        // std::cout<<"recv msg length is wrong\n";
        // std::cout<<"expect data length: "<<buf_len<<std::endl;
        return false;
    }
    else{
        // std::cout<<"got sth, now reply\n";
        inet_aton(inet_ntoa(this->addr_rx.sin_addr),&this->addr_tx.sin_addr);
        return true;
    }
    if(recv_len>0){
        std::cout<<"we got sth\n";
    }
}
void UdpServer::Send(char* buf, int buf_len){
    if(sendto(this->sockfd_tx,buf,buf_len,0,(struct sockaddr*)&this->addr_tx,sizeof this->addr_tx)==-1){
        perror("UDP Send Failed");
    }
}
void UdpServer::GetDataPacket(UDP_DataPacket &return_packet){
    // const std::array<double,SensorHub::NUMENC>& enc_data = SensorHub::GetEncData();
    // std::copy(enc_data.begin(),enc_data.end(),return_packet.enc_data.begin());
    // const std::array<double,SensorHub::NUMPRE>& pre_data = SensorHub::GetPreData();
    // std::copy(pre_data.begin(),pre_data.end(),return_packet.pre_data1.begin());
    
    // const std::array<u_int8_t,PWM_VAL_NUM>& pwm_duty = Valves_hub::GetDuty();
    // std::copy(pwm_duty.begin(),pwm_duty.end(),return_packet.pwm_duty.begin());

    // const std::array<bool,(unsigned)Valves_hub::Joint::kTotal> controller_cond = Valves_hub::GetControlCond();
    // std::copy(controller_cond.begin(),controller_cond.end(),return_packet.con_status.begin());
}   

UdpServer::~UdpServer()
{
    this->server_thread->join();
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
            std::cout<<"got something, reply\n";
            UDP_DataPacket data_packet;
            data_packet.enc_data = std::array<double,6>{1,2,3,4,5,6};
            data_packet.pwm_duty = std::array<uint8_t,16>{1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16};
            this->GetDataPacket(data_packet);
            this->Send((char*)&data_packet,sizeof data_packet);

        }

       
       
        //handle the commands
        
        //we check if there is any commands by summing the bool arrays, >0 means there is some commands
        //reset encoder
        // if(this->CheckCmdSet(cmd_packet.flags.reset_enc.begin(),cmd_packet.flags.reset_enc.size())){
        //     if(cmd_packet.flags.reset_enc[SensorHub::EncName::LHipS]){
        //         SensorHub::ResetEnc(SensorHub::EncName::LHipS);
        //     }
        //     else if(cmd_packet.flags.reset_enc[SensorHub::EncName::LKneS]){
        //         SensorHub::ResetEnc(SensorHub::EncName::LKneS);
        //     }
        //     else if(cmd_packet.flags.reset_enc[SensorHub::EncName::LAnkS]){
        //         SensorHub::ResetEnc(SensorHub::EncName::LAnkS);
        //     }
        //     else if(cmd_packet.flags.reset_enc[SensorHub::EncName::RHipS]){
        //         SensorHub::ResetEnc(SensorHub::EncName::RHipS);
        //     }
        //     else if(cmd_packet.flags.reset_enc[SensorHub::EncName::RKneS]){
        //         SensorHub::ResetEnc(SensorHub::EncName::RKneS);
        //     }
        //     else if(cmd_packet.flags.reset_enc[SensorHub::EncName::RAnkS]){
        //         SensorHub::ResetEnc(SensorHub::EncName::RAnkS);
        //     }
        // }
        // else if(this->CheckCmdSet(cmd_packet.flags.pwm_duty.begin(),cmd_packet.flags.pwm_duty.size())){
        //     if(cmd_packet.flags.pwm_duty[(unsigned)PWM_ID::kLTank]){
        //         Valves_hub::EnableCon(Valves_hub::Joint::kLKne,JointCon::ControlMode::kPreConTank);
        //         Valves_hub::SetDuty(cmd_packet.data.pwm_duty[(unsigned)PWM_ID::kLTank],PWM_ID::kLTank);
        //     }
        //     else if(cmd_packet.flags.pwm_duty[(unsigned)PWM_ID::kLKneExt]){
        //         Valves_hub::EnableCon(Valves_hub::Joint::kLKne,JointCon::ControlMode::kPreConExt);
        //         Valves_hub::SetDuty(cmd_packet.data.pwm_duty[(unsigned)PWM_ID::kLKneExt],PWM_ID::kLKneExt);
        //     }
        //     else if(cmd_packet.flags.pwm_duty[(unsigned)PWM_ID::kLKneFlex]){
        //         Valves_hub::EnableCon(Valves_hub::Joint::kLKne,JointCon::ControlMode::kPreConFlex);
        //         Valves_hub::SetDuty(cmd_packet.data.pwm_duty[(unsigned)PWM_ID::kLKneFlex],PWM_ID::kLKneFlex);
        //     }
        //     else if(cmd_packet.flags.pwm_duty[(unsigned)PWM_ID::kLAnkExt]){
        //         Valves_hub::EnableCon(Valves_hub::Joint::kLAnk,JointCon::ControlMode::kPreConExt);
        //         Valves_hub::SetDuty(cmd_packet.data.pwm_duty[(unsigned)PWM_ID::kLAnkExt],PWM_ID::kLAnkExt);
        //     }
        //     else if(cmd_packet.flags.pwm_duty[(unsigned)PWM_ID::kLAnkFlex]){
        //         Valves_hub::EnableCon(Valves_hub::Joint::kLAnk,JointCon::ControlMode::kPreConFlex);
        //         Valves_hub::SetDuty(cmd_packet.data.pwm_duty[(unsigned)PWM_ID::kLAnkFlex],PWM_ID::kLAnkFlex);
        //     }
        //     else if(cmd_packet.flags.pwm_duty[(unsigned)PWM_ID::kRTank]){
        //         Valves_hub::EnableCon(Valves_hub::Joint::kRKne,JointCon::ControlMode::kPreConTank);
        //         Valves_hub::SetDuty(cmd_packet.data.pwm_duty[(unsigned)PWM_ID::kRTank],PWM_ID::kRTank);
        //     }
        //     else if(cmd_packet.flags.pwm_duty[(unsigned)PWM_ID::kRKneExt]){
        //         Valves_hub::EnableCon(Valves_hub::Joint::kRKne,JointCon::ControlMode::kPreConExt);
        //         Valves_hub::SetDuty(cmd_packet.data.pwm_duty[(unsigned)PWM_ID::kRKneExt],PWM_ID::kRKneExt);
        //     }
        //     else if(cmd_packet.flags.pwm_duty[(unsigned)PWM_ID::kRKneFlex]){
        //         Valves_hub::EnableCon(Valves_hub::Joint::kRKne,JointCon::ControlMode::kPreConFlex);
        //         Valves_hub::SetDuty(cmd_packet.data.pwm_duty[(unsigned)PWM_ID::kRKneFlex],PWM_ID::kRKneFlex);
        //     }
        //     else if(cmd_packet.flags.pwm_duty[(unsigned)PWM_ID::kRAnkExt]){
        //         Valves_hub::EnableCon(Valves_hub::Joint::kRAnk,JointCon::ControlMode::kPreConExt);
        //         Valves_hub::SetDuty(cmd_packet.data.pwm_duty[(unsigned)PWM_ID::kRAnkExt],PWM_ID::kRAnkExt);
        //     }
        //     else if(cmd_packet.flags.pwm_duty[(unsigned)PWM_ID::kRAnkFlex]){
        //         Valves_hub::EnableCon(Valves_hub::Joint::kRAnk,JointCon::ControlMode::kPreConFlex);
        //         Valves_hub::SetDuty(cmd_packet.data.pwm_duty[(unsigned)PWM_ID::kRAnkFlex],PWM_ID::kRAnkFlex);
        //     }
        // }
        // //force control
        // else if(this->CheckCmdSet(cmd_packet.flags.des_force.begin(),cmd_packet.flags.des_force.size())){
        //     if(cmd_packet.flags.des_force[(unsigned)Valves_hub::Joint::kLKne]){
        //         Valves_hub::EnableCon(Valves_hub::Joint::kLKne,JointCon::ControlMode::kForceCon);
        //         Valves_hub::SetDesiredForce(Valves_hub::Joint::kLKne,cmd_packet.data.des_force[(unsigned)Valves_hub::Joint::kLKne]);
        //     }
        //     else if(cmd_packet.flags.des_force[(unsigned)Valves_hub::Joint::kLAnk]){
        //         Valves_hub::EnableCon(Valves_hub::Joint::kLAnk,JointCon::ControlMode::kForceCon);
        //         Valves_hub::SetDesiredForce(Valves_hub::Joint::kLAnk,cmd_packet.data.des_force[(unsigned)Valves_hub::Joint::kLAnk]);
        //     }
        //     else if(cmd_packet.flags.des_force[(unsigned)Valves_hub::Joint::kRKne]){
        //         Valves_hub::EnableCon(Valves_hub::Joint::kRKne,JointCon::ControlMode::kForceCon);
        //         Valves_hub::SetDesiredForce(Valves_hub::Joint::kRKne,cmd_packet.data.des_force[(unsigned)Valves_hub::Joint::kRKne]);
        //     }
        //     else if(cmd_packet.flags.des_force[(unsigned)Valves_hub::Joint::kRAnk]){
        //         Valves_hub::EnableCon(Valves_hub::Joint::kRAnk,JointCon::ControlMode::kForceCon);
        //         Valves_hub::SetDesiredForce(Valves_hub::Joint::kRAnk,cmd_packet.data.des_force[(unsigned)Valves_hub::Joint::kRAnk]);
        //     }
        // }
        // //impedance control
        // else if(this->CheckCmdSet(cmd_packet.flags.des_imp.begin(),cmd_packet.flags.des_imp.size())){
        //     if(cmd_packet.flags.des_imp[(unsigned)Valves_hub::Joint::kLKne]){
        //         Valves_hub::EnableCon(Valves_hub::Joint::kLKne,JointCon::ControlMode::kImpCon);
        //         Valves_hub::SetDesiredImp(Valves_hub::Joint::kLKne,cmd_packet.data.des_imp[(unsigned)Valves_hub::Joint::kLKne]);
        //     }
        //     else if(cmd_packet.flags.des_imp[(unsigned)Valves_hub::Joint::kLAnk]){
        //         Valves_hub::EnableCon(Valves_hub::Joint::kLAnk,JointCon::ControlMode::kImpCon);
        //         Valves_hub::SetDesiredImp(Valves_hub::Joint::kLAnk,cmd_packet.data.des_imp[(unsigned)Valves_hub::Joint::kLAnk]);
        //     }
        //     else if(cmd_packet.flags.des_imp[(unsigned)Valves_hub::Joint::kRKne]){
        //         Valves_hub::EnableCon(Valves_hub::Joint::kRKne,JointCon::ControlMode::kImpCon);
        //         Valves_hub::SetDesiredImp(Valves_hub::Joint::kRKne,cmd_packet.data.des_imp[(unsigned)Valves_hub::Joint::kRKne]);
        //     }
        //     else if(cmd_packet.flags.des_imp[(unsigned)Valves_hub::Joint::kRAnk]){
        //         Valves_hub::EnableCon(Valves_hub::Joint::kRAnk,JointCon::ControlMode::kImpCon);
        //         Valves_hub::SetDesiredImp(Valves_hub::Joint::kRAnk,cmd_packet.data.des_imp[(unsigned)Valves_hub::Joint::kRAnk]);
        //     }

        // }
        
        

        




    }
}
#endif