#include "TCP_server.hpp"
TCP_server::TCP_server()
{
    this->flag=true;
    std::cout<<"SYS:TCP_server:init starts\n";
    this->acceptor.reset(new boost::asio::ip::tcp::acceptor(this->ioc,boost::asio::ip::tcp::endpoint(boost::asio::ip::tcp::v4(),TCP_PORT)));
    
    this->recv_th.reset(new std::thread(&TCP_server::RecvCmd,this));
    
}
void TCP_server::Off(){
    this->flag = false;
    
}
TCP_server::~TCP_server()
{
    this->Off(); //turn itself off when finished
    std::cout<<"SYS:TCP_server:Thread join starts, remember it has to be connected to a client when it ends, otherwise it will stuck in loops\n";
    this->recv_th->join();
    std::cout<<"SYS:TCP_server:Thread join\n";
}
void TCP_server::RecvCallback(const boost::system::error_code& error,
                         std::size_t recv_len,
                         char recv_str[],std::string &ret_str)
{
    if(error)
    {
        std::cout<<"SYS:TCP_server:"<<error.message()<<'\n';
    }
    else{
        ret_str = std::string(recv_str,recv_str+recv_len);
        
    }
}
void TCP_server::RecvCmd(){
    
    
    while(this->flag){
        
        std::shared_ptr<boost::asio::ip::tcp::socket> socket(new boost::asio::ip::tcp::socket(this->ioc));
        this->acceptor->accept(*socket);
        
        char recv_str[1024]={};
        std::string ret_str{""};
        socket->async_receive(boost::asio::buffer(recv_str),
                         std::bind(&TCP_server::RecvCallback,std::placeholders::_1,std::placeholders::_2,recv_str,std::ref(ret_str)));
        
        this->ioc.run();
        this->ioc.restart();
        //cmd interpret
        std::size_t cmd_idx=0;
        std::string cmd_class = this->Sub_cmd(ret_str,cmd_idx,':');
        std::string cmd_subClass = this->Sub_cmd(ret_str,cmd_idx,':');
        
        if(cmd_class.compare("REQ")==0){
            std::string cmd_device = Sub_cmd(ret_str,cmd_idx,'\n');
            if(cmd_subClass.compare("MEAS")==0){
                // std::string cmd_device = Sub_cmd(ret_str,cmd_idx,'\n');
                if(cmd_device.compare("DATA")==0){  
                    
                    std::array<char,(SensorHub::NUMENC+SensorHub::NUMPRE)*sizeof(double)> meaData;
                    const std::array<double,SensorHub::NUMENC> &encData=SensorHub::GetEncData();
                    std::memcpy(meaData.begin(),encData.begin(),sizeof(double)*encData.size());
                    const std::array<double,SensorHub::NUMPRE> &preData=SensorHub::GetPreData();
                    std::memcpy(meaData.begin()+encData.size()*sizeof(double),preData.begin(),sizeof(double)*preData.size());

                    // std::array<char,16> test_char;
                    // std::memcpy(test_char.begin(),preData.begin(),sizeof(double)*8);
                    // std::cout<<"test\n";
                    // std::cout<<(unsigned)meaData[18]<<','<<(unsigned)meaData[19]<<std::endl;
                    // std::cout<<(unsigned)test_char[6]<<','<<(unsigned)test_char[7]<<std::endl;
                    // std::cout<<preData[3]<<std::endl;
                
                    TCP_server::Send_cmd(std::string(meaData.begin(),meaData.end()),socket);
                }
            }
            else if(cmd_subClass.compare("REC")==0){
                // std::string cmd_device = Sub_cmd(ret_str,cmd_idx,'\n');
                if(cmd_device.compare("DATA")==0){
                    const bool &recFlag=Timer::GetDataRec_flag();
                    if(recFlag){
                        TCP_server::Send_cmd(std::string("1"),socket);
                    }
                    else{
                        TCP_server::Send_cmd(std::string("0"),socket);
                    }
                }
            }
            else if(cmd_subClass.compare("PWM")==0){
                // std::string cmd_device = Sub_cmd(ret_str,cmd_idx,'\n');
                if(cmd_device.compare("DUTY")==0){
                    const std::array<uint8_t,PWM_VAL_NUM> pwm_data = Valves_hub::GetDuty();
                    TCP_server::Send_cmd(std::string(pwm_data.begin(),pwm_data.end()),socket);
                }

            }
            else if(cmd_subClass.compare("CONT")==0){
                if(cmd_device.compare("MPC")==0){
                    const std::array<bool,NUM_OF_MPC> mpc_enable = Valves_hub::GetMpcCond();
                    TCP_server::Send_cmd(std::string(mpc_enable.begin(),mpc_enable.end()),socket);
                }

            }
        
            
            
        }
        // else if(cmd_class.compare("CON")==0){
        //     if(cmd_subClass.compare("STOP")==0){
        //         std::string cmd_device = Sub_cmd(ret_str,cmd_idx,'\n');
        //         if(cmd_device.compare("CONN")==0){
        //             this->flag=false;
        //         }
        //     }
        // }
        else if(cmd_class.compare("SET")==0){
            std::string cmd_device = Sub_cmd(ret_str,cmd_idx,':');
            if(cmd_subClass.compare("PWM")==0){
                uint8_t input = std::stoi(Sub_cmd(ret_str,cmd_idx,'\n'));
                
                if(cmd_device.compare("LKNE")==0) {
                    Valves_hub::SetDuty(input,Valves_hub::LKNEPRE);
                    TCP_server::Send_cmd(std::string("1"),socket);
                    }
                else if(cmd_device.compare("LANK")==0){
                    Valves_hub::SetDuty(input,Valves_hub::LANKPRE);
                    TCP_server::Send_cmd(std::string("1"),socket);
                    }
                else if(cmd_device.compare("LTANK")==0){
                    Valves_hub::SetDuty(input,Valves_hub::LTANKPRE);
                    TCP_server::Send_cmd(std::string("1"),socket);
                    }
                else if(cmd_device.compare("RKNE")==0){
                    Valves_hub::SetDuty(input,Valves_hub::RKNEPRE);
                    TCP_server::Send_cmd(std::string("1"),socket);
                    }
                else if(cmd_device.compare("RANK")==0){
                    Valves_hub::SetDuty(input,Valves_hub::RANKPRE);
                    TCP_server::Send_cmd(std::string("1"),socket);
                    }
                else if(cmd_device.compare("RTANK")==0){
                    Valves_hub::SetDuty(input,Valves_hub::RTANKPRE);
                    TCP_server::Send_cmd(std::string("1"),socket);
                    }
                else{
                    TCP_server::Send_cmd(std::string("0"),socket);
                }
                
            }
            else if(cmd_subClass.compare("PRE")==0){
                float input = std::stof(Sub_cmd(ret_str,cmd_idx,'\n'));
                double pre_val = (double)(input*320+8000);
                if(cmd_device.compare("LTANK")==0){
                    Valves_hub::SetDesiredPre(Valves_hub::PWM_ID::LTANKPRE,pre_val);
                    TCP_server::Send_cmd(std::string("1"),socket);
                    
                }
                else if(cmd_device.compare("RTANK")==0){
                    Valves_hub::SetDesiredPre(Valves_hub::PWM_ID::RTANKPRE,pre_val);
                    TCP_server::Send_cmd(std::string("1"),socket);
                    

                }
                else if(cmd_device.compare("LKNE")==0){
                    Valves_hub::SetDesiredPre(Valves_hub::PWM_ID::LKNEPRE,pre_val);
                    TCP_server::Send_cmd(std::string("1"),socket);
                    

                }
                else{
                    TCP_server::Send_cmd(std::string("0"),socket);
                }

            }
            else if(cmd_subClass.compare("IMP")==0){
                double input = std::stod(Sub_cmd(ret_str,cmd_idx,'\n'));
                if(cmd_device.compare("LKNE")==0){
                    Valves_hub::SetDesiredImp(Valves_hub::Joint::kLKne,input);
                    TCP_server::Send_cmd(std::string("1"),socket);
                }

            }
            else if(cmd_subClass.compare("MPOS")==0){
                if(cmd_device.compare("LKNE")==0){
                    Valves_hub::SetCylnMaxPos(Valves_hub::Joint::kLKne);
                }
                else if(cmd_device.compare("LANK")==0){
                    Valves_hub::SetCylnMaxPos(Valves_hub::Joint::kLAnk);
                }
                else if(cmd_device.compare("RKNE")==0){
                    Valves_hub::SetCylnMaxPos(Valves_hub::Joint::kRKne);
                }
                else if(cmd_device.compare("RANK")==0){
                    Valves_hub::SetCylnMaxPos(Valves_hub::Joint::kRAnk);
                }

            }
            else if(cmd_subClass.compare("REC")==0){
                
                if(cmd_device.compare("DATA")==0){
                    std::string input = Sub_cmd(ret_str,cmd_idx,'\n');
                    if(input.compare("1")==0){
                        Timer::StartRec();
                        TCP_server::Send_cmd("1",socket);
                        std::cout<<"Start to record\n";
                    }
                    else{
                        Timer::EndRec();
                        TCP_server::Send_cmd("1",socket);
                        std::cout<<"End recording\n";
                    }
                }
            }
            
            else{
                TCP_server::Send_cmd(std::string("0"),socket);
            }
        }
        else if(cmd_class.compare("ACT")==0){
            std::string cmd_device = Sub_cmd(ret_str,cmd_idx,':');
            std::string input = Sub_cmd(ret_str,cmd_idx,'\n');
            if(cmd_subClass.compare("MPC")==0){
                if(cmd_device.compare("LTANK")==0){
                    if(input.compare("1")==0){
                        Valves_hub::StartMPC(Valves_hub::PWM_ID::LTANKPRE,true);
                        
                        TCP_server::Send_cmd(std::string("1"),socket);
                    }
                    else{
                        Valves_hub::StartMPC(Valves_hub::PWM_ID::LTANKPRE,false);
                        Valves_hub::SetDuty(0,Valves_hub::PWM_ID::LTANKPRE);
                        TCP_server::Send_cmd(std::string("1"),socket);
                    }
                }
                else if(cmd_device.compare("RTANK")==0){
                    if(input.compare("1")==0){
                        Valves_hub::StartMPC(Valves_hub::PWM_ID::RTANKPRE,true);
                        TCP_server::Send_cmd(std::string("1"),socket);
                    }
                    else{
                        Valves_hub::StartMPC(Valves_hub::PWM_ID::RTANKPRE,false);
                        Valves_hub::SetDuty(0,Valves_hub::PWM_ID::RTANKPRE);
                        TCP_server::Send_cmd(std::string("1"),socket);
                    }
                }
                else if(cmd_device.compare("LKNE")==0){
                    if(input.compare("1")==0){
                        Valves_hub::StartMPC(Valves_hub::PWM_ID::LKNEPRE,true);
                        TCP_server::Send_cmd(std::string("1"),socket);
                    }
                    else{
                        Valves_hub::StartMPC(Valves_hub::PWM_ID::LKNEPRE,false);
                        Valves_hub::SetDuty(0,Valves_hub::PWM_ID::LKNEPRE);
                        TCP_server::Send_cmd(std::string("1"),socket);
                    }
                }

                
                else{
                    TCP_server::Send_cmd(std::string("0"),socket);
                }



            }
            else if(cmd_subClass.compare("IMP")==0){
                if(cmd_device.compare("LKNE")==0){
                    if(input.compare("1")==0){
                        Valves_hub::GetInstance().EnableImpCon(Valves_hub::Joint::kLKne,true);
                    }
                    else{
                        Valves_hub::GetInstance().EnableImpCon(Valves_hub::Joint::kLKne,false);
                    }
                    TCP_server::Send_cmd(std::string("1"),socket);
                }
                else if(cmd_device.compare("LANK")==0){
                    if(input.compare("1")==0){
                        Valves_hub::GetInstance().EnableImpCon(Valves_hub::Joint::kLAnk,true);
                    }
                    else{
                        Valves_hub::GetInstance().EnableImpCon(Valves_hub::Joint::kLAnk,false);
                    }
                }
                else if(cmd_device.compare("RKNE")==0){
                    if(input.compare("1")==0){
                        Valves_hub::GetInstance().EnableImpCon(Valves_hub::Joint::kRKne,true);
                    }
                    else{
                        Valves_hub::GetInstance().EnableImpCon(Valves_hub::Joint::kRKne,false);
                    }
                    TCP_server::Send_cmd(std::string("1"),socket);
                }
                else if(cmd_device.compare("RANK")==0){
                    if(input.compare("1")==0){
                        Valves_hub::GetInstance().EnableImpCon(Valves_hub::Joint::kRAnk,true);
                    }
                    else{
                        Valves_hub::GetInstance().EnableImpCon(Valves_hub::Joint::kRAnk,false);
                    }
                }
                else{
                    TCP_server::Send_cmd(std::string("0"),socket);
                }

            }
            else{
                TCP_server::Send_cmd(std::string("0"),socket);

            }


        }
    
        else if(cmd_class.compare("CAL")==0){
            if(cmd_subClass.compare("ENC")==0){
                std::string cmd_device = Sub_cmd(ret_str,cmd_idx,'\n');
                if(cmd_device.compare("LHIP_S")==0){
                    SensorHub::ResetEnc(SensorHub::EncName::LHipS);
                    std::cout<<"DEV::LHip_S encoder reset\n";
                    TCP_server::Send_cmd(std::string("1"),socket);
                }
                else if(cmd_device.compare("LKNE_S")==0){
                    SensorHub::ResetEnc(SensorHub::EncName::LKneS);
                    TCP_server::Send_cmd(std::string("1"),socket);
                    std::cout<<"DEV::LKne_S encoder reset\n";
                }
                else if(cmd_device.compare("LANK_S")==0){
                    SensorHub::ResetEnc(SensorHub::EncName::LAnkS);
                    TCP_server::Send_cmd(std::string("1"),socket);
                    std::cout<<"DEV::LAnk_S encoder reset\n";
                }
                else if(cmd_device.compare("RHIP_S")==0){
                    SensorHub::ResetEnc(SensorHub::EncName::RHipS);
                    TCP_server::Send_cmd(std::string("1"),socket);
                    std::cout<<"DEV::RHip_S encoder reset\n";
                }
                else if(cmd_device.compare("RKNE_S")==0){
                    SensorHub::ResetEnc(SensorHub::EncName::RKneS);
                    TCP_server::Send_cmd(std::string("1"),socket);
                    std::cout<<"DEV::RKne_S encoder reset\n";
                }
                else if(cmd_device.compare("RANK_S")==0){
                    SensorHub::ResetEnc(SensorHub::EncName::RAnkS);
                    TCP_server::Send_cmd(std::string("1"),socket);
                    std::cout<<"DEV::RAnk_S encoder reset\n";
                }
                else{
                    TCP_server::Send_cmd(std::string("0"),socket);
                }
                
            }
            if(cmd_subClass.compare("TIME")==0){
                std::string cmd_device = Sub_cmd(ret_str,cmd_idx,':');
                double inputs = std::stod(Sub_cmd(ret_str,cmd_idx,'\n'));
                if(cmd_device.compare("EPOCH")==0){
                    timeval time;
                    int input_usec = (inputs - floor(inputs))*1000000;
                    time.tv_sec = inputs;
                    time.tv_usec = input_usec;
                    settimeofday(&time,NULL);
                    std::cout<<"Set DateTime\n";
                    TCP_server::Send_cmd(std::string("1"),socket);
                }
            }
        }

        
        this->ioc.run();
        this->ioc.restart();


        
    }
    std::cout<<"SYS:TCP_server:Loop ends\n";
}
std::string TCP_server::Sub_cmd(std::string cmd,std::size_t &idx1,char delim){
    std::size_t idx2 = cmd.find(delim,idx1);
    std::string sub_cmd = cmd.substr(idx1,idx2-idx1);
    idx1 = idx2+1;
    return sub_cmd;
}

void TCP_server::SendCallback(const boost::system::error_code& error,
                             std::size_t bytes_transferred,
                             std::shared_ptr<boost::asio::ip::tcp::socket> socket,
                             std::string str)
{
    if(error)
        std::cout<<error.message()<<std::endl;
    else if (bytes_transferred == str.size()){
        
    }
    else{//recursive so we don't need to write another function to check byte_transferred
        socket->async_send(
            boost::asio::buffer(str.c_str()+bytes_transferred,str.size()-bytes_transferred),
            std::bind(&TCP_server::SendCallback,std::placeholders::_1,std::placeholders::_2,socket,str)
        );
    }
}

void TCP_server::Send_cmd(std::string reply,std::shared_ptr<boost::asio::ip::tcp::socket> socket){
    
    socket->async_send(boost::asio::buffer(reply),
                       std::bind(&TCP_server::SendCallback,std::placeholders::_1,std::placeholders::_2,socket,reply));

}
