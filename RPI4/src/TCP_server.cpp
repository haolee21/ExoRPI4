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
    std::cout<<"SYS:TCP_server:Thread join starts\n";
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
            if(cmd_subClass.compare("MEAS")==0){
                std::string cmd_device = Sub_cmd(ret_str,cmd_idx,'\n');
                if(cmd_device.compare("DATA")==0){
                    //TODO: add callback to reply measurements
                    std::array<char,(SensorHub::NUMENC+SensorHub::NUMPRE)*sizeof(uint16_t)> meaData;
                    const std::array<u_int16_t,SensorHub::NUMENC> &encData=SensorHub::GetEncData();
                    std::memcpy(meaData.begin(),encData.begin(),sizeof(u_int16_t)*encData.size());
                    const std::array<u_int16_t,SensorHub::NUMPRE> &preData=SensorHub::GetPreData();
                    std::memcpy(meaData.begin()+encData.size(),preData.begin(),sizeof(u_int16_t)*preData.size());
                    TCP_server::Send_cmd(std::string(meaData.begin(),meaData.end()),socket);
                }
            }
        }
        else if(cmd_class.compare("ACT")==0){
            if(cmd_subClass.compare("STOP")==0){
                std::string cmd_device = Sub_cmd(ret_str,cmd_idx,'\n');
                if(cmd_device.compare("CONN")==0){
                    this->flag=false;
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
