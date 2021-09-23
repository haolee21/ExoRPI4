#include "TCP_server.h"
#include "Timer.hpp"
using namespace boost::asio;
TCP_server::TCP_server()
{
    this->flag_on=true;
    this->flag_isConnect=false;
    this->thread_checkClient.reset(new std::thread(&TCP_server::CheckConnLoop,this));
    this->thread_interpretCMD.reset(new std::thread(&TCP_server::InterpretCMDLoop,this));
    
    
}

TCP_server::~TCP_server()
{
    this->flag_on=false;
    this->thread_interpretCMD->join();
    this->thread_checkClient->join();
    this->socket->close();

}
bool TCP_server::Listen(){
    try{
        
        io_service ios;
        ip::tcp::acceptor acceptor (ios,ip::tcp::endpoint(ip::tcp::v4(),TCP_PORT));
        this->socket.reset(new ip::tcp::socket(ios));
        acceptor.accept(*(this->socket));
    return true;
        
    }catch(const std::exception &e){
        return false;
    }
    
}

void TCP_server::CheckConnLoop(){
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    while(this->flag_on){

        if(!this->flag_isConnect){
            this->flag_isConnect = this->Listen();
        }

        t.tv_nsec += SEC*this->CHECK_T_S;
        Timer::Sleep(&t);
    }
}
void TCP_server::InterpretCMDLoop(){
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC,&t);
    while(this->flag_on){
        if(this->flag_isConnect){
            //consider the exceptions since it is possible that server disconnected but flag not updated
            std::string cmd;
            std::size_t cmd_len = this->Read(cmd);
            std::size_t cmd_idx=0;
            if(cmd_len!=0){
                std::string cmd_sub = this->Sub_cmd(cmd,cmd_idx,':');
                if(cmd_sub.compare("REQ")==0){
                    cmd_sub = this->Sub_cmd(cmd,cmd_idx,':');
                    if(cmd_sub.compare("MEAS")==0){
                        //TODO: return measurements' data
                    }
                    else if (cmd_sub.compare("CONT")==0)
                    {
                        //TODO: return control parameter(what functions are activated)
                    }
                    else{
                        //TODO: cmd_error
                    }
                    
                }
                


            }
        }
        t.tv_nsec+=MSEC*this->CHECK_CMD_T_MS;
        Timer::Sleep(&t);
    }
}

void TCP_server::SendData(const char* msg, const unsigned len){
    if(this->flag_isConnect){
        this->socket->send(buffer(msg,len));
    }
    //won't show any error if no client connected since it is possible the client disconnected
}
std::size_t TCP_server::Read(std::string &cmd){

    //check if there are bytes to read, ref: https://stackoverflow.com/questions/6748412/check-if-boostasio-buffer-data-is-present-before-read
    socket_base::bytes_readable command(true);
    this->socket->io_control(command);
    std::size_t bytes2Read = command.get();
    if(bytes2Read!=0){
        streambuf buf;
        read_until(*(this->socket),buf,'\n');//TODO: there is possibility that a cmd is sent without \n, need to take care of it
        cmd = buffer_cast<const char*>(buf.data());
    }
    else{
        cmd = "";
    }

    return bytes2Read;
}

std::string TCP_server::Sub_cmd(std::string cmd,std::size_t &idx1,char delim){
    std::size_t idx2= cmd.find(delim,idx1);
    std::string sub_cmd = cmd.substr(idx1,idx2-idx1);
    idx1 = idx2+1;
    return sub_cmd;
}
