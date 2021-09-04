#include "TCP_server.h"
using namespace boost::asio;
TCP_server::TCP_server()
{
    this->searching_flag=true;
    
}

TCP_server::~TCP_server()
{
}
bool TCP_server::Listen(){
    try{
        boost::asio::io_service ios;
        ip::tcp::acceptor acceptor (ios,ip::tcp::endpoint(ip::tcp::v4(),TCP_PORT));
        this->socket.reset(new ip::tcp::socket(ios));
        acceptor.accept(*socket.get());
    return true;
        
    }catch(const std::exception &e){
        return false;
    }
    
}