#ifndef TCP_SERVER_H
#define TCP_SERVER_H
#include <boost/asio.hpp>
#include <cstring>
#include <memory>
#define TCP_PORT 1234
class TCP_server
{

public:
    TCP_server();
    ~TCP_server();
    
    // bool StopListen(); //TODO need to add communication handler to search client or restart connection in a different thread, better be realtime
private:
    
    std::unique_ptr<boost::asio::ip::tcp::socket> socket;
    // void SearchAndConnect();
    bool searching_flag;
    // bool Disconnect();
    bool Listen();

    
};










#endif