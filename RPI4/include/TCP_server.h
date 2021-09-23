#ifndef TCP_SERVER_H
#define TCP_SERVER_H
#include <boost/asio.hpp>
#include <cstring>
#include <memory>
#include <thread>

#define TCP_PORT 1234
class TCP_server
{

public:
    TCP_server();
    ~TCP_server();
    void SendData(const char* msg,const unsigned len); 
    
    // bool StopListen(); //TODO need to add communication handler to search client or restart connection in a different thread, better be realtime
private:
    
    std::unique_ptr<boost::asio::ip::tcp::socket> socket;
    // void SearchAndConnect();
    bool flag_on;
    bool flag_isConnect;
    // bool Disconnect();
    bool Listen();


    std::unique_ptr<std::thread> thread_checkClient;
    void CheckConnLoop();
    const uint8_t CHECK_T_S=5;//check the connection every 5 second
    

    // interpret cmd from client
    std::unique_ptr<std::thread> thread_interpretCMD;
    void InterpretCMDLoop();
    const uint8_t CHECK_CMD_T_MS =  10;

    std::size_t Read(std::string &cmd);
    std::string Sub_cmd(std::string cmd, size_t &idx1,char delim);
    
};










#endif