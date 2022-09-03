#ifndef UDP_SERVER_HPP
#define UDP_SERVER_HPP
#include <arpa/inet.h>
#include <sys/socket.h>
#include <cstring>
#include <memory>
#include <mutex>
#include "UDP_packet.hpp"
#include "Timer.hpp"
#include "Valves_hub.hpp"
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
    
    void ProcessCmd(UDP_CmdPacket cmd_packet);
public:
    UdpServer(/* args */);
    ~UdpServer();
};


#endif