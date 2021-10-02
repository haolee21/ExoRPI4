#ifndef TCP_SERVER_HPP
#define TCP_SERVER_HPP
#include <boost/asio.hpp>
#include <iostream>
#include <memory>
#include <functional>
#include <thread>
#include <cstring>
#include <SensorHub.hpp>
#include <Valves_hub.hpp>
#include <functional>

#define TCP_PORT 1234
class TCP_server
{
private:
    //Boost TCP handler, ref: https://nanxiao.gitbooks.io/boost-asio-network-programming-little-book/content/posts/asynchronous-read-write-operations.html
    //Reading
    static void RecvCallback(const boost::system::error_code& error,
                         std::size_t recv_len,
                         char recv_str[],std::string &ret_str);
    boost::asio::io_context ioc;
    std::unique_ptr<boost::asio::ip::tcp::acceptor> acceptor;
    //Sending
    static void SendCallback(const boost::system::error_code& error,
                             std::size_t bytes_transferred,
                             std::shared_ptr<boost::asio::ip::tcp::socket> socket, //socket is not copyable so we have to use shared_ptr
                             std::string str);

    //recv loop
    bool flag;
    std::unique_ptr<std::thread> recv_th;
    void RecvCmd();
    

    //Cmd Handler
    std::string Sub_cmd(std::string cmd,std::size_t &idx1,char delim);
    void Cmd_handler(char[],size_t len);

    static void Send_cmd(std::string cmd,std::shared_ptr<boost::asio::ip::tcp::socket> socket);

    //Callback functions
    
    
public:
    TCP_server();
    ~TCP_server();
    
    void Off();
    
};
#endif
