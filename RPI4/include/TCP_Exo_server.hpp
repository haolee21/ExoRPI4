// #ifndef TCP_EXO_SERVER_HPP
// #define TCP_EXO_SERVER_HPP

// #include <sys/socket.h>
// #include <netinet/in.h>
// #include <unistd.h>
// #include <iostream>

// class TCP_Exo_server
// {
// private:
//     int sockfd,server_addr_len;
//     struct sockaddr_in server_addr;
// public:
//     TCP_Exo_server(int port);
//     ~TCP_Exo_server();

//     int ListenClient(char *rx_buf,int rx_buf_len);
//     int ReplyClient(char *tx_buf,int tx_buf_len,int client_sock);
// };
// #endif
// TCP_Exo_server::TCP_Exo_server(int port)
// {
//     // Everything in linux is a file, we need a file descriptor "sockfd"
//     if((this->sockfd=socket(AF_INET,SOCK_STREAM,0))==0){
//         perror("TCP Socket Init");
//         exit(EXIT_FAILURE);
//     }
//     // Attach socket to port
//     this->server_addr.sin_family = AF_INET;
//     this->server_addr.sin_port = htons(port);
//     this->server_addr.sin_addr.s_addr = INADDR_ANY;

//     if(bind(this->sockfd,(struct sockaddr*)&this->server_addr,sizeof(this->server_addr))<0){
//         perror("TCP Socket/Port binding failed");
//         exit(EXIT_FAILURE);
//     }
//     if(listen(this->sockfd,5)<0){
//         perror("TCP Server listen failed");
//         exit(EXIT_FAILURE);
//     }
// }
// TCP_Exo_server::~TCP_Exo_server()
// {
// }


