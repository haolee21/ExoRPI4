// Reference from http://robotics.hobbizine.com/raspiduino.html
#ifndef ENCODER_HPP
#define ENCODER_HPP
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <fcntl.h>
#include <iostream>
#include <string>
#include "Pin.hpp"
#include <cstring>

//index for CE of encoders


class Encoder
{
protected:
    void _initCE(int pinId); //set the chip select pin
    virtual void _setCE()=0; //this function will only be realize in Encoder_R and Encoder_L, so we will have to define it pure virtual
    bool pinA;
    bool pinB;
    bool pinC;

    
    int fd;
    char _spiTxRx(unsigned int len);

    char rxBuf[5];
    char txBuf[5];
    char MSB;
    char LSB;

public:
    Encoder(int pinId,int spi_num);
    ~Encoder();
    void SetZero();
    int ReadPos();
};


#endif //ENCODERS