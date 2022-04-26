// Reference from http://robotics.hobbizine.com/raspiduino.html
#ifndef ENCODER_HPP
#define ENCODER_HPP
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <fcntl.h>
#include <iostream>
#include <cstring>
#include "Pin.hpp"
#include <mutex>
#include <memory>
//index for CE of encoders


class Encoder
{
protected:
    void _initCE(uint8_t pinId); //set the chip select pin
    virtual void _setCE()=0; //this function will only be realize in Encoder_R and Encoder_L, so we will have to define it pure virtual
    virtual void Lock()=0; //lock of left/right encoders are different, however, I don't think I can create a virtual lock_guard
    virtual void Unlock()=0;
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
    Encoder(uint8_t pinId,int spi_num); //spi_num has to be int
    ~Encoder();
    void SetZero();
    int ReadPos();
    
};


#endif //ENCODERS