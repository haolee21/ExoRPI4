#ifndef ENCODER_R_HPP
#define ENCODER_R_HPP
#include "Encoder.hpp"

class Encoder_R:public Encoder
{
private:
    static const uint8_t CEA=6; //The gpio pin id of the CE's 
    static const uint8_t CEB=4;
    static const uint8_t CEC=25;
    static const int SPI_IDX=3; //for some reason, even I enable spi5 (pin is correct), in /dev it will show up as spi3

    static const Pin CEA_pin;
    static const Pin CEB_pin;
    static const Pin CEC_pin;

    virtual void _setCE();
    virtual void Lock();
    virtual void Unlock();
    static std::mutex lock;
public:
    Encoder_R(uint8_t pinId);
    ~Encoder_R();



    
};


#endif