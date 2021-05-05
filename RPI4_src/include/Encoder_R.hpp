#ifndef ENCODER_R_HPP
#define ENCODER_R_HPP
#include "Encoder.hpp"
class Encoder_R:public Encoder
{
private:
    static const int CEA=6;
    static const int CEB=4;
    static const int CEC=25;
    static const int SPI_IDX=3; //for some reason, even I enable spi5 (pin is correct), in /dev it will show up as spi3

    static const Pin CEA_pin;
    static const Pin CEB_pin;
    static const Pin CEC_pin;

    virtual void _setCE();
public:
    Encoder_R(int pinId);
    ~Encoder_R();
};


#endif