#ifndef ENCODER_L_HPP
#define ENCODER_L_HPP
#include "Encoder.hpp"
class Encoder_L:public Encoder
{
private:
    static const int CEA=5;
    static const int CEB=1;
    static const int CEC=0;
    static const int SPI_IDX=0;

    static const Pin CEA_pin;
    static const Pin CEB_pin;
    static const Pin CEC_pin;
    virtual void _setCE();
public:
    Encoder_L(int pinId);
    ~Encoder_L();

};


#endif