#ifndef ENCODER_L_HPP
#define ENCODER_L_HPP
#include "Encoder.hpp"
class Encoder_L:public Encoder
{
private:
    static const uint8_t CEA=5;
    static const uint8_t CEB=1;
    static const uint8_t CEC=0;
    static const int SPI_IDX=0;

    static const Pin CEA_pin;
    static const Pin CEB_pin;
    static const Pin CEC_pin;
    virtual void _setCE();
public:
    Encoder_L(uint8_t pinId);
    ~Encoder_L();
    static const uint8_t HIP1 = 0;
    static const uint8_t HIP2 = 1;
    static const uint8_t KNEE = 2;
    static const uint8_t ANK1 = 3;
    static const uint8_t ANK2 = 4;

};


#endif