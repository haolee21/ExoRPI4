#ifndef ENCODER_L_HPP
#define ENCODER_L_HPP
#include "Encoder.hpp"
#include <mutex>
class Encoder_L:public Encoder
{
private:
    static const uint8_t CEA=5; //The gpio pin id of the CE's 
    static const uint8_t CEB=1;
    static const uint8_t CEC=0;
    static const int SPI_IDX=0;

    static const Pin CEA_pin;
    static const Pin CEB_pin;
    static const Pin CEC_pin;
    virtual void _setCE();
    // virtual void Lock();
    // virtual void Unlock();
    // static std::mutex lock;

public:
    Encoder_L(uint8_t pinId);
    ~Encoder_L();
    

};


#endif