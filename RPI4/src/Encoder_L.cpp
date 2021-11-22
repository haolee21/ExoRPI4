#include "Encoder_L.hpp"
std::mutex Encoder_L::lock = std::mutex();
Encoder_L::Encoder_L(uint8_t pinId)
:Encoder(pinId,Encoder_L::SPI_IDX)
{
}

Encoder_L::~Encoder_L()
{
}
const Pin Encoder_L::CEA_pin = Pin(Encoder_L::CEA,Pin::IO_TYPE::Output);
const Pin Encoder_L::CEB_pin = Pin(Encoder_L::CEB,Pin::IO_TYPE::Output);
const Pin Encoder_L::CEC_pin = Pin(Encoder_L::CEC,Pin::IO_TYPE::Output);
void Encoder_L::_setCE()
{
    if (this->pinA)
        Encoder_L::CEA_pin.On();
    else
        Encoder_L::CEA_pin.Off();
    if (this->pinB)
        Encoder_L::CEB_pin.On();
    else
        Encoder_L::CEB_pin.Off();
    if (this->pinC)
        Encoder_L::CEC_pin.On();
    else
        Encoder_L::CEC_pin.Off();
}
void Encoder_L::Lock(){
    Encoder_L::lock.lock();
}
void Encoder_L::Unlock(){
    Encoder_L::lock.unlock();
}