
#include "Encoder_R.hpp"


Encoder_R::Encoder_R(int pinId)
: Encoder(pinId,Encoder_R::SPI_IDX)
{
}

Encoder_R::~Encoder_R()
{
}
const Pin Encoder_R::CEA_pin = Pin(Encoder_R::CEA,Pin::IO_TYPE::Output);
const Pin Encoder_R::CEB_pin = Pin(Encoder_R::CEB,Pin::IO_TYPE::Output);
const Pin Encoder_R::CEC_pin = Pin(Encoder_R::CEC,Pin::IO_TYPE::Output);
void Encoder_R::_setCE()
{
    if (this->pinA)
        Encoder_R::CEA_pin.On();
    else
        Encoder_R::CEA_pin.Off();
    if (this->pinB)
        Encoder_R::CEB_pin.On();
    else
        Encoder_R::CEB_pin.Off();
    if (this->pinC)
        Encoder_R::CEC_pin.On();
    else
        Encoder_R::CEC_pin.Off();
}





