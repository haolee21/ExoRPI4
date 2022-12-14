
#include "Encoder_R.hpp"
// std::mutex Encoder_R::lock = std::mutex();

Encoder_R::Encoder_R(uint8_t pinId)
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


// void Encoder_R::Lock(){
//     Encoder_R::lock.lock();
//     // mtx_R.lock();
// }
// void Encoder_R::Unlock(){
//     Encoder_R::lock.unlock();
//     // mtx_R.unlock();
// }   


