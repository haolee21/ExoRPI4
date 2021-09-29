
#include "ADC.hpp"
#include <chrono>
#include <bitset>
#include <sstream>
using namespace std;
const Pin ADC::conv = Pin(23,Pin::IO_TYPE::Output);
const Pin ADC::reset = Pin(22,Pin::IO_TYPE::Output);
const Pin ADC::eoc = Pin(24,Pin::IO_TYPE::Input);
ADC::ADC(short adc_idx)
{
    //initialize the channel list
    // for some weird reason,
    this->ch_list[0] = 0b00000000;
    this->ch_list[1] = 0b00010000;
    this->ch_list[2] = 0b00100000;
    this->ch_list[3] = 0b00110000;
    this->ch_list[4] = 0b01000000;
    this->ch_list[5] = 0b01010000;
    this->ch_list[6] = 0b01100000;
    this->ch_list[7] = 0b01110000;

    ADC::conv.On();

    ADC::reset.On();

    //reset the adc
    ADC::reset.Off();

    usleep(5);
    ADC::reset.On();

    usleep(5);
    //end reset

    usleep(1000);
    stringstream ss;
    ss<<adc_idx;
    string spi_port="/dev/spidev1."+ss.str();
    cout<<"port: "<<spi_port<<endl;
    this->fd = open(spi_port.c_str(), O_RDWR);
    std::cout << "spi port" << fd << std::endl;
    if (this->fd < 0)
        std::cout << "failed to open spi1\n";

    int ret = 0;
    auto mode = SPI_MODE_2;
    ret = ioctl(this->fd, SPI_IOC_WR_MODE, &mode);
    if (ret == -1)
        std::cout << "cannot set spi write mode\n";

    ret = ioctl(this->fd, SPI_IOC_RD_MODE, &mode);
    if (ret == -1)
        std::cout << "cannot set spi read mode\n";

    uint8_t bits = 8;
    ret = ioctl(this->fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
    if (ret < 0)
        std::cout << "cannot set bits per work\n";

    ret = ioctl(this->fd, SPI_IOC_RD_BITS_PER_WORD, &bits);
    if (ret < 0)
        std::cout << "cannot set bits per work\n";

    static uint32_t speed = 20000000;
    ret = ioctl(this->fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
    if (ret < 0)
        std::cout << "cannot set write speed\n";
    ret = ioctl(this->fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
    if (ret < 0)
        std::cout << "cannot set read speed\n";

    char lsb_first = 0x00;
    ret = ioctl(this->fd, SPI_IOC_RD_LSB_FIRST, &lsb_first);
    if (ret < 0)
        std::cout << "cannot set read MSB first\n";
    ret = ioctl(this->fd, SPI_IOC_WR_LSB_FIRST, &lsb_first);
    if (ret < 0)
        std::cout << "cannot set write MSB first\n";

    this->init_ADC();
}

ADC::~ADC()
{
}

void ADC::init_ADC()
{
    //configure ADC
    //set ADC select channel 0
    this->txBuf[0] = 0b00000000;
    this->_spiTxRx(1);

    //set manual channel select
    this->txBuf[0] = 0b11101110;
    this->txBuf[1] = 0b11111111;
    this->_spiTxRx(2);
    this->txBuf[0] = 0b11000000;
    this->txBuf[1] = 0b00000000;
    this->_spiTxRx(2);

    std::bitset<8> b1(this->rxBuf[0]);
    std::bitset<8> b2(this->rxBuf[1]);
    cout << "current setting" << b1 << ',' << b2 << endl;
}

char ADC::_spiTxRx(unsigned int len)
{

    struct spi_ioc_transfer spi[len];

    memset(&spi, 0, sizeof(spi)); //I seriously cannot understand this line.......

    spi->tx_buf = (unsigned long)this->txBuf;
    spi->rx_buf = (unsigned long)this->rxBuf;
    spi->len = len;

    return ioctl(this->fd, SPI_IOC_MESSAGE(1), spi);

    //since we have already assign rxData to spi.rx_buf, it will directly writes to it
}

const std::array<u_int16_t,8>& ADC::ReadData(){
    for (int i = 0; i < 8; i++)
    {
        // this->txBuf[0] = this->ch_list[0];
        this->txBuf[0] = 0b11010000;
        this->txBuf[1] = 0b11010000;
        
        ADC::conv.On();
  

        this->_spiTxRx(2);
        ADC::conv.Off();
 

        this->data[i] = (short)this->rxBuf[1] + ((short)this->rxBuf[0] << 8);
    }
    return this->data;

}
