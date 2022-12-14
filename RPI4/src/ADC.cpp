
#include "ADC.hpp"
#include <chrono>
#include <bitset>
#include <sstream>
using namespace std;
const Pin ADC::conv = Pin(23, Pin::IO_TYPE::Output);
const Pin ADC::reset = Pin(22, Pin::IO_TYPE::Output);
const Pin ADC::eoc = Pin(24, Pin::IO_TYPE::Input);
ADC::ADC(short adc_idx)
{

    ADC::conv.On(); // start conversion is low-triggered

    ADC::reset.On(); // reset is also low-triggered, since both adc shares the same reset, set it high to avoid resetting another adc

    stringstream ss;
    ss << adc_idx;
    string spi_port = "/dev/spidev1." + ss.str();
    cout << "port: " << spi_port << endl;
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
    // static uint32_t speed = 40000000;
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
    // configure ADC

    // TODO: fix this, for some reason it never works
    // set ADC select channel 0
    //  this->txBuf[0] = 0b00000000;
    //  this->_spiTxRx(1);

    // set CFR, enable auto channel, tag mode, basically the default mode. Can be achieved by just reset both adc, yet leave the liberty to set CFR here for future use
    this->txBuf[0] = 0b11101110;
    this->txBuf[1] = 0b11111111;
    this->_spiTxRx(2);

    // read CFR settings
    this->txBuf[0] = 0b11000000;
    this->txBuf[1] = 0b00000000;
    this->_spiTxRx(2);

    std::bitset<8> b1(this->rxBuf[0]);
    std::bitset<8> b2(this->rxBuf[1]);
    cout << "current setting" << b1 << ',' << b2 << endl;
}

char ADC::_spiTxRx(unsigned int len)
{

    struct spi_ioc_transfer spi[1];

    memset(&spi, 0, sizeof(spi)); // I seriously cannot understand this line.......

    spi->tx_buf = (unsigned long)this->txBuf;
    spi->rx_buf = (unsigned long)this->rxBuf;
    spi->len = len;

    return ioctl(this->fd, SPI_IOC_MESSAGE(1), spi); // I am not 100% sure why SPI_IOC_MESSAGE(1) works even if I send out two bytes.....

    // since we have already assign rxData to spi.rx_buf, it will directly writes to it
}

const std::array<double, 8> &ADC::ReadData()
{
    this->txBuf[0] = 0b11010000;
    this->txBuf[1] = 0b00000000;
    this->txBuf[2] = 0b00000000;

    for (int i = 0; i < 8; i++)
    {

        ADC::conv.Off(); //conversion starts at falling edge, the duration needed is 40ns, which is shorter than sending 3 bytes from MOSI 

        this->_spiTxRx(3);
        ADC::conv.On();
        int tag = (int)this->rxBuf[2] >> 5;
        this->data[tag] = (short)this->rxBuf[1] + ((short)this->rxBuf[0] << 8);
    }

    return this->data;
}
