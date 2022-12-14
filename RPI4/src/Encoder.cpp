#include "Encoder.hpp"
#include <chrono>
#include <sstream>

using namespace std;
Encoder::Encoder(uint8_t pinId, int spi_num)
{
    int ret = 0;

    this->_initCE(pinId);
    stringstream ss;
    ss << spi_num;
    string spi_port = "/dev/spidev" + ss.str() + ".0";
    this->fd = open(spi_port.c_str(), O_RDWR);
    // unsigned int speed = 1000000;
    unsigned speed = 800000;
    this->reset_encoder = false;
    this->reset_not_done = false;

    auto mode = SPI_MODE_0;
    ret = ioctl(this->fd, SPI_IOC_WR_MODE, &mode);
    if (ret == -1)
        cout << "SYS:ENCODER:spi" << spi_num << ":cannot set spi write mode\n";

    ret = ioctl(this->fd, SPI_IOC_RD_MODE, &mode);
    if (ret == -1)
        cout << "SYS:ENCODER:spi" << spi_num << ":cannot set spi read mode\n";

    uint8_t bits = 8;
    ret = ioctl(this->fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
    if (ret < 0)
        cout << "SYS:ENCODER:spi" << spi_num << ":cannot set bits per work\n";

    ret = ioctl(this->fd, SPI_IOC_RD_BITS_PER_WORD, &bits);
    if (ret < 0)
        cout << "SYS:ENCODER:spi" << spi_num << ":cannot set bits per work\n";

    ret = ioctl(this->fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
    if (ret < 0)
        cout << "SYS:ENCODER:spi" << spi_num << ":cannot set write speed\n";
    ret = ioctl(this->fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
    if (ret < 0)
        cout << "SYS:ENCODER:spi" << spi_num << ":cannot set read speed\n";

    int lsb_first = 0;
    ret = ioctl(this->fd, SPI_IOC_RD_LSB_FIRST, &lsb_first);
    if (ret < 0)
        cout << "SYS:ENCODER:spi" << spi_num << ":cannot set read MSB first\n";
    ret = ioctl(this->fd, SPI_IOC_WR_LSB_FIRST, &lsb_first);
    if (ret < 0)
        cout << "SYS:ENCODER:spi" << spi_num << ":cannot set write MSB first\n";
}

Encoder::~Encoder()
{
}
void Encoder::_initCE(uint8_t pinId)
{
    switch (pinId)
    {
    case 0:
        this->pinC = false;
        this->pinB = false;
        this->pinA = false;
        break;
    case 1:
        this->pinC = false;
        this->pinB = false;
        this->pinA = true;
        break;
    case 2:
        this->pinC = false;
        this->pinB = true;
        this->pinA = false;
        break;
    case 3:
        this->pinC = false;
        this->pinB = true;
        this->pinA = true;
        break;
    case 4:
        this->pinC = true;
        this->pinB = false;
        this->pinA = false;
        break;
    case 5:
        this->pinC = true;
        this->pinB = false;
        this->pinA = true;
        break;
    case 6:
        this->pinC = true;
        this->pinB = true;
        this->pinA = false;
        break;
    case 7:
        this->pinC = true;
        this->pinB = true;
        this->pinA = true;
        break;
    default:
        break;
    }
}

void Encoder::SetZero()
{
    // this->Lock();

    // this->Unlock();
    if (!this->reset_not_done)
    {
        this->reset_encoder = true;
    }
}

void Encoder::_SetZero()
{
    // this->_setCE();

    if (this->reset_not_done)
    {
        this->txBuf[0] = 0x00;
    }
    else
    {
        this->txBuf[0] = 0x70;
        this->reset_not_done = true;
        this->reset_encoder = false;
        std::cout << "start reset\n";
    }
    this->_spiTxRx(1);

    if (this->rxBuf[0] == 0x80)
    {
        this->reset_not_done = false;
        std::cout << "done reset\n";
    }

    // while (rxBuf[0] != 0x80)
    // {
    //     // usleep(5);
    //     this->txBuf[0]=0x00;
    //     this->_spiTxRx(1);
    // }
    // this->reset_encoder=false;
    // cout<<"done set zero\n";
}
char Encoder::_spiTxRx(unsigned int len)
{
    // normal spi protocal can take more than 1 byte, however, for our encoder, sending two bytes is not applicable, sending 1 byte at a time
    struct spi_ioc_transfer spi[len];
    memset(&spi, 0, sizeof(spi));

    for(int i=0;i<len;i++){
        spi[i].tx_buf = (unsigned long)(this->txBuf+i);
        spi[i].rx_buf = (unsigned long)(this->rxBuf+i);
        spi[i].len=1;
    }

    // spi->tx_buf = (unsigned long)this->txBuf;
    // spi->rx_buf = (unsigned long)this->rxBuf;
    // spi->len = 1;
    return ioctl(this->fd, SPI_IOC_MESSAGE(len), &spi); //I am not 100% sure why SPI_IOC_MESSAGE(1) works even if I send out two bytes.....
}

int Encoder::ReadPos()
{
    // this->Lock();
    // preset the CE pins
    // auto start = chrono::high_resolution_clock::now();
    

    this->_setCE();

    if (this->reset_encoder || this->reset_not_done)
    {
        this->_SetZero();
    }
    else
    {

        // check if there are still data in the buffer, will just read the buffer data

        // this->txBuf[0] = 0x00;
        // this->_spiTxRx(1);

        // if (this->rxBuf[0] == 0xA5)
        {
            // only send new read pos cmd if nothing left in the buffer
            // send read_pos
            this->txBuf[0] = 0x10;
            this->_spiTxRx(1);

            if (this->rxBuf[0] == 0x10)
            {
                // std::cout<<"got mea\n";
                this->txBuf[0] = 0x00;
                // this->txBuf[1] = 0x00;
                this->_spiTxRx(1);
                this->MSB = this->rxBuf[0];
                this->_spiTxRx(1);
                this->LSB = this->rxBuf[0];
                // return (int)(this->MSB << 8) + (int)this->LSB;
                // this is to avoid tasks pile up
                // when reset encoder, there might be up to 1 ms latency, the threads will pile up
                // however, the encoder value during reset is not important, we should just use the old value instead
            }

            // usleep(20);
            // The encoder has issues reading two bytes consecutively, you will have to send one byte per time.
        }
        // else
        // {
        //     this->txBuf[0] = 0x00;
        //     if (this->rxBuf[0] == 0xA5)
        //         std::cout << "buf clear\n";
        // }
        
    }
    // this->txBuf[0]=0x00;

    // while (this->rxBuf[0] != 0x10)
    // {
    //     // usleep(20);
    //     this->_spiTxRx(1);

    // }
    // std::chrono::time_point<std::chrono::high_resolution_clock> cur_time = chrono::high_resolution_clock::now();
    // // auto elapsed = chrono::high_resolution_clock::now() - start;
    // auto elapsed = cur_time-this->old_time;
    // long long dur_time = chrono::duration_cast<chrono::microseconds>(elapsed).count();
    // cout << "time: " << dur_time << endl;

    // this->old_time = cur_time;
    // this->Unlock();
    return (int)(this->MSB << 8) + (int)this->LSB;
}
