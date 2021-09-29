#include "Teensy.hpp"

TeensyI2C::TeensyI2C(int i2c_idx)
{
    std::stringstream ss;
    ss << i2c_idx;
    std::string i2c_port = "/dev/i2c-" + ss.str();
    this->fd = open(i2c_port.c_str(), O_RDWR);
    if (fd < 0)
        std::cout << "Error opening i2c port " << ss.str() << std::endl;
    if (ioctl(fd, I2C_SLAVE, VALVE_CON_ADDRESS) < 0)
        printf("ioctl error: %s\n", strerror(errno));
}
TeensyI2C::~TeensyI2C()
{   
    std::cout<<"TeensyI2C closes\n";
    close(this->fd);
}



void TeensyI2C::WriteCmd(const std::array<char,TeensyI2C::CMDLEN>&cmd){
    if(write(this->fd,&cmd,TeensyI2C::CMDLEN)!=TeensyI2C::CMDLEN) //write length is different from the requirement (has to be identical to pwm_num+val_num)
        std::cout<<"I2C error writing\n";
}