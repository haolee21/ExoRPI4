#ifndef TEENSY_HPP
#define TEENSY_HPP
#include <linux/i2c-dev.h>
#include <fcntl.h>  /* For O_RDWR */
#include <unistd.h> /* For open(), */
#include <sys/ioctl.h>
#include <errno.h>
#include <sstream>
#include <cstring>
#include <iostream>
#include <array>
#include "TeensyCommon.h"
class TeensyI2C
{
public:

    TeensyI2C(int i2c_idx);
    ~TeensyI2C();
    const static int CMDLEN=PWM_VAL_NUM+SW_VAL_NUM;
    void WriteCmd(const std::array<char,TeensyI2C::CMDLEN>&cmd);
private:
    int fd;
    
    
};



#endif