#ifndef PWM_HPP
#define PWM_HPP
#include<string>
class PWM
{
private:
    int duty;
    std::string name;
    
public:
    PWM(std::string name);
    ~PWM();
    void SetDuty(int);
    void Off();
    const int &GetDuty();
};

#endif //PWM_HPP
