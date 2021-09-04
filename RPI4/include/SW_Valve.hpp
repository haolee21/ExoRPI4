#ifndef SW_VALVE_HPP
#define SW_VALVE_HPP
#include <string>
class SW_Valve
{
private:
    bool condition=false;
    std::string name;
public:
    SW_Valve(std::string name);
    ~SW_Valve();
    const bool& GetValCond();
    void On();
    void Off();
};


#endif //SW_VALVE_HPP