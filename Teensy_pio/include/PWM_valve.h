#ifndef PWM_VALVE_H
#define PWM_VALVE_H
#include <Arduino.h>
class PWM_valve
{
private:
    /* data */
    int duty;
    int duty_unit;
    bool curCond;
    

public:
    int idx;
    int pin_id;
    PWM_valve();
    PWM_valve(int,int,int);
    ~PWM_valve();
    void setDuty(int duty);
    bool operator >(PWM_valve &other_pwm){
        if(this->duty>other_pwm.duty){return true;}
        else{return false;}
    }
    bool operator <(PWM_valve &other_pwm){
        if(this->duty<other_pwm.duty){return true;}
        else{return false;}
    }
    bool operator ==(PWM_valve &other_pwm){
        if(this->duty == other_pwm.duty) {return true;}
        else {return false;}
    }
    bool operator >=(PWM_valve &other_pwm){
        if(this->duty>=other_pwm.duty){return true;}
        else{return false;}
    }
    bool operator <=(PWM_valve &other_pwm){
        if(this->duty<=other_pwm.duty){return true;}
        else{return false;}
    }
    void on();
    void off(); 
    int cal_off_t(int &pre_off_t);//when it turned off, it will return it's duty
               //the next valve will sleep duty-pre_duty time
    int GetDuty();
    
};
PWM_valve::PWM_valve() //this is for creating empty list
{
    this->curCond = true; //this flag was to avoid unnecessary valve on/off (already off, turn off again), however, we need to turn it off at the beginning
    this->off();

}
PWM_valve::PWM_valve(int idx,int _pin_id,int _duty_unit)
{
    this->idx = idx;
    this->pin_id = _pin_id;
    this->duty_unit = _duty_unit;
    pinMode(pin_id, OUTPUT);
}


PWM_valve::~PWM_valve()
{
}
void PWM_valve::setDuty(int _duty){
    this->duty = _duty;
}


void PWM_valve::on(){
    if((this->duty!=0) & (!this->curCond)){
    // if(this->duty!=0){
        digitalWrite(this->pin_id, true);
        this->curCond = true;
    }
}
void PWM_valve::off(){
    if(this->curCond){
        digitalWrite(this->pin_id, false);
        this->curCond = false;
    }
    
      
    
}
int PWM_valve::cal_off_t(int &pre_duty){
  int off_time = this->duty - pre_duty;
  pre_duty = this->duty;
  return off_time;
    

}
int PWM_valve::GetDuty(){
    return this->duty;
}
#endif
