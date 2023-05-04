#ifndef FSM_HPP
#define FSM_HPP
#include <Eigen/Dense>

#include "Recorder.hpp"
#include "DigitalFilter.hpp"
#include "FilterParam.hpp"
#include "SensorHub.hpp"

class FSM
{

public:
    enum class State{
        kLeftAnkPushOff,
        kLeftToeOff,
        kDDLeftFrontRightRear, //double support with left leg front
        kRightAnkPushOff,
        kRightToeOff,
        kDDRightFrontLeftRear,
        kTurnOff,//we need a special state to end all the valve actuation during fsm, but only once 
        kNone
    };
    
    FSM(const FSM&)=delete;
    static FSM& GetInstance();
    ~FSM();
    static void Update();
    static const State GetFSM_State();

    static void FSM_LeftStart();
    static void FSM_RightStart();
    static void TurnOffFSM();
    static void SetNetualPos();
    
    //FSM control params
    static void GetLKneImpParams(double &imp,double &neutral_pos,double &init_force);
    static void GetRKneImpParams(double &imp,double &neutral_pos,double &init_force);
    static double GetLAnkPreParams();
    static double GetRAnkPreParams();
    static void SetImpParams(const double left_swing_left_load_ratio,const double right_swing_right_load_ratio, const double l_kne_imp,const double r_kne_imp,const double l_kne_initF,const double r_kne_initF,const double l_ank_idle_pre,const double r_ank_idle_pre);
    
private:
    FSM(/* args */);
    State cur_state;
    Recorder<double,10> fsm_rec;


    const std::array<double,SensorHub::NUMENC> &joint_pos;
    const std::array<double,SensorHub::NUMENC> &joint_diff;
    const double kVelTh = 0.01; //if less than  1/100 deg/sec, consider stop
                                                                          

    double l_hip_s_neu,l_kne_s_neu,l_ank_s_neu,r_hip_s_neu,r_kne_s_neu,r_ank_s_neu;
    


    double l_kne_s_imp,l_ank_s_imp,r_kne_s_imp,r_ank_s_imp;
    double l_kne_s_initF,r_kne_s_initF;

    double l_ank_s_idle_p,r_ank_s_idle_p;

    
    Eigen::Vector2d exo_momentum;
    double min_y_momentum,max_y_momentum,pre_y_momentum_loss;
    bool momentum_erase=true;


};

#endif