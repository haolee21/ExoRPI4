#ifndef JOINT_CON_HPP
#define JOINT_CON_HPP
#include <array>
#include <osqp/osqp.h>
#include "MPC.hpp"
#include "SensorHub.hpp"
#include "DigitalFilter.hpp"
#include "CylinderParam.hpp"
#include "Recorder.hpp"
#include "ExoConfig.hpp"
#include "FSM.hpp"
class JointCon
{
public:
    JointCon(ExoConfig::MPC_Params knee_ext_params, ExoConfig::MPC_Params ank_ext_params, ExoConfig::MPC_Params kne_ank_params,
    ExoConfig::MPC_Params tank_params, ExoConfig::CylnPhyParams knee_cyln_params,ExoConfig::CylnPhyParams ank_cyln_params, std::string joint_con_name);
    ~JointCon();


    enum class ConMode
    {
        kNone,
        kPreCon,
        kForceCon,
        kImpCon,
        kTurnOff
    };
    enum class PreCon
    {
        kSubTank,
        kKneExt,
        kAnkPlant,
        kKneFlex,
        kAnkDorsi,
        kTotal
    };
    enum class ForceCon
    {
        kKneExt,
        kAnkPlant,
        kTotal
    };
    enum class ForceRedType //two ways to reduce the force output
    {
        kRec,
        kBalance
    };
    
    static const unsigned  kNumOfChambers = 3;

    void SetPreControl(PreCon pre_con_type,double des_pre);
    // void SetControl(ConMode con_mode,PreCon pre_con_type,double des_pre);
    void SetControl(ConMode con_mode, ForceCon force_con_type,ForceRedType force_red_type,double cmd_value);
    void SetImpControl(ForceCon _force_con_type, ForceRedType _force_red_type, double cmd_imp, double cmd_init_force);
    void SetImpControl(ForceCon _force_con_type, ForceRedType _force_red_type, double cmd_imp, double cmd_init_force,double neutral_knee_pos); //not doing imp control on ankle
    // void SetControl(ConMode con_mode,ForceCon force_con_type,ForceRedType force_red_type,double cmd_val1,double cmd_val2);
    void ResetControl();
    void ShutDown();

    
    
    void PushMeas(const double &p_knee_ext,const double &p_knee_flex,const double &p_ank_pla,const double &p_ank_dorsi, const double &p_sub_tank,const double &p_main_tank,const double &knee_angle,const double &ankle_angle,const u_int8_t knee_ext_duty,const u_int8_t knee_flex_duty,const u_int8_t ank_ext_duty, const u_int8_t knee_ank_duty,const u_int8_t tank_duty);
    // void PushMeas(const double &p_joint_ext,const double &p_joint_flex, const double &p_joint_rec, const double &p_tank, const double &p_main_tank,const double &pos,const u_int8_t tank_duty, const u_int8_t knee_ext_duty, const u_int8_t knee_flex_duty,const u_int8_t knee_ank_duty, const u_int8_t ank_ext_duty);
    void RecData();
    

    bool GetValveDuty(u_int8_t &knee_ext_duty,u_int8_t &knee_flex_duty, u_int8_t &ank_pla_duty,u_int8_t &ank_dor_duty, u_int8_t &sub_tank_duty,u_int8_t &knee_ank_duty);

    

private:
    const double kForceTol = 0.5; //if abs(force_err) < kForceTol, don't activate 
    // MPC ext_con, flex_con, tank_con;
    MPC knee_ext_con,ank_ext_con,knee_ank_con,tank_con;//knee_flex_con
    ExoConfig::CylnPhyParams knee_cyln_params,ank_cyln_params;

    ConMode con_mode = ConMode::kNone;
    PreCon pre_con_type;
    ForceCon force_con_type;
    ForceRedType force_red_type;

    std::array<double,(unsigned)PreCon::kTotal> cmd_pre;
    std::array<std::array<double,MPC_TIME_HORIZON>,(unsigned)ForceCon::kTotal> cmd_force;
    std::array<double,(unsigned)ForceCon::kTotal> cmd_imp;
    std::array<double,(unsigned)ForceCon::kTotal> cmd_init_force;
    std::array<double,(unsigned)ForceCon::kTotal> cmd_imp_tor;

    double cur_knee_ang,cur_ankle_ang;
    
    double knee_pos_diff,ank_pos_diff; //difference in position, if the joint is translational: dx, if the joint is revolute, dq
    double knee_cyln_ext_len,knee_cyln_shrk_len,ank_cyln_ext_len,ank_cyln_shrk_len;  //ext: extend, shrk: shrink, unit: mm
    double knee_moment_arm,ankle_moment_arm;
    double knee_cyln_len_diff, ank_cyln_len_diff; //for calculating frction
    double knee_len_ext_old,ank_len_ext_old;
    // const double spring_k = 55.4 * 0.0393701 * 4.44822; // unit: N/mm
    double max_knee_spring_compress,max_ank_spring_compress;

    double cur_knee_force, cur_ank_force; //unit: N
    
    double p_knee_ext,p_knee_flex,p_ank_pla,p_ank_dorsi,p_sub_tank,p_main_tank;
    
    // double cur_pre_force;
    // double des_force;
    // double des_ext_pre;

    void GetPreCon(double des_pre, u_int8_t &duty, PreCon pre_con);
    void GetForceCon(std::array<double,MPC_TIME_HORIZON> des_force,u_int8_t &charge_duty,u_int8_t &rec_duty,u_int8_t &balance_duty, u_int8_t &tank_duty, ForceCon force_con_type,ForceRedType force_red_type);
    void GetTorCon(std::array<double,MPC_TIME_HORIZON> des_tor,u_int8_t &charge_duty,u_int8_t &rec_duty,u_int8_t &balance_duty, u_int8_t &tank_duty, ForceCon force_con_type,ForceRedType force_red_type);
    void GetImpCon(double des_imp, double init_F, u_int8_t&charge_duty,u_int8_t&rec_duty,u_int8_t &balance_duty,u_int8_t&tank_duty,ForceCon force_con_type,ForceRedType force_red_type);




    // const double volume_slope_6in = 0.0006351973436310972;  // FIXME: these are only used for linear calibrations
    // const double volume_intercept_6in = 115.68133521647316; // unit: mm/adc(pos)


    //Energy recycled based imp control
    // we believe the impedance will peak in the middle, and reduce after it
    // e.g., 
    // Pos:      0 pos            mid pos                max pos
    // Imp:      5                5  (decrease linearly)    2             when compressing
    // Imp:      2                2                         2             when extending 

    //since the impdeance switches, we need a FSM to make sure it does not switch frequently between these two stats
    //logic:  velocity< -v_th, => MODE: kCompress
    //        velocity> +v_th => MODE: kExtend
    enum class Imp_FSM{
        kLoadPrep,kCompress_inc,kCompress_dec,kExtend,kFree
    };
    Imp_FSM imp_fsm_state;
    static constexpr double vel_th = 300;
    static constexpr double min_moment_arm = 5; //if the moment arm is <5mm, just lock 
    double neutral_knee_pos,neutral_ank_pos;
    // const double kExtImp=2; //   N/mm
    // double imp_deflect_point;
    double recover_imp;

    
    // calculating current cylinder length, external force
    double GetExternalForce(double pre_ext, double pre_flex, double delta_x, double x_dot,ExoConfig::CylnPhyParams cyln_params); // unit: newton
    
    // double GetLenLinear_mm(double pos);

    // filter
    DigitalFilter<double, FilterParam::Filter20Hz_2::Order, 2> vel_filter;
    DigitalFilter<double, FilterParam::Filter5Hz_2::Order, 2> force_filter;
    DigitalFilter<double,FilterParam::Filter5Hz_2::Order,1> force_pre_filter;

    DigitalFilter<double,FilterParam::Filter20Hz_2::Order,1> p_ext_rec_diff_filter;


    double GetPre_KPa(double pre_adc);

    Recorder<double,8> joint_con_rec;

   
   


};

#endif