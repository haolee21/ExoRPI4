#ifndef JOINT_CON_HPP
#define JOINT_CON_HPP
#include <array>
#include <osqp/osqp.h>
#include "MPC.hpp"
#include "DigitalFilter.hpp"
#include "CylinderParam.hpp"
#include "Recorder.hpp"
class JointCon
{
public:
    JointCon(PneumaticParam::CylinderParam cyln_param, PneumaticParam::ReservoirParam reservoir_param,std::string joint_con_name);
    ~JointCon();
    enum class Chamber
    {
        kExt,
        kFlex,
        kTank
    };
    enum class ControlMode
    {
        kNone,
        kPreConExt,
        kPreConFlex,
        kPreConTank,
        kForceCon,
        kImpCon
    };

    void GetForceCon(const std::array<double,MPC_TIME_HORIZON> &des_force, u_int8_t& ext_duty, u_int8_t &flex_duty, u_int8_t &tank_duty);
    void GetImpCon(const double des_imp, u_int8_t& ext_duty, u_int8_t& flex_duty, u_int8_t& tank_duty);
    void GetPreCon(const double des_pre, u_int8_t &duty, Chamber chamber); // Pressure control
    void PushMeas(const double &p_joint_ext,const double &p_joint_flex, const double &p_joint_rec, const double &p_tank, const double &p_main_tank, const u_int8_t& joint_duty, const u_int8_t &rec_duty, const u_int8_t &tank_duty, const double &pos);
    void RecData();
    void SetControlMode(ControlMode con_mode);
    const ControlMode GetControlMode();

private:
    MPC ext_con, flex_con, tank_con;
    ControlMode control_mode = ControlMode::kNone;
    double cur_pos = 0;
    double pre_pos = 0;
    double pos_diff = 0;
    const double spring_k = 55.4 * 0.0393701 * 4.44822; // unit: N/mm
    double cur_max_spring_compress;
    const double piston_area_ext;
    const double piston_area_flex;
    double fric_coeff;
    double max_pos;
    double max_len_mm;
    double cur_delta_x; // the difference between max_pos - cur_x
    double pre_ext;
    double pre_rec;
    double pre_tank;
    double pre_main_tank;
    double cur_force;                                       // unit: N
    double L_ext;                                      // unit: mm
    double L_flex;                                     // unit: mm
    
    const double volume_slope_6in = 0.0006351973436310972;  // FIXME: these are only used for linear calibrations
    const double volume_intercept_6in = 115.68133521647316; // unit: mm/adc(pos)

    // calculating current cylinder length, external force
    double GetExternalForce(double pre_ext, double pre_flex, double delta_x, double x_dot); // unit: newton
    double GetLenLinear_mm(double pos);

    // filter
    DigitalFilter<double, FilterParam::Filter20Hz_2::Order, 1> vel_filter;
    DigitalFilter<double, FilterParam::Filter15Hz_5::Order, 1> force_filter;


    // std::unique_ptr<OSQPSettings> osqp_settings;
    // std::unique_ptr<OSQPData> osqp_data;
    // OSQPWorkspace *work;

    double GetPre_KPa(double pre_adc);

    Recorder<double,6> joint_con_rec;

   


};

#endif