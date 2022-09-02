#include "JointCon.hpp"
JointCon::JointCon(PneumaticParam::CylinderParam cyln_param,PneumaticParam::ReservoirParam reservoir_param)
    : ext_con(cyln_param.cl_ext,cyln_param.ch_ext),flex_con(cyln_param.cl_flex,cyln_param.ch_flex), tank_con(reservoir_param.cl,reservoir_param.ch),
      piston_area_ext(cyln_param.piston_area_ext), piston_area_flex(cyln_param.piston_area_flex), fric_coeff(cyln_param.fri_coeff), max_pos(cyln_param.max_pos), vel_filter(FilterParam::Filter20Hz_2::a, FilterParam::Filter20Hz_2::b), force_filter(FilterParam::Filter20Hz_2::a, FilterParam::Filter20Hz_2::b)
{
    this->max_len_mm = this->GetLenLinear_mm(this->max_pos);
}

JointCon::~JointCon()
{
}

void JointCon::PushMeas(const double &p_joint_ext, const double &p_joint_flex, const double &p_tank, const double &p_main_tank, const u_int8_t &ext_duty, const u_int8_t &flex_duty, const u_int8_t &tank_duty, const double &pos)
{
    this->ext_con.PushMeas(p_tank,p_joint_ext,ext_duty);
    this->flex_con.PushMeas(p_joint_ext,p_joint_flex,flex_duty);
    this->tank_con.PushMeas(p_main_tank,p_tank,tank_duty);
    
    this->pre_tank = p_tank;
    this->pre_main_tank = p_main_tank;
    this->pos_diff = pos - this->pre_pos;
    this->pre_pos = pos;
    this->pre_flex = p_joint_flex;
    this->pre_ext = p_joint_ext;
    this->cur_pos = pos;
    this->cur_max_spring_compress = (p_joint_ext * this->piston_area_ext - p_joint_flex * this->piston_area_flex) * 2.1547177056884764e-05 / this->spring_k; // unit in mm, piston area=0 for air reservoir
    this->cur_delta_x = this->max_pos - pos;

    this->cur_force = this->GetExternalForce();
}
void JointCon::GetForceCon(const double des_force, u_int8_t &ext_duty, u_int8_t &flex_duty, u_int8_t &tank_duty)
{
    /**
     * @brief Get the required duty cycle based on the commanded force, return in joint_duty and bal_duty
     * To calculate the current force,
     * 1. If desired force > current force & pre_tank > pre_ext
     *    => increase pressure
     *    a. calculate desired pre_ext
     *    b. calculate required joint_duty, set bal_duty=0, tank_duty=0
     *
     * 2. If desired force > current force & pre_tank < pre_ext
     *    => increase tank pressure
     *    a. calculate desired p_ext, assigned pre_ext + const to pre_tank
     *       (note: the const shouldn't be too high, otherwise we will have trouble recycleing energy later)
     *    b. calculate tank_duty, set joint_duty=0, bal_duty=0
     *
     * 3. If desired force < current force & pre_tank>pre_ext
     *    => reduce pre_ext-pre_flex
     *    a. calcualte desired (p_ext-p_flex), based on the volume ratio between side of the cylinder and pos_diff, calculate p_ext, p_flex
     *    b. calculate bal_duty, set joint_duty=0, tank_duty=0
     * 4. If desired force < current force & pre_tank<pre_ext
     *    => reduce pre_ext by energy recycling
     *    a. calculate desired pre_ext
     *    b. calculate joint_duty, set tank_duty=0, bal_duty=0
     */

    if ((des_force > this->cur_force) && (this->pre_tank > pre_ext))
    {
        double des_pre = ((des_force + this->fric_coeff * this->pos_diff) / 2.1547177056884764e-05 + this->pre_flex * this->piston_area_flex) / this->piston_area_ext;
        ext_duty = this->ext_con.GetPreControl(des_pre, this->pre_ext, this->pre_tank, this->GetLenLinear_mm(this->cur_pos) / this->max_len_mm);
        flex_duty = 0;
        tank_duty = 0;
    }
    else if ((des_force > this->cur_force) && (this->pre_tank < this->pre_ext))
    {
        double des_pre = ((des_force + this->fric_coeff * this->pos_diff) / 2.1547177056884764e-05 + this->pre_flex * this->piston_area_flex) / this->piston_area_ext;
        tank_duty = this->tank_con.GetPreControl(des_pre, pre_tank, pre_main_tank, 1);
        flex_duty = 0;
        ext_duty = 0;
    }
    else if ((des_force < this->cur_force) && (this->pre_tank > this->pre_ext))
    {
        double des_PA = des_force + this->fric_coeff * this->pos_diff; // pre_ext*area_ext - pre_flex*area_flex
        // TODO: finish, this is actually a quadratic problem
    }
    else if ((des_force < this->cur_force) && (this->pre_tank < this->pre_ext))
    {
        double des_pre = ((des_force + this->fric_coeff * this->pos_diff) / 2.1547177056884764e-05 + this->pre_flex * this->piston_area_flex) / this->piston_area_ext;
        ext_duty = this->ext_con.GetPreControl(des_pre, this->pre_ext, this->pre_tank, this->GetLenLinear_mm(this->cur_pos) / this->max_len_mm);
        flex_duty = 0;
        tank_duty = 0;
    }
}

double JointCon::GetExternalForce()
{

    if ((this->cur_delta_x - 4700) > this->cur_max_spring_compress / this->volume_slope_6in)
    { // It turned out the the spring start to compress earlier, the 4700 is an experimental value

        return (this->pre_ext * this->piston_area_ext - this->pre_flex * this->piston_area_flex) * 2.1547177056884764e-05 - this->fric_coeff * this->pos_diff; // unit newton
        // return (pre/65536*4.096-0.5)/4*200*0.31-0.001*this->pos_diff;
    }
    else
    {
        return (this->cur_delta_x) * this->volume_slope_6in * this->spring_k; // unit: newton
        // return (this->max_pos-x)*0.0006351973436310972*this->spring_k/25.4;
    }
}

double JointCon::GetLenLinear_mm(double pos)
{
    return pos * this->volume_slope_6in + this->volume_intercept_6in;
}

void JointCon::GetImpCon(const double des_imp, u_int8_t &ext_duty, u_int8_t &flex_duty, u_int8_t &tank_duty)
{
    // steps:
    //        1. calculate desired force based on current position
    //        2. calculate desired pressure based on current velocity and desired force
    double des_force = des_imp * this->cur_delta_x;
    this->GetForceCon(des_force, ext_duty, tank_duty, flex_duty);
}

void JointCon::GetPreCon(const double des_pre, u_int8_t &duty, Chamber chamber)
{
    switch (chamber)
    {
    case JointCon::Chamber::kExt:
        duty = this->ext_con.GetPreControl(des_pre, this->pre_ext, this->pre_tank, this->GetLenLinear_mm(this->cur_pos) / this->max_len_mm);
        break;
    case JointCon::Chamber::kFlex:
        break;
    case JointCon::Chamber::kTank:
        duty = this->tank_con.GetPreControl(des_pre, this->pre_tank, this->pre_main_tank, 1);
        break;

    default:
        duty = 0;
    }
}

void JointCon::SetControlMode(JointCon::ControlMode con_mode){
    this->control_mode = con_mode;
}
const JointCon::ControlMode JointCon::GetControlMode(){
    return this->control_mode;

}