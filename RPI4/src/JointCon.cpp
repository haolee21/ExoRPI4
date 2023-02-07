#include <cmath>
#include <osqp/osqp.h>

#include "JointCon.hpp"

JointCon::JointCon(ExoConfig::MPC_Params knee_ext_params, ExoConfig::MPC_Params ank_ext_params, ExoConfig::MPC_Params knee_ank_params,
                   ExoConfig::MPC_Params tank_params, ExoConfig::CylnPhyParams knee_cyln_params, ExoConfig::CylnPhyParams ank_cyln_params, std::string joint_con_name)
    : knee_ext_con(knee_ext_params, joint_con_name + "_knee_ext"),
      // knee_flex_con(knee_flex_params, joint_con_name + "_knee_flex"),
      ank_ext_con(ank_ext_params, joint_con_name + "_ank_ext"), knee_ank_con(knee_ank_params, joint_con_name + "_knee_ank"), tank_con(tank_params, joint_con_name + "_tank"),
      knee_cyln_params(knee_cyln_params), ank_cyln_params(ank_cyln_params),
      vel_filter(FilterParam::Filter20Hz_2::a, FilterParam::Filter20Hz_2::b), force_filter(FilterParam::Filter5Hz_2::a, FilterParam::Filter5Hz_2::b), force_pre_filter(FilterParam::Filter5Hz_2::a, FilterParam::Filter5Hz_2::b),
      p_ext_rec_diff_filter(FilterParam::Filter20Hz_2::a, FilterParam::Filter20Hz_2::b),
      joint_con_rec(joint_con_name, "Time,L_ext,L_flex,cur_force,max_spring_compress,delta_x,x_dot,des_force,pre_force,des_ext_pre")
{
    this->imp_fsm_state = Imp_FSM::kLoadPrep;
    // //setup osqp solver
    // this->osqp_data.reset(new OSQPData);
    // this->osqp_settings.reset(new OSQPSettings);
    // osqp_set_default_settings(this->osqp_settings.get());
    // this->osqp_settings->alpha = 1.0;
    // this->osqp_settings->verbose = false;
}

JointCon::~JointCon()
{
}

void JointCon::SetControl(JointCon::ConMode _con_mode, PreCon _pre_con_type, double _cmd_pre)
{
    this->con_mode = _con_mode;
    this->pre_con_type = _pre_con_type;
    this->cmd_pre[(unsigned)_pre_con_type] = _cmd_pre;
}
void JointCon::SetControl(ConMode _con_mode, ForceCon _force_con_type, ForceRedType _force_red_type, double cmd_force)
{
    std::array<double, MPC_TIME_HORIZON> cmd_force_array;

    std::fill_n(cmd_force_array.begin(), cmd_force_array.size(), cmd_force);
    this->cmd_force[(unsigned)_force_con_type] = cmd_force_array;
    this->con_mode = _con_mode;
    this->force_con_type = _force_con_type;
    this->force_red_type = _force_red_type;
}
void JointCon::SetControl(ConMode _con_mode, ForceCon _force_con_type, ForceRedType _force_red_type, double cmd_imp, double cmd_init_force)
{
    this->cmd_init_force[(unsigned)_force_con_type] = cmd_init_force;
    this->cmd_imp[(unsigned)_force_con_type] = cmd_imp;
    this->con_mode = _con_mode;
    this->force_con_type = _force_con_type;
    this->force_red_type = _force_red_type;
}
void JointCon::ResetControl()
{
    this->con_mode = ConMode::kNone;
}
// void JointCon::SetKneeMaxPos(double max_pos_val)
// {
//     this->max_len_mm = this->GetLenLinear_mm(max_pos_val); // TODO: need to modify this function when integrating to the exo
//     // std::cout<<"max pos reset\n";
// }
// void JointCon::PushMeas(const double &p_joint_ext, const double &p_joint_flex, const double &p_joint_rec, const double &p_tank, const double &p_main_tank, const double &pos, const u_int8_t tank_duty, const u_int8_t knee_ext_duty, const u_int8_t knee_flex_duty, const u_int8_t knee_ank_duty, const u_int8_t ank_ext_duty)
void JointCon::PushMeas(const double &p_knee_ext, const double &p_knee_flex, const double &p_ank_ext, const double &p_sub_tank, const double &p_main_tank, const double &knee_angle, const double &ankle_angle, const u_int8_t knee_ext_duty, const u_int8_t knee_flex_duty, const u_int8_t ank_ext_duty, const u_int8_t knee_ank_duty, const u_int8_t tank_duty)
{
    this->knee_ext_con.UpdateMeas(p_sub_tank, p_knee_ext, knee_ext_duty);
    // this->knee_flex_con.UpdateMeas(p_knee_ext, p_knee_flex, knee_flex_duty);
    this->ank_ext_con.UpdateMeas(p_sub_tank, p_ank_ext, ank_ext_duty);
    this->knee_ank_con.UpdateMeas(p_knee_ext, p_ank_ext, knee_ank_duty);
    this->tank_con.UpdateMeas(p_main_tank, p_sub_tank, tank_duty);

    this->p_knee_ext = p_knee_ext;
    this->p_ank_ext = p_ank_ext;
    this->p_knee_flex = p_knee_flex;
    this->p_sub_tank = p_sub_tank;
    this->p_main_tank = p_main_tank;
    this->cur_knee_ang = knee_angle;
    this->cur_ankle_ang = ankle_angle;
    // calculate force related values
    std::array<double, 2> cur_force = this->force_filter.GetFilteredMea(
        std::array<double, 2>{(p_knee_ext * this->knee_cyln_params.piston_area[0] - p_knee_flex * this->knee_cyln_params.piston_area[1]) * 2.1547177056884764e-05,
                              p_ank_ext * this->ank_cyln_params.piston_area[0] * 2.1547177056884764e-05});
    this->cur_knee_force = cur_force[0];
    this->cur_ank_force = cur_force[1];

    // this->cur_max_spring_compress = ((p_joint_ext-0.5/4.096*65536) * this->piston_area_ext ) * 2.1547177056884764e-05 / this->spring_k; // unit in mm, piston area=0 for air reservoir

    // calculate cylinder volume

    this->max_knee_spring_compress = this->cur_knee_force / this->knee_cyln_params.spring_const; // unit in mm, piston area=0 for air reservoir
    this->max_ank_spring_compress = this->cur_ank_force / this->ank_cyln_params.spring_const;

    double knee_tot_len = sqrt(this->knee_cyln_params.cyln_eqn[0] - this->knee_cyln_params.cyln_eqn[1] * cos((180 - knee_angle - this->knee_cyln_params.cyln_eqn[2]) / 180 * M_PI)); // the cylinder is calculated by assuming knee angle =180 at full extension
    double ank_tot_len = sqrt(this->ank_cyln_params.cyln_eqn[0] - this->ank_cyln_params.cyln_eqn[1] * cos((ankle_angle + 180 - this->ank_cyln_params.cyln_eqn[2]) / 180 * M_PI));

    // std::cout<<"knee angle: "<<knee_angle<<std::endl;

    this->knee_cyln_ext_len = this->knee_cyln_params.chamber_max_len - this->knee_cyln_params.mech_max_len + knee_tot_len - this->max_knee_spring_compress;
    // this->knee_cyln_ext_len = knee_tot_len - this->knee_cyln_params.mech_max_len + this->max_knee_spring_compress;
    // this->ank_cyln_ext_len = ank_tot_len - this->ank_cyln_params.mech_max_len + this->max_ank_spring_compress;

    this->ank_cyln_ext_len = this->ank_cyln_params.chamber_max_len - this->ank_cyln_params.mech_max_len + ank_tot_len - this->max_ank_spring_compress;

    this->knee_cyln_shrk_len = this->knee_cyln_params.chamber_max_len - this->knee_cyln_ext_len;
    this->ank_cyln_shrk_len = this->ank_cyln_params.chamber_max_len - this->ank_cyln_ext_len;

    std::array<double, 2> cyln_len_diff = this->vel_filter.GetFilteredMea(std::array<double, 2>{this->knee_cyln_ext_len - this->knee_len_ext_old, this->ank_cyln_ext_len - this->ank_len_ext_old});
    this->knee_cyln_len_diff = cyln_len_diff[0];
    this->ank_cyln_len_diff = cyln_len_diff[1];

    // calculate moment arm
    this->knee_moment_arm = 0.5 * this->knee_cyln_params.cyln_eqn[1] * sin((180 - knee_angle - this->knee_cyln_params.cyln_eqn[2]) / 180 * M_PI) / knee_tot_len;
    this->ankle_moment_arm = 0.5 * this->ank_cyln_params.cyln_eqn[1] * sin((ankle_angle + 180 - this->ank_cyln_params.cyln_eqn[2]) / 180 * M_PI) / ank_tot_len;


    //get fsm state
    




    // record old data

    // TODO: add recorder
    this->knee_len_ext_old = this->knee_cyln_ext_len;
    this->ank_len_ext_old = this->ank_cyln_ext_len;


    
}

void JointCon::RecData()
{

    this->knee_ext_con.RecData();
    // this->knee_flex_con.RecData();
    this->ank_ext_con.RecData();
    this->knee_ank_con.RecData();
    this->tank_con.RecData();
}
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
// void JointCon::GetForceCon(const std::array<double,MPC_TIME_HORIZON> &des_force,u_int8_t &ext_duty,u_int8_t &ctra_duty,u_int8_t &tank_duty, JointCon::ForceImpCon force_imp_con,JointCon::ForceRedType force_red_type){
void JointCon::GetForceCon(u_int8_t &charge_duty, u_int8_t &rec_duty, u_int8_t &balance_duty, u_int8_t &tank_duty, ForceCon force_con_type, ForceRedType force_red_type)
{

    // }
    // void JointCon::GetForceCon(const std::array<double, MPC_TIME_HORIZON> &des_force, u_int8_t &act_duty, u_int8_t &rec_duty, u_int8_t &tank_duty,ControlMode con_mode)
    // {
    auto des_force = this->cmd_force[(unsigned)force_con_type];
    const ExoConfig::CylnPhyParams *cyln_phy_param;
    const double *pos_diff, *cur_force, *p_act, *p_rec;
    MPC *act_con;
    const double *cur_act_chamber_len;

    if (force_con_type == ForceCon::kKneExt)
    {
        cyln_phy_param = &this->knee_cyln_params;
        pos_diff = &this->knee_pos_diff;
        cur_force = &this->cur_knee_force;
        act_con = &this->knee_ext_con;

        cur_act_chamber_len = &this->knee_cyln_ext_len;
        p_act = &this->p_knee_ext;
        p_rec = &this->p_ank_ext;
    }
    else if (force_con_type == ForceCon::kAnkPlant)
    {
        cyln_phy_param = &this->ank_cyln_params;
        pos_diff = &this->ank_pos_diff;
        cur_force = &this->cur_ank_force;
        act_con = &this->ank_ext_con;
        cur_act_chamber_len = &this->ank_cyln_ext_len;
        p_act = &this->p_ank_ext;
        p_rec = &this->p_knee_ext;
    }
    else
    {
        std::cerr << "cannot find force type" << std::endl;
        return;
    }

    // std::cout<<"desired force: "<<des_force[0]<<std::endl;
    // std::cout<<"current force: "<<this->cur_force<<std::endl;
    // std::cout<<"des pre: ";
    std::array<double, MPC_TIME_HORIZON> des_pre;
    for (unsigned i = 0; i < des_pre.size(); i++)
    {
        des_pre[i] = ((des_force[i] + cyln_phy_param->fri_coeff * (*pos_diff)) / 2.1547177056884764e-05) / cyln_phy_param->piston_area[0] + 8000;
        // des_pre[i] = ((des_force[i]) / 2.1547177056884764e-05) / this->piston_area_ext + 8000;
        // std::cout<<des_pre[i]<<',';
    }
    this->des_force = des_force[0];
    this->des_ext_pre = des_pre[0];
    // std::cout<<std::endl;
    // std::cout<<"current pressure: "<<this->pre_ext<<std::endl;
    // std::cout<<"force gap: "<<des_force[0]-this->cur_force<<std::endl;
    if (std::abs(des_force[0] - *cur_force) > JointCon::kForceTol)
    {

        if ((des_force[0] > *cur_force) && (*cur_force > 0))
        {
            // the des_pre is definitely larger than pre_ext, otherwise des_force < this->cur_force

            // to charge the cylinder, we need to make sure the tank is > than des_pre
            if (this->p_sub_tank < des_pre[0])
            {
                // std::cout << "increase tank pressure\n";
                // std::cout<<"ltank pre: "<<this->pre_tank<<", des pre: "<<des_pre[0]<<std::endl;
                tank_duty = this->tank_con.GetPreControl(des_pre, this->p_sub_tank, this->p_main_tank, 1);
                charge_duty = 0;
            }
            else
            {
                // std::cout << "increase cylinder pressure\n";
                // still charge the cylinder with whatever pressure in the tank
                charge_duty = act_con->GetPreControl(des_pre, *p_act, this->p_sub_tank, act_con->GetMpcCalibLen() / *cur_act_chamber_len);
                tank_duty = 0;

                // std::cout<<"desired force: "<<des_force[0]<<std::endl;
                // std::cout<<"current force: "<<this->cur_force<<std::endl;
                // std::cout << "desired duty: " << ((int)rec_duty) << std::endl;
                // std::cout << "desired pressure: " << des_pre[0] << std::endl;
                // std::cout << "current pressure: " << this->pre_ext << std::endl;
                // std::cout << "current tank pressure: " << this->pre_tank << std::endl;
            }
            rec_duty = 0;
        }
        else
        {
            // std::cout<<"pre force: "<<this->cur_pre_force<<std::endl;
            // std::cout<<"desired force: "<<des_force[0]<<std::endl;
            // if the force is too high, we have 3 choices
            //  1. If pre_ext > pre_tank: pump the air in the cylinder back to reservior
            //  2. If pre_ext > pre_rec, pump the air to the recycle end
            if (*p_act < this->p_sub_tank)
            {
                if (this->p_ext_rec_diff > 160)
                {
                    rec_duty = this->knee_ank_con.GetPreControl(des_pre, *p_act, *p_rec, this->knee_ank_con.GetMpcCalibLen() / *cur_act_chamber_len);
                    // rec_duty = this->flex_con.GetPreControl(des_pre, this->pre_ext, this->pre_rec, 0.6);
                    // std::cout<<"recycle to rec cylinder\n";
                }
                else
                {
                    rec_duty = 0;
                    // std::cout<<"recycle cylinder pressure is too high\n";
                }
                charge_duty = 0;
                tank_duty = 0;
            }
            else
            {
                // we can recycle the energy to the tank
                // std::cout<<"recycle to ltank\n";
                charge_duty = act_con->GetPreControl(des_pre, *p_act, this->p_sub_tank, act_con->GetMpcCalibLen() / *cur_act_chamber_len);
                rec_duty = 0;
                tank_duty = 0;
            }
        }
    }
    else
    {
        // std::cout<<"force is close enough\n";
        charge_duty = 0;
        rec_duty = 0;
        tank_duty = 0;
    }
}

double JointCon::GetExternalForce(double pre_ext, double pre_flex, double delta_x, double x_dot, ExoConfig::CylnPhyParams cyln_params)
{

    // if ((delta_x - 4700) > this->cur_max_spring_compress / this->volume_slope_6in)
    // { // It turned out the the spring start to compress earlier, the 4700 is an experimental value

    //     return (pre_ext* this->piston_area_ext-pre_flex*this->piston_area_flex) * 2.1547177056884764e-05 - this->fric_coeff * x_dot; // unit newton
    //     // return (pre/65536*4.096-0.5)/4*200*0.31-0.001*this->pos_diff;
    // }
    // else
    // {
    //     return (delta_x) * this->volume_slope_6in * this->spring_k; // unit: newton
    //     // return (this->max_pos-x)*0.0006351973436310972*this->spring_k/25.4;
    // }

    // double delta_x_switch = delta_x - this->cur_max_spring_compress / this->volume_slope_6in;
    // // It turned out the the spring start to compress earlier, the 4700 is an experimental value
    // double coeff_0 = 1 / (1 + exp(-0.1 * delta_x_switch));
    // double coeff_1 = 1 - coeff_0;

    // double f_0 = coeff_0 * ((pre_ext * this->piston_area_ext - pre_flex * this->piston_area_flex) * 2.1547177056884764e-05 - this->fric_coeff * x_dot);
    // double f_1 = coeff_1 * (delta_x * this->volume_slope_6in * this->spring_k - this->fric_coeff * x_dot);
    // return f_0 + f_1;

    return (pre_ext * cyln_params.piston_area[0] - pre_flex * cyln_params.piston_area[1]) * 2.1547177056884764e-05 - cyln_params.fri_coeff * x_dot;
}

// double JointCon::GetLenLinear_mm(double pos)
// {
//     return pos * this->volume_slope_6in + this->volume_intercept_6in;
// }
void JointCon::GetImpCon(u_int8_t &charge_duty, u_int8_t &rec_duty, u_int8_t &balance_duty, u_int8_t &tank_duty, JointCon::ForceCon force_con_type, JointCon::ForceRedType force_red_type)
// void JointCon::GetImpCon(double des_imp, u_int8_t &ext_duty, u_int8_t &rec_duty, u_int8_t &tank_duty,ControlMode con_mode, double force_offset)
{
    // steps:
    //        1. calculate desired force based on current position
    //        2. calculate desired pressure based on current velocity and desired force

    // check if we need to switch Imp_FSM
    double des_imp = this->cmd_imp[(unsigned)force_con_type];
    double force_offset = this->cmd_init_force[(unsigned)force_con_type];
    double des_force, des_force_step;

    if (force_con_type == ForceCon::kKneExt)
    {
        des_force = des_imp * (this->cur_knee_ang - this->knee_cyln_params.neutral_pos);
        des_force_step = des_imp * this->knee_pos_diff;
    }
    else if (force_con_type == ForceCon::kAnkPlant)
    {
        des_force = des_imp * (this->cur_ankle_ang - this->ank_cyln_params.neutral_pos);
        des_force_step = des_imp * this->ank_pos_diff;
    }
    else
    {
        std::cerr << "Incorrect force type (Impedance control)" << std::endl;
        return;
    }

    std::array<double, MPC_TIME_HORIZON> des_force_array;
    for (int i = 0; i < MPC_TIME_HORIZON; i++)
    {
        des_force_array[i] = des_force + force_offset;
        des_force -= des_force_step; // TODO: right now I just make it 5% more in the future, note that it will fail when the impedance is negative
    }
    this->cmd_force[(unsigned)force_con_type] = des_force_array;

    this->GetForceCon(charge_duty, rec_duty, balance_duty, tank_duty, force_con_type, force_red_type);
}

// void JointCon::GetPreCon(const double des_pre, u_int8_t &duty, ControlMode con_mode)
// {
//     std::array<double, MPC_TIME_HORIZON> des_pre_array;
//     std::fill_n(des_pre_array.begin(), des_pre_array.size(), des_pre);
//     switch (con_mode)
//     {

//     case JointCon::ControlMode::kPreConKneeExt:

//         duty = this->knee_ext_con.GetPreControl(des_pre_array,this->p_knee_ext, this->p_sub_tank, this->knee_ext_con.GetMpcCalibLen() / this->knee_cyln_ext_len);
//         break;
//     case JointCon::ControlMode::kPreConSubTank:
//         duty = this->tank_con.GetPreControl(des_pre_array, this->p_sub_tank, this->p_main_tank, 1);
//         break;

//     default:
//         duty = 0;
//     }
// }

void JointCon::GetPreCon(u_int8_t &duty, JointCon::PreCon pre_con_mode)
{
    std::array<double, MPC_TIME_HORIZON> des_pre_array;
    std::fill_n(des_pre_array.begin(), des_pre_array.size(), this->cmd_pre[(unsigned)pre_con_mode]);
    switch (pre_con_mode)
    {

    case JointCon::PreCon::kKneExt:
        duty = this->knee_ext_con.GetPreControl(des_pre_array, this->p_knee_ext, this->p_sub_tank, this->knee_cyln_params.chamber_max_len / this->knee_cyln_ext_len);
        break;
    case JointCon::PreCon::kAnkPlant:
        duty = this->ank_ext_con.GetPreControl(des_pre_array, this->p_ank_ext, this->p_sub_tank, this->ank_cyln_params.chamber_max_len / this->ank_cyln_shrk_len);
        break;
    case JointCon::PreCon::kSubTank:
        duty = this->tank_con.GetPreControl(des_pre_array, this->p_sub_tank, this->p_main_tank, 1);
        break;

    default:
        duty = 0;
    }
}

const JointCon::ConMode JointCon::GetControlMode()
{
    return this->con_mode;
}
const JointCon::PreCon JointCon::GetPreConMode()
{
    return this->pre_con_type;
}
const JointCon::ForceCon JointCon::GetForceImpConMode()
{
    return this->force_con_type;
}
const JointCon::ForceRedType JointCon::GetForceImpRedMode()
{
    return this->force_red_type;
}

double JointCon::GetPre_KPa(double pre_adc)
{
    return (pre_adc / 65536 * 4.096 - 0.5) * 50 * 6.89476;
}

bool JointCon::GetValveDuty(u_int8_t &knee_ext_duty, u_int8_t &knee_flex_duty, u_int8_t &ank_pla_duty, u_int8_t &ank_dor_duty, u_int8_t &sub_tank_duty, u_int8_t &knee_ank_duty)
{

    if (this->con_mode == ConMode::kPreCon)
    {
        switch (this->pre_con_type)
        {
        case PreCon::kKneExt:
            this->GetPreCon(knee_ext_duty, this->pre_con_type);
            break;
        case PreCon::kAnkPlant:
            this->GetPreCon(ank_pla_duty, this->pre_con_type);
            break;
        case PreCon::kSubTank:
            this->GetPreCon(sub_tank_duty, this->pre_con_type);
            break;
        default:
            return false;
        }
        return true;
    }
    else if (this->con_mode == ConMode::kForceCon)
    {
        switch (this->force_con_type)
        {
        case ForceCon::kKneExt:
            this->GetForceCon(knee_ext_duty, knee_ank_duty, knee_flex_duty, sub_tank_duty, this->force_con_type, this->force_red_type);
            break;
        case ForceCon::kAnkPlant:
            this->GetForceCon(ank_pla_duty, knee_ank_duty, ank_dor_duty, sub_tank_duty, this->force_con_type, this->force_red_type);
            break;

        default:
            return false;
        }
        return true;
    }

    else if(this->con_mode==ConMode::kImpCon)
    {
        switch (this->force_con_type)
        {
        case ForceCon::kKneExt:
            this->GetImpCon(knee_ext_duty,knee_ank_duty,ank_dor_duty,sub_tank_duty, this->force_con_type,this->force_red_type);
            break;
        case ForceCon::kAnkPlant:
            this->GetImpCon(ank_pla_duty,knee_ank_duty,ank_dor_duty,sub_tank_duty,this->force_con_type,this->force_red_type);
            break;
        default:
            return false;
        }
        return true;
    }

    return false;
}
