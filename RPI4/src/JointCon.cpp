#include <cmath>
#include <osqp/osqp.h>

#include "JointCon.hpp"

JointCon::JointCon(ExoConfig::MPC_Params knee_ext_params, ExoConfig::MPC_Params ank_ext_params, ExoConfig::MPC_Params knee_ank_params,
                   ExoConfig::MPC_Params tank_params, ExoConfig::CylnPhyParams knee_cyln_params, ExoConfig::CylnPhyParams ank_cyln_params, std::string joint_con_name)
    : knee_ext_con(knee_ext_params, joint_con_name + "_knee_ext"),
      // knee_flex_con(knee_flex_params, joint_con_name + "_knee_flex"),
      ank_ext_con(ank_ext_params, joint_con_name + "_ank_ext"), knee_ank_con(knee_ank_params, joint_con_name + "_knee_ank"), tank_con(tank_params, joint_con_name + "_tank"),
      knee_cyln_params(knee_cyln_params), ank_cyln_params(ank_cyln_params),
      neutral_knee_pos(knee_cyln_params.neutral_pos), neutral_ank_pos(ank_cyln_params.neutral_pos),
      vel_filter(FilterParam::Filter20Hz_2::a, FilterParam::Filter20Hz_2::b), force_filter(FilterParam::Filter5Hz_2::a, FilterParam::Filter5Hz_2::b), force_pre_filter(FilterParam::Filter5Hz_2::a, FilterParam::Filter5Hz_2::b),
      p_ext_rec_diff_filter(FilterParam::Filter20Hz_2::a, FilterParam::Filter20Hz_2::b),
      joint_con_rec(joint_con_name, "Time,KneForce,AnkForce,KneExtLen,AnkShrkLen,KneMomentArm,AnkMomentArm,KneSpringMaxCompress,AnkSpringMaxCompress,TankDes,KneExtDes,AnkPlaDes")
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

void JointCon::SetPreControl(double _cmd_pre, Chamber controlled, Chamber followed)
{

    this->con_mode = ConMode::kPreCon;
    this->controlled_chamber = controlled;
    this->followed_chamber = followed;
    this->cmd_pre[(unsigned)controlled] = _cmd_pre;
}
void JointCon::SetForceControl(ForceCon _force_con_type, double cmd_force)
{
    std::array<double, MPC_TIME_HORIZON> cmd_force_array;

    std::fill_n(cmd_force_array.begin(), cmd_force_array.size(), cmd_force);
    this->cmd_force[(unsigned)_force_con_type] = cmd_force_array;
    this->con_mode = ConMode::kForceCon;
    this->force_con_type = _force_con_type;
}
void JointCon::SetImpControl(ForceCon _force_con_type, double cmd_imp, double cmd_init_force)
{

    this->cmd_init_force[(unsigned)_force_con_type] = cmd_init_force;
    this->cmd_imp[(unsigned)_force_con_type] = cmd_imp;
    this->con_mode = ConMode::kImpCon;
    this->force_con_type = _force_con_type;
}
void JointCon::SetImpControl(ForceCon _force_con_type, double cmd_imp, double cmd_init_force, double neutral_pos)
{

    this->cmd_init_force[(unsigned)_force_con_type] = cmd_init_force;
    this->cmd_imp[(unsigned)_force_con_type] = cmd_imp;
    this->con_mode = ConMode::kImpCon;
    this->force_con_type = _force_con_type;
    this->neutral_knee_pos = neutral_pos;
}
void JointCon::ResetControl()
{
    this->con_mode = ConMode::kNone;
    this->cmd_pre[(unsigned)Chamber::kSubTank] = 0;
    this->cmd_pre[(unsigned)Chamber::kKneExt] = 0;
    this->cmd_pre[(unsigned)Chamber::kAnkPla] = 0;
}
void JointCon::ShutDown()
{
    // std::cout<<"shutdown joint controller\n";
    this->con_mode = ConMode::kTurnOff;
}

void JointCon::SetKneeReverse(bool is_reverse)
{
    if (this->is_knee_reverse ^ is_reverse)
        this->is_knee_reverse = is_reverse;
}
bool JointCon::GetKneeReverse()
{
    return this->is_knee_reverse;
}

// void JointCon::SetKneeMaxPos(double max_pos_val)
// {
//     this->max_len_mm = this->GetLenLinear_mm(max_pos_val); // TODO: need to modify this function when integrating to the exo
//     // std::cout<<"max pos reset\n";
// }
// void JointCon::PushMeas(const double &p_joint_ext, const double &p_joint_flex, const double &p_joint_rec, const double &p_tank, const double &p_main_tank, const double &pos, const u_int8_t tank_duty, const u_int8_t knee_ext_duty, const u_int8_t knee_flex_duty, const u_int8_t knee_ank_duty, const u_int8_t ank_ext_duty)
void JointCon::PushMeas(const double &p_knee_ext, const double &p_knee_flex, const double &p_ank_pla, const double &p_ank_dorsi, const double &p_sub_tank, const double &p_main_tank, const double &knee_angle, const double &ankle_angle, const u_int8_t knee_ext_duty, const u_int8_t knee_flex_duty, const u_int8_t ank_ext_duty, const u_int8_t knee_ank_duty, const u_int8_t tank_duty)
{
    this->knee_ext_con.UpdateMeas(p_sub_tank, p_knee_ext, knee_ext_duty);
    // this->knee_flex_con.UpdateMeas(p_knee_ext, p_knee_flex, knee_flex_duty);
    this->ank_ext_con.UpdateMeas(p_sub_tank, p_ank_pla, ank_ext_duty);
    this->knee_ank_con.UpdateMeas(p_knee_ext, p_ank_pla, knee_ank_duty);
    this->tank_con.UpdateMeas(p_main_tank, p_sub_tank, tank_duty);

    this->p_knee_ext = p_knee_ext;
    this->p_ank_pla = p_ank_pla;
    this->p_ank_dorsi = p_ank_dorsi;
    this->p_knee_flex = p_knee_flex;
    this->p_sub_tank = p_sub_tank;
    this->p_main_tank = p_main_tank;

    this->knee_pos_diff = knee_angle - this->cur_knee_ang;
    this->ank_pos_diff = ankle_angle - this->cur_ankle_ang;

    this->cur_knee_ang = knee_angle;
    this->cur_ankle_ang = ankle_angle;
    // calculate force related values

    std::array<double, 2> cur_force;
    if (this->is_knee_reverse)
    {
        cur_force = this->force_filter.GetFilteredMea(
            std::array<double, 2>{(p_knee_ext * this->knee_cyln_params.piston_area[0] - p_knee_flex * this->knee_cyln_params.piston_area[1]) * 2.1547177056884764e-05,
                                  (p_ank_pla * this->ank_cyln_params.piston_area[0] - p_ank_dorsi * this->knee_cyln_params.piston_area[1]) * 2.1547177056884764e-05});
    }
    else
    {
        cur_force = this->force_filter.GetFilteredMea(
            std::array<double, 2>{(p_knee_ext * this->knee_cyln_params.piston_area[0] - p_knee_flex * this->knee_cyln_params.piston_area[1]) * 2.1547177056884764e-05,
                                  (p_ank_pla * this->ank_cyln_params.piston_area[1] - p_ank_dorsi * this->knee_cyln_params.piston_area[0]) * 2.1547177056884764e-05});
    }
    this->cur_knee_force = cur_force[0];
    this->cur_ank_force = cur_force[1];

    // this->cur_max_spring_compress = ((p_joint_ext-0.5/4.096*65536) * this->piston_area_ext ) * 2.1547177056884764e-05 / this->spring_k; // unit in mm, piston area=0 for air reservoir

    // calculate cylinder volume

    this->max_knee_spring_compress = this->cur_knee_force / this->knee_cyln_params.spring_const; // unit in mm, piston area=0 for air reservoir
    this->max_ank_spring_compress = this->cur_ank_force / this->ank_cyln_params.spring_const;

    double knee_tot_len = sqrt(this->knee_cyln_params.cyln_eqn[0] - this->knee_cyln_params.cyln_eqn[1] * cos((180 - knee_angle - this->knee_cyln_params.cyln_eqn[2]) / 180 * M_PI)); // the cylinder is calculated by assuming knee angle =180 at full extension
    double ank_tot_len = sqrt(this->ank_cyln_params.cyln_eqn[0] - this->ank_cyln_params.cyln_eqn[1] * cos((ankle_angle + 180 - this->ank_cyln_params.cyln_eqn[2]) / 180 * M_PI));

    // std::cout<<"knee angle: "<<knee_angle<<std::endl;

    this->knee_cyln_shrk_len = this->knee_cyln_params.mech_max_len - knee_tot_len - this->max_knee_spring_compress;
    this->knee_cyln_ext_len = this->knee_cyln_params.chamber_max_len - this->knee_cyln_shrk_len;
    // this->knee_cyln_ext_len = knee_tot_len - this->knee_cyln_params.mech_max_len + this->max_knee_spring_compress;
    // this->ank_cyln_ext_len = ank_tot_len - this->ank_cyln_params.mech_max_len + this->max_ank_spring_compress;

    this->ank_cyln_shrk_len = this->ank_cyln_params.mech_max_len - ank_tot_len - this->max_ank_spring_compress;
    this->ank_cyln_ext_len = this->ank_cyln_params.chamber_max_len - this->ank_cyln_shrk_len;

    std::array<double, 2> cyln_len_diff;
    if(this->is_knee_reverse)//for forward knee, we should use ank_cyln_ext_len instead of ank_cyln_ahrk_len, yet, for vel it is just adding a opposite sign
        cyln_len_diff = this->vel_filter.GetFilteredMea(std::array<double, 2>{this->knee_cyln_ext_len - this->knee_len_ext_old, this->ank_cyln_shrk_len - this->ank_len_shrk_old});
    else
        cyln_len_diff= this->vel_filter.GetFilteredMea(std::array<double, 2>{this->knee_cyln_ext_len - this->knee_len_ext_old, -this->ank_cyln_shrk_len + this->ank_len_shrk_old});
    this->knee_cyln_len_diff = cyln_len_diff[0];
    this->ank_cyln_len_diff = cyln_len_diff[1];

    // calculate moment arm
    this->knee_moment_arm = 0.5 * this->knee_cyln_params.cyln_eqn[1] * sin((180 - knee_angle - this->knee_cyln_params.cyln_eqn[2]) / 180 * M_PI) / knee_tot_len;
    this->ankle_moment_arm = 0.5 * this->ank_cyln_params.cyln_eqn[1] * sin((ankle_angle + 180 - this->ank_cyln_params.cyln_eqn[2]) / 180 * M_PI) / ank_tot_len;

    // std::cout<<"knee moment arm: "<<this->knee_moment_arm<<std::endl;
    // std::cout<<"ankle moment arm: "<<this->ankle_moment_arm<<std::endl;

    // get fsm state

    // record old data
    this->joint_con_rec.PushData(std::array<double, 11>{this->cur_knee_force,
                                                        this->cur_ank_force,
                                                        this->knee_cyln_ext_len,
                                                        this->ank_cyln_shrk_len,
                                                        this->knee_moment_arm,
                                                        this->ankle_moment_arm,
                                                        this->max_knee_spring_compress,
                                                        this->max_ank_spring_compress,
                                                        this->cmd_pre[(unsigned)Chamber::kSubTank],
                                                        this->cmd_pre[(unsigned)Chamber::kKneExt],
                                                        this->cmd_pre[(unsigned)Chamber::kAnkPla]});

    this->knee_len_ext_old = this->knee_cyln_ext_len;
    this->ank_len_shrk_old = this->ank_cyln_shrk_len;
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
void JointCon::GetForceCon(std::array<double, MPC_TIME_HORIZON> des_force, std::array<u_int8_t, (unsigned)ValveDuty::kTotal> &valve_duty, ForceCon force_con_type)
{

    const ExoConfig::CylnPhyParams *cyln_phy_param;
    const double *pos_diff, *cur_force, *p_act, *p_rec, *piston_area;
    double *cur_cmd_pre;
    // MPC *act_con;
    // const double *cur_act_chamber_len;
    if (force_con_type == ForceCon::kKneExt)
    {
        cyln_phy_param = &this->knee_cyln_params;
        pos_diff = &this->knee_pos_diff;
        cur_force = &this->cur_knee_force;
        // act_con = &this->knee_ext_con;

        // cur_act_chamber_len = &this->knee_cyln_ext_len;
        p_act = &this->p_knee_ext;
        p_rec = &this->p_ank_pla;

        cur_cmd_pre = &this->cmd_pre[(unsigned)Chamber::kKneExt];
        piston_area = &cyln_phy_param->piston_area[0];
    }
    else if (force_con_type == ForceCon::kAnkPlant)
    {
        cyln_phy_param = &this->ank_cyln_params;
        pos_diff = &this->ank_pos_diff;
        cur_force = &this->cur_ank_force;
        // act_con = &this->ank_ext_con;
        // cur_act_chamber_len = &this->ank_cyln_ext_len;
        p_act = &this->p_ank_pla;
        p_rec = &this->p_knee_ext;

        cur_cmd_pre = &this->cmd_pre[(unsigned)Chamber::kAnkPla];

        if(this->is_knee_reverse)
            piston_area = &cyln_phy_param->piston_area[1];
        else
            piston_area = &cyln_phy_param->piston_area[0];

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
        des_pre[i] = ((des_force[i] + cyln_phy_param->fri_coeff * (*pos_diff)) / 2.1547177056884764e-05) / *piston_area + 8000;
        // des_pre[i] = ((des_force[i]) / 2.1547177056884764e-05) / this->piston_area_ext + 8000;
        // std::cout<<des_pre[i]<<',';
    }
    *cur_cmd_pre = des_pre[0];
    // this->des_force = des_force[0];
    // std::cout<<"desired pressure: "<<des_pre[0]<<std::endl;
    // this->des_ext_pre = des_pre[0];
    // std::cout<<std::endl;
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

                this->GetPreCon(des_pre, valve_duty, Chamber::kSubTank, Chamber::kMainTank);
                // valve_duty[(unsigned)ValveDuty::kSubTank] = this->tank_con.GetPreControl(des_pre, this->p_sub_tank, this->p_main_tank, 1);//make it 5 psi higher than required
                valve_duty[(unsigned)ValveDuty::kKneExt] = 0;
            }
            else
            {
                // std::cout << "increase cylinder pressure\n";
                // still charge the cylinder with whatever pressure in the tank
                // this->GetPreCon(des_pre,charge_duty,PreCon::kKneExt);
                this->GetPreCon(des_pre, valve_duty, Chamber::kKneExt, Chamber::kSubTank);
                // valve_duty[(unsigned)ValveDuty::kKneExt] = act_con->GetPreControl(des_pre, *p_act, this->p_sub_tank, act_con->GetMpcCalibLen() / *cur_act_chamber_len);
                valve_duty[(unsigned)ValveDuty::kSubTank] = 0;

                // std::cout<<"desired force: "<<des_force[0]<<std::endl;
                // std::cout<<"current force: "<<this->cur_force<<std::endl;
                // std::cout << "desired duty: " << ((int)rec_duty) << std::endl;
                // std::cout << "desired pressure: " << des_pre[0] << std::endl;
                // std::cout << "current pressure: " << this->pre_ext << std::endl;
                // std::cout << "current tank pressure: " << this->pre_tank << std::endl;
            }
            valve_duty[(unsigned)ValveDuty::kKneAnk] = 0;
        }
        else
        {
            // std::cout<<"pre force: "<<this->cur_pre_force<<std::endl;
            // std::cout<<"desired force: "<<des_force[0]<<std::endl;
            // if the force is too high, we have 3 choices
            //  1. If pre_ext > pre_tank: pump the air in the cylinder back to reservior
            //  2. If pre_ext > pre_rec, pump the air to the recycle end
            // if (*p_act < this->p_sub_tank)
            // {
            //     if (this->p_knee_ext-this->p_ank_pla > 160)
            //     {
            //         // std::cout<<"recycle to another cylinder\n";
            //         this->GetPreCon(des_pre,valve_duty,Chamber::kKneExt,Chamber::kAnkPla);
            //         // valve_duty[(unsigned)ValveDuty::kKneAnk] = this->knee_ank_con.GetPreControl(des_pre, *p_act, *p_rec, this->knee_ank_con.GetMpcCalibLen() / *cur_act_chamber_len);
            //         // rec_duty = this->flex_con.GetPreControl(des_pre, this->pre_ext, this->pre_rec, 0.6);
            //         // std::cout<<"recycle to rec cylinder\n";
            //     }
            //     else
            //     {
            //         valve_duty[(unsigned)ValveDuty::kKneAnk] = 0;
            //         // std::cout<<"recycle cylinder pressure is too high\n";
            //     }
            //     valve_duty[(unsigned)ValveDuty::kKneExt]=0;
            //     valve_duty[(unsigned)ValveDuty::kSubTank]=0;
            // }
            // else
            {
                // we can recycle the energy to the tank
                // std::cout<<"recycle to ltank\n";
                this->GetPreCon(des_pre, valve_duty, Chamber::kKneExt, Chamber::kSubTank);
                // valve_duty[(unsigned)ValveDuty::kKneExt] = act_con->GetPreControl(des_pre, *p_act, this->p_sub_tank, act_con->GetMpcCalibLen() / *cur_act_chamber_len);
                valve_duty[(unsigned)ValveDuty::kKneAnk] = 0;
                valve_duty[(unsigned)ValveDuty::kSubTank] = 0;
            }
        }
    }
    else
    {
        // std::cout<<"force is close enough\n";
        valve_duty[(unsigned)ValveDuty::kKneExt] = 0;
        valve_duty[(unsigned)ValveDuty::kKneAnk] = 0;
        valve_duty[(unsigned)ValveDuty::kSubTank] = 0;
    }
}

double JointCon::GetExternalForce(double pre_ext, double pre_flex, double delta_x, double x_dot, ExoConfig::CylnPhyParams cyln_params,bool is_ankle)
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

    if(is_ankle){
        if(this->is_knee_reverse)
            return (pre_ext * cyln_params.piston_area[1] - pre_flex * cyln_params.piston_area[0]) * 2.1547177056884764e-05 - cyln_params.fri_coeff * x_dot;

    }


    return (pre_ext * cyln_params.piston_area[0] - pre_flex * cyln_params.piston_area[1]) * 2.1547177056884764e-05 - cyln_params.fri_coeff * x_dot;
}

// double JointCon::GetLenLinear_mm(double pos)
// {
//     return pos * this->volume_slope_6in + this->volume_intercept_6in;
// }
// void JointCon::GetImpCon(double des_imp, double force_offset, u_int8_t &charge_duty, u_int8_t &rec_duty, u_int8_t &balance_duty, u_int8_t &tank_duty, JointCon::ForceCon force_con_type, JointCon::ForceRedType force_red_type)
// // void JointCon::GetImpCon(double des_imp, u_int8_t &ext_duty, u_int8_t &rec_duty, u_int8_t &tank_duty,ControlMode con_mode, double force_offset)
// {
//     // steps:
//     //        1. calculate desired force based on current position
//     //        2. calculate desired pressure based on current velocity and desired force

//     // check if we need to switch Imp_FSM
//     // double des_imp = this->cmd_imp[(unsigned)force_con_type];
//     // double force_offset = this->cmd_init_force[(unsigned)force_con_type];
//     double des_force, des_force_step;

//     if (force_con_type == ForceCon::kKneExt)
//     {
//         des_force = des_imp * (this->cur_knee_ang - this->knee_cyln_params.neutral_pos);
//         des_force_step = des_imp * this->knee_pos_diff;
//     }
//     else if (force_con_type == ForceCon::kAnkPlant)
//     {
//         des_force = des_imp * (this->cur_ankle_ang - this->ank_cyln_params.neutral_pos);
//         des_force_step = des_imp * this->ank_pos_diff;
//     }
//     else
//     {
//         std::cerr << "Incorrect force type (Impedance control)" << std::endl;
//         return;
//     }

//     std::array<double, MPC_TIME_HORIZON> des_force_array;
//     for (int i = 0; i < MPC_TIME_HORIZON; i++)
//     {
//         des_force_array[i] = des_force + force_offset;
//         des_force -= des_force_step; // TODO: right now I just make it 5% more in the future, note that it will fail when the impedance is negative
//     }
//     this->cmd_force[(unsigned)force_con_type] = des_force_array;

//     this->GetForceCon(des_force_array, charge_duty, rec_duty, balance_duty, tank_duty, force_con_type, force_red_type);
// }
void JointCon::GetImpCon(double des_imp, double torque_offset, std::array<u_int8_t, (unsigned)ValveDuty::kTotal> &valve_duty, JointCon::ForceCon force_con_type)
// void JointCon::GetImpCon(double des_imp, u_int8_t &ext_duty, u_int8_t &rec_duty, u_int8_t &tank_duty,ControlMode con_mode, double force_offset)
{
    // steps:
    //        1. calculate desired force based on current position
    //        2. calculate desired pressure based on current velocity and desired force

    // check if we need to switch Imp_FSM
    // double des_imp = this->cmd_imp[(unsigned)force_con_type];
    // double force_offset = this->cmd_init_force[(unsigned)force_con_type];
    double des_torque, des_torque_step;
    // std::cout<<"knee pos diff: "<<this->knee_pos_diff<<std::endl;
    if (force_con_type == ForceCon::kKneExt)
    {
        des_torque = des_imp * (this->cur_knee_ang - this->neutral_knee_pos) + torque_offset;
        des_torque_step = des_imp * this->knee_pos_diff;
    }
    else if (force_con_type == ForceCon::kAnkPlant)
    {
        des_torque = des_imp * (this->cur_ankle_ang - this->neutral_ank_pos) + torque_offset;
        des_torque_step = des_imp * this->ank_pos_diff;
    }
    else
    {
        std::cerr << "Incorrect force type (Impedance control)" << std::endl;
        return;
    }
    // std::cout<<"desired torque: "<<des_torque<<std::endl;
    // std::cout<<"torque step: "<<des_torque_step<<std::endl;
    std::array<double, MPC_TIME_HORIZON> des_torque_array;
    for (int i = 0; i < MPC_TIME_HORIZON; i++)
    {
        des_torque_array[i] = des_torque;
        des_torque += des_torque_step; // TODO: right now I just make it 5% more in the future, note that it will fail when the impedance is negative
    }
    // this->cmd_force[(unsigned)force_con_type] = des_torque_array;
    // for (double tor : des_torque_array)
    // {
    //     std::cout << tor << ',';
    // }
    // std::cout << std::endl;
    this->GetTorCon(des_torque_array, valve_duty, force_con_type);
}

// void JointCon::GetPreCon(const std::array<double, MPC_TIME_HORIZON> &des_pre, u_int8_t &duty, JointCon::PreCon pre_con_mode)
// {
//     // std::array<double, MPC_TIME_HORIZON> des_pre_array;
//     // // std::fill_n(des_pre_array.begin(), des_pre_array.size(), this->cmd_pre[(unsigned)pre_con_mode]);
//     // std::fill_n(des_pre_array.begin(), des_pre_array.size(), des_pre);
//     switch (pre_con_mode)
//     {

//     case JointCon::PreCon::kKneExt:
//         duty = this->knee_ext_con.GetPreControl(des_pre, this->p_knee_ext, this->p_sub_tank, this->knee_cyln_params.chamber_max_len / this->knee_cyln_ext_len);
//         break;
//     case JointCon::PreCon::kAnkPlant:
//         duty = this->ank_ext_con.GetPreControl(des_pre, this->p_ank_pla, this->p_sub_tank, this->ank_cyln_params.chamber_max_len / this->ank_cyln_shrk_len);
//         break;

//     case JointCon::PreCon::kSubTank:
//         duty = this->tank_con.GetPreControl(des_pre, this->p_sub_tank, this->p_main_tank, 1);
//         break;

//     default:
//         duty = 0;
//     }
// }
void JointCon::GetPreCon(const std::array<double, MPC_TIME_HORIZON> &des_pre, std::array<u_int8_t, (unsigned)ValveDuty::kTotal> &valve_duty, Chamber controlled, Chamber followed)
{
    if (controlled == Chamber::kSubTank && followed == Chamber::kMainTank)
    {
        valve_duty[(unsigned)ValveDuty::kSubTank] = this->tank_con.GetPreControl(des_pre, this->p_sub_tank, this->p_main_tank, 1);
    }
    else if (controlled == Chamber::kKneExt && followed == Chamber::kSubTank)
    {
        valve_duty[(unsigned)ValveDuty::kKneExt] = this->knee_ext_con.GetPreControl(des_pre, this->p_knee_ext, this->p_sub_tank, this->knee_cyln_params.chamber_max_len / this->knee_cyln_ext_len);
    }
    else if (controlled == Chamber::kAnkPla && followed == Chamber::kSubTank)
    {
        valve_duty[(unsigned)ValveDuty::kAnkPla] = this->ank_ext_con.GetPreControl(des_pre, this->p_ank_pla, this->p_sub_tank, this->ank_cyln_params.chamber_max_len / this->ank_cyln_shrk_len);
    }
    else if (controlled == Chamber::kKneExt && followed == Chamber::kAnkPla)
    {
        valve_duty[(unsigned)ValveDuty::kKneAnk] = this->knee_ank_con.GetPreControl(des_pre, this->p_knee_ext, this->p_ank_pla, this->knee_cyln_params.chamber_max_len / this->knee_cyln_ext_len / this->ank_cyln_params.chamber_max_len * this->ank_cyln_shrk_len);
    }
    else if (controlled == Chamber::kAnkPla && followed == Chamber::kKneExt)
    {
        valve_duty[(unsigned)ValveDuty::kKneAnk] = this->knee_ank_con.GetPreControl(des_pre, this->p_ank_pla, this->p_knee_ext, this->ank_cyln_params.chamber_max_len / this->ank_cyln_shrk_len / this->knee_cyln_params.chamber_max_len * this->knee_cyln_ext_len);
    }
}

double JointCon::GetPre_KPa(double pre_adc)
{
    return (pre_adc / 65536 * 4.096 - 0.5) * 50 * 6.89476;
}
bool JointCon::GetValveDuty(u_int8_t &knee_ext_duty, u_int8_t &knee_flex_duty, u_int8_t &ank_pla_duty, u_int8_t &ank_dor_duty, u_int8_t &sub_tank_duty, u_int8_t &knee_ank_duty)
{
    std::array<u_int8_t, (unsigned)ValveDuty::kTotal> valve_duty;
    valve_duty[(unsigned)ValveDuty::kSubTank] = sub_tank_duty;
    valve_duty[(unsigned)ValveDuty::kKneExt] = knee_ext_duty;
    valve_duty[(unsigned)ValveDuty::kAnkPla] = ank_pla_duty;
    valve_duty[(unsigned)ValveDuty::kKneAnk] = knee_ank_duty;
    valve_duty[(unsigned)ValveDuty::kKneFlex] = knee_flex_duty;
    valve_duty[(unsigned)ValveDuty::kAnkDor] = ank_dor_duty;
    if (this->con_mode == ConMode::kPreCon)
    {

        std::array<double, MPC_TIME_HORIZON> des_pre;
        std::fill_n(des_pre.begin(), MPC_TIME_HORIZON, this->cmd_pre[(unsigned)this->controlled_chamber]);
        this->GetPreCon(des_pre, valve_duty, this->controlled_chamber, this->followed_chamber);

        // return true;
    }
    else if (this->con_mode == ConMode::kForceCon)
    {
        // std::cout<<"run force cont\n";
        // u_int8_t *chamber_pos_duty, *chamber_neg_duty;
        // switch (this->force_con_type)
        // {

        // case ForceCon::kKneExt:
        //     chamber_pos_duty = &knee_ext_duty;
        //     chamber_neg_duty = &knee_flex_duty;

        //     break;
        // case ForceCon::kAnkPlant:
        //     chamber_pos_duty = &ank_pla_duty;
        //     chamber_neg_duty = &ank_dor_duty;
        //     break;
        // }

        this->GetForceCon(this->cmd_force[(unsigned)this->force_con_type], valve_duty, this->force_con_type);
        // return true;
    }

    else if (this->con_mode == ConMode::kImpCon)
    {
        // std::cout<<"run imp con\n";
        // u_int8_t *charge_duty, *balance_duty;
        // switch (this->force_con_type)
        // {
        // case ForceCon::kKneExt:
        //     charge_duty = &knee_ext_duty;
        //     balance_duty = &knee_flex_duty;

        //     break;
        // case ForceCon::kAnkPlant:
        //     // this->GetImpCon(ank_pla_duty, knee_ank_duty, ank_dor_duty, sub_tank_duty, this->force_con_type, this->force_red_type);
        //     charge_duty = &ank_pla_duty;
        //     balance_duty = &ank_dor_duty;
        //     break;
        // // default:
        //     // return false;
        // }

        this->GetImpCon(this->cmd_imp[(unsigned)force_con_type], this->cmd_init_force[(unsigned)force_con_type], valve_duty, this->force_con_type);
        // return true;
    }
    else if (this->con_mode == ConMode::kTurnOff)
    {
        // std::cout<<"turn off\n";

        valve_duty[(unsigned)ValveDuty::kSubTank] = 0;
        valve_duty[(unsigned)ValveDuty::kKneExt] = 0;
        valve_duty[(unsigned)ValveDuty::kAnkPla] = 0;
        valve_duty[(unsigned)ValveDuty::kKneAnk] = 0;
        valve_duty[(unsigned)ValveDuty::kKneFlex] = 0;
        valve_duty[(unsigned)ValveDuty::kAnkDor] = 0;

        // this->con_mode = ConMode::kNone;
        this->ResetControl();
    }

    // u_int8_t &knee_ext_duty, u_int8_t &knee_flex_duty, u_int8_t &ank_pla_duty, u_int8_t &ank_dor_duty, u_int8_t &sub_tank_duty, u_int8_t &knee_ank_duty
    if (this->con_mode != ConMode::kNone)
    {
        sub_tank_duty = valve_duty[(unsigned)ValveDuty::kSubTank];
        knee_ext_duty = valve_duty[(unsigned)ValveDuty::kKneExt];
        ank_pla_duty = valve_duty[(unsigned)ValveDuty::kAnkPla];
        knee_ank_duty = valve_duty[(unsigned)ValveDuty::kKneAnk];
        knee_flex_duty = valve_duty[(unsigned)ValveDuty::kKneFlex];
        ank_dor_duty = valve_duty[(unsigned)ValveDuty::kAnkDor];

        // std::cout<<(int)knee_ext_duty<<std::endl;
    }
    return true;
}

void JointCon::GetTorCon(std::array<double, MPC_TIME_HORIZON> des_tor, std::array<u_int8_t, (unsigned)ValveDuty::kTotal> &valve_duty, ForceCon force_con_type)
{

    std::array<double, MPC_TIME_HORIZON> des_force;
    double moment_arm = 10;
    switch (force_con_type)
    {
    case ForceCon::kKneExt:
        moment_arm = this->knee_moment_arm;
        break;
    case ForceCon::kAnkPlant:
        moment_arm = this->ankle_moment_arm;
        break;
    default:
        moment_arm = 10;
        break;
    }
    // std::cout<<"moment arm: "<<moment_arm<<std::endl;
    // std::cout<<"desired torque: "<<des_tor[0]<<std::endl;
    // std::cout<<"moment arm: "<<moment_arm<<std::endl;
    // std::cout<<"desired force: ";
    for (unsigned i = 0; i < des_force.size(); i++)
    {
        des_force[i] = des_tor[i] / moment_arm * 1000; // desired torque is in N*m
        // std::cout<<des_force[i]<<',';
        // ideally moment arm should change at different angles, but here we ignore it for simplification
    }
    // std::cout<<"moment arm: "<<moment_arm<<std::endl;
    // std::cout<<"desired force: "<<des_force[0]<<std::endl;
    // std::cout<<std::endl;
    this->GetForceCon(des_force, valve_duty, force_con_type);
}
