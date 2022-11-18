#include <math.h>
#include <osqp/osqp.h>
#include "JointCon.hpp"
JointCon::JointCon(PneumaticParam::CylinderParam ext_param, PneumaticParam::ReservoirParam reservoir_param, std::string joint_con_name)
    : ext_con(ext_param.cl_ext, ext_param.ch_ext, joint_con_name + "_ext"), flex_con(ext_param.cl_flex, ext_param.ch_flex, joint_con_name + "_flex"), tank_con(reservoir_param.cl, reservoir_param.ch, joint_con_name + "_tank"),
      piston_area_ext(ext_param.piston_area_ext), piston_area_flex(ext_param.piston_area_flex), fric_coeff(ext_param.fri_coeff), max_pos(ext_param.max_pos), vel_filter(FilterParam::Filter20Hz_2::a, FilterParam::Filter20Hz_2::b), force_filter(FilterParam::Filter5Hz_2::a, FilterParam::Filter5Hz_2::b),force_pre_filter(FilterParam::Filter5Hz_2::a,FilterParam::Filter5Hz_2::b),
      p_ext_rec_diff_filter(FilterParam::Filter20Hz_2::a,FilterParam::Filter20Hz_2::b),
      joint_con_rec(joint_con_name, "Time,L_ext,L_flex,cur_force,max_spring_compress,delta_x,x_dot,des_force,pre_force,des_ext_pre")
{
    this->SetKneeMaxPos(ext_param.max_pos);
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
void JointCon::SetKneeMaxPos(double max_pos_val)
{
    this->max_pos = max_pos_val;
    this->max_len_mm = this->GetLenLinear_mm(max_pos_val); // TODO: need to modify this function when integrating to the exo
    // std::cout<<"max pos reset\n";
}
void JointCon::PushMeas(const double &p_joint_ext, const double &p_joint_flex, const double &p_joint_rec, const double &p_tank, const double &p_main_tank, const double &pos, const u_int8_t tank_duty, const u_int8_t knee_ext_duty, const u_int8_t knee_flex_duty, const u_int8_t knee_ank_duty, const u_int8_t ank_ext_duty)
{
    this->ext_con.PushMeas(p_tank, p_joint_ext, knee_ext_duty);
    this->flex_con.PushMeas(p_joint_rec, p_joint_ext, knee_ank_duty);
    this->tank_con.PushMeas(p_main_tank, p_tank, tank_duty);

    this->pre_tank = p_tank;
    this->pre_main_tank = p_main_tank;
    this->pos_diff = this->vel_filter.GetFilteredMea(std::array<double, 1>{pos - this->pre_pos})[0];
    this->pre_pos = pos;
    this->pre_rec = p_joint_rec;
    this->pre_ext = p_joint_ext;
    this->cur_pos = pos;
    // this->cur_max_spring_compress = ((p_joint_ext-0.5/4.096*65536) * this->piston_area_ext ) * 2.1547177056884764e-05 / this->spring_k; // unit in mm, piston area=0 for air reservoir
    this->cur_max_spring_compress = (p_joint_ext * this->piston_area_ext - p_joint_flex * this->piston_area_flex) * 2.1547177056884764e-05 / this->spring_k; // unit in mm, piston area=0 for air reservoir
    this->cur_delta_x = this->max_pos - pos;

    // calculate current volume
    // TODO: add the spring's effect back, but right now just forget about it
    this->L_ext = pos * this->volume_slope_6in + this->volume_intercept_6in;
    this->L_flex = this->max_len_mm - this->L_ext;

    this->cur_force = this->force_filter.GetFilteredMea(std::array<double, 1>{this->GetExternalForce(p_joint_ext, p_joint_flex, this->cur_delta_x, this->pos_diff)})[0];
    this->cur_pre_force = this->force_pre_filter.GetFilteredMea(std::array<double,1>{(p_joint_ext * this->piston_area_ext - p_joint_flex * this->piston_area_flex) * 2.1547177056884764e-05 - this->fric_coeff * this->pos_diff})[0];
    this->p_ext_rec_diff = this->p_ext_rec_diff_filter.GetFilteredMea(std::array<double,1>{p_joint_ext-p_joint_rec})[0];


    this->joint_con_rec.PushData(std::array<double, 9>{this->L_ext, this->L_flex, this->cur_force, this->cur_max_spring_compress, this->cur_delta_x, this->pos_diff, this->des_force,this->cur_pre_force,this->des_ext_pre});
}
void JointCon::RecData()
{
    this->ext_con.RecData();
    this->flex_con.RecData();
    this->tank_con.RecData();
}

void JointCon::GetForceCon(const std::array<double, MPC_TIME_HORIZON> &des_force, u_int8_t &ext_duty, u_int8_t &rec_duty, u_int8_t &tank_duty)
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

    // std::cout<<"desired force: "<<des_force[0]<<std::endl;
    // std::cout<<"current force: "<<this->cur_force<<std::endl;
    // std::cout<<"des pre: ";
    std::array<double, MPC_TIME_HORIZON> des_pre;
    for (unsigned i = 0; i < des_pre.size(); i++)
    {
        des_pre[i] = ((des_force[i] + this->fric_coeff * this->pos_diff) / 2.1547177056884764e-05) / this->piston_area_ext + 8000;
        // des_pre[i] = ((des_force[i]) / 2.1547177056884764e-05) / this->piston_area_ext + 8000;
        // std::cout<<des_pre[i]<<',';
    }
    this->des_force = des_force[0];
    this->des_ext_pre = des_pre[0];
    // std::cout<<std::endl;
    // std::cout<<"current pressure: "<<this->pre_ext<<std::endl;
    // std::cout<<"force gap: "<<des_force[0]-this->cur_force<<std::endl;
    if (std::abs(des_force[0] - this->cur_force) > JointCon::kForceTol)
    {

        if ((des_force[0] > this->cur_force) && (this->cur_force > 0))
        {
            // the des_pre is definitely larger than pre_ext, otherwise des_force < this->cur_force

            // to charge the cylinder, we need to make sure the tank is > than des_pre
            if (this->pre_tank < des_pre[0])
            {
                // std::cout << "increase tank pressure\n";
                // std::cout<<"ltank pre: "<<this->pre_tank<<", des pre: "<<des_pre[0]<<std::endl;
                tank_duty = this->tank_con.GetPreControl(des_pre, this->pre_tank, pre_main_tank, 1);
                ext_duty = 0;
            }
            else
            {
                // std::cout << "increase cylinder pressure\n";
                // still charge the cylinder with whatever pressure in the tank
                ext_duty = this->ext_con.GetPreControl(des_pre, this->pre_ext, this->pre_tank, this->max_len_mm / this->GetLenLinear_mm(this->cur_pos));
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
            if (this->pre_ext < this->pre_tank)
            {
                if(this->p_ext_rec_diff>160){
                    rec_duty = this->flex_con.GetPreControl(des_pre, this->pre_ext, this->pre_rec, this->max_len_mm / this->GetLenLinear_mm(this->cur_pos));
                    // rec_duty = this->flex_con.GetPreControl(des_pre, this->pre_ext, this->pre_rec, 0.6);
                    // std::cout<<"recycle to rec cylinder\n";
                }
                else{
                    rec_duty=0;
                    // std::cout<<"recycle cylinder pressure is too high\n";
                }
                ext_duty = 0;
                tank_duty = 0;
            }
            else
            {
                // we can recycle the energy to the tank
                // std::cout<<"recycle to ltank\n";
                ext_duty = this->ext_con.GetPreControl(des_pre, this->pre_ext, this->pre_tank, this->max_len_mm / this->GetLenLinear_mm(this->cur_pos));
                rec_duty = 0;
                tank_duty = 0;
            }
        }
    }
    else
    {
        // std::cout<<"force is close enough\n";
        ext_duty = 0;
        rec_duty = 0;
        tank_duty = 0;
    }
}

double JointCon::GetExternalForce(double pre_ext, double pre_flex, double delta_x, double x_dot)
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

    double delta_x_switch = delta_x - this->cur_max_spring_compress / this->volume_slope_6in;
    // It turned out the the spring start to compress earlier, the 4700 is an experimental value
    double coeff_0 = 1 / (1 + exp(-0.1 * delta_x_switch));
    double coeff_1 = 1 - coeff_0;

    double f_0 = coeff_0 * ((pre_ext * this->piston_area_ext - pre_flex * this->piston_area_flex) * 2.1547177056884764e-05 - this->fric_coeff * x_dot);
    double f_1 = coeff_1 * (delta_x * this->volume_slope_6in * this->spring_k-this->fric_coeff*x_dot);
    return f_0 + f_1;
}

double JointCon::GetLenLinear_mm(double pos)
{
    return pos * this->volume_slope_6in + this->volume_intercept_6in;
}

void JointCon::GetImpCon(double des_imp, u_int8_t &ext_duty, u_int8_t &rec_duty, u_int8_t &tank_duty, double force_offset)
{
    // steps:
    //        1. calculate desired force based on current position
    //        2. calculate desired pressure based on current velocity and desired force

    // check if we need to switch Imp_FSM

    double des_force = des_imp * this->cur_delta_x * this->volume_slope_6in;
    double des_force_step = des_imp * this->pos_diff * this->volume_slope_6in;

    std::array<double, MPC_TIME_HORIZON> des_force_array;
    for (int i = 0; i < MPC_TIME_HORIZON; i++)
    {
        des_force_array[i] = des_force + force_offset;
        des_force -= des_force_step; // TODO: right now I just make it 5% more in the future, note that it will fail when the impedance is negative
    }
    
    this->GetForceCon(des_force_array, ext_duty, rec_duty, tank_duty);
}

void JointCon::GetPreCon(const double des_pre, u_int8_t &duty, Chamber chamber)
{
    std::array<double, MPC_TIME_HORIZON> des_pre_array;
    std::fill_n(des_pre_array.begin(), des_pre_array.size(), des_pre);
    switch (chamber)
    {

    case JointCon::Chamber::kExt:

        duty = this->ext_con.GetPreControl(des_pre_array, this->pre_ext, this->pre_tank, this->max_len_mm / this->GetLenLinear_mm(this->cur_pos));
        break;
    case JointCon::Chamber::kFlex:
        break;
    case JointCon::Chamber::kTank:
        duty = this->tank_con.GetPreControl(des_pre_array, this->pre_tank, this->pre_main_tank, 1);
        break;

    default:
        duty = 0;
    }
}

void JointCon::SetControlMode(JointCon::ControlMode con_mode)
{
    this->control_mode = con_mode;
}
const JointCon::ControlMode JointCon::GetControlMode()
{
    return this->control_mode;
}

double JointCon::GetPre_KPa(double pre_adc)
{
    return (pre_adc / 65536 * 4.096 - 0.5) * 50 * 6.89476;
}

void JointCon::GetImpactCon(const double init_force, const double init_imp, u_int8_t &ext_duty, u_int8_t &rec_duty, u_int8_t &tank_duty, u_int8_t &flex_duty, u_int8_t &exhaust_duty)
{
   
    flex_duty = 0;
    exhaust_duty = 0;
    
    switch (this->imp_fsm_state)
    {

    case Imp_FSM::kCompress_inc:
        std::cout<<"fsm: compress inc\n";
        if (this->L_flex > 5)
        {
            this->imp_fsm_state = Imp_FSM::kCompress_dec;
            std::cout << "switch to compress dec\n";
        }

        break;
    case Imp_FSM::kCompress_dec:
        std::cout<<"fsm: compress dec\n";
        if (this->pos_diff > this->vel_th)
        {
            this->imp_fsm_state = Imp_FSM::kExtend;
            this->recover_imp = (this->cur_force-init_force) / this->L_flex;
            std::cout << "switch to extend\n";
        }

        break;
    case Imp_FSM::kExtend:
        std::cout<<"fsm: extend\n";
        if(std::abs(this->pos_diff)<this->vel_th){
            this->imp_fsm_state=Imp_FSM::kLoadPrep;
            std::cout<<"switch to load prep\n";
        }
        break;
    case Imp_FSM::kLoadPrep:
        std::cout<<"fsm: load prep\n";
        // if(this->pos_diff<-this->vel_th){
        //     this->imp_fsm_state = Imp_FSM::kCompress;
        //     std::cout<<"switch to compress\n";
        // }
        if (this->cur_force >= init_force)
        {
            this->imp_fsm_state = Imp_FSM::kCompress_inc;
            std::cout << "switch to compress inc\n";
        }
        break;
    default:
        break;
    }

    if (this->imp_fsm_state == Imp_FSM::kLoadPrep)
    {
        double des_pre = (init_force / 2.1547177056884764e-05) / this->piston_area_ext + 8000;
        this->GetPreCon(des_pre, ext_duty, Chamber::kExt);
        rec_duty = 0;
        tank_duty = 0;
        flex_duty = 0;
    }

    else if (this->imp_fsm_state == Imp_FSM::kCompress_inc)
    {
        // std::cout << "init force: " << init_force << std::endl;
        // std::cout << "current force: " << this->cur_force << std::endl;
        this->GetImpCon(init_imp, ext_duty, rec_duty, tank_duty, init_force);
        flex_duty = 0;
        std::cout << "rec duty: " << (int)rec_duty << std::endl;
    }
    else if (this->imp_fsm_state == Imp_FSM::kCompress_dec)
    {
        this->GetImpCon(-init_imp, ext_duty, rec_duty, tank_duty, init_force);
        flex_duty = 0;
        std::cout<<"rec duty: " << (int)rec_duty<<std::endl;
    }
    else if (this->imp_fsm_state == Imp_FSM::kExtend)
    {
        // double rec_imp = this->cur_force / this->cur_delta_x;
        // this->GetImpCon(this->recover_imp, ext_duty, rec_duty, tank_duty);
        ext_duty = 0;
        
        tank_duty = 0;
        flex_duty = 0;
        if(this->pre_rec-this->pre_ext>200){
            rec_duty=100;
        }
        else{
            rec_duty=0;
        }
    }
    else if (this->imp_fsm_state == Imp_FSM::kFree)
    {
        ext_duty = 0;
        rec_duty = 0;
        tank_duty = 0;
        flex_duty = 100;
        exhaust_duty = 100;
    }
}
