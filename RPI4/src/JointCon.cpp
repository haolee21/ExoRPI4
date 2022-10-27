#include <math.h>
#include <osqp/osqp.h>
#include "JointCon.hpp"
JointCon::JointCon(PneumaticParam::CylinderParam ext_param, PneumaticParam::ReservoirParam reservoir_param, std::string joint_con_name)
    : ext_con(ext_param.cl_ext, ext_param.ch_ext, joint_con_name + "_ext"), flex_con(ext_param.cl_flex, ext_param.ch_flex, joint_con_name + "_flex"), tank_con(reservoir_param.cl, reservoir_param.ch, joint_con_name + "_tank"),
      piston_area_ext(ext_param.piston_area_ext), piston_area_flex(ext_param.piston_area_flex), fric_coeff(ext_param.fri_coeff), max_pos(ext_param.max_pos), vel_filter(FilterParam::Filter20Hz_2::a, FilterParam::Filter20Hz_2::b), force_filter(FilterParam::Filter15Hz_5::a, FilterParam::Filter15Hz_5::b),
      joint_con_rec(joint_con_name, "Time,L_ext,L_flex,cur_force,max_spring_compress,delta_x,x_dot")
{
    this->SetKneeMaxPos(ext_param.max_pos);
    this->imp_fsm_state = Imp_FSM::kCompress;
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
void JointCon::SetKneeMaxPos(double max_pos_val){
    this->max_pos = max_pos_val;
    this->max_len_mm = this->GetLenLinear_mm(max_pos_val); //TODO: need to modify this function when integrating to the exo
    // std::cout<<"max pos reset\n";
}
void JointCon::PushMeas(const double &p_joint_ext, const double &p_joint_flex, const double &p_joint_rec, const double &p_tank, const double &p_main_tank, const double &pos)
{
    this->ext_con.PushMeas(p_tank, p_joint_ext);
    this->flex_con.PushMeas(p_joint_rec, p_joint_ext);
    this->tank_con.PushMeas(p_main_tank, p_tank);

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

    this->joint_con_rec.PushData(std::array<double, 6>{this->L_ext, this->L_flex, this->cur_force, this->cur_max_spring_compress, this->cur_delta_x, this->pos_diff});
}
void JointCon::RecData()
{
    this->ext_con.RecData();
    this->flex_con.RecData();
    this->tank_con.RecData();
}

void JointCon::GetForceCon(const std::array<double, MPC_TIME_HORIZON> &des_force, u_int8_t &ext_duty, u_int8_t &flex_duty, u_int8_t &tank_duty)
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

    // std::cout<<"desired force: "<<des_force<<std::endl;
    // std::cout<<"current force: "<<this->cur_force<<std::endl;
    // std::cout<<"des pre: ";
    std::array<double, MPC_TIME_HORIZON> des_pre;
    for (unsigned i = 0; i < des_pre.size(); i++)
    {
        des_pre[i] = ((des_force[i] + this->fric_coeff * this->pos_diff) / 2.1547177056884764e-05) / this->piston_area_ext + 8000;
        // std::cout<<des_pre[i]<<',';
    }
    // std::cout<<std::endl;
    // std::cout<<"current pressure: "<<this->pre_ext<<std::endl;
    // std::cout<<"force gap: "<<des_force[0]-this->cur_force<<std::endl;
    if ((des_force[0] > this->cur_force) && (this->cur_force>0))
    {
        // the des_pre is definitely larger than pre_ext, otherwise des_force < this->cur_force

        // to charge the cylinder, we need to make sure the tank is > than des_pre
        if (this->pre_tank < des_pre[0])
        {
            // std::cout << "increase tank pressure\n";
            tank_duty = this->tank_con.GetPreControl(des_pre, this->pre_tank, pre_main_tank, 1);
            ext_duty = 0;
        }
        else
        {
            // std::cout << "increase cylinder pressure\n";
            // still charge the cylinder with whatever pressure in the tank
            ext_duty = this->ext_con.GetPreControl(des_pre, this->pre_ext, this->pre_tank, this->max_len_mm/this->GetLenLinear_mm(this->cur_pos));
            tank_duty = 0;

            // std::cout<<"desired force: "<<des_force[0]<<std::endl;
            // std::cout<<"current force: "<<this->cur_force<<std::endl;
            // std::cout << "desired duty: " << ((int)rec_duty) << std::endl;
            // std::cout << "desired pressure: " << des_pre[0] << std::endl;
            // std::cout << "current pressure: " << this->pre_ext << std::endl;
            // std::cout << "current tank pressure: " << this->pre_tank << std::endl;
        }
        flex_duty = 0;
    }
    else
    {
        // if the force is too high, we have 3 choices
        //  1. If pre_ext > pre_tank: pump the air in the cylinder back to reservior
        //  2. If pre_ext > pre_rec, pump the air to the recycle end
        if (this->pre_ext < this->pre_tank)
        {
            // we cannot recycle if the desired pressure is lower than the current tank pressure
            //  double des_force_mN = (des_force + this->fric_coeff * this->pos_diff)*1000; //to make our life easier, we use mN to match kPa and mm^2
            //  double p_ext_kpa = this->GetPre_KPa(this->pre_ext);

            // double c = (p_ext_kpa*this->piston_area_ext*this->L_ext+p_flex_kpa*this->piston_area_flex*this->L_flex);

            // double des_p_ext_num = this->piston_area_ext*this->L_ext*des_force_mN/this->L_flex+2*this->piston_area_ext*this->L_ext*c/this->L_flex/this->L_flex+2*this->piston_area_ext*des_force_mN+2*this->piston_area_ext*c/this->L_flex;
            // double des_p_ext_den = 2*this->piston_area_ext*this->piston_area_ext*this->L_ext*this->L_ext/this->L_flex/this->L_flex+4*this->piston_area_ext*this->piston_area_ext*this->L_ext/this->L_flex+2*this->piston_area_ext*this->piston_area_ext;
            // double des_p_ext_kpa = des_p_ext_num/des_p_ext_den;
            // double des_p_flex_kpa = (c-des_p_ext_kpa*this->L_ext*this->piston_area_ext)/(this->piston_area_flex*this->L_flex);

            // // //due to the way we program mpc, we only need p_flex_kpa's adc reading
            // double p_flex_adc = des_p_flex_kpa*46.41216+8000;
            // double p_ext_adc = des_p_ext_kpa*46.41216+8000;

            // double checking = des_p_ext_kpa*this->L_ext+des_p_flex_kpa*this->L_flex-p_ext_kpa*this->L_ext-p_flex_kpa*this->L_flex;
            // std::cout<<"checking: "<<checking<<std::endl;
            // std::cout<<"recycle to ankle:\n";
            // std::cout<<"pre_ext: "<<this->pre_ext<<", rec_tank pre: "<<this->pre_rec<<std::endl;
            flex_duty = this->flex_con.GetPreControl(des_pre, this->pre_ext, this->pre_rec, this->max_len_mm/this->GetLenLinear_mm(this->cur_pos));

            // flex_duty = this->flex_con.GetPreControl(p_flex_adc,this->pre_flex,this->pre_ext,1);
            // std::cout<<"flex duty: "<<(int)flex_duty<<std::endl;
            // std::cout<<"flex duty: "<<(int)flex_duty<<std::endl;//TODO: remove it later

            ext_duty = 0;
            tank_duty = 0;
        }
        else
        {
            // we can recycle the energy to the tank
            
            ext_duty = this->ext_con.GetPreControl(des_pre, this->pre_ext, this->pre_tank, this->max_len_mm/this->GetLenLinear_mm(this->cur_pos));
            flex_duty = 0;
            tank_duty = 0;
        }
    }

    // if ((des_force > this->cur_force) && (this->pre_tank > pre_ext))
    // {
    //     std::cout<<"need to increase force, tank pressure is sufficient\n";

    //     std::cout<<"des_pre: "<<des_pre<<std::endl;
    //     std::cout<<"cur_pre: "<<this->pre_ext<<std::endl;
    //     std::cout<<"tank pressure: "<<this->pre_tank<<std::endl;
    //     ext_duty = this->ext_con.GetPreControl(des_pre, this->pre_ext, this->pre_tank, this->GetLenLinear_mm(this->cur_pos) / this->max_len_mm);
    //     flex_duty = 0;
    //     tank_duty = 0;
    //     std::cout<<"ext duty: "<<(int)ext_duty<<std::endl;
    // }
    // else if ((des_force > this->cur_force) && (this->pre_tank < this->pre_ext))
    // {
    //     // std::cout<<"need to increase force, tank pressure is not sufficient\n";
    //     double des_pre = ((des_force + this->fric_coeff * this->pos_diff) / 2.1547177056884764e-05 + this->pre_flex * this->piston_area_flex) / this->piston_area_ext;
    //     tank_duty = this->tank_con.GetPreControl(des_pre, this->pre_tank, pre_main_tank, 1);
    //     flex_duty = 0;
    //     ext_duty = 0;
    //     std::cout<<"tank duty: "<<(int)tank_duty<<std::endl;
    // }
    // else if ((des_force < this->cur_force) && (this->pre_tank > this->pre_ext))
    // {
    //     // std::cout<<"need to decrease force but cannot recycle\n";
    //     double des_force_mN = (des_force + this->fric_coeff * this->pos_diff)*1000; //to make our life easier, we use mN to match kPa and mm^2
    //     // c_double P_x[3] = {this->piston_area_ext,this->piston_area_ext*this->piston_area_flex,this->piston_area_flex*this->piston_area_flex};
    //     // c_int P_nnz = 3;
    //     // c_int P_i[3] = {0,0,1};
    //     // c_int P_p[3]={0,1,3};
    //     // c_double q[2]={-des_force*this->piston_area_ext,des_force*this->piston_area_flex};

    //     // std::cout<<"num: "<<des_p_ext_num<<std::endl;
    //     // std::cout<<"den: "<<des_p_ext_den<<std::endl;
    //     // std::cout<<"des pre: "<<des_p_ext_num/des_p_ext_den<<std::endl;

    //     flex_duty=0;
    //     ext_duty=0;
    //     tank_duty=0;

    // }
    // else if ((des_force < this->cur_force) && (this->pre_tank < this->pre_ext))
    // {
    //     // std::cout<<"need to decrease force, energy recycle\n";
    //     double des_pre = ((des_force + this->fric_coeff * this->pos_diff) / 2.1547177056884764e-05 + this->pre_flex * this->piston_area_flex) / this->piston_area_ext;
    //     ext_duty = this->ext_con.GetPreControl(des_pre, this->pre_ext, this->pre_tank, this->GetLenLinear_mm(this->cur_pos) / this->max_len_mm);
    //     flex_duty = 0;
    //     tank_duty = 0;
    // }
    // else{
    //     std::cout<<"do nothing\n";
    // }
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
    double f_1 = coeff_1 * (delta_x * this->volume_slope_6in * this->spring_k);
    return f_0 + f_1;
}

double JointCon::GetLenLinear_mm(double pos)
{
    return pos * this->volume_slope_6in + this->volume_intercept_6in;
}

void JointCon::GetImpCon(double des_imp, u_int8_t &ext_duty, u_int8_t &rec_duty, u_int8_t &tank_duty)
{
    // steps:
    //        1. calculate desired force based on current position
    //        2. calculate desired pressure based on current velocity and desired force

    //check if we need to switch Imp_FSM
   
    if(this->imp_fsm_state == Imp_FSM::kCompress){
        if(this->pos_diff>this->vel_th){
            this->imp_fsm_state = Imp_FSM::kExtend;
            this->imp_deflect_point = this->cur_pos;
            // std::cout<<"extend\n";
        }
        
    }
    else{
        if(this->pos_diff< -1*this->vel_th){
            this->imp_fsm_state=Imp_FSM::kCompress;
            // std::cout<<"compressed\n";
        }
    }

    //if current Imp_FSM state is extension, change des_imp to kExtImp
    // if(this->imp_fsm_state==Imp_FSM::kExtend){

    //     des_imp = (this->max_pos-this->cur_pos)*des_imp+(this->cur_pos-this->imp_deflect_point)*this->kExtImp/(this->max_pos-this->imp_deflect_point);
    //     // des_imp = this->kExtImp;
    //     std::cout<<"ext imp: "<<des_imp<<std::endl;
    // }
    

    

    double des_force = des_imp * this->cur_delta_x * this->volume_slope_6in;
    double des_force_step = des_imp*this->pos_diff*this->volume_slope_6in;

    
    std::array<double, MPC_TIME_HORIZON> des_force_array;
    for (int i = 0; i < MPC_TIME_HORIZON; i++)
    {
        des_force_array[i] = des_force;
        des_force += des_force_step; // TODO: right now I just make it 5% more in the future, note that it will fail when the impedance is negative
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

        duty = this->ext_con.GetPreControl(des_pre_array, this->pre_ext, this->pre_tank, this->max_len_mm/this->GetLenLinear_mm(this->cur_pos));
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
