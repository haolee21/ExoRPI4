#include <cmath>
#include <iostream>

#include "MPC.hpp"
#include "MPC_param.hpp"

using namespace std;

// const float MPC::kArea =  0.31*645.16f;  //unit: mm^2

MPC::MPC(std::array<std::array<float, MPC_STATE_NUM>, 2> cl, std::array<std::array<float, MPC_STATE_NUM>, 2> ch)
    : ah(ch[0]), bh(ch[1]), al(cl[0]), bl(cl[1])

{
    // this->max_pos = param.max_pos;
    // this->max_len_mm = this->GetLenLinear_mm(max_pos);

    // setup osqp solver
    //  this->osqp_data.reset(new OSQPData);
    //  this->osqp_settings.reset(new OSQPSettings);
    //  osqp_set_default_settings(this->osqp_settings.get());
    //  this->osqp_settings->alpha = 1.0;
    //  this->osqp_settings->verbose = false;

    this->phi_scale = 1.0;
    // this->pre_pos = 0.0;
}

MPC::~MPC()
{
}
void MPC::UpdateParamH(array<float, MPC_STATE_NUM> new_a, array<float, MPC_STATE_NUM> new_b)
{

    this->ah = new_a;
    this->bh = new_b;
}
void MPC::UpdateParamL(array<float, MPC_STATE_NUM> new_a, array<float, MPC_STATE_NUM> new_b)
{
    this->al = new_a;
    this->bl = new_b;
}
void MPC::UpdatePhi(const std::array<float, MPC_DELAY> p_h, const std::array<float, MPC_DELAY> p_l,
                    const std::array<float, MPC_DELAY> u, const std::array<float, MPC_STATE_NUM> &a, const std::array<float, MPC_STATE_NUM> &b,
                    Eigen::Matrix<float, 2, 1> &_phi)
{
    // Record the old Phi

    float x0 = p_h[0] * u[0];
    float x1 = x0 / p_l[0];
    float x2 = p_h[2] * u[2];
    float x3 = x2 / p_l[2];
    float x4 = p_h[3] * u[3];
    float x5 = x4 / p_l[3];
    float x6 = p_h[1] * u[1];
    float x7 = x6 / p_l[1];
    float x8 = -p_l[1] / p_h[1];
    float x9 = x6 * (x8 + 3);
    float x10 = -p_l[2] / p_h[2];
    float x11 = x2 * (x10 + 1);
    float x12 = x2 * (x10 + 3);
    float x13 = -p_l[0] / p_h[0];
    float x14 = x0 * (x13 + 1);
    float x15 = -p_l[3] / p_h[3];
    float x16 = x4 * (x15 + 1);
    float x17 = x4 * (x15 + 3);
    float x18 = x0 * (x13 + 3);
    float x19 = x6 * (x8 + 1);
    float x20 = -(p_l[1] * p_l[1]) / (p_h[1] * p_h[1]);
    float x21 = x20 + 1;
    float x22 = u[1] * x21;
    float x23 = -(p_l[2] * p_l[2]) / (p_h[2] * p_h[2]);
    float x24 = x23 + 1;
    float x25 = u[2] * x24;
    float x26 = -(p_l[3] * p_l[3]) / (p_h[3] * p_h[3]);
    float x27 = x26 + 1;
    float x28 = u[3] * x27;
    float x29 = -(p_l[0] * p_l[0]) / (p_h[0] * p_h[0]);
    float x30 = x29 + 1;
    float x31 = u[0] * x30;
    float x32 = p_h[1] * x22;
    float x33 = x6 * (x20 + 3);
    float x34 = p_h[2] * x25;
    float x35 = x2 * (x23 + 3);
    float x36 = p_h[3] * x28;
    float x37 = x4 * (x26 + 3);
    float x38 = p_h[0] * x31;
    float x39 = x0 * (x29 + 3);
    float x40 = (u[1] * u[1]);
    float x41 = x21 * x40;
    float x42 = x40 * (1 - (p_l[1] * p_l[1] * p_l[1]) / (p_h[1] * p_h[1] * p_h[1]));
    float x43 = (u[2] * u[2]);
    float x44 = x24 * x43;
    float x45 = x43 * (1 - (p_l[2] * p_l[2] * p_l[2]) / (p_h[2] * p_h[2] * p_h[2]));
    float x46 = (u[3] * u[3]);
    float x47 = x27 * x46;
    float x48 = x46 * (1 - (p_l[3] * p_l[3] * p_l[3]) / (p_h[3] * p_h[3] * p_h[3]));
    float x49 = (u[0] * u[0]);
    float x50 = x30 * x49;
    float x51 = x49 * (1 - (p_l[0] * p_l[0] * p_l[0]) / (p_h[0] * p_h[0] * p_h[0]));

    _phi << a[0] * x1 + a[10] * x9 + a[11] * x32 + a[12] * x33 + a[13] * x22 + a[14] * x41 + a[15] * x42 + a[16] * x3 + a[17] * x11 + a[18] * x12 + a[19] * x34 + a[1] * x14 + a[20] * x35 + a[21] * x25 + a[22] * x44 + a[23] * x45 + a[24] * x5 + a[25] * x16 + a[26] * x17 + a[27] * x36 + a[28] * x37 + a[29] * x28 + a[2] * x18 + a[30] * x47 + a[31] * x48 + a[3] * x38 + a[4] * x39 + a[5] * x31 + a[6] * x50 + a[7] * x51 + a[8] * x7 + a[9] * x19, b[0] * x1 + b[10] * x9 + b[11] * x32 + b[12] * x33 + b[13] * x22 + b[14] * x41 + b[15] * x42 + b[16] * x3 + b[17] * x11 + b[18] * x12 + b[19] * x34 + b[1] * x14 + b[20] * x35 + b[21] * x25 + b[22] * x44 + b[23] * x45 + b[24] * x5 + b[25] * x16 + b[26] * x17 + b[27] * x36 + b[28] * x37 + b[29] * x28 + b[2] * x18 + b[30] * x47 + b[31] * x48 + b[3] * x38 + b[4] * x39 + b[5] * x31 + b[6] * x50 + b[7] * x51 + b[8] * x7 + b[9] * x19;
}
void MPC::Update_dPhi_dxL(const std::array<float, MPC_DELAY> &p_h, const std::array<float, MPC_DELAY> &p_l, const std::array<float, MPC_DELAY> &u, const std::array<float, MPC_STATE_NUM> &a, const std::array<float, MPC_STATE_NUM> &b)
{

    float x0 = a[25] * u[3];
    float x1 = a[26] * u[3];
    float x2 = (p_l[3] * p_l[3]);
    float x3 = p_h[3] * u[3] / x2;
    float x4 = p_l[3] / p_h[3];
    float x5 = 2 * u[3];
    float x6 = x4 * x5;
    float x7 = (p_h[3]);
    float x8 = p_l[3] * x7;
    float x9 = a[29] * x5;
    float x10 = (u[3] * u[3]);
    float x11 = 2 * x10;
    float x12 = a[30] * x11;
    float x13 = x2 / (p_h[3] * p_h[3] * p_h[3]);
    float x14 = 3 * x10;
    float x15 = a[31] * x14;
    float x16 = u[3] / p_l[3];
    float x17 = x2 * x7;
    float x18 = x17 * x5;
    float x19 = -x4;
    float x20 = x19 + 1;
    float x21 = x19 + 3;
    float x22 = (p_l[3] * p_l[3] * p_l[3]) / (p_h[3] * p_h[3] * p_h[3] * p_h[3]);
    float x23 = -x17;
    float x24 = u[3] * (x23 + 1);
    float x25 = u[3] * (x23 + 3);
    float x26 = b[25] * u[3];
    float x27 = b[26] * u[3];
    float x28 = b[29] * x5;
    float x29 = b[30] * x11;
    float x30 = b[31] * x14;

    this->dPhi_dx_T << -a[24] * x3 - a[27] * x6 - a[28] * x6 - x0 - x1 - x12 * x8 - x13 * x15 - x8 * x9, a[24] * x16 + a[27] * x18 + a[27] * x24 + a[28] * x18 + a[28] * x25 + x0 * x20 + x0 * x4 + x1 * x21 + x1 * x4 + x12 * x13 + x13 * x9 + x15 * x22, -b[24] * x3 - b[27] * x6 - b[28] * x6 - x13 * x30 - x26 - x27 - x28 * x8 - x29 * x8, b[24] * x16 + b[27] * x18 + b[27] * x24 + b[28] * x18 + b[28] * x25 + x13 * x28 + x13 * x29 + x20 * x26 + x21 * x27 + x22 * x30 + x26 * x4 + x27 * x4;
}

void MPC::Update_dPhi_dxH(const std::array<float, MPC_DELAY> &p_h, const std::array<float, MPC_DELAY> &p_l, const std::array<float, MPC_DELAY> &u, const std::array<float, MPC_STATE_NUM> &a, const std::array<float, MPC_STATE_NUM> &b)
{
    float x0 = u[3] / p_l[3];
    float x1 = p_l[3] / p_h[3];
    float x2 = a[25] * u[3];
    float x3 = a[26] * u[3];
    float x4 = (p_h[3]);
    float x5 = (p_l[3] * p_l[3]);
    float x6 = x4 * x5;
    float x7 = 2 * u[3];
    float x8 = x6 * x7;
    float x9 = x5 / (p_h[3] * p_h[3] * p_h[3]);
    float x10 = a[29] * x7;
    float x11 = -x1;
    float x12 = x11 + 1;
    float x13 = x11 + 3;
    float x14 = (u[3] * u[3]);
    float x15 = x14 * x9;
    float x16 = 2 * a[30];
    float x17 = 3 * a[31];
    float x18 = (p_l[3] * p_l[3] * p_l[3]) * x14 / (p_h[3] * p_h[3] * p_h[3] * p_h[3]);
    float x19 = -x6;
    float x20 = u[3] * (x19 + 1);
    float x21 = u[3] * (x19 + 3);
    float x22 = p_h[3] * u[3] / x5;
    float x23 = x1 * x7;
    float x24 = p_l[3] * x4;
    float x25 = x14 * x24;
    float x26 = b[25] * u[3];
    float x27 = b[26] * u[3];
    float x28 = b[29] * x7;
    float x29 = 2 * b[30];
    float x30 = 3 * b[31];
    this->dPhi_dx_T << a[24] * x0 + a[27] * x20 + a[27] * x8 + a[28] * x21 + a[28] * x8 + x1 * x2 + x1 * x3 + x10 * x9 + x12 * x2 + x13 * x3 + x15 * x16 + x17 * x18, -a[24] * x22 - a[27] * x23 - a[28] * x23 - x10 * x24 - x15 * x17 - x16 * x25 - x2 - x3, b[24] * x0 + b[27] * x20 + b[27] * x8 + b[28] * x21 + b[28] * x8 + x1 * x26 + x1 * x27 + x12 * x26 + x13 * x27 + x15 * x29 + x18 * x30 + x28 * x9, -b[24] * x22 - b[27] * x23 - b[28] * x23 - x15 * x30 - x24 * x28 - x25 * x29 - x26 - x27;
}
void MPC::Update_dPhi_du(const std::array<float, MPC_DELAY> &p_h, const std::array<float, MPC_DELAY> &p_l, const std::array<float, MPC_DELAY> &u, const std::array<float, MPC_STATE_NUM> &a, const std::array<float, MPC_STATE_NUM> &b)
{
    float x0 = p_h[3] / p_l[3];
    float x1 = -p_l[3] / p_h[3];
    float x2 = p_h[3] * (x1 + 1);
    float x3 = p_h[3] * (x1 + 3);
    float x4 = -(p_l[3] * p_l[3]) / (p_h[3] * p_h[3]);
    float x5 = x4 + 1;
    float x6 = p_h[3] * x5;
    float x7 = p_h[3] * (x4 + 3);
    float x8 = 2 * u[3];
    float x9 = x5 * x8;
    float x10 = x8 * (1 - (p_l[3] * p_l[3] * p_l[3]) / (p_h[3] * p_h[3] * p_h[3]));

    this->dPhi_du_T << a[24] * x0 + a[25] * x2 + a[26] * x3 + a[27] * x6 + a[28] * x7 + a[29] * x5 + a[30] * x9 + a[31] * x10, b[24] * x0 + b[25] * x2 + b[26] * x3 + b[27] * x6 + b[28] * x7 + b[29] * x5 + b[30] * x9 + b[31] * x10;
}

void MPC::UpdateDyn(bool increase_pre)
{
    this->UpdateHistory();
    if (increase_pre)
    {
        // if we are increasing the pressure

        this->UpdatePhi(this->p_tank_his, this->p_set_his, this->u_his, this->ah, this->bh, this->Phi);
        this->UpdatePhi(this->p_tank_hat, this->p_set_hat, this->u_hat, this->ah, this->bh, this->Phi_hat);
        this->Update_dPhi_dxH(this->p_tank_hat, this->p_set_hat, this->u_hat, this->ah, this->bh);
        this->Update_dPhi_du(this->p_tank_hat, this->p_set_hat, this->u_hat, this->ah, this->bh);
    }
    else
    {
        this->UpdatePhi(this->p_set_his, this->p_tank_his, this->u_his, this->al, this->bl, this->Phi);
        this->UpdatePhi(this->p_set_hat, this->p_tank_hat, this->u_hat, this->al, this->bl, this->Phi_hat);
        this->Update_dPhi_dxL(this->p_set_hat, this->p_tank_hat, this->u_hat, this->al, this->bl);
        this->Update_dPhi_du(this->p_set_hat, this->p_tank_hat, this->u_hat, this->al, this->bl);
    }

    // Scale the phi, dphi_du, dphi_dx since the volume may change
    this->Phi = this->Phi / this->phi_scale;
    this->Phi_hat = this->Phi_hat / this->phi_scale;
    this->dPhi_du_T = this->dPhi_du_T / this->phi_scale;
    this->dPhi_dx_T = this->dPhi_dx_T / this->phi_scale;

    Eigen::Matrix2f K_mat = Eigen::Matrix2f::Identity() - this->dPhi_dx_T;

    // std::cout<<"dPhi_dx: "<<this->dPhi_dx_T<<std::endl;
    // std::cout<<"dPhi_du: "<<this->dPhi_du_T<<std::endl;
    // std::cout<<"K_mat: "<<K_mat<<std::endl;
    // std::cout<<"Phi: "<<this->Phi<<std::endl;

    this->B = K_mat.inverse() * this->dPhi_du_T;
    this->alpha = K_mat.inverse() * (this->Phi_hat - this->dPhi_du_T * this->u_hat[MPC_DELAY - 1]);
}

int MPC::GetPreControl(const double &p_des, const double &ps, const double &pt, float scale)
{

    double p_diff = (p_des - ps) / 2; // we scaled the p_diff with the assumption that pressure will have the momentum to go

    if (std::abs(p_diff) > 320)
    { // if desired pressure has 1 psi difference, Caution: calculate the diff does not need to consider the 0.5V dc bias

        // double lb;
        // if(std::abs(pt-ps)<1310){ //if the difference is less than 10 psi, we can operate the valve with lower duty
        //     lb = 10.0;
        // }
        // else if(std::abs(pt-ps)<2621){ //20 psi
        //     lb = 20.0;
        // }

        // else{
        //     lb=60.0;
        // }

        // double lb = 20; //set the lower bound 20 duty

        if ((p_des > ps) & (pt > ps) & (pt > p_des))
        {
            // increasing pressure
            //  std::cout<<"increase\n";
            this->UpdateDyn(true);
        }
        else if ((p_des < ps) & (ps > pt) & (p_des > pt))
        {
            // decreasing pressure
            //  std::cout<<"decrease\n";
            this->UpdateDyn(false);
        }
        else
        {
            this->Phi << 0, 0;
            this->P_val = 0;
            this->q_val = 0;
            return 0;
        }

        // format question to osqp format
        // std::cout<<"diff: "<<p_diff<<std::endl;
        this->q_val = 2 * (this->B.coeff(1, 0) * this->alpha.coeff(1, 0) - p_diff / 65536 * this->B.coeff(1, 0));
        this->P_val = 2 * this->B.coeff(1, 0) * this->B.coeff(1, 0); // scale up the u to duty instead of duty/100, since q_val/100 and p_val/10000, I just scale p_val/100

        // c_int P_nnz = 1;
        // c_float P_x[1]={P_val};
        // c_int P_i[1]={0};
        // c_int P_p[2]={0,1};
        // this->osqp_data->n =1;
        // this->osqp_data->P = csc_matrix(this->osqp_data->n,this->osqp_data->n,P_nnz,P_x,P_i,P_p);

        // std::cout<<"pval: "<<this->P_val<<std::endl;
        // std::cout<<"qval: "<<this->q_val<<std::endl;
        // float ideal_duty = -this->q_val/this->P_val+0.5; //I scale the result by 100, for some reason the real gradient is way larger TODO: figure out why!
        float ideal_duty = -100 * this->q_val / this->P_val + 0.5;
        // std::cout<<"ideal duty: "<<ideal_duty<<std::endl;
        if (ideal_duty < 20)
        {
            return 20;
        }
        else if (ideal_duty > 100)
            return 100;
        else
        {
            return ideal_duty;
        }

        /* osqp is not working......


        c_float q[1]={q_val};
        this->osqp_data->q = q;
        // std::cout<<"pval: "<<p_val<<std::endl;

        c_int A_nnz = 1;
        this->osqp_data->m = 1;
        c_float A_x[1]={1};
        c_int A_i[1]={0};
        c_int A_p[2]={0,1};
        this->osqp_data->A = csc_matrix(this->osqp_data->m,this->osqp_data->n,A_nnz,A_x,A_i,A_p);

        c_float l[1]={lb};
        c_float u[1]={100};

        this->osqp_data->l = l;
        this->osqp_data->u = u;



        int set_up_flag = osqp_setup(&this->work,this->osqp_data.get(),this->osqp_settings.get());

        int solve_err=0;
        int duty=0;
        if(!set_up_flag)
            solve_err = osqp_solve(this->work);
        if(!solve_err){
            duty = *this->work->solution->x+0.5;
            // if(duty<30)
            //     duty =0;
        }
        else{
            std::cout<<"osqp failed\n";
        }


        // std::cout<<"pval: "<<p_val<<std::endl;
        // std::cout<<"qval: "<<q_val<<std::endl;
        // std::cout<<"ps: "<<ps<<std::endl;
        // std::cout<<"pt: "<<pt<<std::endl;
        // std::cout<<"p_des: "<<p_des<<std::endl;

        return duty;
        */
    }
    else
    {
        this->Phi << 0, 0;
        this->P_val = 0;
        this->q_val = 0;
        return 0;
    }
}

std::array<double, 10> MPC::GetMpcRec()
{ // record dPhi_du, dPhi_dx
    return std::array<double, 10>({this->Phi.coeff(0, 0), this->Phi.coeff(1, 0), this->P_val, this->q_val, this->dPhi_du_T.coeff(0, 0), this->dPhi_du_T.coeff(1, 0), this->dPhi_dx_T.coeff(0, 0), this->dPhi_dx_T.coeff(0, 1), this->dPhi_dx_T.coeff(1, 0), this->dPhi_dx_T.coeff(1, 1)});
}

void MPC::PushMeas(const double p_tank, const double p_set, const double duty)
{
    this->p_tank_mem[this->meas_idx] = ((float)p_tank - 3297.312) / 65536.0; // the substraction is to remove the 0.5 V pressure sensor bias and add 1 atm to the equation
    this->p_set_mem[this->meas_idx] = ((float)p_set - 3297.312) / 65536.0;   // in the lasso regression, we have proved it increases the testing accuracy to stable 90% up
    this->u_mem[this->meas_idx] = (float)duty / 100;
    this->meas_idx++;
    if (this->meas_idx >= MPC_DELAY)
    {
        this->meas_idx = 0;
    }

    // //also calculate force
    // this->pre_pos = this->cur_pos;
    // this->cur_pos = _pos;
    // double temp_pos_diff = this->cur_pos-this->pre_pos;
    // // this->pos_diff = (this->vel_filter.GetFilteredMea(std::array<double,1>{temp_pos_diff}))[0];
    // this->pos_diff = temp_pos_diff;

    // this->cur_max_spring_compress = (p_ext-p_flex)*2.1547177056884764e-05*this->piston_area/this->spring_k; //unit in mm, piston area=0 for air reservoir
    // this->cur_force =  this->force_filter.GetFilteredMea(std::array<double,1>{this->GetExternalForce(p_ext,p_flex,_pos)})[0];
}

void MPC::UpdateHistory()
{

    for (int i = 0; i < MPC_DELAY - 1; i++)
    {
        this->p_tank_his[i] = this->p_tank_mem[(this->meas_idx + i) % MPC_DELAY];
        this->p_set_his[i] = this->p_set_mem[(this->meas_idx + i) % MPC_DELAY];
        this->u_his[i] = this->u_mem[(this->meas_idx + i) % MPC_DELAY];

        this->p_tank_hat[i] = this->p_tank_mem[(this->meas_idx + i + 1) % MPC_DELAY];
        this->p_set_hat[i] = this->p_set_mem[(this->meas_idx + i + 1) % MPC_DELAY];
        this->u_hat[i] = this->u_mem[(this->meas_idx + i + 1) % MPC_DELAY];
    }

    this->p_tank_his[MPC_DELAY - 1] = this->p_tank_mem[(this->meas_idx + MPC_DELAY - 1) % MPC_DELAY];
    this->p_set_his[MPC_DELAY - 1] = this->p_set_mem[(this->meas_idx + MPC_DELAY - 1) % MPC_DELAY];
    this->u_his[MPC_DELAY - 1] = this->u_mem[(this->meas_idx + MPC_DELAY - 1) % MPC_DELAY];

    this->p_tank_hat[MPC_DELAY - 1] = this->p_tank_mem[(this->meas_idx + MPC_DELAY - 1) % MPC_DELAY];
    this->p_set_hat[MPC_DELAY - 1] = this->p_set_mem[(this->meas_idx + MPC_DELAY - 1) % MPC_DELAY];
    this->u_hat[MPC_DELAY - 1] = this->u_mem[(this->meas_idx + MPC_DELAY - 1) % MPC_DELAY];

    // std::cout<<"mem values: "<<this->p_tank_his[0]<<std::endl;
}

// double MPC::GetLenLinear_mm(double pos){ //TODO: this only fit linear case, need to do nonlinear equation when using it on the exoskeleton
//     return pos*this->volume_slope_6in+this->volume_intercept_6in; //152.4 is 6 in cylinder max length
//                                                                                //TODO: need to consider 5" cylinder
//                                                                                //6.71 is just the offset of linear encoder
// }

// double MPC::GetExternalForce(double pre_ext,double pre_flex,double x){
//     //return the external force in newton

//     double cur_delta_x =this->max_pos-x;
//     if((cur_delta_x-4700)>this->cur_max_spring_compress/this->volume_slope_6in){ //It turned out the the spring start to compress earlier, the 4700 is an experimental value

//         return (pre_ext-pre_flex)*2.1547177056884764e-05*this->piston_area-this->fric_coeff*this->pos_diff; //unit newton
//         // return (pre/65536*4.096-0.5)/4*200*0.31-0.001*this->pos_diff;
//     }
//     else{
//         return (this->max_pos-x)*this->volume_slope_6in*this->spring_k;// unit: newton
//         // return (this->max_pos-x)*0.0006351973436310972*this->spring_k/25.4;
//     }

// }

// void MPC::SetCylinderMaxPos(){
// this->max_pos = this->cur_pos;
// }
// double MPC::GetCylinderScale(double pre,double pos) //get the (cylinder length)/(max cylinder length)
// {

//     double cur_pos_mm = this->GetLenLinear_mm(pos);
//     double cur_len = this->max_len_mm - cur_pos_mm-4700*this->volume_slope_6in;
//     if(cur_len<this->cur_max_spring_compress){

//         return (cur_pos_mm-this->cur_max_spring_compress)/this->max_len_mm;
//     }
//     else{
//         return 1;
//     }

// }

// void MPC::GetImpControl(const double& imp_des, const double& p_ext,const double& p_flex, const double& p_tank, const double& pos,float scale,int& joint_val_duty,int& joint_bal_duty,int& tank_duty){
//     //the impedance controller will use the current velocity to estimate the displacement

//     //steps:
//     //       1. calculate desired force based on current position
//     //       2. calculate desired pressure based on current velocity and desired force
//     //       3. command desired pressure

//     // if((this->pos_diff*this->volume_slope_6in<0.01)&&(this->pos_diff*this->volume_slope_6in>-0.01)) return 0; //if it does not move more than 0.5mm, no control

//     double cur_delta_x = this->max_len_mm-this->GetLenLinear_mm(pos);
//     double des_force = imp_des*cur_delta_x;
//     double des_pre = (des_force+this->fric_coeff*this->pos_diff)/this->piston_area/2.1547177056884764e-05+8000;

//     // double p_des = (imp_des*this->pos_diff*this->volume_slope_6in)/this->piston_area*4.641208782079999e+04+p_cur;  //N/mm2 * in2/lbf * 4/200 * 2^16/4.096

//     joint_val_duty= this->GetPreControl(des_pre,p_ext,p_tank,scale);

// }
