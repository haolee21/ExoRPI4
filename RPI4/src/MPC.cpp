#include <cmath>
#include <iostream>

#include "MPC.hpp"
#include "MPC_param.hpp"

using namespace std;
MPC::MPC(array<array<float,MPC_STATE_NUM>,2> init_cl,array<array<float,MPC_STATE_NUM>,2> init_ch)
{

    //load default values for the model parameters, it is data from one of the experiment
    this->ah = init_ch[0];
    this->bh = init_ch[1];

    this->al = init_cl[0];
    this->bl = init_cl[1];


    
    //setup osqp solver
    this->osqp_data.reset(new OSQPData);
    this->osqp_settings.reset(new OSQPSettings);
    osqp_set_default_settings(this->osqp_settings.get());
    this->osqp_settings->alpha = 1.0;
    this->osqp_settings->verbose = false;
   
    
}

MPC::~MPC()
{
}
void MPC::UpdateParamH(array<float,MPC_STATE_NUM> new_a,array<float,MPC_STATE_NUM> new_b){

        
    this->ah = new_a;
    this->bh = new_b;

}
void MPC::UpdateParamL(array<float,MPC_STATE_NUM> new_a,array<float,MPC_STATE_NUM> new_b){
    this->al = new_a;
    this->bl = new_b;
}
void MPC::UpdatePhi(const std::array<float,MPC_DELAY> p_h,const std::array<float,MPC_DELAY> p_l,
        const std::array<float,MPC_DELAY> u,const std::array<float,MPC_STATE_NUM>& a,const std::array<float,MPC_STATE_NUM> &b){
    //Record the old Phi

    float x0 = p_h[0]*u[0];
    float x1 = x0/p_l[0];
    float x2 = p_h[2]*u[2];
    float x3 = x2/p_l[2];
    float x4 = p_h[3]*u[3];
    float x5 = x4/p_l[3];
    float x6 = p_h[4]*u[4];
    float x7 = x6/p_l[4];
    float x8 = p_h[5]*u[5];
    float x9 = x8/p_l[5];
    float x10 = p_h[6]*u[6];
    float x11 = x10/p_l[6];
    float x12 = p_h[7]*u[7];
    float x13 = x12/p_l[7];
    float x14 = p_h[8]*u[8];
    float x15 = x14/p_l[8];
    float x16 = p_h[9]*u[9];
    float x17 = x16/p_l[9];
    float x18 = p_h[1]*u[1];
    float x19 = x18/p_l[1];
    float x20 = -(p_l[2]*p_l[2])/(p_h[2]*p_h[2]);
    float x21 = x20 + 1;
    float x22 = u[2]*x21;
    float x23 = -(p_l[3]*p_l[3])/(p_h[3]*p_h[3]);
    float x24 = x23 + 1;
    float x25 = u[3]*x24;
    float x26 = -(p_l[4]*p_l[4])/(p_h[4]*p_h[4]);
    float x27 = x26 + 1;
    float x28 = u[4]*x27;
    float x29 = -(p_l[5]*p_l[5])/(p_h[5]*p_h[5]);
    float x30 = x29 + 1;
    float x31 = u[5]*x30;
    float x32 = -(p_l[6]*p_l[6])/(p_h[6]*p_h[6]);
    float x33 = x32 + 1;
    float x34 = u[6]*x33;
    float x35 = -(p_l[0]*p_l[0])/(p_h[0]*p_h[0]);
    float x36 = x35 + 1;
    float x37 = u[0]*x36;
    float x38 = -(p_l[7]*p_l[7])/(p_h[7]*p_h[7]);
    float x39 = x38 + 1;
    float x40 = u[7]*x39;
    float x41 = -(p_l[8]*p_l[8])/(p_h[8]*p_h[8]);
    float x42 = x41 + 1;
    float x43 = u[8]*x42;
    float x44 = -(p_l[9]*p_l[9])/(p_h[9]*p_h[9]);
    float x45 = x44 + 1;
    float x46 = u[9]*x45;
    float x47 = -(p_l[1]*p_l[1])/(p_h[1]*p_h[1]);
    float x48 = x47 + 1;
    float x49 = u[1]*x48;
    float x50 = p_h[2]*x22;
    float x51 = x2*(x20 + 3);
    float x52 = p_h[3]*x25;
    float x53 = p_h[0]*x37;
    float x54 = x4*(x23 + 3);
    float x55 = p_h[4]*x28;
    float x56 = x6*(x26 + 3);
    float x57 = x0*(x35 + 3);
    float x58 = p_h[5]*x31;
    float x59 = x8*(x29 + 3);
    float x60 = p_h[6]*x34;
    float x61 = x10*(x32 + 3);
    float x62 = p_h[7]*x40;
    float x63 = x12*(x38 + 3);
    float x64 = p_h[8]*x43;
    float x65 = x14*(x41 + 3);
    float x66 = p_h[9]*x46;
    float x67 = x16*(x44 + 3);
    float x68 = p_h[1]*x49;
    float x69 = x18*(x47 + 3);
    float x70 = (u[1]*u[1]);
    float x71 = x48*x70;
    float x72 = x70*(1 - (p_l[1]*p_l[1]*p_l[1])/(p_h[1]*p_h[1]*p_h[1]));
    float x73 = (u[2]*u[2]);
    float x74 = x21*x73;
    float x75 = x73*(1 - (p_l[2]*p_l[2]*p_l[2])/(p_h[2]*p_h[2]*p_h[2]));
    float x76 = (u[3]*u[3]);
    float x77 = x24*x76;
    float x78 = x76*(1 - (p_l[3]*p_l[3]*p_l[3])/(p_h[3]*p_h[3]*p_h[3]));
    float x79 = (u[4]*u[4]);
    float x80 = x27*x79;
    float x81 = x79*(1 - (p_l[4]*p_l[4]*p_l[4])/(p_h[4]*p_h[4]*p_h[4]));
    float x82 = (u[5]*u[5]);
    float x83 = x30*x82;
    float x84 = x82*(1 - (p_l[5]*p_l[5]*p_l[5])/(p_h[5]*p_h[5]*p_h[5]));
    float x85 = (u[6]*u[6]);
    float x86 = x33*x85;
    float x87 = x85*(1 - (p_l[6]*p_l[6]*p_l[6])/(p_h[6]*p_h[6]*p_h[6]));
    float x88 = (u[7]*u[7]);
    float x89 = x39*x88;
    float x90 = x88*(1 - (p_l[7]*p_l[7]*p_l[7])/(p_h[7]*p_h[7]*p_h[7]));
    float x91 = (u[0]*u[0]);
    float x92 = x36*x91;
    float x93 = (u[8]*u[8]);
    float x94 = x42*x93;
    float x95 = x93*(1 - (p_l[8]*p_l[8]*p_l[8])/(p_h[8]*p_h[8]*p_h[8]));
    float x96 = (u[9]*u[9]);
    float x97 = x45*x96;
    float x98 = x96*(1 - (p_l[9]*p_l[9]*p_l[9])/(p_h[9]*p_h[9]*p_h[9]));
    float x99 = x91*(1 - (p_l[0]*p_l[0]*p_l[0])/(p_h[0]*p_h[0]*p_h[0]));

    this->Phi<<a[0]*x1 + a[10]*x71 + a[11]*x72 + a[12]*x3 + a[13]*x50 + a[14]*x51 + a[15]*x22 + a[16]*x74 + a[17]*x75 + a[18]*x5 + a[19]*x52 + a[1]*x53 + a[20]*x54 + a[21]*x25 + a[22]*x77 + a[23]*x78 + a[24]*x7 + a[25]*x55 + a[26]*x56 + a[27]*x28 + a[28]*x80 + a[29]*x81 + a[2]*x57 + a[30]*x9 + a[31]*x58 + a[32]*x59 + a[33]*x31 + a[34]*x83 + a[35]*x84 + a[36]*x11 + a[37]*x60 + a[38]*x61 + a[39]*x34 + a[3]*x37 + a[40]*x86 + a[41]*x87 + a[42]*x13 + a[43]*x62 + a[44]*x63 + a[45]*x40 + a[46]*x89 + a[47]*x90 + a[48]*x15 + a[49]*x64 + a[4]*x92 + a[50]*x65 + a[51]*x43 + a[52]*x94 + a[53]*x95 + a[54]*x17 + a[55]*x66 + a[56]*x67 + a[57]*x46 + a[58]*x97 + a[59]*x98 + a[5]*x99 + a[6]*x19 + a[7]*x68 + a[8]*x69 + a[9]*x49, b[0]*x1 + b[10]*x71 + b[11]*x72 + b[12]*x3 + b[13]*x50 + b[14]*x51 + b[15]*x22 + b[16]*x74 + b[17]*x75 + b[18]*x5 + b[19]*x52 + b[1]*x53 + b[20]*x54 + b[21]*x25 + b[22]*x77 + b[23]*x78 + b[24]*x7 + b[25]*x55 + b[26]*x56 + b[27]*x28 + b[28]*x80 + b[29]*x81 + b[2]*x57 + b[30]*x9 + b[31]*x58 + b[32]*x59 + b[33]*x31 + b[34]*x83 + b[35]*x84 + b[36]*x11 + b[37]*x60 + b[38]*x61 + b[39]*x34 + b[3]*x37 + b[40]*x86 + b[41]*x87 + b[42]*x13 + b[43]*x62 + b[44]*x63 + b[45]*x40 + b[46]*x89 + b[47]*x90 + b[48]*x15 + b[49]*x64 + b[4]*x92 + b[50]*x65 + b[51]*x43 + b[52]*x94 + b[53]*x95 + b[54]*x17 + b[55]*x66 + b[56]*x67 + b[57]*x46 + b[58]*x97 + b[59]*x98 + b[5]*x99 + b[6]*x19 + b[7]*x68 + b[8]*x69 + b[9]*x49;

}
void MPC::Update_dPhi_dxL(const std::array<float,MPC_DELAY>& p_h, const std::array<float,MPC_DELAY> &p_l,const std::array<float,MPC_DELAY> &u
                        ,const std::array<float,MPC_STATE_NUM>& a,const std::array<float,MPC_STATE_NUM> &b){
    
    float x0 = (p_l[9]*p_l[9]);
    float x1 = p_h[9]*u[9]/x0;
    float x2 = 2*p_l[9];
    float x3 = u[9]*x2;
    float x4 = x3/p_h[9];
    float x5 = (p_h[9]);
    float x6 = x3*x5;
    float x7 = (u[9]*u[9]);
    float x8 = a[58]*x7;
    float x9 = x2*x5;
    float x10 = x0/(p_h[9]*p_h[9]*p_h[9]);
    float x11 = 3*x7;
    float x12 = a[59]*x11;
    float x13 = u[9]/p_l[9];
    float x14 = x0*x5;
    float x15 = 2*u[9];
    float x16 = x14*x15;
    float x17 = x10*x15;
    float x18 = 2*x10;
    float x19 = (p_l[9]*p_l[9]*p_l[9])/(p_h[9]*p_h[9]*p_h[9]*p_h[9]);
    float x20 = -x14;
    float x21 = u[9]*(x20 + 1);
    float x22 = u[9]*(x20 + 3);
    float x23 = b[58]*x7;
    float x24 = b[59]*x11;


    this->dPhi_dx_T<<-a[54]*x1 - a[55]*x4 - a[56]*x4 - a[57]*x6 - x10*x12 - x8*x9, a[54]*x13 + a[55]*x16 + a[55]*x21 + a[56]*x16 + a[56]*x22 + a[57]*x17 + x12*x19 + x18*x8, -b[54]*x1 - b[55]*x4 - b[56]*x4 - b[57]*x6 - x10*x24 - x23*x9, b[54]*x13 + b[55]*x16 + b[55]*x21 + b[56]*x16 + b[56]*x22 + b[57]*x17 + x18*x23 + x19*x24;
    
}

void MPC::Update_dPhi_dxH(const std::array<float,MPC_DELAY>& p_h,const std::array<float,MPC_DELAY> &p_l,const std::array<float,MPC_DELAY>& u
                          ,const std::array<float,MPC_STATE_NUM>& a,const std::array<float,MPC_STATE_NUM> &b) {
    float x0 = u[9]/p_l[9];
    float x1 = (p_h[9]);
    float x2 = (p_l[9]*p_l[9]);
    float x3 = x1*x2;
    float x4 = 2*u[9];
    float x5 = x3*x4;
    float x6 = x2/(p_h[9]*p_h[9]*p_h[9]);
    float x7 = a[57]*x4;
    float x8 = (u[9]*u[9]);
    float x9 = x6*x8;
    float x10 = 2*a[58];
    float x11 = 3*a[59];
    float x12 = (p_l[9]*p_l[9]*p_l[9])*x8/(p_h[9]*p_h[9]*p_h[9]*p_h[9]);
    float x13 = -x3;
    float x14 = u[9]*(x13 + 1);
    float x15 = u[9]*(x13 + 3);
    float x16 = p_h[9]*u[9]/x2;
    float x17 = p_l[9]*x4;
    float x18 = x17/p_h[9];
    float x19 = p_l[9]*x1;
    float x20 = x19*x8;
    float x21 = 2*b[58];
    float x22 = 3*b[59];
    this->dPhi_dx_T<<a[54]*x0 + a[55]*x14 + a[55]*x5 + a[56]*x15 + a[56]*x5 + x10*x9 + x11*x12 + x6*x7, -a[54]*x16 - a[55]*x18 - a[56]*x18 - x10*x20 - x11*x9 - x19*x7, b[54]*x0 + b[55]*x14 + b[55]*x5 + b[56]*x15 + b[56]*x5 + b[57]*x4*x6 + x12*x22 + x21*x9, -b[54]*x16 - b[55]*x18 - b[56]*x18 - b[57]*x1*x17 - x20*x21 - x22*x9;

}
void MPC::Update_dPhi_du(const std::array<float,MPC_DELAY>& p_h,const std::array<float,MPC_DELAY> &p_l,const std::array<float,MPC_DELAY>& u
                        ,const std::array<float,MPC_STATE_NUM>& a,const std::array<float,MPC_STATE_NUM> &b){
    float x0 = p_h[9]/p_l[9];
    float x1 = -(p_l[9]*p_l[9])/(p_h[9]*p_h[9]);
    float x2 = x1 + 1;
    float x3 = p_h[9]*x2;
    float x4 = p_h[9]*(x1 + 3);
    float x5 = 2*u[9];
    float x6 = x2*x5;
    float x7 = x5*(1 - (p_l[9]*p_l[9]*p_l[9])/(p_h[9]*p_h[9]*p_h[9]));

    this->dPhi_du_T<<a[54]*x0 + a[55]*x3 + a[56]*x4 + a[57]*x2 + a[58]*x6 + a[59]*x7, b[54]*x0 + b[55]*x3 + b[56]*x4 + b[57]*x2 + b[58]*x6 + b[59]*x7;
    
}


void MPC::UpdateDyn(bool increase_pre)
{
    this->UpdateHistory();
    if(increase_pre){
        //if we are increasing the pressure
        
        this->UpdatePhi(this->p_tank_his,this->p_set_his,this->u_his,this->ah,this->bh);
        this->Update_dPhi_dxH(this->p_tank_hat,this->p_set_hat,this->u_hat,this->ah,this->bh);
        this->Update_dPhi_du(this->p_tank_hat,this->p_set_hat,this->u_hat,this->ah,this->bh);
        
        

    }
    else{
        this->UpdatePhi(this->p_set_his,this->p_tank_his,this->u_his,this->al,this->bl);
        this->Update_dPhi_dxL(this->p_set_hat,this->p_tank_hat,this->u_hat,this->al,this->bl);
        this->Update_dPhi_du(this->p_set_hat,this->p_tank_hat,this->u_hat, this->al,this->bl);
    }


    Eigen::Matrix2f K_mat = 2*Eigen::Matrix2f::Identity()-this->dPhi_dx_T;
    
    // std::cout<<"dPhi_dx: "<<this->dPhi_dx_T<<std::endl;
    // std::cout<<"dPhi_du: "<<this->dPhi_du_T<<std::endl;
    // std::cout<<"K_mat: "<<K_mat<<std::endl;
    // std::cout<<"Phi: "<<this->Phi<<std::endl;

    this->B = K_mat.inverse()*this->dPhi_du_T;
    this->alpha = K_mat.inverse()*(this->Phi-this->dPhi_du_T*this->u_his[MPC_DELAY-1]);

    

}







int MPC::GetControl(const double& p_des,const double& ps, const double& pt){
    



    int p_diff = (p_des - ps);
    
    
    if(std::abs(p_diff)>320){ //if desired pressure has 1 psi difference, Caution: calculate the diff does not need to consider the 0.5V dc bias
        
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

        
        if((p_des>ps) & (pt>ps)){
            //increasing pressure
            // std::cout<<"increase\n";
            this->UpdateDyn(true);
        
        }
        else if((p_des<ps)&(ps>pt)){
            //decreasing pressure
            // std::cout<<"decrease\n";
            this->UpdateDyn(false);
        }
        else{
            this->Phi<<0,0;
            this->P_val=0;
            this->q_val=0;
            return 0;
        }
        
        // format question to osqp format
        // std::cout<<"diff: "<<p_diff<<std::endl;
        this->q_val = 2*(this->B.coeff(1,0)*this->alpha.coeff(1,0)-p_diff/65536*this->B.coeff(1,0));
        this->P_val = 2*this->B.coeff(1,0)*this->B.coeff(1,0)/100;//scale up the u to duty instead of duty/100, since q_val/100 and p_val/10000, I just scale p_val/100
        
        
       
        c_int P_nnz = 1;
        c_float P_x[1]={P_val};
        c_int P_i[1]={0};
        c_int P_p[2]={0,1};
        this->osqp_data->n =1;
        this->osqp_data->P = csc_matrix(this->osqp_data->n,this->osqp_data->n,P_nnz,P_x,P_i,P_p);


        // std::cout<<"pval: "<<this->P_val<<std::endl;
        // std::cout<<"qval: "<<this->q_val<<std::endl;
        float ideal_duty = -this->q_val/this->P_val+0.5; //for some reason the q_val's sign is opposite TODO: figure out why!

        // std::cout<<"ideal duty: "<<ideal_duty<<std::endl;
        if(ideal_duty<20){
            return 20;
        }
        else if(ideal_duty>100)
            return 100;
        else{
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
    else{
        this->Phi<<0,0;
        this->P_val=0;
        this->q_val=0;
        return 0;
    }

}

std::array<float,10> MPC::GetMpcRec(){ //record dPhi_du, dPhi_dx
    return std::array<float,10>({this->Phi.coeff(0,0),this->Phi.coeff(1,0),this->P_val,this->q_val
                                ,this->dPhi_du_T.coeff(0,0),this->dPhi_du_T.coeff(1,0)
                                ,this->dPhi_dx_T.coeff(0,0),this->dPhi_dx_T.coeff(0,1),this->dPhi_dx_T.coeff(1,0),this->dPhi_dx_T.coeff(1,1)});
}



void MPC::PushPreMeas(const double p_tank,const double p_set,const double duty)
{
    this->p_tank_mem[this->meas_idx] = ((float)p_tank - 8000)/65536;
    this->p_set_mem[this->meas_idx]=((float)p_set - 8000)/65536;
    this->u_mem[this->meas_idx]=(float)duty/100;
    this->meas_idx++;
    if(this->meas_idx>=MPC_DELAY){
        this->meas_idx=0;
    }
}

void MPC::UpdateHistory(){
    
    for(int i=0;i<MPC_DELAY-1;i++){
        this->p_tank_his[i]=this->p_tank_mem[(this->meas_idx+i)%MPC_DELAY];
        this->p_set_his[i]=this->p_set_mem[(this->meas_idx+i)%MPC_DELAY];
        this->u_his[i]=this->u_mem[(this->meas_idx+i)%MPC_DELAY];

        this->p_tank_hat[i]=this->p_tank_mem[(this->meas_idx+i+1)%MPC_DELAY];
        this->p_set_hat[i]=this->p_set_mem[(this->meas_idx+i+1)%MPC_DELAY];
        this->u_hat[i]=this->u_mem[(this->meas_idx+i+1)%MPC_DELAY];
    }

    this->p_tank_his[MPC_DELAY-1] = this->p_tank_mem[(this->meas_idx+MPC_DELAY-1)%MPC_DELAY];
    this->p_set_his[MPC_DELAY-1] = this->p_set_mem[(this->meas_idx+MPC_DELAY-1)%MPC_DELAY];
    this->u_his[MPC_DELAY-1] = this->u_mem[(this->meas_idx+MPC_DELAY-1)%MPC_DELAY];

    this->p_tank_hat[MPC_DELAY-1] = this->p_tank_mem[(this->meas_idx+MPC_DELAY-1)%MPC_DELAY];
    this->p_set_hat[MPC_DELAY-1] = this->p_set_mem[(this->meas_idx+MPC_DELAY-1)%MPC_DELAY];
    this->u_hat[MPC_DELAY-1] = this->u_mem[(this->meas_idx+MPC_DELAY-1)%MPC_DELAY];

    // std::cout<<"mem values: "<<this->p_tank_his[0]<<std::endl;


}






