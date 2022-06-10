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


    float x0 = 1.0/p_l[0];
    float x1 = p_h[0]*u[0];
    float x2 = x0*x1;
    float x3 = 1.0/p_l[1];
    float x4 = p_h[1]*u[1];
    float x5 = x3*x4;
    float x6 = 1.0/p_l[2];
    float x7 = p_h[2]*u[2];
    float x8 = x6*x7;
    float x9 = 1.0/p_l[3];
    float x10 = p_h[3]*u[3];
    float x11 = x10*x9;
    float x12 = 1.0/p_l[4];
    float x13 = p_h[4]*u[4];
    float x14 = x12*x13;
    float x15 = (u[1]*u[1]);
    float x16 = p_h[1]*x15;
    float x17 = x16*x3;
    float x18 = (u[2]*u[2]);
    float x19 = p_h[2]*x18;
    float x20 = x19*x6;
    float x21 = (u[3]*u[3]);
    float x22 = p_h[3]*x21;
    float x23 = x22*x9;
    float x24 = (u[4]*u[4]);
    float x25 = p_h[4]*x24;
    float x26 = x12*x25;
    float x27 = (u[0]*u[0]);
    float x28 = p_h[0]*x27;
    float x29 = x0*x28;
    float x30 = -p_l[1]/p_h[1];
    float x31 = x30 + 1;
    float x32 = u[1]*x31;
    float x33 = -p_l[2]/p_h[2];
    float x34 = x33 + 1;
    float x35 = u[2]*x34;
    float x36 = -p_l[3]/p_h[3];
    float x37 = x36 + 1;
    float x38 = u[3]*x37;
    float x39 = -p_l[0]/p_h[0];
    float x40 = x39 + 1;
    float x41 = u[0]*x40;
    float x42 = -p_l[4]/p_h[4];
    float x43 = x42 + 1;
    float x44 = u[4]*x43;
    float x45 = x30 + 3;
    float x46 = x4*x45;
    float x47 = x33 + 3;
    float x48 = x47*x7;
    float x49 = x39 + 3;
    float x50 = x1*x49;
    float x51 = x36 + 3;
    float x52 = x10*x51;
    float x53 = x42 + 3;
    float x54 = x13*x53;
    float x55 = x27*x40;
    float x56 = -(p_l[1]*p_l[1])/(p_h[1]*p_h[1]);
    float x57 = x56 + 1;
    float x58 = u[1]*x57;
    float x59 = 1 - (p_l[1]*p_l[1]*p_l[1])/(p_h[1]*p_h[1]*p_h[1]);
    float x60 = u[1]*x59;
    float x61 = x15*x31;
    float x62 = -(p_l[2]*p_l[2])/(p_h[2]*p_h[2]);
    float x63 = x62 + 1;
    float x64 = u[2]*x63;
    float x65 = 1 - (p_l[2]*p_l[2]*p_l[2])/(p_h[2]*p_h[2]*p_h[2]);
    float x66 = u[2]*x65;
    float x67 = x18*x34;
    float x68 = -(p_l[3]*p_l[3])/(p_h[3]*p_h[3]);
    float x69 = x68 + 1;
    float x70 = u[3]*x69;
    float x71 = 1 - (p_l[3]*p_l[3]*p_l[3])/(p_h[3]*p_h[3]*p_h[3]);
    float x72 = u[3]*x71;
    float x73 = x21*x37;
    float x74 = -(p_l[4]*p_l[4])/(p_h[4]*p_h[4]);
    float x75 = x74 + 1;
    float x76 = u[4]*x75;
    float x77 = 1 - (p_l[4]*p_l[4]*p_l[4])/(p_h[4]*p_h[4]*p_h[4]);
    float x78 = u[4]*x77;
    float x79 = x24*x43;
    float x80 = -(p_l[0]*p_l[0])/(p_h[0]*p_h[0]);
    float x81 = x80 + 1;
    float x82 = u[0]*x81;
    float x83 = 1 - (p_l[0]*p_l[0]*p_l[0])/(p_h[0]*p_h[0]*p_h[0]);
    float x84 = u[0]*x83;
    float x85 = p_h[1]*x58;
    float x86 = x4*(x56 + 2);
    float x87 = x56 + 3;
    float x88 = x4*x87;
    float x89 = p_h[0]*x82;
    float x90 = x16*x45;
    float x91 = p_h[2]*x64;
    float x92 = x1*(x80 + 2);
    float x93 = x7*(x62 + 2);
    float x94 = x62 + 3;
    float x95 = x7*x94;
    float x96 = x19*x47;
    float x97 = p_h[3]*x70;
    float x98 = x10*(x68 + 2);
    float x99 = x68 + 3;
    float x100 = x10*x99;
    float x101 = x80 + 3;
    float x102 = x1*x101;
    float x103 = x22*x51;
    float x104 = p_h[4]*x76;
    float x105 = x13*(x74 + 2);
    float x106 = x74 + 3;
    float x107 = x106*x13;
    float x108 = x25*x53;
    float x109 = x28*x49;
    float x110 = x27*x81;
    float x111 = x27*x83;
    float x112 = x15*x57;
    float x113 = x15*x59;
    float x114 = x18*x63;
    float x115 = x18*x65;
    float x116 = x21*x69;
    float x117 = x21*x71;
    float x118 = x24*x75;
    float x119 = x24*x77;
    float x120 = x101*x28;
    float x121 = x16*x87;
    float x122 = x19*x94;
    float x123 = x22*x99;
    float x124 = x106*x25;

    this->Phi<<a[0]*x2 + a[10]*x120 + a[11]*x55 + a[12]*x110 + a[13]*x111 + a[14]*x5 + a[15]*x85 + a[16]*x86 + a[17]*x46 + a[18]*x88 + a[19]*x32 + a[1]*x89 + a[20]*x58 + a[21]*x60 + a[22]*x17 + a[23]*x90 + a[24]*x121 + a[25]*x61 + a[26]*x112 + a[27]*x113 + a[28]*x8 + a[29]*x91 + a[2]*x92 + a[30]*x93 + a[31]*x48 + a[32]*x95 + a[33]*x35 + a[34]*x64 + a[35]*x66 + a[36]*x20 + a[37]*x96 + a[38]*x122 + a[39]*x67 + a[3]*x50 + a[40]*x114 + a[41]*x115 + a[42]*x11 + a[43]*x97 + a[44]*x98 + a[45]*x52 + a[46]*x100 + a[47]*x38 + a[48]*x70 + a[49]*x72 + a[4]*x102 + a[50]*x23 + a[51]*x103 + a[52]*x123 + a[53]*x73 + a[54]*x116 + a[55]*x117 + a[56]*x14 + a[57]*x104 + a[58]*x105 + a[59]*x54 + a[5]*x41 + a[60]*x107 + a[61]*x44 + a[62]*x76 + a[63]*x78 + a[64]*x26 + a[65]*x108 + a[66]*x124 + a[67]*x79 + a[68]*x118 + a[69]*x119 + a[6]*x82 + a[7]*x84 + a[8]*x29 + a[9]*x109, b[0]*x2 + b[10]*x120 + b[11]*x55 + b[12]*x110 + b[13]*x111 + b[14]*x5 + b[15]*x85 + b[16]*x86 + b[17]*x46 + b[18]*x88 + b[19]*x32 + b[1]*x89 + b[20]*x58 + b[21]*x60 + b[22]*x17 + b[23]*x90 + b[24]*x121 + b[25]*x61 + b[26]*x112 + b[27]*x113 + b[28]*x8 + b[29]*x91 + b[2]*x92 + b[30]*x93 + b[31]*x48 + b[32]*x95 + b[33]*x35 + b[34]*x64 + b[35]*x66 + b[36]*x20 + b[37]*x96 + b[38]*x122 + b[39]*x67 + b[3]*x50 + b[40]*x114 + b[41]*x115 + b[42]*x11 + b[43]*x97 + b[44]*x98 + b[45]*x52 + b[46]*x100 + b[47]*x38 + b[48]*x70 + b[49]*x72 + b[4]*x102 + b[50]*x23 + b[51]*x103 + b[52]*x123 + b[53]*x73 + b[54]*x116 + b[55]*x117 + b[56]*x14 + b[57]*x104 + b[58]*x105 + b[59]*x54 + b[5]*x41 + b[60]*x107 + b[61]*x44 + b[62]*x76 + b[63]*x78 + b[64]*x26 + b[65]*x108 + b[66]*x124 + b[67]*x79 + b[68]*x118 + b[69]*x119 + b[6]*x82 + b[7]*x84 + b[8]*x29 + b[9]*x109;

}
void MPC::Update_dPhi_dxL(const std::array<float,MPC_DELAY>& p_h, const std::array<float,MPC_DELAY> &p_l,const std::array<float,MPC_DELAY> &u
                        ,const std::array<float,MPC_STATE_NUM>& a,const std::array<float,MPC_STATE_NUM> &b){
    
    float x0 = a[59]*u[4];
    float x1 = (u[4]*u[4]);
    float x2 = a[65]*x1;
    float x3 = 1.0/p_h[4];
    float x4 = u[4]*x3;
    float x5 = (p_l[4]*p_l[4]);
    float x6 = p_h[4]/x5;
    float x7 = u[4]*x6;
    float x8 = x1*x3;
    float x9 = p_l[4]*x3;
    float x10 = 2*x9;
    float x11 = u[4]*x10;
    float x12 = (p_h[4]);
    float x13 = p_l[4]*x12;
    float x14 = u[4]*x13;
    float x15 = 2*a[62];
    float x16 = x1*x6;
    float x17 = x5/(p_h[4]*p_h[4]*p_h[4]);
    float x18 = 3*u[4];
    float x19 = a[63]*x18;
    float x20 = a[66]*x1;
    float x21 = x1*x13;
    float x22 = 2*a[68];
    float x23 = x1*x17;
    float x24 = 3*a[69];
    float x25 = 1.0/p_l[4];
    float x26 = u[4]*x25;
    float x27 = x1*x25;
    float x28 = x12*x5;
    float x29 = 2*x28;
    float x30 = u[4]*x29;
    float x31 = u[4]*x17;
    float x32 = (p_l[4]*p_l[4]*p_l[4])/(p_h[4]*p_h[4]*p_h[4]*p_h[4]);
    float x33 = 3 - x9;
    float x34 = x1*x32;
    float x35 = -x28;
    float x36 = u[4]*(x35 + 1);
    float x37 = u[4]*(x35 + 2);
    float x38 = x35 + 3;
    float x39 = u[4]*x38;
    float x40 = x1*x38;
    float x41 = b[59]*u[4];
    float x42 = b[65]*x1;
    float x43 = 2*b[62];
    float x44 = b[63]*x18;
    float x45 = b[66]*x1;
    float x46 = 2*b[68];
    float x47 = 3*b[69];


    this->dPhi_dx_T<<-a[56]*x7 - a[57]*x11 - a[58]*x11 - a[60]*x11 - a[61]*x4 - a[64]*x16 - a[67]*x8 - x0 - x10*x20 - x14*x15 - x17*x19 - x2 - x21*x22 - x23*x24, a[56]*x26 + a[57]*x30 + a[57]*x36 + a[58]*x30 + a[58]*x37 + a[60]*x30 + a[60]*x39 + a[61]*x14 + a[64]*x27 + a[66]*x40 + a[67]*x21 + x0*x33 + x0*x9 + x15*x31 + x19*x32 + x2*x33 + x2*x9 + x20*x29 + x22*x23 + x24*x34, -b[56]*x7 - b[57]*x11 - b[58]*x11 - b[60]*x11 - b[61]*x4 - b[64]*x16 - b[67]*x8 - x10*x45 - x14*x43 - x17*x44 - x21*x46 - x23*x47 - x41 - x42, b[56]*x26 + b[57]*x30 + b[57]*x36 + b[58]*x30 + b[58]*x37 + b[60]*x30 + b[60]*x39 + b[61]*x14 + b[64]*x27 + b[66]*x40 + b[67]*x21 + x23*x46 + x29*x45 + x31*x43 + x32*x44 + x33*x41 + x33*x42 + x34*x47 + x41*x9 + x42*x9;
    
}

void MPC::Update_dPhi_dxH(const std::array<float,MPC_DELAY>& p_h,const std::array<float,MPC_DELAY> &p_l,const std::array<float,MPC_DELAY>& u
                          ,const std::array<float,MPC_STATE_NUM>& a,const std::array<float,MPC_STATE_NUM> &b) {
    float x0 = 1.0/p_l[4];
    float x1 = 1.0/p_h[4];
    float x2 = p_l[4]*x1;
    float x3 = a[59]*u[4];
    float x4 = (p_h[4]);
    float x5 = p_l[4]*x4;
    float x6 = u[4]*x5;
    float x7 = (u[4]*u[4]);
    float x8 = a[65]*x7;
    float x9 = x5*x7;
    float x10 = a[57]*u[4];
    float x11 = (p_l[4]*p_l[4]);
    float x12 = x11*x4;
    float x13 = 2*x12;
    float x14 = a[58]*u[4];
    float x15 = a[60]*u[4];
    float x16 = x11/(p_h[4]*p_h[4]*p_h[4]);
    float x17 = 2*u[4];
    float x18 = 3*u[4];
    float x19 = (p_l[4]*p_l[4]*p_l[4])/(p_h[4]*p_h[4]*p_h[4]*p_h[4]);
    float x20 = 3 - x2;
    float x21 = a[66]*x7;
    float x22 = x16*x7;
    float x23 = -x12;
    float x24 = x23 + 3;
    float x25 = a[56]*u[4]*x0 + a[61]*x6 + a[62]*x16*x17 + a[63]*x18*x19 + a[64]*x0*x7 + a[67]*x9 + 2*a[68]*x22 + 3*a[69]*x19*x7 + x10*x13 + x10*(x23 + 1) + x13*x14 + x13*x15 + x13*x21 + x14*(x23 + 2) + x15*x24 + x2*x3 + x2*x8 + x20*x3 + x20*x8 + x21*x24;
    float x26 = p_h[4]/x11;
    float x27 = x17*x2;
    float x28 = -b[56]*u[4]*x26 - b[57]*x27 - b[58]*x27 - b[59]*u[4] - b[60]*x27 - b[61]*u[4]*x1 - 2*b[62]*x6 - b[63]*x16*x18 - b[64]*x26*x7 - b[65]*x7 - 2*b[66]*x2*x7 - b[67]*x1*x7 - 2*b[68]*x9 - 3*b[69]*x22;

    this->dPhi_dx_T<<x25, x25, x28, x28;

}
void MPC::Update_dPhi_du(const std::array<float,MPC_DELAY>& p_h,const std::array<float,MPC_DELAY> &p_l,const std::array<float,MPC_DELAY>& u
                        ,const std::array<float,MPC_STATE_NUM>& a,const std::array<float,MPC_STATE_NUM> &b){
    float x0 = p_h[4]/p_l[4];
    float x1 = 2*u[4];
    float x2 = x0*x1;
    float x3 = -p_l[4]/p_h[4];
    float x4 = x3 + 1;
    float x5 = p_h[4]*(x3 + 3);
    float x6 = -(p_l[4]*p_l[4])/(p_h[4]*p_h[4]);
    float x7 = x6 + 1;
    float x8 = 1 - (p_l[4]*p_l[4]*p_l[4])/(p_h[4]*p_h[4]*p_h[4]);
    float x9 = x1*x4;
    float x10 = p_h[4]*x7;
    float x11 = p_h[4]*(x6 + 2);
    float x12 = p_h[4]*(x6 + 3);
    float x13 = x1*x5;
    float x14 = x1*x7;
    float x15 = x1*x8;
    float x16 = x1*x12;

    this->dPhi_du_T<<a[56]*x0 + a[57]*x10 + a[58]*x11 + a[59]*x5 + a[60]*x12 + a[61]*x4 + a[62]*x7 + a[63]*x8 + a[64]*x2 + a[65]*x13 + a[66]*x16 + a[67]*x9 + a[68]*x14 + a[69]*x15, b[56]*x0 + b[57]*x10 + b[58]*x11 + b[59]*x5 + b[60]*x12 + b[61]*x4 + b[62]*x7 + b[63]*x8 + b[64]*x2 + b[65]*x13 + b[66]*x16 + b[67]*x9 + b[68]*x14 + b[69]*x15;
    
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







int MPC::GetControl(const u_int16_t& p_des,const u_int16_t& ps, const u_int16_t& pt){
    



    int p_diff = (p_des - ps);
    
    
    if(std::abs(p_diff)>262){ //if desired pressure has 1 psi difference, Caution: calculate the diff does not need to consider the 0.5V dc bias
        
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

        double lb = 20; //set the lower bound 20 duty

        
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
        this->q_val = 2*(this->B.coeff(1,0)*this->alpha.coeff(1,0)-p_diff/65536*this->B.coeff(1,0))*100;
        this->P_val = 2*this->B.coeff(1,0)*this->B.coeff(1,0);//scale up the u to duty instead of duty/100
        

       
        c_int P_nnz = 1;
        c_float P_x[1]={P_val};
        c_int P_i[1]={0};
        c_int P_p[2]={0,1};
        this->osqp_data->n =1;
        this->osqp_data->P = csc_matrix(this->osqp_data->n,this->osqp_data->n,P_nnz,P_x,P_i,P_p);


        // std::cout<<"pval: "<<this->P_val<<std::endl;
        // std::cout<<"qval: "<<this->q_val<<std::endl;
        float ideal_duty = -this->q_val/this->P_val+0.5;

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

std::array<float,4> MPC::GetMpcRec(){
    return std::array<float,4>({this->Phi.coeff(0,0),this->Phi.coeff(1,0),this->P_val,this->q_val});
}



void MPC::PushPreMeas(const u_int16_t p_tank,const u_int16_t p_set,const u_int16_t duty)
{
    this->p_tank_mem[this->meas_idx] = ((float)p_tank - 6553.6)/65536;
    this->p_set_mem[this->meas_idx]=((float)p_set - 6553.6)/65536;
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






