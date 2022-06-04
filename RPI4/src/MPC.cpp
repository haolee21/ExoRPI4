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
void MPC::UpdatePhi(float p_h,float p_l,float d,const std::array<float,MPC_STATE_NUM>& a,const std::array<float,MPC_STATE_NUM> &b){
    float x0 = 1.0/p_l ;
    float x1 = d*p_h ;
    float x2 = x0*x1 ;
    float x3 = d*d ;
    float x4 = p_h*x3 ;
    float x5 = x0*x4 ;
    float x6 = -p_l/p_h ;
    float x7 = x6 + 1 ;
    float x8 = d*x7 ;
    float x9 = x6 + 3 ;
    float x10 = x1*x9 ;
    float x11 = -p_l*p_l/p_h/p_h ;
    float x12 = x11 + 1 ;
    float x13 = d*x12 ;
    float x14 = 1 - p_l*p_l*p_l/p_h/p_h/p_h ;
    float x15 = d*x14 ;
    float x16 = x3*x7 ;
    float x17 = p_h*x13 ;
    float x18 = x1*(x11 + 2) ;
    float x19 = x11 + 3 ;
    float x20 = x1*x19 ;
    float x21 = x4*x9 ;
    float x22 = x12*x3 ;
    float x23 = x14*x3 ;
    float x24 = x19*x4 ;


    this->Phi<< a[0]*x2 + a[1]*x17 + a[2]*x18 + a[3]*x10 + a[4]*x20 + a[5]*x8 + a[6]*x13 + a[7]*x15 + a[8]*x5 + a[9]*x21 + a[10]*x24 + a[11]*x16 + a[12]*x22 + a[13]*x23
              , b[0]*x2 + b[1]*x17 + b[2]*x18 + b[3]*x10 + b[4]*x20 + b[5]*x8 + b[6]*x13 + b[7]*x15 + b[8]*x5 + b[9]*x21 + b[10]*x24 + b[11]*x16 + b[12]*x22 + b[13]*x23;

}
void MPC::Update_dPhi_dxL(float p_h,float p_l,float d,const std::array<float,MPC_STATE_NUM>& a,const std::array<float,MPC_STATE_NUM> &b){
    float x0 = a[3]*d ;
    float x1 = d*d ;
    float x2 = a[9]*x1 ;
    float x3 = 1.0/p_h ;
    float x4 = d*x3 ;
    float x5 = p_l*p_l;
    float x6 = p_h/x5 ;
    float x7 = d*x6 ;
    float x8 = x1*x3 ;
    float x9 = p_l*x3 ;
    float x10 = 2*x9 ;
    float x11 = d*x10 ;
    float x12 = 1/p_h/p_h;
    float x13 = p_l*x12 ;
    float x14 = d*x13 ;
    float x15 = 2*a[6] ;
    float x16 = x1*x6 ;
    float x17 = x5/p_h/p_h/p_h;
    float x18 = 3*d ;
    float x19 = a[7]*x18 ;
    float x20 = a[10]*x1 ;
    float x21 = x1*x13 ;
    float x22 = 2*a[12] ;
    float x23 = x1*x17 ;
    float x24 = 3*a[13] ;
    float x25 = 1.0/p_l ;
    float x26 = d*x25 ;
    float x27 = x1*x25 ;
    float x28 = x12*x5 ;
    float x29 = 2*x28 ;
    float x30 = d*x29 ;
    float x31 = d*x17 ;
    float x32 = p_l*p_l*p_l/p_h/p_h/p_h/p_h;
    float x33 = 3 - x9 ;
    float x34 = x1*x32 ;
    float x35 = -x28 ;
    float x36 = d*(x35 + 1) ;
    float x37 = d*(x35 + 2) ;
    float x38 = x35 + 3 ;
    float x39 = d*x38 ;
    float x40 = x1*x38 ;
    float x41 = b[3]*d ;
    float x42 = b[9]*x1 ;
    float x43 = 2*b[6] ;
    float x44 = b[7]*x18 ;
    float x45 = b[10]*x1 ;
    float x46 = 2*b[12] ;
    float x47 = 3*b[13] ;

    this->dPhi_dx<< -a[0]*x7 - a[1]*x11 - a[2]*x11 - a[4]*x11 - a[5]*x4 - a[8]*x16 - a[11]*x8 - x0 - x10*x20 - x14*x15 - x17*x19 - x2 - x21*x22 - x23*x24
                  , a[0]*x26 + a[1]*x30 + a[1]*x36 + a[2]*x30 + a[2]*x37 + a[4]*x30 + a[4]*x39 + a[5]*x14 + a[8]*x27 + a[10]*x40 + a[11]*x21 + x0*x33 + x0*x9 + x15*x31 + x19*x32 + x2*x33 + x2*x9 + x20*x29 + x22*x23 + x24*x34
                  , -b[0]*x7 - b[1]*x11 - b[2]*x11 - b[4]*x11 - b[5]*x4 - b[8]*x16 - b[11]*x8 - x10*x45 - x14*x43 - x17*x44 - x21*x46 - x23*x47 - x41 - x42
                  , b[0]*x26 + b[1]*x30 + b[1]*x36 + b[2]*x30 + b[2]*x37 + b[4]*x30 + b[4]*x39 + b[5]*x14 + b[8]*x27 + b[10]*x40 + b[11]*x21 + x23*x46 + x29*x45 + x31*x43 + x32*x44 + x33*x41 + x33*x42 + x34*x47 + x41*x9 + x42*x9;

}
void MPC::Update_dPhi_duL(float p_h,float p_l,float d,const std::array<float,MPC_STATE_NUM>& a,const std::array<float,MPC_STATE_NUM> &b){
    float x0 = p_h/p_l ;
    float x1 = 2*d ;
    float x2 = x0*x1 ;
    float x3 = -p_l/p_h ;
    float x4 = x3 + 1 ;
    float x5 = p_h*(x3 + 3) ;
    float x6 = -p_l*p_l/p_h/p_h ;
    float x7 = x6 + 1 ;
    float x8 = 1 - p_l*p_l*p_l/p_h/p_h/p_h ;
    float x9 = x1*x4 ;
    float x10 = p_h*x7 ;
    float x11 = p_h*(x6 + 2) ;
    float x12 = p_h*(x6 + 3) ;
    float x13 = x1*x5 ;
    float x14 = x1*x7 ;
    float x15 = x1*x8 ;
    float x16 = x1*x12 ;
    this->dPhi_du<< a[0]*x0 + a[1]*x10 + a[2]*x11 + a[3]*x5 + a[4]*x12 + a[5]*x4 + a[6]*x7 + a[7]*x8 + a[8]*x2 + a[9]*x13 + a[10]*x16 + a[11]*x9 + a[12]*x14 + a[13]*x15
                  , b[0]*x0 + b[1]*x10 + b[2]*x11 + b[3]*x5 + b[4]*x12 + b[5]*x4 + b[6]*x7 + b[7]*x8 + b[8]*x2 + b[9]*x13 + b[10]*x16 + b[11]*x9 + b[12]*x14 + b[13]*x15;

}
void MPC::Update_dPhi_dxH(float p_h,float p_l,float d,const std::array<float,MPC_STATE_NUM>& a,const std::array<float,MPC_STATE_NUM> &b){
    float x0 = 1.0/p_l ;
    float x1 = d*x0 ;
    float x2 = 1.0/p_h ;
    float x3 = p_l*x2 ;
    float x4 = a[3]*d ;
    float x5 = 1/p_h/p_h;
    float x6 = p_l*x5 ;
    float x7 = d*x6 ;
    float x8 = d*d;
    float x9 = x0*x8 ;
    float x10 = a[9]*x8 ;
    float x11 = x6*x8 ;
    float x12 = p_l*p_l;
    float x13 = x12*x5 ;
    float x14 = 2*d ;
    float x15 = x13*x14 ;
    float x16 = x12/p_h/p_h/p_h;
    float x17 = x14*x16 ;
    float x18 = p_l*p_l*p_l/p_h/p_h/p_h/p_h;
    float x19 = 3*d ;
    float x20 = a[7]*x19 ;
    float x21 = 3 - x3 ;
    float x22 = 2*x8 ;
    float x23 = a[10]*x22 ;
    float x24 = x16*x22 ;
    float x25 = 3*x8 ;
    float x26 = a[13]*x25 ;
    float x27 = -x13 ;
    float x28 = d*(x27 + 1) ;
    float x29 = d*(x27 + 2) ;
    float x30 = x27 + 3 ;
    float x31 = d*x30 ;
    float x32 = x30*x8 ;
    float x33 = d*x2 ;
    float x34 = p_h/x12 ;   
    float x35 = d*x34 ;
    float x36 = x2*x8 ;
    float x37 = x14*x3 ;
    float x38 = 2*x7 ;
    float x39 = x34*x8 ;
    float x40 = 2*x11 ;
    float x41 = b[3]*d ;
    float x42 = b[9]*x8 ;
    float x43 = b[7]*x19 ;
    float x44 = b[10]*x22 ;
    float x45 = b[13]*x25 ;
    this->dPhi_dx<<a[0]*x1 + a[1]*x15 + a[1]*x28 + a[2]*x15 + a[2]*x29 + a[4]*x15 + a[4]*x31 + a[5]*x7 + a[6]*x17 + a[8]*x9 + a[10]*x32 + a[11]*x11 + a[12]*x24 + x10*x21 + x10*x3 + x13*x23 + x18*x20 + x18*x26 + x21*x4 + x3*x4
        , -a[0]*x35 - a[1]*x37 - a[2]*x37 - a[4]*x37 - a[5]*x33 - a[6]*x38 - a[8]*x39 - a[11]*x36 - a[12]*x40 - x10 - x16*x20 - x16*x26 - x23*x3 - x4
        , b[0]*x1 + b[1]*x15 + b[1]*x28 + b[2]*x15 + b[2]*x29 + b[4]*x15 + b[4]*x31 + b[5]*x7 + b[6]*x17 + b[8]*x9 + b[10]*x32 + b[11]*x11 + b[12]*x24 + x13*x44 + x18*x43 + x18*x45 + x21*x41 + x21*x42 + x3*x41 + x3*x42
        , -b[0]*x35 - b[1]*x37 - b[2]*x37 - b[4]*x37 - b[5]*x33 - b[6]*x38 - b[8]*x39 - b[11]*x36 - b[12]*x40 - x16*x43 - x16*x45 - x3*x44 - x41 - x42;

}
void MPC::Update_dPhi_duH(float p_h,float p_l,float d,const std::array<float,MPC_STATE_NUM>& a,const std::array<float,MPC_STATE_NUM> &b){
    float x0 = p_h/p_l ;
    float x1 = 2*d ;
    float x2 = x0*x1 ;
    float x3 = -p_l/p_h ;
    float x4 = x3 + 1 ;
    float x5 = p_h*(x3 + 3) ;
    float x6 = -p_l*p_l/p_h/p_h;
    float x7 = x6 + 1 ;
    float x8 = 1 - p_l*p_l*p_l/p_h/p_h/p_h;
    float x9 = x1*x4 ;
    float x10 = p_h*x7 ;
    float x11 = p_h*(x6 + 2) ;
    float x12 = p_h*(x6 + 3) ;
    float x13 = x1*x5 ;
    float x14 = x1*x7 ;
    float x15 = x1*x8 ;
    float x16 = x1*x12 ;

    this->dPhi_du<< a[0]*x0 + a[1]*x10 + a[2]*x11 + a[3]*x5 + a[4]*x12 + a[5]*x4 + a[6]*x7 + a[7]*x8 + a[8]*x2 + a[9]*x13 + a[10]*x16 + a[11]*x9 + a[12]*x14 + a[13]*x15
                   , b[0]*x0 + b[1]*x10 + b[2]*x11 + b[3]*x5 + b[4]*x12 + b[5]*x4 + b[6]*x7 + b[7]*x8 + b[8]*x2 + b[9]*x13 + b[10]*x16 + b[11]*x9 + b[12]*x14 + b[13]*x15;

}


void MPC::UpdateDyn(float p_h,float p_l,float d,bool increase_pre){
    // Eigen::Matrix<float,2,1> Phi;
    
    if(increase_pre){
        //if we are increasing the pressure
        
        this->UpdatePhi(p_h,p_l,d,this->ah,this->bh);
        this->Update_dPhi_dxH(p_h,p_l,d,this->ah,this->bh);
        this->Update_dPhi_duH(p_h,p_l,d,this->ah,this->bh);
        
        

    }
    else{
        this->UpdatePhi(p_h,p_l,d,this->al,this->bl);
        this->Update_dPhi_dxL(p_h,p_l,d,this->al,this->bl);
        this->Update_dPhi_duL(p_h,p_l,d,this->al,this->bl);
    }
    Eigen::Matrix2f K_mat = 2*Eigen::Matrix2f::Identity()-this->dPhi_dx;

    this->B = K_mat.inverse()*this->dPhi_du;
    this->alpha = K_mat.inverse()*(this->Phi-this->dPhi_du*d);

}







int MPC::GetControl(int p_des,int pt,int ps,int duty){
    // duty = 50; //TODO: force override
    int p_diff = p_des - ps;
    float pt_scaled = ((float)pt - 6553.6)/65536;
    float ps_scaled = ((float)ps - 6553.6)/65536;
    float duty_scaled = (float)duty/100;
    
    if(std::abs(p_diff)>262){ //if desired pressure has 1 psi difference, Caution: calculate the diff does not need to consider the 0.5V dc bias
        
        double lb;
        if(std::abs(pt-ps)<1310){ //if the difference is less than 10 psi, we can operate the valve with lower duty
            lb = 10.0;
        }
        else if(std::abs(pt-ps)<2621){ //20 psi
            lb = 20.0;
        }

        else{
            lb=60.0;
        }

        // if(duty==0){
        //     duty=50;//if duty=0, there will be no output
        // }
        
        if((p_des>ps) & (pt>ps)){
            //increasing pressure
            // std::cout<<"increase\n";
            this->UpdateDyn(pt_scaled,ps_scaled,duty_scaled,true);
        
        }
        else if((p_des<ps)&(ps>pt)){
            //decreasing pressure
            // std::cout<<"decrease\n";
            this->UpdateDyn(ps_scaled,pt_scaled,duty_scaled,false);
        }
        else{
            return 0;
        }
        
        // format question to osqp format
        float q_val = 2*(this->B.coeff(1,0)*this->alpha.coeff(1,0)-p_diff*this->B.coeff(1,0))/100;
        float p_val = 2*this->B.coeff(1,0)*this->B.coeff(1,0)/10000;//scale up the u to duty instead of duty/100
        // std::cout<<"q_val: "<<q_val<<std::endl;
        // std::cout<<"p_val: "<<p_val<<std::endl;
        // std::cout<<"B: "<<this->B<<std::endl;
        // std::cout<<"P_diff: "<<p_diff<<std::endl;
        std::cout<<"Phi: "<<this->Phi<<std::endl;
        // std::cout<<"duty: "<<duty<<std::endl;

        c_int P_nnz = 1;
        c_float P_x[1]={p_val};
        c_int P_i[1]={0};
        c_int P_p[2]={0,1};
        this->osqp_data->n =1;
        this->osqp_data->P = csc_matrix(this->osqp_data->n,this->osqp_data->n,P_nnz,P_x,P_i,P_p);
        
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
        
    }
    else{
        return 0;
    }

}



