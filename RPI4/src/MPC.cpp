#include <cmath>
#include <iostream>
#include "MPC.hpp"
#include "MPC_param.hpp"

using namespace std;
MPC::MPC(array<array<float,13>,2> init_cl,array<array<float,13>,2> init_ch)
{

    //load default values for the model parameters, it is data from one of the experiment
    this->cl = init_cl;
    this->ch = init_ch;

    
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
void MPC::UpdateParamH(array<float,13> new_0,array<float,13> new_1){
    
    this->ch[0] = new_0;
    this->ch[1] = new_1;
}
void MPC::UpdateParamL(array<float,13> new_0,array<float,13> new_1){
    this->cl[0] = new_0;
    this->cl[1] = new_1;
}
void MPC::UpdateL(int _pt,int _ps, int _duty){
    float pt = ((float)_pt - 6553.6)/65536; //The zero pressure measurement is 0.5V
    float ps = ((float)_ps - 6553.6)/65536;
    float duty = (float)_duty/100;
   
    float a00,a01,a10,a11,b0,b1;
    a00 = (-this->cl[0][2]*ps/pt/pt
          -2*this->cl[0][4]*pt/ps
          -2*this->cl[0][6]*pt/ps
          -2*this->cl[0][8]*pt/ps
          -this->cl[0][10]/ps
          -2*this->cl[0][11]*pt/ps/ps)*duty;
    
    a01 = (this->cl[0][2]/pt
         +this->cl[0][4]*(1-pt*pt/ps/ps)
         +2*this->cl[0][4]*pt*pt/ps/ps
         +this->cl[0][6]*(2-pt*pt/ps/ps)
         +2*this->cl[0][6]*pt*pt/ps/ps
         +this->cl[0][8]*(3-pt*pt/ps/ps)
         +2*this->cl[0][8]*pt*pt/ps/ps
         +this->cl[0][10]*pt/ps/ps
         +2*this->cl[0][11]*pt*pt/ps/ps/ps)*duty;
    
    
    a10 = (-this->cl[1][2]*ps/pt/pt
          -2*this->cl[1][4]*pt/ps
          -2*this->cl[1][6]*pt/ps
          -2*this->cl[1][8]*pt/ps
          -3*this->cl[1][12]*pt*pt/ps/ps/ps)*duty;

    a11 = (this->cl[1][2]/pt
         +this->cl[1][4]*(1-pt*pt/ps/ps)
         +2*this->cl[1][4]*pt*pt/ps/ps
         +this->cl[1][6]*(2-pt*pt/ps/ps)
         +2*this->cl[1][6]*pt*pt/ps/ps
         +this->cl[1][8]*(3-pt*pt/ps/ps)
         +2*this->cl[1][8]*pt*pt/ps/ps
         +3*this->cl[1][12]*pt*pt*pt/ps/ps/ps/ps)*duty;

    this->matA<<a00,a01,a10,a11;
    
    b0 = this->cl[0][2]*ps/pt
        +this->cl[0][4]*ps*(1-pt*pt/ps/ps)
        +this->cl[0][6]*ps*(2-pt*pt/ps/ps)
        +this->cl[0][8]*ps*(3-pt*pt/ps/ps)
        +this->cl[0][10]*(1-pt/ps)
        +this->cl[0][11]*(1-pt*pt/ps/ps);

    b1 = this->cl[1][2]*ps/pt
        +this->cl[1][4]*ps*(1-pt*pt/ps/ps)
        +this->cl[1][6]*ps*(2-pt*pt/ps/ps)
        +this->cl[1][8]*ps*(3-pt*pt/ps/ps)
        +this->cl[1][12]*(1-pt*pt*pt/ps/ps/ps);

    this->matB<<b0,b1;
}

void MPC::UpdateH(int _pt,int _ps, int _duty){
    float pt = ((float)_pt - 6553.6)/65536; //The zero pressure measurement is 0.5V
    float ps = ((float)_ps - 6553.6)/65536;
    float duty = (float)_duty/100;
   
    float a00,a01,a10,a11,b0,b1;
    a00 =(this->ch[0][2]/ps
          +2*this->ch[0][8]*ps*ps/pt/pt
          +this->ch[0][8]*(3-ps*ps/pt/pt)
          +3*this->ch[0][12]*ps*ps*ps/pt/pt/pt/pt)*duty;
         

    a01 = -(this->ch[0][2]*pt/ps/ps
          +2*this->ch[0][8]*ps/pt
          +3*this->ch[0][12]*ps*ps/pt/pt/pt)*duty;
    
    
    a10 = (this->ch[1][2]/ps
          +2*this->ch[1][8]*ps*ps/pt/pt
          +this->ch[1][8]*(3-ps*ps/pt/pt)
          +3*this->ch[1][12]*ps*ps*ps/pt/pt/pt/pt)*duty;

    a11 = -(this->ch[1][2]*pt/ps/ps
            +2*this->ch[1][8]*ps/pt
            +3*this->ch[1][12]*ps*ps/pt/pt/pt)*duty;

    this->matA<<a00,a01,a10,a11;
    
    b0 = this->ch[0][2]*pt/ps
        +this->ch[0][8]*pt*(3-ps*ps/pt/pt)
        +this->ch[0][12]*(1-ps*ps*ps/pt/pt/pt);

    b1 = this->ch[1][2]*pt/ps
        +this->ch[1][8]*pt*(3-ps*ps/pt/pt)
        +this->ch[1][12]*(1-ps*ps*ps/pt/pt/pt);

    this->matB<<b0,b1;

}

int MPC::GetControl(int p_des,int pt,int ps,int duty){
    int p_diff = p_des - ps;
    
    if(std::abs(p_diff)>525){ //if desired pressure has 2 psi difference, Caution: calculate the diff does not need to consider the 0.5V dc bias
        
        double lb = 50.0;
        // if(std::abs(pt-ps)<7865){ //if the difference is less than 5 psi, we can operate the valve with lower duty
        //     lb = 50.0;
        // }

        if(duty==0){
            duty=50;//if duty=0, there will be no output
        }

        if((p_des>ps) & (pt>ps)){
            this->UpdateH(pt,ps,duty);
        
        }
        else if((p_des<ps)&(ps>pt)){
            this->UpdateL(pt,ps,duty);
        }
        else{
            return 0;
        }

        
        // format question to osqp format
        float q_val = 2*(pt*this->matA.coeff(1,0)*this->matB.coeff(1,0)+ps*this->matA.coeff(1,1)*this->matB(1,0)-this->matB(1,0)*p_diff);
        float p_val = this->matB.coeff(1,0)*this->matB.coeff(1,0);
 

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
            duty = *this->work->solution->x;
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

