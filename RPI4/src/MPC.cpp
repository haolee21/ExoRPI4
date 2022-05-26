#include <cmath>
#include <iostream>
#include "MPC.hpp"
#include "MPC_param.hpp"

using namespace std;
MPC::MPC(array<array<float,MPC_STATE_NUM>,2> init_cl,array<array<float,MPC_STATE_NUM>,2> init_ch)
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
void MPC::UpdateParamH(array<float,MPC_STATE_NUM> new_0,array<float,MPC_STATE_NUM> new_1){
    
    this->ch[0] = new_0;
    this->ch[1] = new_1;
}
void MPC::UpdateParamL(array<float,MPC_STATE_NUM> new_0,array<float,MPC_STATE_NUM> new_1){
    this->cl[0] = new_0;
    this->cl[1] = new_1;
}
void MPC::Update(int _ph,int _pl, int _duty,std::array<std::array<float,MPC_STATE_NUM>,2> &c){
    float ph = ((float)_ph - 6553.6)/65536; //The zero pressure measurement is 0.5V
    float pl = ((float)_pl - 6553.6)/65536;
    float duty = (float)_duty/100;
   
    float a00,a01,a10,a11,b0,b1;
    a00 = (-c[0][0]*ph
          -c[0][1]
          -2*c[0][2]*pl/ph
          -c[0][3]
          -2*c[0][4]*pl/ph
          -c[0][5]
          -2*c[0][6]*pl/ph
          -c[0][7]
          -c[0][8]/ph
          -2*c[0][9]*pl/ph/ph
          -3*c[0][10]*pl*pl/ph/ph/ph
    )*duty;
    
    
    a01 = (c[0][0]/pl
          +c[0][1]*(1-pl/ph)
          +c[0][1]*pl/ph
          +c[0][2]*(1-pl*pl/ph/ph)
          +c[0][3]*(2-pl/ph)
          +c[0][3]*pl/ph
          +c[0][4]*(2-pl*pl/ph/ph)
          +2*c[0][4]*pl*pl/ph/ph
          +c[0][5]*(3-pl/ph)
          +c[0][5]*pl/ph
          +c[0][6]*(3-pl*pl/ph/ph)
          +2*c[0][6]*pl*pl/ph/ph
          +c[0][7]
          +c[0][8]*pl/ph/ph
          +2*c[0][9]*pl*pl/ph/ph/ph
          +3*c[0][10]*pl*pl*pl/ph/ph/ph/ph)*duty;
    
    
    a10 = -(c[1][0]*ph/pl/pl
          +c[1][1]
          +2*c[1][2]*pl/ph
          +c[1][3]
          +2*c[1][4]*pl/ph
          +c[1][5]
          +2*c[1][6]*pl/ph
          +c[1][7]
          +c[1][8]/ph
          +2*c[1][9]*pl/ph/ph
          +3*c[1][10]*pl*pl/ph/ph/ph)*duty;

    a11 = (c[1][0]/pl
          +c[1][1]*(1-pl/ph)
          +c[1][1]*pl/ph
          +c[1][2]*(1-pl*pl/ph/ph)
          +2*c[1][2]*pl*pl/ph/ph
          +c[1][3]*(2-pl/ph)
          +c[1][3]*pl/ph
          +c[1][4]*(2-pl*pl/ph/ph)
          +2*c[1][4]*pl*pl/ph/ph
          +c[1][5]*(3-pl/ph)
          +c[1][5]*pl/ph
          +c[1][6]*(3-pl*pl/ph/ph)
          +2*c[1][6]*pl*pl/ph/ph
          +c[1][7]
          +c[1][8]*pl/ph/ph
          +2*c[1][9]*pl*pl/ph/ph/ph
          +3*c[1][10]*pl*pl*pl/ph/ph/ph/ph)*duty;

    this->matA<<a00,a01,a10,a11;
    
    b0 = c[0][0]*ph/pl
        +c[0][1]*ph*(1-pl/ph)
        +c[0][2]*ph*(1-pl*pl/ph/ph)
        +c[0][3]*ph*(2-pl/ph)
        +c[0][4]*ph*(2-pl*pl/ph/ph)
        +c[0][5]*ph*(3-pl/ph)
        +c[0][6]*ph*(3-pl*pl/ph/ph)
        +c[0][7]*(ph-pl)
        +c[0][8]*(1-pl/ph)
        +c[0][9]*(1-pl*pl/ph/ph)
        +c[0][10]*(1-pl*pl*pl/ph/ph/ph);

    b1 = c[1][0]*ph/pl
        +c[1][1]*ph*(1-pl/ph)
        +c[1][2]*ph*(1-pl*pl/ph/ph)
        +c[1][3]*ph*(2-pl/ph)
        +c[1][4]*ph*(2-pl*pl/ph/ph)
        +c[1][5]*ph*(3-pl/ph)
        +c[1][6]*ph*(3-pl*pl/ph/ph)
        +c[1][7]*(ph-pl)
        +c[1][8]*(1-pl/ph)
        +c[1][9]*(1-pl*pl/ph/ph)
        +c[1][10]*(1-pl*pl*pl/ph/ph/ph);

    this->matB<<b0,b1;
}


int MPC::GetControl(int p_des,int pt,int ps,int duty){
    int p_diff = p_des - ps;
    
    if(std::abs(p_diff)>525){ //if desired pressure has 2 psi difference, Caution: calculate the diff does not need to consider the 0.5V dc bias
        
        double lb = 50.0;
        if(std::abs(pt-ps)<2621){ //if the difference is less than 10 psi, we can operate the valve with lower duty
            lb = 40.0;
        }

        if(duty==0){
            duty=50;//if duty=0, there will be no output
        }

        if((p_des>ps) & (pt>ps)){
            this->Update(pt,ps,duty,this->ch);
        
        }
        else if((p_des<ps)&(ps>pt)){
            this->Update(ps,pt,duty,this->cl);
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

