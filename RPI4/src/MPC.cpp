#include <cmath>
#include "MPC.hpp"
MPC::MPC(/* args */)
{

    //load default values for the model parameters, it is data from one of the experiment
    this->c00 = 8.79462549e-01;
    this->c02 = 6.14512679;
    this->c03 =  2.15175439e+03;
    this->c06 = -1.52124169;
    this->c07 = 1.0181582;
    this->c08 = 2.24744155;
    this->c09 = -1.96773051;

    this->c12 = 1.59100979;
    this->c13 = -2.31679389e+03;
    this->c16 = 1.38348116;
    this->c17 = 4.84105880e-01;
    this->c18 = -1.68793486e+01;
    this->c19 = 2.66752892;

    //setup osqp solver
    this->osqp_data.reset(new OSQPData);
    this->osqp_settings.reset(new OSQPSettings);

   
    
}

MPC::~MPC()
{
}
void MPC::UpdateParam(float _c00,float _c02,float _c03,float _c06,float _c07,float _c08,
                      float _c09,float _c12,float _c13,float _c16,float _c17,float _c18,float _c19){
    this->c00 = _c00;
    this->c02 = _c02;
    this->c03 = _c03;
    this->c06 = _c06;
    this->c07 = _c07;
    this->c08 = _c08;
    this->c09 = _c09;
    this->c12 = _c12;
    this->c13 = _c13;
    this->c16 = _c16;
    this->c17 = _c17;
    this->c18 = _c18;
    this->c19 = _c19;
}
void MPC::UpdateA(int _p_tank,int _p_lt, int _duty){
    float p_tank = ((float)_p_tank - 6553.6)/65536; //The zero pressure measurement is 0.5V
    float p_lt = ((float)_p_lt - 6553.6)/65536;
    float duty = (float)_duty/100;
   
    float a00,a01,a10,a11,b0,b1;
    a00 =  duty*this->c00
          -duty*this->c02*p_lt/p_tank/p_tank
          +duty*this->c03*p_lt*p_lt/p_tank/p_tank
          +duty*this->c06
          +duty*this->c07*p_lt
          +duty*this->c08/p_lt
          -3*duty*this->c09*(1-p_tank/p_lt)*(1-p_tank/p_lt)/p_lt;

    a01 = duty*this->c02/p_tank
         -duty*this->c03*p_lt/p_tank
         +duty*this->c03*(1-p_lt/p_tank)
         +duty*this->c07*p_tank
         -duty*this->c08*p_tank/p_lt/p_lt
         +3*duty*this->c09*p_tank*(1-p_tank/p_lt)*(1-p_tank/p_lt)/p_lt;
    
    
    a10 = -duty*this->c12*p_lt/p_tank/p_tank
          +duty*this->c13*p_lt*p_lt/p_tank/p_tank
          +duty*this->c16
          +duty*this->c17*p_lt
          +duty*this->c18/p_lt
          -3*duty*this->c19*(1-p_tank/p_lt)*(1-p_tank/p_lt)/p_lt;

    a11 = duty*this->c12/p_tank
         -duty*this->c13*p_lt/p_tank
         +duty*this->c13*(1-p_lt/p_tank)
         +duty*this->c17*p_tank
         -duty*this->c18*p_tank/p_lt/p_lt
         +3*duty*this->c19*p_tank*(1-p_tank/p_lt)*(1-p_tank/p_lt)/p_lt/p_lt;

    this->matA<<a00,a01,a10,a11;
    
    b0 = this->c00*p_tank
                  +this->c02*p_lt/p_tank
                  +this->c03*p_lt*(1-p_lt/p_tank)
                  +this->c06*p_tank
                  +this->c07*p_lt*p_tank
                  +this->c08*p_tank/p_lt
                  +this->c09*(1-p_tank/p_lt)*(1-p_tank/p_lt);

    b1 = this->c12*p_lt/p_tank
                  +this->c13*p_lt*(1-p_lt/p_tank)
                  +this->c16*p_tank
                  +this->c17*p_lt*p_tank
                  +this->c18*p_tank/p_lt
                  +this->c19*(1-p_tank/p_lt)*(1-p_tank/p_lt);

    this->matB<<b0,b1;
    
}

int MPC::GetControl(int p_des,int p_tank,int p_lt,int duty){

    if(std::abs(p_des-p_lt)>35390){ //if desired pressure has 2 psi difference
        this->UpdateA(p_tank,p_lt,duty);

        // format question to osqp format
        float q_val = 2*(p_tank*this->matA.coeff(0,1)*this->matB.coeff(1,0)+p_lt*this->matA.coeff(1,1)*this->matB(1,0));
        float p_val = this->matB.coeff(0,1)*this->matB.coeff(0,1);

        c_int P_nnz = 1;
        c_float P_x[1]={p_val};
        c_int P_i[1]={0};
        c_int P_p[2]={0,1};
        this->osqp_data->n =1;
        this->osqp_data->P = csc_matrix(this->osqp_data->n,this->osqp_data->n,P_nnz,P_x,P_i,P_p);

        c_float q[1]={q_val};
        this->osqp_data->q = q;


        c_int A_nnz = 1;
        this->osqp_data->m = 1;
        c_float A_x[1]={1};
        c_int A_i[1]={0};
        c_int A_p[2]={0,1};
        this->osqp_data->A = csc_matrix(this->osqp_data->m,this->osqp_data->n,A_nnz,A_x,A_i,A_p);

        c_float l[1]={0};
        c_float u[1]={95};

        this->osqp_data->l = l;
        this->osqp_data->u = u;

        osqp_set_default_settings(this->osqp_settings.get());
        this->osqp_settings->alpha = 1.0;
    
        int set_up_flag = osqp_setup(&this->work,this->osqp_data.get(),this->osqp_settings.get());

        int solve_err=0;
        int duty=0;
        if(!set_up_flag)
            solve_err = osqp_solve(this->work);
        if(!solve_err){
            duty = *this->work->solution->x;
            if(duty<30)
                duty =0;
        }
        return duty;
    }
    else{
        return 0;
    }

}

