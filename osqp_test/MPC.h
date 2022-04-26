#ifndef MPC_H
#define MPC_H

#define LIN_COST_LEN 2
#define QUAD_COST_LEN 3//only consider the upper traingular part
#define CONS_BOUND_LEN 3
#define CONS_A_LEN 4
#define X_LEN 2
#define NUM_OF_CONS 3
#include "osqp/osqp.h"
#include <memory>
#include <array>
#include <iostream>
class MPC
{
    //This is directly reference from https://osqp.org/docs/examples/setup-and-solve.html
    //The library required is OSQP, which is written in C, thus, the Class is defined in c-style
private:
    
    OSQPWorkspace *work;
    OSQPSettings  *settings = (OSQPSettings *)c_malloc(sizeof(OSQPSettings));
    OSQPData      *data     = (OSQPData *)c_malloc(sizeof(OSQPData));

    
    c_int exitflag = 0;

public:
    MPC(/* args */);
    ~MPC();
    int Solve();
    std::array<float,X_LEN> GetResult();
    void UpdateLinCost(std::array<float,LIN_COST_LEN>);
    void UpdateQuadCost(std::array<float,QUAD_COST_LEN>);
    void UpdateConsBound(std::array<float,CONS_BOUND_LEN> up, std::array<float,CONS_BOUND_LEN> low);
    void UpdateConsMat(std::array<float,CONS_A_LEN>);

};

MPC::MPC(/* args */)
{
    std::cout<<"sizeof OSQP data"<<sizeof(OSQPData)<<std::endl;
    //some initial values
    c_float P_x[3] = {4.0, 1.0, 2.0, };
    c_int P_nnz = 3;
    c_int P_i[3] = {0, 0, 1, };
    c_int P_p[3] = {0, 1, 3, };
    c_float q[2] = {1.0, 1.0, };
    c_float A_x[4] = {1.0, 1.0, 1.0, 1.0, };
    c_int A_nnz = 4;
    c_int A_i[4] = {0, 1, 0, 2, };
    c_int A_p[3] = {0, 2, 4, };
    c_float l[3] = {1.0, 0.0, 0.0, };
    c_float u[3] = {1.0, 0.7, 0.7, };
    c_int n = X_LEN;
    c_int m = NUM_OF_CONS;

    if(this->data){
        this->data->n = n;
        this->data->m = m;
        this->data->P = csc_matrix(this->data->n, this->data->n, P_nnz, P_x, P_i, P_p);
        this->data->q = q;
        this->data->A = csc_matrix(this->data->m, this->data->n, A_nnz, A_x, A_i, A_p);
        this->data->l = l;
        this->data->u = u;
    }
        // Define solver settings as default
    if (this->settings) {
        osqp_set_default_settings(this->settings);
        this->settings->alpha = 1.0; // Change alpha parameter
    }
    this->exitflag = osqp_setup(&this->work, this->data, this->settings);
}

MPC::~MPC()
{
    osqp_cleanup(this->work);
    if (this->data) {
        if (this->data->A) c_free(this->data->A);
        if (this->data->P) c_free(this->data->P);
        c_free(this->data);
    }
    if (this->settings) c_free(this->settings);

}

int MPC::Solve(){
    int flag = osqp_solve(this->work);
   
    return flag;
}


void MPC::UpdateLinCost(std::array<float,LIN_COST_LEN> q_new){
    c_float q[LIN_COST_LEN];

    std::copy(q_new.begin(),q_new.end(),q);
    osqp_update_lin_cost(this->work,q);


}
void MPC::UpdateQuadCost(std::array<float,QUAD_COST_LEN> P_new){
    c_float P[QUAD_COST_LEN];
    std::copy(P_new.begin(),P_new.end(),P);
    osqp_update_P(this->work,P,OSQP_NULL,QUAD_COST_LEN);


}
void MPC::UpdateConsBound(std::array<float,CONS_BOUND_LEN> up_new, std::array<float,CONS_BOUND_LEN> low_new){
    c_float up[CONS_BOUND_LEN], low[CONS_BOUND_LEN];
    std::copy(up_new.begin(),up_new.end(),up);
    std::copy(low_new.begin(),low_new.end(),low);
    osqp_update_bounds(this->work,low,up);


}
void MPC::UpdateConsMat(std::array<float,CONS_A_LEN> A_new){
    c_float A[CONS_A_LEN];
    std::copy(A_new.begin(),A_new.end(),A);
    osqp_update_A(this->work,A,OSQP_NULL,CONS_A_LEN);


}
std::array<float,X_LEN> MPC::GetResult(){
    std::array<float,X_LEN> result;
    std::copy(this->work->solution->x,this->work->solution->x+X_LEN,result.begin());
    
    return result;

}
#endif //MPC.h