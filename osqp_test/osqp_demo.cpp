#include <osqp/osqp.h>
#include "MPC.h"


int main(int argc, char **argv) {
    

    MPC ins;
    ins.Solve();
    std::array<float,X_LEN> res1 = ins.GetResult();
    for(const auto& e:res1){
        std::cout<<e<<std::endl;
    }
    std::array<float,LIN_COST_LEN> new_q{1,2};
    ins.UpdateLinCost(new_q);
    int solve_flag = ins.Solve();
    if(solve_flag)
        std::cout<<"solve flag is true\n";
    std::cout<<"solve flag is: "<<solve_flag<<std::endl;
    std::array<float,X_LEN> res2 = ins.GetResult();
    for(const auto& e:res2){
        std::cout<<e<<std::endl;
    }
};