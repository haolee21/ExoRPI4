#ifndef EXO_CONFIG_HPP
#define EXO_CONFIG_HPP
#include <array>
#include <cstring>
#include <json.hpp>
#define MPC_STATE_NUM 125
#define MPC_DELAY 25

namespace ExoConfig
{
    enum class ConfigType{
        MPC,
        Phy,
    };
    struct MPC_Params
    {
        std::array<std::array<double, MPC_STATE_NUM>, 2> ch;
        std::array<std::array<double, MPC_STATE_NUM>, 2> cl;
    };
    struct CylnPhyParams
    {
        double cyln_len_mm;
        double fri_coeff;
        std::array<double,2> piston_area;
        // double piston_area_ext;
        // double piston_area_flex;
    };
    struct SystemParam
    {
        // MPC controller parameters
        MPC_Params left_tank;
        MPC_Params left_knee_ext;
        // MPC_Param left_ank_ext;
        // MPC_Param lk_ra_ext; // left knee to right ankle

        // MPC_Param right_tank;
        // MPC_Param right_knee_ext;
        // MPC_Param right_ank_ext;
        // MPC_Param rk_la_ext;

        // Cylinder parameters
        CylnPhyParams left_knee_phy;
        CylnPhyParams left_ankle_phy;
        CylnPhyParams right_knee_phy;
        CylnPhyParams right_ankle_phy;

        std::array<double,2> left_knee_eqn;   //cylinder length = coeff[0]-coeff[1]*angle
        std::array<double,2> left_ankle_eqn;
        std::array<double,2> right_knee_eqn;
        std::array<double,2> right_ankle_eqn;

    };
    nlohmann::json LoadJson(std::string file_name);
    void LoadConfigFile(const nlohmann::json &file, SystemParam &sys_param,ConfigType config_type);
    // void SaveConfigFile(std::string file_name);

    

}

#endif
