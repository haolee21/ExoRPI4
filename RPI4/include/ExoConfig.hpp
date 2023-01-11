#ifndef EXO_CONFIG_HPP
#define EXO_CONFIG_HPP
#include <array>
#include <cstring>
#include <json.hpp>
#define MPC_STATE_NUM 125
#define MPC_DELAY 25

class ExoConfig
{
public:
    enum class ConfigType
    {
        MPC,
        Phy,
    };
    struct MPC_Params
    {
        std::array<std::array<double, MPC_STATE_NUM>, 2> ch;
        std::array<std::array<double, MPC_STATE_NUM>, 2> cl;
        double cali_chamber_len; //unit: mm, the length of the chamber during static calibration
    };
    struct CylnPhyParams
    {
        double mech_max_len; // unit: mm, the length excludes the pneumatic chambers
        double chamber_max_len; //unit: mm, the total length, includes the springs and other parts
        
        double fri_coeff;
        std::array<double, 2> piston_area; // unit: mm^2, 0: ext, 1: flex
        double spring_const;               // unit N/mm 55.4 * 0.0393701 * 4.44822;
        std::array<double,3> cyln_eqn;// cylinder length = coeff[0]-coeff[1]*(angle-coeff[2]) 
        double neutral_pos;
    };
    struct SystemParam
    {
        // MPC controller parameters
        MPC_Params left_tank_subtank;
        MPC_Params left_subtank_knee;
        MPC_Params left_subtank_ank;
        MPC_Params left_knee_ank;
        MPC_Params left_knee_ext_flex;
        MPC_Params left_ank_ext_flex;

        MPC_Params right_tank_subtank;
        MPC_Params right_subtank_knee;
        MPC_Params right_subtank_ank;
        MPC_Params right_knee_ank;
        MPC_Params right_knee_ext_flex;
        MPC_Params right_ank_ext_flex;
        
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
    };

    static nlohmann::json LoadJson(std::string file_name);
    static void LoadConfigFile(const nlohmann::json &file, SystemParam &sys_param, ConfigType config_type);
    // void SaveConfigFile(std::string file_name);
    ExoConfig(const ExoConfig &)=delete;
    static ExoConfig &GetInstance();
    ~ExoConfig();
    static const SystemParam& GetConfig();
private:
    ExoConfig();
    SystemParam sys_param;
    static void LoadMPC_param(const nlohmann::json &config_file,MPC_Params &mpc_params,std::string name);

};

#endif
