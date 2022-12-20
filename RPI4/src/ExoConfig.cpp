#include "ExoConfig.hpp"
#include <iostream>
#include <fstream>
using json = nlohmann::json;
using namespace ExoConfig;
template <class T, unsigned N>
std::array<T, N> LoadJsonArray(json json_file, std::string name)
{
    std::array<T, N> result;
    auto json_array = json_file[name];
    if (N > json_array.size())
    {
        std::cout << "JSON Read Error: json array " << name << " size mismatch (required " << N << " members)\n";
    }
    else
    {
        for (int i = 0; i < N; i++)
        {
            result[i] = json_array[i];
        }
    }

    return result;
}
json ExoConfig::LoadJson(std::string file_name)
{
    json file;
    std::ifstream ifs(file_name);
    file = json::parse(ifs);
    return file;
}
void ExoConfig::LoadConfigFile(const json &config_file, SystemParam &sys_param, ConfigType config_type)
{
    if (config_type == ConfigType::MPC)
    {
        sys_param.left_tank.ch[0] = LoadJsonArray<double, MPC_STATE_NUM>(config_file["MPC_Params"]["LTank"]["ch"], "a");
        sys_param.left_tank.ch[1] = LoadJsonArray<double, MPC_STATE_NUM>(config_file["MPC_Params"]["LTank"]["ch"], "b");
        sys_param.left_tank.cl[0] = LoadJsonArray<double, MPC_STATE_NUM>(config_file["MPC_Params"]["LTank"]["cl"], "a");
        sys_param.left_tank.cl[1] = LoadJsonArray<double, MPC_STATE_NUM>(config_file["MPC_Params"]["LTank"]["cl"], "b");

        sys_param.left_knee_ext.ch[0] = LoadJsonArray<double, MPC_STATE_NUM>(config_file["MPC_Params"]["LKneExt"]["ch"], "a");
        sys_param.left_knee_ext.ch[1] = LoadJsonArray<double, MPC_STATE_NUM>(config_file["MPC_Params"]["LKneExt"]["ch"], "b");
        sys_param.left_knee_ext.cl[0] = LoadJsonArray<double, MPC_STATE_NUM>(config_file["MPC_Params"]["LKneExt"]["cl"], "a");
        sys_param.left_knee_ext.cl[1] = LoadJsonArray<double, MPC_STATE_NUM>(config_file["MPC_Params"]["LKneExt"]["cl"], "b");

        std::cout << "size: " << config_file["MPC_Params"]["LTank"]["ch"]["a"].size() << std::endl;
    }
    else if(config_type==ConfigType::Phy){
        //cylinder length equation
        sys_param.left_knee_eqn = LoadJsonArray<double,2>(config_file["CylnEqn"],"LeftKnee");
        sys_param.left_ankle_eqn = LoadJsonArray<double,2>(config_file["CylnEqn"],"LeftAnkle");
        sys_param.right_knee_eqn = LoadJsonArray<double,2>(config_file["CylnEqn"],"RightKnee");
        sys_param.right_ankle_eqn = LoadJsonArray<double,2>(config_file["CylnEqn"],"RightAnkle");
        //Cylinder length
        sys_param.left_knee_phy.cyln_len_mm = config_file["CylinderLen_mm"]["LeftKnee"];
        sys_param.left_ankle_phy.cyln_len_mm = config_file["CylinderLen_mm"]["AnkleKnee"];
        sys_param.right_knee_phy.cyln_len_mm = config_file["CylinderLen_mm"]["RightKnee"];
        sys_param.right_ankle_phy.cyln_len_mm = config_file["CylinderLen_mm"]["RightAnkle"];
        //Friction coeff
        sys_param.left_knee_phy.fri_coeff = config_file["FrictionCoeff"]["LeftKnee"];
        sys_param.left_ankle_phy.fri_coeff = config_file["FrictionCoeff"]["LeftAnkle"];
        sys_param.right_knee_phy.fri_coeff = config_file["FrictionCoeff"]["RightKnee"];
        sys_param.right_ankle_phy.fri_coeff = config_file["FrictionCoeff"]["RightAnkle"];
        //Piston area
        sys_param.left_knee_phy.piston_area = LoadJsonArray<double,2>(config_file["PistonArea_mm2"],"LeftKnee");
        sys_param.left_ankle_phy.piston_area = LoadJsonArray<double,2>(config_file["PistonArea_mm2"],"LeftAnkle");
        sys_param.right_knee_phy.piston_area = LoadJsonArray<double,2>(config_file["PistonArea_mm2"],"RightKnee");
        sys_param.right_ankle_phy.piston_area = LoadJsonArray<double,2>(config_file["PistonArea_mm2"],"RightAnkle");
    }
    std::cout << config_file["Name"] <<"  "<< config_file["ConfigVersion"] << std::endl;
    std::cout << "done loading\n";
}

// void ExoConfig::SaveConfigFile(std::string file_name,SystemParam sys_param){
//     json new_config;

// }