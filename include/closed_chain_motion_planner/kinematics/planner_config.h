#pragma once
#include <iostream>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <fstream>
#include <memory>
using namespace Eigen;

class planner_config
{
public:
    planner_config();
    void loadConfig(const std::string &file_name = "dumbbell");
    Isometry3d base_left, base_right, base_top;
    VectorXd start, goal;
    Isometry3d t_wo_start, t_wo_goal;

    std::vector<Eigen::Isometry3d> t_wb;
    std::vector<Eigen::Isometry3d> t_7e;
    std::string debug_file_prefix_, mesh_file_;
    std::string arm_name1, arm_name2, obj_name;
    int arm_index1;
    int arm_index2;
};

typedef std::shared_ptr<planner_config> ConfigPtr;
