#pragma once
#include <iostream>
#include <yaml-cpp/yaml.h>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Geometry>
struct YAMLConfig
{
  void loadConfig(std::string file_name)
  {
    YAML::Node yamlnode;

    yamlnode = YAML::LoadFile(file_name);
    Eigen::Vector3d t_wo_start_pos(yamlnode["t_wo_start_pos"].as<std::vector<double> >().data());
    Eigen::Vector4d t_wo_start_quat(yamlnode["t_wo_start_quat"].as<std::vector<double> >().data());
    Eigen::QuaternionMapd qq(t_wo_start_quat.data());
    t_wo_start.setIdentity();
    t_wo_start.translation() = t_wo_start_pos;
    t_wo_start.linear() = qq.toRotationMatrix();
    
    Eigen::Vector3d t_wo_goal_pos(yamlnode["t_wo_goal_pos"].as<std::vector<double> >().data());
    Eigen::Vector4d t_wo_goal_quat(yamlnode["t_wo_goal_quat"].as<std::vector<double> >().data());
    Eigen::QuaternionMapd qq2(t_wo_goal_quat.data());
    t_wo_goal.setIdentity();
    t_wo_goal.translation() = t_wo_goal_pos;
    t_wo_goal.linear() = qq2.toRotationMatrix();

    debug_file_prefix_ = yamlnode["debug_file_prefix_"].as<std::string>();
    mesh_file_ = yamlnode["mesh_file_"].as<std::string>();
    start = Eigen::VectorXd(yamlnode["start_joint"].as<std::vector<double> >().data());
  }

    Eigen::Isometry3d base_left, base_right, base_top;
    Eigen::VectorXd start, goal;
    Eigen::Vector3d z_offset, t_wo_start_pos;
    Eigen::Isometry3d t_wo_start, t_wo_goal;

    std::vector<Eigen::Isometry3d> t_wb;
    std::vector<Eigen::Isometry3d> t_7e;
    std::string debug_file_prefix_, mesh_file_;
};