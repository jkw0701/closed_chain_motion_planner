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
    Eigen::Vector3d t_wo_goal_pos(yamlnode["t_wo_goal_pos"].as<std::vector<double> >().data());
    Eigen::Vector4d t_wo_start_quat(yamlnode["t_wo_start_quat"].as<std::vector<double> >().data());
    Eigen::Vector4d t_wo_goal_quat(yamlnode["t_wo_goal_quat"].as<std::vector<double> >().data());

    debug_file_prefix_ = yamlnode["debug_file_prefix_"].as<std::string>();
    mesh_file_ = yamlnode["mesh_file_"].as<std::string>();
  }

    Eigen::Isometry3d base_left, base_right, base_top;
    Eigen::Matrix<double, 14, 1> start, goal;
    Eigen::Vector3d z_offset, t_wo_start_pos;
    Eigen::Isometry3d t_wo_start, t_wo_goal;

    std::vector<Eigen::Isometry3d> t_wb;
    std::vector<Eigen::Isometry3d> t_7e;
    Eigen::Matrix<double, 7, 4> calib_dh_left, calib_dh_right, calib_dh_top;
    std::vector<Eigen::Matrix<double, 7, 4> > calib_dh;
    std::string debug_file_prefix_, mesh_file_;
};