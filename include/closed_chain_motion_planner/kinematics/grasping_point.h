#pragma once
#include <iostream>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
using namespace Eigen;
struct ContinuousGraspCandid
{
  std::vector< std::pair< std::pair<Eigen::Vector3d, Eigen::Vector3d>, Eigen::Quaterniond> > candids;

  void loadConfig(std::string file_name);

  Eigen::Isometry3d getGrasp(int index, double ratio);
  Eigen::Isometry3d getGrasp(double ratio);
};
class grasping_point
{
public:
    grasping_point();
    Isometry3d base_left, base_right, base_top;
    Matrix<double, 14, 1> start, goal;
    Vector3d z_offset, t_wo_start_pos;
    Quaterniond t_wo_start_quat;
    Isometry3d t_wo_start, t_wo_goal;

    std::vector<Eigen::Isometry3d> t_wb;
    std::vector<Eigen::Isometry3d> t_7e;
    Eigen::Matrix<double, 7, 4> calib_dh_left, calib_dh_right, calib_dh_top;
    std::vector<Eigen::Matrix<double, 7, 4> > calib_dh;
    std::string debug_file_prefix_, mesh_file_;
};

