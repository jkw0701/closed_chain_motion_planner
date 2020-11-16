#pragma once

#include <Eigen/Dense>
#include <closed_chain_motion_planner/kinematics/panda_tracik.h>
#include <closed_chain_motion_planner/kinematics/panda_rbdl.h>

struct ArmModel
{
  std::string name;
  int index;
  TrackIKAdaptorPtr trac_ik_model;
  RobotModelPtr rbdl_model;

  Eigen::Isometry3d t_7e; // end effector frame
  Eigen::Isometry3d t_wb; // base frame wrt world frame
  Eigen::Isometry3d t_o7;
  ArmModel()
  {
    t_7e.setIdentity();
    t_wb.setIdentity();
    t_o7.setIdentity();
  }
};

typedef std::shared_ptr<ArmModel> ArmModelPtr;
