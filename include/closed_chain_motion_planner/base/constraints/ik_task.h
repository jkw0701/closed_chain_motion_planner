#pragma once

#include <iostream>
#include <map>
#include <closed_chain_motion_planner/kinematics/panda_model.h>
#include <queue>
#include <algorithm>
// #include <closed_chain_motion_planner/base/constraints/ik_constraint.h>
#include <Eigen/Dense>
class IKTask
{
public:
  IKTask(const std::string & name = "");
  virtual ~IKTask();

  bool solve(const Eigen::Ref<const Eigen::VectorXd> & q0, Eigen::Ref<Eigen::VectorXd> q_out);
  bool random_solve(Eigen::Ref<Eigen::VectorXd> q_out);
  void setMaxOptTrials(double trials) { max_opt_trials_ = trials; }
  void setTargetPose(const Eigen::Isometry3d & target_pose);
  void setArmName(const std::string & arm_name)
  {
    arm_name_ = arm_name;
  }
  void setArmModels(const std::map<std::string, ArmModelPtr> &arm_models)
  {
    arm_models_ = arm_models;
  }  
  void setSigma(double sigma) { sigma_ = sigma; }
  Eigen::Isometry3d compute_t_wo(const Eigen::Ref<const Eigen::VectorXd> & q0);
  std::map<std::string, ArmModelPtr> arm_models_;
protected:
  Eigen::Isometry3d target_pose_;
  int max_opt_trials_ {50};
  int max_trials_ {50};
  double sigma_ {0.3}; ///< random variation
  std::string arm_name_;
  std::mutex mutex_;
};

typedef std::shared_ptr<IKTask> IKTaskPtr;
