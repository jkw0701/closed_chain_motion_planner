#include <closed_chain_motion_planner/base/constraints/ik_task.h>

IKTask::IKTask(const std::string & name)
{
}

IKTask::~IKTask()
{}

Eigen::Isometry3d IKTask::compute_t_wo(const Eigen::Ref<const Eigen::VectorXd> & q0)
{
  ArmModelPtr target_arm = arm_models_["panda_left"];
  return target_arm->t_wb * target_arm->rbdl_model->getTransform(q0) * target_arm->t_o7.inverse();
}

bool IKTask::solve(const Eigen::Ref<const Eigen::VectorXd> & q0, Eigen::Ref<Eigen::VectorXd> q_out)
{
  ArmModelPtr target_arm = arm_models_[arm_name_];

  int arm_index = target_arm->index;

  const Eigen::Isometry3d &t_wb = target_arm->t_wb;
  const Eigen::Isometry3d &t_o7 = target_arm->t_o7;
  Eigen::Isometry3d t_b7 = t_wb.inverse() * target_pose_ * t_o7;
  bool result = target_arm->trac_ik_model->solve(q0, t_b7, q_out);
  
  return result;
}

bool IKTask::random_solve(Eigen::Ref<Eigen::VectorXd> q_out)
{
  ArmModelPtr target_arm = arm_models_[arm_name_];
  int arm_index = target_arm->index;

  const Eigen::Isometry3d &t_wb = target_arm->t_wb;
  const Eigen::Isometry3d &t_o7 = target_arm->t_o7;
  Eigen::Isometry3d t_b7 = t_wb.inverse() * target_pose_ * t_o7;
  
  // Phase 3: random
  int iter = max_trials_;
  while (iter --)
  {
    if (target_arm->trac_ik_model->randomSolve(t_b7, q_out))
    {
      return true;
    }
  }
  return false;
}


void IKTask::setTargetPose(const Eigen::Isometry3d & target_pose)
{
  target_pose_ = target_pose;
}
