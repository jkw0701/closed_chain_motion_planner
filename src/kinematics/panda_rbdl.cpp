#include <closed_chain_motion_planner/kinematics/panda_rbdl.h>
#include <Eigen/Geometry>

PandaModel::PandaModel()
{
  initModel();
}

Eigen::MatrixXd PandaModel::getJacobianMatrix(const Eigen::VectorXd &q) const
{
  Eigen::MatrixXd j_temp;
  j_temp.setZero(6, kDof);
  Eigen::Matrix<double, 6, kDof> j;
  RigidBodyDynamics::CalcPointJacobian6D(*model_, q, body_id_[kDof - 1], ee_position_, j_temp, true);

  for (int i = 0; i < 2; i++)
  {
    j.block<3, kDof>(i * 3, 0) = j_temp.block<3, kDof>(3 - i * 3, 0);
  }

  return j;
}

Eigen::Vector3d PandaModel::getTranslation(const Eigen::VectorXd &q) const
{
  return RigidBodyDynamics::CalcBodyToBaseCoordinates(*model_, q, body_id_[kDof - 1], ee_position_, true);
}

Eigen::Matrix3d PandaModel::getRotation(const Eigen::VectorXd &q) const
{
  auto M = rot_ee_ * Eigen::AngleAxisd(-M_PI/4., Eigen::Vector3d::UnitZ());
  return RigidBodyDynamics::CalcBodyWorldOrientation(*model_, q, body_id_[kDof - 1], true).transpose() * M;
}

Eigen::Isometry3d PandaModel::getTransform(const Eigen::VectorXd &q) const
{
  Eigen::Isometry3d transform;
  transform.linear() = getRotation(q);
  transform.translation() = getTranslation(q);

  return transform;
}

Eigen::MatrixXd PandaModel::getJointLimit() const
{
  Eigen::Matrix<double, kDof, 2> joint_limits;
  joint_limits << -2.8973, 2.8973,
      -1.7628, 1.7628,
      -2.8973, 2.8973,
      -3.0718, -0.0698,
      -2.8973, 2.8973,
      -0.0175, 3.7525,
      -2.8973, 2.8973;
  return joint_limits;
}

Eigen::VectorXd PandaModel::getInitialConfiguration() const
{
}

int PandaModel::getDof()
{
  return kDof;
}

void PandaModel::initModel()
{
  Eigen::Matrix<double, 7, 4> dh;
  dh.setZero();
  initModel(dh);
}

void PandaModel::initModel(const Eigen::Ref<const Eigen::Matrix<double, 7, 4>> &dh)
{
  model_ = std::make_shared<RigidBodyDynamics::Model>();

  model_->gravity = Eigen::Vector3d(0., 0., -kGravity);

  double mass[kDof];
  mass[0] = 1.0;
  mass[1] = 1.0;
  mass[2] = 1.0;
  mass[3] = 1.0;
  mass[4] = 1.0;
  mass[5] = 1.0;
  mass[6] = 1.0;

  Eigen::Vector3d global_joint_position[kDof], global_joint_position_new[kDof], axis[kDof];
  Eigen::VectorXd dh_al(7), dh_a(7), dh_d(7), dh_q(7);
  Eigen::Isometry3d transform_joint;
  transform_joint.setIdentity();
  const Eigen::Ref<const Eigen::VectorXd> &a_offset = dh.col(0);
  const Eigen::Ref<const Eigen::VectorXd> &d_offset = dh.col(1);
  const Eigen::Ref<const Eigen::VectorXd> &q_offset = dh.col(2);
  const Eigen::Ref<const Eigen::VectorXd> &alpha_offset = dh.col(3);

  dh_al << 0.0, -1.0 * M_PI_2, M_PI_2, M_PI_2, -1.0 * M_PI_2, M_PI_2, M_PI_2;
  dh_a << 0.0, 0.0, 0.0, 0.0825, -0.0825, 0.0, 0.088;
  dh_d << 0.333, 0.0, 0.316, 0.0, 0.384, 0.0, 0.0;

  global_joint_position[0] = Eigen::Vector3d(0.0, 0.0, 0.3330);
  global_joint_position[1] = Eigen::Vector3d(0.0, 0.0, 0.3330);
  global_joint_position[2] = Eigen::Vector3d(0.0, 0.0, 0.6490);
  global_joint_position[3] = Eigen::Vector3d(0.0825, 0.0, 0.6490);
  global_joint_position[4] = Eigen::Vector3d(0.0, 0.0, 1.0330);
  global_joint_position[5] = Eigen::Vector3d(0.0, 0.0, 1.0330);
  global_joint_position[6] = Eigen::Vector3d(0.0880, 0.0, 1.0330);

  com_position_[0] = Eigen::Vector3d(0.000096, -0.0346, 0.2575);
  com_position_[1] = Eigen::Vector3d(0.0002, 0.0344, 0.4094);
  com_position_[2] = Eigen::Vector3d(0.0334, 0.0266, 0.6076);
  com_position_[3] = Eigen::Vector3d(0.0331, -0.0266, 0.6914);
  com_position_[4] = Eigen::Vector3d(0.0013, 0.0423, 0.9243);
  com_position_[5] = Eigen::Vector3d(0.0421, -0.0103, 1.0482);
  com_position_[6] = Eigen::Vector3d(0.1, -0.0120, 0.9536);

  for (int i = 0; i < kDof; i++)
  {
    transform_joint = transform_joint * transformDH(dh_a(i) + a_offset(i), dh_d(i) + d_offset(i), dh_al(i) + alpha_offset(i), q_offset(i));
    axis[i] = transform_joint.matrix().block<3, 1>(0, 2);
    global_joint_position_new[i] = transform_joint.translation();
  }

  rot_ee_ = transform_joint.linear();
  ee_position_ = Eigen::Vector3d(0.0, 0.0, 0.107);
  ee_position_ = rot_ee_ * ee_position_;

  joint_posision_[0] = global_joint_position_new[0];
  for (int i = 1; i < kDof; i++)
    joint_posision_[i] = global_joint_position_new[i] - global_joint_position_new[i - 1];

  for (int i = 0; i < kDof; i++)
    com_position_[i] -= global_joint_position[i];

  RigidBodyDynamics::Math::Vector3d inertia[kDof];
  for (int i = 0; i < kDof; i++)
    inertia[i] = Eigen::Vector3d::Identity() * 0.001;

  for (int i = 0; i < kDof; i++)
  {
    body_[i] = RigidBodyDynamics::Body(mass[i], com_position_[i], inertia[i]);
    joint_[i] = RigidBodyDynamics::Joint(RigidBodyDynamics::JointTypeRevolute, axis[i]);
    if (i == 0)
      body_id_[i] = model_->AddBody(0, RigidBodyDynamics::Math::Xtrans(joint_posision_[i]), joint_[i], body_[i]);
    else
      body_id_[i] = model_->AddBody(body_id_[i - 1], RigidBodyDynamics::Math::Xtrans(joint_posision_[i]), joint_[i], body_[i]);
  }
}

Eigen::Isometry3d PandaModel::transformDH(const double a, const double d, const double alpha, const double theta)
{
  Eigen::Isometry3d transform_dh;
  double st = sin(theta), ct = cos(theta);
  double sa = sin(alpha), ca = cos(alpha);
  transform_dh.setIdentity();
  transform_dh.linear() << ct, -1 * st, 0.0,
      st * ca, ct * ca, -1 * sa,
      st * sa, ct * sa, ca;
  transform_dh.translation() << a, -1 * sa * d, ca * d;
  return transform_dh;
}