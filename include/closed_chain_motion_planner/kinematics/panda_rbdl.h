#pragma once

#include <memory>

#include <rbdl/rbdl.h>
#include <Eigen/Dense>

class RobotModel
{
public:
  RobotModel() {}

  virtual Eigen::MatrixXd getJacobianMatrix(const Eigen::VectorXd &q) const = 0;
  virtual Eigen::Vector3d getTranslation(const Eigen::VectorXd &q) const = 0;
  virtual Eigen::Matrix3d getRotation(const Eigen::VectorXd &q) const = 0;
  virtual Eigen::Isometry3d getTransform(const Eigen::VectorXd &q) const = 0; 
  virtual Eigen::MatrixXd getJointLimit() const = 0;
  virtual Eigen::VectorXd getInitialConfiguration() const = 0;

  virtual void initModel() = 0;
  virtual void initModel(const Eigen::Ref<const Eigen::Matrix<double, 7, 4> > & dh) = 0;
  
  virtual int getDof() = 0;

  Eigen::Vector3d getPhi(const Eigen::Matrix3d a, const Eigen::Matrix3d b)
  {
    Eigen::Vector3d phi;
    Eigen::Vector3d s[3], v[3], w[3];

    for (int i = 0; i < 3; i++) {
      v[i] = a.block<3, 1>(0, i);
      w[i] = b.block<3, 1>(0, i);
      s[i] = v[i].cross(w[i]);
    }
    phi = s[0] + s[1] + s[2];
    phi = -0.5* phi;

    return phi;
  }
};


class PandaModel : public RobotModel
{
public:
  static constexpr int kDof=7;
  static constexpr double kGravity=9.8;

  PandaModel();

  Eigen::MatrixXd getJacobianMatrix(const Eigen::VectorXd &q) const override;
  Eigen::Vector3d getTranslation(const Eigen::VectorXd &q) const override;
  Eigen::Matrix3d getRotation(const Eigen::VectorXd &q) const override;
  Eigen::Isometry3d getTransform(const Eigen::VectorXd &q) const override;
  Eigen::MatrixXd getJointLimit() const override;
  Eigen::VectorXd getInitialConfiguration() const override;

  int getDof() override;

  void initModel();
  void initModel(const Eigen::Ref<const Eigen::Matrix<double, 7, 4> > & dh); // 7x4
  
private:
  Eigen::Isometry3d transformDH(const double a, const double d, const double alpha, const double theta);
  
  RigidBodyDynamics::Math::Vector3d com_position_[kDof];
  RigidBodyDynamics::Math::Vector3d ee_position_;
  Eigen::Vector3d joint_posision_[kDof];
  Eigen::Matrix3d rot_ee_;

  std::shared_ptr<RigidBodyDynamics::Model> model_;
  unsigned int body_id_[kDof];
  RigidBodyDynamics::Body body_[kDof];
  RigidBodyDynamics::Joint joint_[kDof];
};

typedef std::shared_ptr<RobotModel> RobotModelPtr;