#pragma once

#include <random>

#include <ros/ros.h>
#include <Eigen/Dense>
#include <cstdint>
#include <trac_ik/trac_ik.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <iostream>
#include <memory>
#include <mutex>
using namespace Eigen;

Eigen::Matrix3d getEigenRotation(const KDL::Rotation & r);
Eigen::Vector3d getEigenVector(const KDL::Vector& v);
Eigen::Isometry3d getEigenFrame(const KDL::Frame & frame);
KDL::Rotation getKDLRotation(const Eigen::Matrix3d & matrix);
KDL::Vector getKDLVector(const Eigen::Vector3d& vector);
KDL::Frame getKDLFrame(const Eigen::Isometry3d & transform);

class TrackIKAdaptor
{
public:
    TrackIKAdaptor() {}
    virtual bool solve(const Eigen::Ref<const Eigen::VectorXd> &q0, const Eigen::Isometry3d &transform, Eigen::Ref<Eigen::VectorXd> solution) = 0;
    virtual Eigen::Isometry3d fk(const Eigen::Ref<const Eigen::VectorXd> &q) = 0;

    bool solve(const Eigen::Isometry3d &transform, Eigen::Ref<Eigen::VectorXd> solution);
    Eigen::VectorXd getRandomConfig();
    bool randomSolve(const Eigen::Isometry3d &transform, Eigen::Ref<Eigen::VectorXd> solution);
    Eigen::VectorXd getLowerBound() const { return lb_; }
    Eigen::VectorXd getUpperBound() const { return ub_; }
    bool isValid(const Eigen::Ref<const Eigen::VectorXd> &q);
    double gaussian(double mean, double stddev);

protected:
    unsigned int n_joint_{7};
    KDL::JntArray nominal_;
    Eigen::VectorXd lb_, ub_;
    std::normal_distribution<> normalDist_{0, 1};
    std::mt19937 generator_;
    std::mutex mutex_;
};

class panda_ik : public TrackIKAdaptor
{
public:
    panda_ik(const std::string & arm_name);
    
    bool solve(const Eigen::Ref<const Eigen::VectorXd> &q0, const Eigen::Isometry3d &transform, Eigen::Ref<Eigen::VectorXd> solution) override;
    Eigen::Isometry3d fk(const Eigen::Ref<const Eigen::VectorXd> &q);
    Eigen::Isometry3d random_fk();
    void SetSolveType(TRAC_IK::SolveType type);

private:
    std::string chain_start{"panda_left_link0"};
    std::string chain_end{"panda_left_hand"};
    std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver;
    KDL::Chain chain;
    TRAC_IK::TRAC_IK tracik_solver_;
};

typedef std::shared_ptr<TrackIKAdaptor> TrackIKAdaptorPtr;
typedef std::shared_ptr<panda_ik> panda_ikPtr;