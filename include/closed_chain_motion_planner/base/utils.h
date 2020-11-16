#pragma once

#include <Eigen/Core>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/SO3StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
using namespace ompl;
using namespace Eigen;
namespace ob = ompl::base;

namespace StateEigenUtils
{

    static Eigen::Isometry3d StateToIsometry(ompl::base::State *obj_state_)
    {
        auto *se3state = obj_state_->as<ob::SE3StateSpace::StateType>();
        auto *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);
        auto *rot = se3state->as<ob::SO3StateSpace::StateType>(1);
        Isometry3d base_obj;
        base_obj.setIdentity();
        base_obj.translation() = Eigen::Map<const Eigen::Vector3d>(pos->values);
        base_obj.linear() = Quaterniond(rot->w, rot->x, rot->y, rot->z).toRotationMatrix();
        return base_obj;
    }

    static std::pair<Eigen::Vector3d, Eigen::Vector4d> StateToPosQuat(ompl::base::State *obj_state_)
    {
        auto *se3state = obj_state_->as<ob::SE3StateSpace::StateType>();
        auto *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);
        auto *rot = se3state->as<ob::SO3StateSpace::StateType>(1);
        std::pair<Eigen::Vector3d, Eigen::Vector4d> result;
        result.first = Eigen::Map<const Eigen::Vector3d>(pos->values);
        result.second << rot->x, rot->y, rot->z, rot->w;
        return result;
    }

    static void IsometryToState(Eigen::Isometry3d base_obj, ompl::base::State *obj_state_)
    {
        obj_state_->as<ob::SE3StateSpace::StateType>()->setXYZ(base_obj.translation()[0], base_obj.translation()[1], base_obj.translation()[2]);
        auto *rot = obj_state_->as<ob::SE3StateSpace::StateType>()->as<ob::SO3StateSpace::StateType>(1);
        Eigen::Quaterniond quat(base_obj.linear());
        rot->w = quat.w();
        rot->x = quat.x();
        rot->y = quat.y();
        rot->z = quat.z();
    }

    // Eigen::Isometry3d vectorsToIsometry(const Eigen::Ref<const Eigen::Vector3d> &pos, const Eigen::Ref<const Eigen::Vector4d> &quat)
    // {
    //     Eigen::Isometry3d transform;
    //     transform.setIdentity();
    //     transform.linear() = Eigen::Quaterniond(quat).toRotationMatrix();
    //     transform.translation() = pos;
    //     return transform;
    // }
}