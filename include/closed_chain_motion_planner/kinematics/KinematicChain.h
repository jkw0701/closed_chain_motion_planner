#pragma once

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <boost/math/constants/constants.hpp>
#include <boost/format.hpp>
#include <fstream>

#include <Eigen/Dense>
#include <chrono>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/collision_detection/collision_common.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <urdf_parser/urdf_parser.h>

#include <moveit/kinematic_constraints/utils.h>
#include <sstream>
#include <algorithm>
#include <ctype.h>
#include <fstream>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometric_shapes/shape_operations.h>
#include <shape_msgs/SolidPrimitive.h>
// #include <moveit/planning_scene_monitor/planning_scene_monitor.h>
// #include <moveit/collision_detection/collision_tools.h>

// #include <geometry_msgs/Pose.h>
// #include <tf/transform_datatypes.h>

#include <ompl/base/ConstrainedSpaceInformation.h>

#include <mutex>

namespace ob = ompl::base;
using namespace Eigen;

class KinematicChainProjector : public ompl::base::ProjectionEvaluator
{
public:
    KinematicChainProjector(const ompl::base::StateSpace *space) : ompl::base::ProjectionEvaluator(space)
    {
        int dimension = std::max(2, (int)ceil(log((double)space->getDimension()))); //ceil : 올림함수, dimension = 3
        projectionMatrix_.computeRandom(space->getDimension(), dimension);
    }
    unsigned int getDimension() const override
    {
        return projectionMatrix_.mat.rows();
    }
    void project(const ompl::base::State *state, Eigen::Ref<Eigen::VectorXd> projection) const override
    {
        std::vector<double> v(space_->getDimension());
        space_->copyToReals(v, state);
        projectionMatrix_.project(&v[0], projection);
    }

protected:
    /* A projection matrix -- it allows multiplication of
            real vectors by a specified matrix. The matrix can also be
            randomly generated. */
    ompl::base::ProjectionMatrix projectionMatrix_;
};

class KinematicChainSpace : public ompl::base::RealVectorStateSpace
{
public:
    KinematicChainSpace(unsigned int numLinks)
        : ompl::base::RealVectorStateSpace(numLinks)
    {
        ompl::base::RealVectorBounds bounds(numLinks);

        for (int i = 0; i < 2; i++)
        {
            bounds.setLow(0 + i * 7, -2.8973);
            bounds.setHigh(0 + i * 7, 2.8973);

            bounds.setLow(1 + i * 7, -1.7628);
            bounds.setHigh(1 + i * 7, 1.7628);

            bounds.setLow(2 + i * 7, -2.8973);
            bounds.setHigh(2 + i * 7, 2.8973);

            bounds.setLow(3 + i * 7, -3.0718);
            bounds.setHigh(3 + i * 7, -0.0698);

            bounds.setLow(4 + i * 7, -2.8973);
            bounds.setHigh(4 + i * 7, 2.8973);

            bounds.setLow(5 + i * 7, -0.0175);
            bounds.setHigh(5 + i * 7, 3.7525);

            bounds.setLow(6 + i * 7, -2.8973);
            bounds.setHigh(6 + i * 7, 2.8973);
        }
        setBounds(bounds);
        std::cout << "  - min: ";
        for (unsigned int i = 0; i < dimension_; ++i)
            std::cout << bounds.low[i] << " ";
        std::cout << std::endl;
        std::cout << "  - max: ";
        for (unsigned int i = 0; i < dimension_; ++i)
            std::cout << bounds.high[i] << "  ";
        std::cout << std::endl;

        type_ = ompl::base::STATE_SPACE_SO2;
    }

    void registerProjections() override
    {
        registerDefaultProjection(std::make_shared<KinematicChainProjector>(this));
    }

    void enforceBounds(ompl::base::State *state) const override
    {
        auto *statet = state->as<StateType>();
        for (unsigned int i = 0; i < dimension_; ++i)
        {
            double v = fmod(statet->values[i], 2.0 * boost::math::constants::pi<double>());
            if (v < -boost::math::constants::pi<double>())
                v += 2.0 * boost::math::constants::pi<double>();
            else if (v >= boost::math::constants::pi<double>())
                v -= 2.0 * boost::math::constants::pi<double>();
            statet->values[i] = v;
        }
    }

    bool equalStates(const ompl::base::State *state1, const ompl::base::State *state2) const override
    {

        const double *s1 = static_cast<const StateType *>(state1)->values;
        const double *s2 = static_cast<const StateType *>(state2)->values;
        for (unsigned int i = 0; i < dimension_; ++i)
        {
            double diff = (*s1++) - (*s2++);
            if (fabs(diff) > 1e-10) // 10
                return false;
        }
        return true;
    }
    void interpolate(const ompl::base::State *from, const ompl::base::State *to, const double t,
                     ompl::base::State *state) const override
    {
        const auto *fromt = from->as<StateType>();
        const auto *tot = to->as<StateType>();
        auto *statet = state->as<StateType>();

        for (unsigned int i = 0; i < dimension_; ++i)
        {
            double diff = tot->values[i] - fromt->values[i];
            if (fabs(diff) <= boost::math::constants::pi<double>())
                statet->values[i] = fromt->values[i] + diff * t;
            else
            {
                if (diff > 0.0)
                    diff = 2.0 * boost::math::constants::pi<double>() - diff;
                else
                    diff = -2.0 * boost::math::constants::pi<double>() - diff;

                statet->values[i] = fromt->values[i] - diff * t;
                if (statet->values[i] > boost::math::constants::pi<double>())
                    statet->values[i] -= 2.0 * boost::math::constants::pi<double>();
                else if (statet->values[i] < -boost::math::constants::pi<double>())
                    statet->values[i] += 2.0 * boost::math::constants::pi<double>();
            }
        }
    }

protected:
};


class KinematicChainValidityChecker : public ompl::base::StateValidityChecker // to find valid state space configurations
{
public:
    KinematicChainValidityChecker(const ompl::base::SpaceInformationPtr &si);
    bool isValid(const ob::State *state) const override;
    void addMeshFromFile(const std::string & file_name, geometry_msgs::Pose pose, const std::string &id);
    void attachObject(const std::string &object_id, const std::string &link_name, const std::string &touch_links);
    void setArmNames(const std::vector<std::string> &arm_name);
    void setStartStates(const Eigen::Ref<const Eigen::VectorXd> &q);
    bool IKValid(int arm_index, Eigen::Matrix<double, 7, 1> temp_sol);
private:
    robot_model::RobotModelPtr robot_model;
    std::shared_ptr<planning_scene::PlanningScene> planning_scene;
    collision_detection::AllowedCollisionMatrixPtr acm_;
    moveit_msgs::PlanningScene moveit_scene;

    robot_state::JointModelGroup *planning_group;
    void addBox(const Eigen::Ref<const Eigen::Vector3d> &dim, geometry_msgs::Pose pose, const std::string &id)
    {
        shape_msgs::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[0] = dim(0);
        primitive.dimensions[1] = dim(1);
        primitive.dimensions[2] = dim(2);

        moveit_msgs::CollisionObject co;
        co.header.frame_id = "/base";
        co.id = id;
        co.primitives.push_back(primitive);
        co.primitive_poses.push_back(pose);
        co.operation = moveit_msgs::CollisionObject::ADD;
        
        planning_scene->processCollisionObjectMsg(co);
    }

protected:
    bool isValidImpl(const Eigen::Ref<const Eigen::VectorXd> &q) const;
    mutable std::mutex locker_;
    ros::Publisher scene_pub_;
    ros::NodeHandle nh_;
    std::vector<std::string> arm_names_;
    std::mutex graphMutex_;
};


typedef std::shared_ptr<KinematicChainValidityChecker> KinematicChainValidityCheckerPtr;