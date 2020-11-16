#pragma once

#include <sstream>
#include <string>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/SO3StateSpace.h>
#include <boost/math/constants/constants.hpp>
#include <boost/format.hpp>

#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <chrono>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/collision_detection/collision_common.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometric_shapes/shape_operations.h>

#include <moveit/collision_detection/collision_tools.h>

#include <geometry_msgs/Pose.h>

#include <mutex>

namespace ob = ompl::base;
using namespace std;
using namespace Eigen;
class IKValidityChecker
{
public:
    IKValidityChecker()
    {
        robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
        robot_model = robot_model_loader.getModel();
        planning_scene = std::make_shared<planning_scene::PlanningScene>(robot_model);
        acm_ = std::make_shared<collision_detection::AllowedCollisionMatrix>(planning_scene->getAllowedCollisionMatrix());
        robot_state::RobotState &current_state = planning_scene->getCurrentStateNonConst();

        Eigen::VectorXd default_start(7);
        default_start << 0, -0.785, 0, -1.571, 0, 1.571, 0.785;
        current_state.setJointGroupPositions("panda_left", default_start);
        current_state.setJointGroupPositions("panda_right", default_start);
        current_state.setJointGroupPositions("panda_top", default_start);
        current_state.update();

        /* Define a box to be attached */
        geometry_msgs::Pose box_pose;
        box_pose.position.x = 0.69;
        box_pose.position.y = -0.04;
        box_pose.position.z = 1.0826;
        box_pose.orientation.w = 1.0;
        addBox(Eigen::Vector3d(0.36, 0.21, 0.165), box_pose, "sub_table1");

        geometry_msgs::Pose box_pose2;
        box_pose2.position.x = 0.465;
        box_pose2.position.y = -0.505;
        box_pose2.position.z = 1.0826;
        box_pose2.orientation.w = 1.0;
        addBox(Eigen::Vector3d(0.21, 0.16, 0.165), box_pose2, "sub_table2");

        geometry_msgs::Pose box_pose3;
        box_pose3.position.x = 0.595;
        box_pose3.position.y = 0.355;
        box_pose3.position.z = 1.0826;
        box_pose3.orientation.w = 1.0;
        addBox(Eigen::Vector3d(0.16, 0.21, 0.165), box_pose3, "sub_table3");

        geometry_msgs::Pose box_pose4;
        box_pose4.position.x = 0.42;
        box_pose4.position.y = 0.1;
        box_pose4.position.z = 1.0826;
        box_pose4.orientation.w = 1.0;
        addBox(Eigen::Vector3d(0.21, 0.21, 0.165), box_pose4, "sub_table4");
    }

    void setArmNames(const std::vector<std::string> & arm_names)
    {
        arm_names_ = arm_names;
    }

    void addMeshFromFile(const std::string &file_name, geometry_msgs::Pose pose, const std::string &id)
    {
        shapes::Mesh *mesh = shapes::createMeshFromResource(file_name);
        shapes::ShapeMsg shape_msg;
        shapes::constructMsgFromShape(mesh, shape_msg);

        shape_msgs::Mesh obj_mesh = boost::get<shape_msgs::Mesh>(shape_msg);

        moveit_msgs::CollisionObject co;
        co.header.frame_id = "/base";
        co.id = id;
        co.meshes.push_back(obj_mesh);
        co.mesh_poses.push_back(pose);
        co.operation = moveit_msgs::CollisionObject::ADD;
        {
            planning_scene->processCollisionObjectMsg(co);
        }
        std::vector<std::string> collision_list{"panda_left_rightfinger", "panda_left_leftfinger", "panda_top_finger_left_link", "panda_top_finger_right_link", "panda_top_hand"};
        for (auto link : collision_list)
        {
            acm_->setEntry(id, link, true);
        }
    }

    void attachObject(const std::string &object_id, const std::string &link_name, const std::vector<std::string> &touch_links)
    {
        // link name : hand 
        moveit_msgs::AttachedCollisionObject aco;
        aco.object.id = object_id;
        aco.object.operation = moveit_msgs::CollisionObject::ADD;
        aco.link_name = link_name;
        aco.touch_links = robot_model->getJointModelGroup(link_name)->getLinkModelNames();
        planning_scene->processAttachedCollisionObjectMsg(aco);

        for (int i = 0; i < 2; i++)
            acm_->setEntry(object_id, arm_names_[i] + "_hand", true);

        std::vector<std::string> collision_list{"panda_left_rightfinger", "panda_left_leftfinger", "panda_top_finger_left_link", "panda_top_finger_right_link"};
        for (auto link : collision_list)
        {
            acm_->setEntry(object_id, link, true);
        }
    }


    void moveOBJ(const std::string &id, Isometry3d base_obj)
    {
        moveit_msgs::CollisionObject co;
        co.header.frame_id = "/base";
        co.id = id;
        geometry_msgs::Pose pose;
        pose.position.x = base_obj.translation()(0);
        pose.position.y = base_obj.translation()(1);
        pose.position.z = base_obj.translation()(2);
        Eigen::Quaterniond quat(base_obj.linear());
        pose.orientation.x = quat.x();
        pose.orientation.y = quat.y();
        pose.orientation.z = quat.z();
        pose.orientation.w = quat.w();

        co.mesh_poses.push_back(pose);
        co.operation = moveit_msgs::CollisionObject::MOVE;
        
        planning_scene->processCollisionObjectMsg(co);
    }

    bool IKValid(int arm_index, Eigen::Matrix<double, 7, 1> temp_sol)
    {
        // graphMutex_.lock();
        robot_state::RobotState robot_state = planning_scene->getCurrentState();
        robot_state.setJointGroupPositions(arm_names_[arm_index], temp_sol);
        robot_state.update();
        collision_detection::CollisionRequest req;
        req.verbose = true;
        req.group_name = arm_names_[arm_index];
        collision_detection::CollisionResult res;
        planning_scene->checkCollision(req, res, robot_state, *acm_);
        // graphMutex_.unlock();
        return !res.collision;
    }

    bool totalValid(Eigen::Matrix<double, 7, 1> sol1, Eigen::Matrix<double, 7, 1> sol2, Eigen::Matrix<double, 7, 1> sol3)
    {
        collision_detection::CollisionRequest req;
        collision_detection::CollisionResult res;
        robot_state::RobotState robot_state = planning_scene->getCurrentState();
        Eigen::Matrix<double, 21, 1> sol;
        sol << sol1, sol2 , sol3;
        robot_state.setJointGroupPositions("panda_triple", sol);
        robot_state.update();
        req.verbose = false;
        planning_scene->checkCollision(req, res, robot_state, *acm_);
        return !res.collision;
    }

protected:
private:
    robot_model::RobotModelPtr robot_model;
    std::shared_ptr<planning_scene::PlanningScene> planning_scene;
    collision_detection::AllowedCollisionMatrixPtr acm_;
    // moveit_msgs::PlanningScene moveit_scene;
    moveit_msgs::CollisionObject stefan_obj;
    std::vector<string> arm_names_;
    mutable std::mutex graphMutex_;

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
    // ROS_INFO("ADD BOX!!");
    // planning_scene_->getAllowedCollisionMatrixNonConst().setEntry('hand', id);
    }
};
