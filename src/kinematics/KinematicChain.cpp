#include <closed_chain_motion_planner/kinematics/KinematicChain.h>

KinematicChainValidityChecker::KinematicChainValidityChecker(const ompl::base::SpaceInformationPtr &si)
    : ompl::base::StateValidityChecker(si)
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

    Eigen::Vector2d gpos;
    gpos << 0.04, 0.04;
    current_state.setJointGroupPositions("hand_left", gpos);
    current_state.setJointGroupPositions("hand_right", gpos);
    current_state.setJointGroupPositions("hand_top", gpos);
    moveit_scene.is_diff = true;

    geometry_msgs::Pose box_pose;
    box_pose.position.x = 0.65;
    box_pose.position.y = 0.0;
    box_pose.position.z = 1.1;
    box_pose.orientation.w = 1.0;
    addBox(Eigen::Vector3d(0.65, 1.0, 0.2), box_pose, "sub_table");

    scene_pub_ = nh_.advertise<moveit_msgs::PlanningScene>("/planning_scenes_suhan", 1);
}

void KinematicChainValidityChecker::setStartStates(const Eigen::Ref<const Eigen::VectorXd> &q)
{
    robot_state::RobotState &robot_state = planning_scene->getCurrentStateNonConst();
    // std::lock_guard<std::mutex> lg(locker_);
    for (int i = 0; i < arm_names_.size(); i++)
    {
        const auto &q_seg = q.segment<7>(i * 7);
        const robot_model::JointModelGroup *joint_model_group = robot_state.getJointModelGroup(arm_names_[i]);
        robot_state.setJointGroupPositions(joint_model_group, q_seg);
        robot_state.update();
        break;
    }
    moveit_msgs::PlanningScene scene_msg;
    planning_scene->getPlanningSceneMsg(scene_msg);
    scene_pub_.publish(scene_msg);
}

void KinematicChainValidityChecker::addMeshFromFile(const std::string &file_name, geometry_msgs::Pose pose, const std::string &id)
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
}

void KinematicChainValidityChecker::attachObject(const std::string &object_id, const std::string &link_name, const std::string &touch_links)
{
    // link name : hand
    moveit_msgs::AttachedCollisionObject aco;
    aco.object.id = object_id;
    aco.object.operation = moveit_msgs::CollisionObject::ADD;
    aco.link_name = link_name;
    aco.touch_links = robot_model->getJointModelGroup(touch_links)->getLinkModelNames();
    planning_scene->processAttachedCollisionObjectMsg(aco);
    moveit_msgs::PlanningScene scene_msg;
    planning_scene->getPlanningSceneMsg(scene_msg);

    // while (true)
    //     scene_pub_.publish(scene_msg);
    // std::vector<std::string> collision_list{"panda_left_rightfinger", "panda_left_leftfinger", "panda_top_finger_left_link", "panda_top_finger_right_link", "panda_top_hand"};
    std::vector<std::string> collision_list{"panda_left_rightfinger", "panda_left_leftfinger"};
    for (auto link : collision_list)
    {
        acm_->setEntry(object_id, link, true);
    }
}

bool KinematicChainValidityChecker::isValid(const ob::State *state) const
{
    // return true;
    Eigen::Map<Eigen::VectorXd> q = *state->as<ob::ConstrainedStateSpace::StateType>();
    return isValidImpl(q);
}

bool KinematicChainValidityChecker::isValidImpl(const Eigen::Ref<const Eigen::VectorXd> &q) const
{
    collision_detection::CollisionRequest req;
    collision_detection::CollisionResult res;
    // req.verbose = true;
    // req.max_contacts = 100;
    {
        robot_state::RobotState robot_state = planning_scene->getCurrentState();
        for (int i = 0; i < arm_names_.size(); i++)
        {
            const auto &q_seg = q.segment<7>(i * 7);
            // std::cout << arm_names_[i] << " " << q_seg.transpose() << std::endl;
            const robot_model::JointModelGroup *joint_model_group = robot_state.getJointModelGroup(arm_names_[i]);
            robot_state.setJointGroupPositions(joint_model_group, q_seg);
            robot_state.update();
        }
        planning_scene->checkCollision(req, res, robot_state, *acm_);
        // moveit_msgs::PlanningScene scene_msg;
        // planning_scene->getPlanningSceneMsg(scene_msg);
        // scene_pub_.publish(scene_msg);
    }
    return !res.collision;
}
void KinematicChainValidityChecker::setArmNames(const std::vector<std::string> &arm_name)
{
    arm_names_ = arm_name;
}

bool KinematicChainValidityChecker::IKValid(int arm_index, Eigen::Matrix<double, 7, 1> temp_sol)
{
    // graphMutex_.lock();
    // robot_state::RobotState &robot_state = planning_scene->getCurrentStateNonConst();
    // // std::lock_guard<std::mutex> lg(locker_);
    // for (int i = 0; i < arm_names_.size(); i++)
    // {
    //     const auto & q_seg = q.segment<7>(i*7);
    //     std::cout << arm_names_[i] << " " << q_seg.transpose() << std::endl;
    //     const robot_model::JointModelGroup* joint_model_group = robot_state.getJointModelGroup(arm_names_[i]);
    //     robot_state.setJointGroupPositions(joint_model_group, q_seg);
    //     robot_state.update();
    // }
    // moveit_msgs::PlanningScene scene_msg;
    // planning_scene->getPlanningSceneMsg(scene_msg);
    // scene_pub_.publish(scene_msg);

    robot_state::RobotState &robot_state = planning_scene->getCurrentStateNonConst();
    robot_state.setJointGroupPositions(arm_names_[arm_index], temp_sol);
    robot_state.update();
    collision_detection::CollisionRequest req;
    // req.verbose = true;
    req.group_name = arm_names_[arm_index];
    collision_detection::CollisionResult res;
    planning_scene->checkCollision(req, res, robot_state, *acm_);
    // graphMutex_.unlock();
    //  moveit_msgs::PlanningScene scene_msg;
    // planning_scene->getPlanningSceneMsg(scene_msg);
    // scene_pub_.publish(scene_msg);
    // scene_pub_.publish(scene_msg);
    // scene_pub_.publish(scene_msg);
    return !res.collision;
}