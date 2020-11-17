#include <closed_chain_motion_planner/kinematics/grasping_point.h>
#include <cmath>
#include <yaml-cpp/yaml.h>

grasping_point::grasping_point()
{
    base_left = Eigen::Isometry3d::Identity();
    base_right = Eigen::Isometry3d::Identity();
    base_top = Eigen::Isometry3d::Identity();

    base_left.translation() = Eigen::Vector3d(0, 0.3, 1.006);
    base_right.translation() = Eigen::Vector3d(0, -0.3, 1.006);
    base_top.translation() = Eigen::Vector3d(1.35, 0.3, 1.006); // 1.6
    base_top.linear() << -1, 0, 0,
                            0, -1, 0,
                            0, 0, 1;

    t_wb.push_back(base_left);
    t_wb.push_back(base_right);
    t_wb.push_back(base_top);

    Isometry3d T_normal, T_normal_R;
    T_normal.setIdentity();
    T_normal.translation() = Vector3d(0, 0, 0.103);

    T_normal_R.setIdentity();
    T_normal_R.translation() = Vector3d(0, 0, 0.0825);

    t_7e.push_back(T_normal);  // panda_left
    t_7e.push_back(T_normal_R); // panda_right
    t_7e.push_back(T_normal_R);  // panda_top

}
void grasping_point::loadConfig(std::string file_name)
{
    YAML::Node yamlnode;
    yamlnode = YAML::LoadFile("/home/jiyeong/catkin_ws/src/2_social/closed_chain_motion_planner/config/" + file_name + ".yaml");
    Eigen::Vector3d t_wo_start_pos(yamlnode["t_wo_start_pos"].as<std::vector<double>>().data());
    Eigen::Vector4d t_wo_start_quat(yamlnode["t_wo_start_quat"].as<std::vector<double>>().data());
    Eigen::QuaternionMapd qq(t_wo_start_quat.data());
    t_wo_start.setIdentity();
    t_wo_start.translation() = t_wo_start_pos;
    t_wo_start.linear() = qq.toRotationMatrix();

    Eigen::Vector3d t_wo_goal_pos(yamlnode["t_wo_goal_pos"].as<std::vector<double>>().data());
    Eigen::Vector4d t_wo_goal_quat(yamlnode["t_wo_goal_quat"].as<std::vector<double>>().data());
    Eigen::QuaternionMapd qq2(t_wo_goal_quat.data());
    t_wo_goal.setIdentity();
    t_wo_goal.translation() = t_wo_goal_pos;
    t_wo_goal.linear() = qq2.toRotationMatrix();

    debug_file_prefix_ = yamlnode["debug_file_prefix_"].as<std::string>();
    mesh_file_ = yamlnode["mesh_file_"].as<std::string>();
    // start = Eigen::VectorXd(yamlnode["start_joint"].as<std::vector<double>>().data(), 14);
    start = Eigen::Map<const Eigen::VectorXd>(yamlnode["start_joint"].as<std::vector<double>>().data(), 14);
    std::cout << start.transpose() << std::endl;
    
    obj_name = yamlnode["obj_name"].as<std::string>();
    arm_name1 = yamlnode["arm1"]["name"].as<std::string>();
    arm_name2 = yamlnode["arm2"]["name"].as<std::string>();

    arm_index1 = yamlnode["arm1"]["index"].as<int>();
    arm_index2 = yamlnode["arm2"]["index"].as<int>();
    
}
