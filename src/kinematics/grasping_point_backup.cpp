#include <closed_chain_motion_planner/kinematics/grasping_point.h>
#include <cmath>

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

    t_7e.push_back(T_normal);   // panda_left
    t_7e.push_back(T_normal_R); // panda_right
    t_7e.push_back(T_normal_R); // panda_top

    start << -0.16661368, -0.7661184, -0.03369873, -2.37254935, -0.09888003, 1.6927669, 0.17440837,
        0.40648265944447426, 0.2046333102301547, -0.10520152309796396, -1.2851498105581256, 0.11502121452082804, 1.4620952500450504, 0.4454484761905;

    t_wo_start.setIdentity();
    t_wo_start_pos = Vector3d(0.958 - 0.007, 0.213 + 0.01, 1.557);
    t_wo_start_quat = Quaterniond(0.4418, -0.0446, 0.0220, 0.8958);
    t_wo_start.translation() = t_wo_start_pos;
    t_wo_start.linear() = t_wo_start_quat.toRotationMatrix();

    t_wo_goal.translation() = Vector3d(0.7, 0.45, 1.557 + 0.05);
    t_wo_goal.linear() = Quaterniond(-0.10912217, -0.03777281, -0.004149, 0.99330174).toRotationMatrix();

    debug_file_prefix_ = "/home/jiyeong/catkin_ws/src/2_social/closed_chain_motion_planner/debug/";
    mesh_file_ = "package://grasping_point/STEFAN/stl/assembly_without_bottom.stl";
}
