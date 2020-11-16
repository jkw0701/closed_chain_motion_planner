#include <closed_chain_motion_planner/kinematics/grasping_point.h>
#include <cmath>
#include <yaml-cpp/yaml.h>

Eigen::Isometry3d getGrpYaml(std::string file_name, std::string direction, double ratio)
{
    YAML::Node yamlnode;
    yamlnode = YAML::LoadFile(file_name);

    Eigen::Vector3d lb = Eigen::Vector3d::Map(yamlnode[direction][0]["lower_bound"].as<std::vector<double>>().data());
    Eigen::Vector3d ub = Eigen::Vector3d::Map(yamlnode[direction][0]["upper_bound"].as<std::vector<double>>().data());
    Eigen::Quaterniond ori;
    ori.coeffs() = Eigen::Vector4d::Map(yamlnode[direction][0]["orientation"].as<std::vector<double>>().data());
    Eigen::Isometry3d result;
    result.translation() = (ub - lb) * ratio + lb;
    result.linear() = ori.toRotationMatrix();
    return result;
}

void ContinuousGraspCandid::loadConfig(std::string file_name)
{
  YAML::Node yamlnode;
  yamlnode = YAML::LoadFile(file_name);
  for (int i=0 ;i< yamlnode["grasp_points"].size(); i++)
  {
    std::pair< std::pair<Eigen::Vector3d, Eigen::Vector3d>, Eigen::Quaterniond> pose;
    pose.first.first = Eigen::Vector3d::Map(yamlnode["grasp_points"][i]["upper_bound"].as<std::vector<double> >().data());
    pose.first.second = Eigen::Vector3d::Map(yamlnode["grasp_points"][i]["lower_bound"].as<std::vector<double> >().data());
    pose.second.coeffs() = Eigen::Vector4d::Map(yamlnode["grasp_points"][i]["orientation"].as<std::vector<double> >().data());
    candids.push_back(pose);
  }
}

Eigen::Isometry3d ContinuousGraspCandid::getGrasp(int index, double ratio)
{
  if (index >= candids.size())
    throw std::out_of_range("out of grasp index (index is greater than candids.size() )");

  const auto & ub = candids[index].first.first;
  const auto & lb = candids[index].first.second;

  const auto & quat = candids[index].second;

  Eigen::Isometry3d pose;
  pose.setIdentity();
  pose.translation() = (ub-lb) * ratio + lb;
  pose.linear() = quat.matrix();

  return pose;
}

Eigen::Isometry3d ContinuousGraspCandid::getGrasp(double ratio)
{
  int n = candids.size();
  int index = n * ratio;
  if (index == n) --index;  // if ratio == 1.0 -> index -> candids.size()-1
  
  double new_ratio = ratio * n - index;
  // std::cout << "index: " << index << std::endl 
  //           << "new_ratio: " << new_ratio << std::endl;
  return getGrasp(index, new_ratio);
}


grasping_point::grasping_point()
{
    ContinuousGraspCandid cgd;
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

    calib_dh_left << 0.000474560429981023,	 0.000483166682165302,	 -0.00304883355188383,	  0.00148667086907321,
                    -3.69828865924539e-05,	-0.000288069909956647,	 -0.00812428761844092,	  0.00421567136144437,
                    -0.000154357131719552,	  -0.0010921364777817,	  0.00031894496234845,	  -0.0030474925191138,
                    -0.000117600404870226,	 0.000712982958085577,	 -0.00571261767823764,	  0.00176867969486185,
                    -0.00058993701921134,	-0.000326649645213642,	  0.00939394386245098,	  0.00123723772258799,
                    -0.000433705606644922,	-0.000293762477507038,	  -0.0156742348127345,	 -0.00529320945025931,
                    -0.000589815315429364,	  6.2389274666678e-05,	   0.0291501803388187,	  0.00113202442328629;

    calib_dh_right << -0.00153055068483914,	-0.000772485919009139,	 -0.00374187596482555,	 -0.00183369895482027,
                      0.000163834922530543,	-0.000212890734425727,	  -0.0041768339596184,	  0.00292223805919776,
                      -2.60271767179549e-05,	 -0.00100930869860036,	  0.00208915153420725,	 -0.00362801030623702,
                      0.0002126348171224,	  0.00108679894872676,	 9.81965015663654e-05,	  0.00292912798333623,
                      -0.00136529072609946,	-5.62799006356164e-05,	   0.0139799357258803,	 -0.00122374898174726,
                      -0.000502587880406569,	 0.000336192838117574,	  -0.0139808833528828,	 -0.00268675457630875,
                      -0.00104647166032287,	 0.000135297170834114,	  0.00498364620994882,	 0.000349775306996936;

    calib_dh_top << -0.000171539356569936,	 0.000591551783574421,	  0.00119525002639617,	 -0.00650097874689066,
                  -0.000330274824097498,	-0.000124214443868683,	 7.10962210595555e-05,	  0.00166835357174836,
                  -0.00027921463294522,	-0.000683991542242173,	  0.00203760570771006,	  -0.0018819778569208,
                  -0.000355247972007034,	 0.000891508331427601,	 -0.00513318787489872,	 0.000168196477584221,
                  0.000190817101859644,	-0.000635118518697479,	  0.00445732420517835,	 -0.00167223312645046,
                  -0.000244350284995939,	-0.000209543585115151,	 0.000118913154576129,	 -0.00356076901549127,
                  -0.000484192538997231,	 4.41640702632187e-05,	  -0.0155483388661319,	 0.000602582057747216;


    calib_dh.push_back(calib_dh_left);
    calib_dh.push_back(calib_dh_right);
    calib_dh.push_back(calib_dh_top);

    start << -0.16661368, -0.7661184, -0.03369873, -2.37254935, -0.09888003 , 1.6927669, 0.17440837,
            0.40648265944447426, 0.2046333102301547, -0.10520152309796396, -1.2851498105581256, 0.11502121452082804, 1.4620952500450504, 0.4454484761905;
    
    t_wo_start.setIdentity();
    t_wo_start_pos =  Vector3d(0.958 - 0.007, 0.213 + 0.01, 1.557);
    t_wo_start_quat = Quaterniond(0.4418, -0.0446,  0.0220,  0.8958);
    t_wo_start.translation() = t_wo_start_pos;
    t_wo_start.linear() = t_wo_start_quat.toRotationMatrix();

    t_wo_goal.translation() = Vector3d(0.7, 0.45, 1.557 + 0.05);
    t_wo_goal.linear() = Quaterniond(-0.10912217, -0.03777281, -0.004149 ,0.99330174).toRotationMatrix();

    debug_file_prefix_ = "/home/jiyeong/catkin_ws/src/2_social/closed_chain_motion_planner/debug/";
    mesh_file_ = "package://grasping_point/STEFAN/stl/assembly_without_bottom.stl";
}