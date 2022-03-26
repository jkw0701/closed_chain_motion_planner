#include <closed_chain_motion_planner/base/constraints/ConstrainedPlanningCommon.h>
#include <geometry_msgs/Point.h>

#include <ompl/tools/benchmark/Benchmark.h>
using namespace std;
using namespace Eigen;
ConstrainedProblem::ConstrainedProblem(ob::StateSpacePtr space_, ChainConstraintPtr constraint_, ConfigPtr config_)
    : space(std::move(space_)), constraint(std::move(constraint_)), config(std::move(config_))
{
  // C = std::move(A) 를 하면 move 하기 때문에 A.data 는 C.data 로 이동이 되어서 A.data 는 빈 문자열이 되고 
  // C.data 는 “aaa” 를 가진다. 얻을 수 있는 이점은, 새로 메모리를 할당(malloc)하지 않아도 되고 이미 메모리에 할당된 것을 소유권만 C 에게 넘겨주기 때문에 copy 동작보다 빠르다. 
  css = std::make_shared<jy_ProjectedStateSpace>(space, constraint); // joint space + constraint ?
  csi = std::make_shared<ob::ConstrainedSpaceInformation>(css);
  css->setup();
  // no contents??
  ss = std::make_shared<og::SimpleSetup>(csi);

  arm_name_map_[config->arm_name1] = config->arm_index1;
  arm_name_map_[config->arm_name2] = config->arm_index2;

  _setEnvironment(arm_name_map_);
  geometry_msgs::Pose mesh_pose;
  mesh_pose.position.x = config->t_wo_start.translation()[0];
  mesh_pose.position.y = config->t_wo_start.translation()[1];
  mesh_pose.position.z = config->t_wo_start.translation()[2]; // translations or rotations 
  Eigen::Quaterniond mesh_quat(config->t_wo_start.linear());
  mesh_pose.orientation.w = mesh_quat.w();
  mesh_pose.orientation.x = mesh_quat.x();
  mesh_pose.orientation.y = mesh_quat.y();
  mesh_pose.orientation.z = mesh_quat.z();

  KinematicChainValidityCheckerPtr valid_checker_ = std::make_shared<KinematicChainValidityChecker>(csi);
  valid_checker_->setArmNames(arm_names_);
  valid_checker_->setStartStates(config->start);
  valid_checker_->addMeshFromFile(config->mesh_file_, mesh_pose, config->obj_name);
  valid_checker_->attachObject(config->obj_name, "panda_left_hand", "hand_left");

  ss->setStateValidityChecker(valid_checker_);

  setObjSpace(config->t_wo_start.translation(), config->t_wo_goal.translation());

  valid_sampler_ = std::make_shared<jy_ValidStateSampler>(csi);
  valid_sampler_->setValidityChecker(valid_checker_);
  valid_sampler_->setArmModels(arm_models_, arm_names_);
  valid_sampler_->setStartState(config->start);
  
}


void ConstrainedProblem::setObjSpace(Eigen::Vector3d lb, Eigen::Vector3d ub) // bounding object's sampling space
{
  Gts = std::make_shared<ob::CompoundStateSpace>();
  ob::StateSpacePtr obj_space_ = std::make_shared<ob::SE3StateSpace>();
  ompl::base::RealVectorBounds bounds(3);
  double delta = 0.2;
  bounds.setLow(0, std::min(lb[0], ub[0]) - delta);
  bounds.setHigh(0, std::max(lb[0], ub[0]) + delta);
  bounds.setLow(1, std::min(lb[1], ub[1]) - delta);
  bounds.setHigh(1, std::max(lb[1], ub[1]) + delta);
  bounds.setLow(2, std::min(lb[2], ub[2]) - delta);
  bounds.setHigh(2, std::max(lb[2], ub[2]) + delta + 0.1);

  //for (unsigned int i = 0; i < 3; ++i)
  //  std::cout << bounds.low[i] << " ";

  //for (unsigned int i = 0; i < 3; ++i)
  //  std::cout << bounds.high[i] << " ";

  obj_space_->as<ob::SE3StateSpace>()->setBounds(bounds);

  Gts->addSubspace(css, 1.0); // ob::CompoundStateSpace
  Gts->addSubspace(obj_space_, 1.0); // ob::CompoundStateSpace + joint space
  tsi = std::make_shared<ob::SpaceInformation>(Gts); // 
  Gts->setup();

  obj_start_ = obj_space_->allocState();
  obj_goal_ = obj_space_->allocState();
  setObjStartGoal();
  setConstrainedOptions();
}

void ConstrainedProblem::setObjStartGoal()
{
  if (!Gts->getSubspace(1))
    OMPL_ERROR("Add OBJ Space First!");
  StateEigenUtils::IsometryToState(config->t_wo_start, obj_start_);
  StateEigenUtils::IsometryToState(config->t_wo_goal, obj_goal_);
}

void ConstrainedProblem::_setEnvironment(const std::map<std::string, int> &arm_name_map)
{
  arm_name_map_ = arm_name_map;

  for (auto it = arm_name_map.begin(); it != arm_name_map.end(); ++it)
  {
    arm_names_.push_back(it->first);
    arm_models_[it->first] = std::make_shared<ArmModel>();
    arm_models_[it->first]->name = it->first;
    arm_models_[it->first]->index = it->second;
    arm_models_[it->first]->trac_ik_model = std::make_shared<panda_ik>(it->first); //ik_solver_;
    arm_models_[it->first]->rbdl_model = std::make_shared<PandaModel>();
    // arm_models_[it->first]->rbdl_model->initModel(config->calib_dh[it->second]);
    arm_models_[it->first]->t_wb = config->t_wb[it->second];
    arm_models_[it->first]->t_7e = config->t_7e[it->second];
  }

  const Eigen::Ref<const Eigen::VectorXd> &arm_seg_1 = config->start.segment<7>(0);
  const Eigen::Ref<const Eigen::VectorXd> &arm_seg_2 = config->start.segment<7>(7);

  Eigen::Isometry3d t_b71 = arm_models_[arm_names_[0]]->rbdl_model->getTransform(arm_seg_1);
  Eigen::Isometry3d t_b72 = arm_models_[arm_names_[1]]->rbdl_model->getTransform(arm_seg_2);
  Eigen::Isometry3d t_w71 = arm_models_[arm_names_[0]]->t_wb * t_b71;
  Eigen::Isometry3d t_w72 = arm_models_[arm_names_[1]]->t_wb * t_b72;

  arm_models_[arm_names_[0]]->t_o7 = config->t_wo_start.inverse() * t_w71;
  arm_models_[arm_names_[1]]->t_o7 = config->t_wo_start.inverse() * t_w72;
}

/* . The distance between each point in the discrete geodesic is tuned by the "delta" parameter
         Valid step size for manifold traversal with delta*/
void ConstrainedProblem::setConstrainedOptions()
{
  c_opt.delta = 0.1; // 0.25
  c_opt.lambda = 2.0;
  c_opt.tolerance1 = 0.002; //0.002
  c_opt.tolerance2 = 0.008; // 0.025
  c_opt.time = 200.0;
  c_opt.tries = 1000;
  c_opt.range = 1.0;

  constraint->setArmModels(arm_models_[arm_names_[0]], arm_models_[arm_names_[1]]);
  constraint->setInitialPosition(config->start);
  // constraint->setTolerance(c_opt.tolerance1, c_opt.tolerance2);
  constraint->setTolerance(1e-2);
  constraint->setMaxIterations(c_opt.tries);
  css->setDelta(c_opt.delta);
  css->setLambda(c_opt.lambda);
}

void ConstrainedProblem::setStartState()
{
  ob::ScopedState<> sstart(css);
  sstart->as<ob::ConstrainedStateSpace::StateType>()->copy(config->start);
  ss->setStartState(sstart);

  // Eigen::VectorXd goal(14);
  // goal << -0.00670725, 0.293223, -0.973256, -1.30336, 0.311805, 1.52998, -1.71431, 0.176124, 0.375089, -0.137823, -0.884194, 0.038757, 1.1646, -0.943639;
  // ob::ScopedState<> sgoal(css);
  // sgoal->as<ob::ConstrainedStateSpace::StateType>()->copy(goal);
  // ss->setStartAndGoalStates(sstart, sgoal);
}

void ConstrainedProblem::goalSampling()
{
  Eigen::Isometry3d base_obj = config->t_wo_goal; ///StateEigenUtils::StateToIsometry(obj_goal_);
  ss->setup();
  goals = std::make_shared<ompl::base::GoalStates>(ss->getSpaceInformation());
  ob::State *goal_state = csi->allocState();
  ob::State *start_state = csi->allocState();

  if (valid_sampler_->sampleCalibGoal(base_obj, goal_state))
  {
    goals->clear(); // why clear??
    goals->addState(goal_state);
    csi->printState(goal_state);
  }

}

void ConstrainedProblem::benchmarkGoalSampling()
{
  Eigen::Isometry3d base_obj = config->t_wo_goal; ///StateEigenUtils::StateToIsometry(obj_goal_);
  // ss->setup();
  ob::State *goal_state = csi->allocState();
  bool suc = false;
  do
  {
    suc = valid_sampler_->sampleCalibGoal(base_obj, goal_state);
  }while (!suc);

  csi->printState(goal_state);
  
  Eigen::Map<Eigen::VectorXd> &sol = *goal_state->as<ob::ConstrainedStateSpace::StateType>();       
  ob::ScopedState<> sgoal(css);
  sgoal->as<ob::ConstrainedStateSpace::StateType>()->copy(sol);  
  ss->setGoalState(sgoal);
  
}
bool ConstrainedProblem::goalSampling(const ob::jy_GoalLazySamples *gls, ob::State *result)
{
  bool cont = false;
  for (int i = 0 ; i < 100 ; ++i) // try 100 times
  {
    if (valid_sampler_->sampleRandomGoal(config->t_wo_goal, result) && gls->getSpaceInformation()->isValid(result))
    {
      cont = true;
      break;
    }
  }
  if (cont)
  {
      std::cout << "Found goal state: " << std::endl;
      csi->printState(result);
  }
  
  return cont && gls->maxSampleCount() < 20;
}

void ConstrainedProblem::solveOnce(bool goalsampling)
{
  setStartState();
  ob::jy_GoalSamplingFn samplingFunction = [&](const ob::jy_GoalLazySamples *gls, ob::State *result) 
  {
    return goalSampling(gls, result);
  };
  std::shared_ptr<ompl::base::jy_GoalLazySamples> goalregion;
  if (goalsampling) // false
  {
    goalregion = std::make_shared<ompl::base::jy_GoalLazySamples>(ss->getSpaceInformation(), samplingFunction, false);
    ob::State *first_goal = csi->allocState();
    if (valid_sampler_->sampleCalibGoal(config->t_wo_goal, first_goal));
    {
        std::cout << "Found First goal state: " << std::endl;
        csi->printState(first_goal);
        goalregion->addState(first_goal);
    }
    goalregion->startSampling();
    ss->setGoal(goalregion); // ss : SimpleSetup
  }
  else
  {
    ss->setGoal(goals);
  }

  ss->setup();
  ob::PlannerStatus stat = ss->solve(c_opt.time);
  if (stat)
  {
    auto path = ss->getSolutionPath();
    if (stat == ob::PlannerStatus::APPROXIMATE_SOLUTION)
      OMPL_WARN("Solution is approximate.");


    std::vector<base::State *> states_raw = path.getStates();
    std::cout << "total states is " << states_raw.size() << std::endl;

    //path.printAsMatrix(std::cout);
    // Interpolate and validate interpolated solution path.
    OMPL_INFORM("Interpolating path...");
    path.interpolate();

    std::vector<base::State *> states = path.getStates();
    int states_num = states.size();
    for (int i=0;i<states_num;i++)
    {
      if(!valid_sampler_->checkValidity(states[i]))
        std::cout << "collision!!" << std::endl;
    }

    //std::cout << "sampled num \t" << css->sampled_num << std::endl;
    //std::cout << "feasible num \t" << css->feasible_num << std::endl;


    OMPL_INFORM("Dumping path to `%s_path.txt`.", config->obj_name.c_str());
    std::ofstream pathfile(config->debug_file_prefix_ + config->obj_name + "_path.txt");
    path.printAsMatrix(pathfile);
    pathfile.close();



    dumpGraph();
  }

  else
    OMPL_WARN("No solution found.");
  OMPL_INFORM("Sove once finished.");
  
  // if (goalsampling) goalregion->stopSampling();
}
