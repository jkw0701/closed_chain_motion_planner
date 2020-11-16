#include <closed_chain_motion_planner/base/constraints/ConstrainedPlanningCommon.h>
#include <geometry_msgs/Point.h>
using namespace std;
using namespace Eigen;
ConstrainedProblem::ConstrainedProblem(ob::StateSpacePtr space_, ChainConstraintPtr constraint_)
    : space(std::move(space_)), constraint(std::move(constraint_))
{
  css = std::make_shared<jy_ProjectedStateSpace>(space, constraint);
  csi = std::make_shared<ob::ConstrainedSpaceInformation>(css);
  css->setup();
  ss = std::make_shared<og::SimpleSetup>(csi);
  
  arm_name_map_["panda_left"] = 0;
  // arm_name_map_["panda_right"] = 1;
  arm_name_map_["panda_top"] = 2;
  _setEnvironment(arm_name_map_);

  geometry_msgs::Pose mesh_pose;
  mesh_pose.position.x = grp.t_wo_start_pos[0];
  mesh_pose.position.y = grp.t_wo_start_pos[1];
  mesh_pose.position.z = grp.t_wo_start_pos[2];
  mesh_pose.orientation.w = grp.t_wo_start_quat.w();
  mesh_pose.orientation.x = grp.t_wo_start_quat.x();
  mesh_pose.orientation.y = grp.t_wo_start_quat.y();
  mesh_pose.orientation.z = grp.t_wo_start_quat.z();

  KinematicChainValidityCheckerPtr valid_checker_ = std::make_shared<KinematicChainValidityChecker>(csi, grp);
  valid_checker_->setArmNames(arm_names_);
  valid_checker_->setStartStates(grp.start);
  valid_checker_->addMeshFromFile(grp.mesh_file_, mesh_pose, "stefan");
  valid_checker_->attachObject("stefan", "panda_left_hand", "hand_left");
  
  ss->setStateValidityChecker(valid_checker_);
  // csi->setMotionValidator(std::make_shared<jy_MotionValidator>(csi));

  
  setObjSpace(grp.t_wo_start.translation(), grp.t_wo_goal.translation());

  valid_sampler_ = std::make_shared<jy_ValidStateSampler>(csi);
  valid_sampler_->setValidityChecker(valid_checker_);
  // valid_sampler_->IKvc->addMeshFromFile(grp.mesh_file_, mesh_pose, "stefan");
  valid_sampler_->setArmModels(arm_models_, arm_names_);
  valid_sampler_->setStartState(grp.start);
}

void ConstrainedProblem::setObjSpace(Eigen::Vector3d lb, Eigen::Vector3d ub)
{
  Gts = std::make_shared<ob::CompoundStateSpace>();
  ob::StateSpacePtr obj_space_ = std::make_shared<ob::SE3StateSpace>();
  ompl::base::RealVectorBounds bounds(3);
  double delta = 0.075;
  bounds.setLow(0, std::min(lb[0], ub[0]) - delta);
  bounds.setHigh(0, std::max(lb[0], ub[0]) + delta);
  bounds.setLow(1, std::min(lb[1], ub[1]) - delta);
  bounds.setHigh(1, std::max(lb[1], ub[1]) + delta);
  bounds.setLow(2, std::min(lb[2], ub[2]) - delta);
  bounds.setHigh(2, std::max(lb[2], ub[2]) + delta );

  for (unsigned int i = 0; i < 3; ++i)
      std::cout << bounds.low[i] << " ";

  for (unsigned int i = 0; i < 3; ++i)
      std::cout << bounds.high[i] << " ";
  
  obj_space_->as<ob::SE3StateSpace>()->setBounds(bounds);

  Gts->addSubspace(css, 1.0);
  Gts->addSubspace(obj_space_, 1.0);
  tsi = std::make_shared<ob::SpaceInformation>(Gts);
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
  StateEigenUtils::IsometryToState(grp.t_wo_start, obj_start_);
  StateEigenUtils::IsometryToState(grp.t_wo_goal, obj_goal_);
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
    arm_models_[it->first]->rbdl_model->initModel(grp.calib_dh[it->second]);
    arm_models_[it->first]->t_wb = grp.t_wb[it->second];
    arm_models_[it->first]->t_7e = grp.t_7e[it->second];
  }
  
  const Eigen::Ref<const Eigen::VectorXd> & arm_seg_1 = grp.start.segment<7>(0);
  const Eigen::Ref<const Eigen::VectorXd> & arm_seg_2 = grp.start.segment<7>(7);

  Eigen::Isometry3d t_b71 = arm_models_[arm_names_[0]]->rbdl_model->getTransform(arm_seg_1);
  Eigen::Isometry3d t_b72 = arm_models_[arm_names_[1]]->rbdl_model->getTransform(arm_seg_2);
  Eigen::Isometry3d t_w71 = arm_models_[arm_names_[0]]->t_wb * t_b71;
  Eigen::Isometry3d t_w72 = arm_models_[arm_names_[1]]->t_wb * t_b72;
  
  arm_models_[arm_names_[0]]->t_o7 = grp.t_wo_start.inverse() * t_w71;
  arm_models_[arm_names_[1]]->t_o7 = grp.t_wo_start.inverse() * t_w72;

}

/* . The distance between each point in the discrete geodesic is tuned by the "delta" parameter
         Valid step size for manifold traversal with delta*/
void ConstrainedProblem::setConstrainedOptions()
{
  c_opt.delta = 0.5;
  c_opt.lambda = 2.0;
  c_opt.tolerance1 = 0.001; //0.002
  c_opt.tolerance2 = 0.005; // 0.025
  c_opt.time = 180.;
  c_opt.tries = 1000;
  c_opt.range = 0.1;

  constraint->setArmModels(arm_models_[arm_names_[0]], arm_models_[arm_names_[1]]);
  constraint->setInitialPosition(grp.start);
  constraint->setTolerance(c_opt.tolerance1, c_opt.tolerance2);
  constraint->setMaxIterations(c_opt.tries);
  css->setDelta(c_opt.delta);
  css->setLambda(c_opt.lambda);
}

void ConstrainedProblem::setStartState()
{
  ob::ScopedState<> sstart(css);
  sstart->as<ob::ConstrainedStateSpace::StateType>()->copy(grp.start);
  // ss->setStartState(sstart);

  Eigen::VectorXd goal(14);
  goal << -0.00670725, 0.293223, -0.973256, -1.30336, 0.311805, 1.52998, -1.71431, 0.176124, 0.375089, -0.137823, -0.884194, 0.038757, 1.1646, -0.943639;
  ob::ScopedState<> sgoal(css);
  sgoal->as<ob::ConstrainedStateSpace::StateType>()->copy(goal);
  ss->setStartAndGoalStates(sstart, sgoal);
}

void ConstrainedProblem::goalSampling()
{
  Eigen::Isometry3d base_obj = StateEigenUtils::StateToIsometry(obj_goal_);
  ss->setup();
  goals = std::make_shared<ompl::base::GoalStates>(ss->getSpaceInformation());
  ob::State *goal_state = csi->allocState();
  ob::State *start_state = csi->allocState();

  if (valid_sampler_->sampleCalibGoal(base_obj, goal_state))
  {
    goals->clear();
    goals->addState(goal_state);
    csi->printState(goal_state);
  }
}

void ConstrainedProblem::solveOnce()
{
  setStartState();
  ss->setGoal(goals);

  ob::PlannerStatus stat = ss->solve(c_opt.time);

  dumpGraph();
  if (stat)
  {
    auto path = ss->getSolutionPath();
    if (stat == ob::PlannerStatus::APPROXIMATE_SOLUTION)
        OMPL_WARN("Solution is approximate.");

    path.printAsMatrix(std::cout);
    // Interpolate and validate interpolated solution path.
    OMPL_INFORM("Interpolating path...");
    path.interpolate();

    OMPL_INFORM("Dumping path to `%s_path.txt`.", grp.debug_file_prefix_.c_str());
    std::ofstream pathfile(grp.debug_file_prefix_ + "_path.txt");
    path.printAsMatrix(pathfile);
    pathfile.close();

        
  }

  else
    OMPL_WARN("No solution found.");
  OMPL_INFORM("Sove once finished.");
}


void ConstrainedProblem::setupBenchmark(std::vector<enum PLANNER_TYPE> &planners, const std::string &problem)
{
    bench = new ot::Benchmark(*ss, problem);

    bench->addExperimentParameter("n", "INTEGER", std::to_string(constraint->getAmbientDimension()));
    bench->addExperimentParameter("k", "INTEGER", std::to_string(constraint->getManifoldDimension()));
    bench->addExperimentParameter("n - k", "INTEGER", std::to_string(constraint->getCoDimension()));

    request = ot::Benchmark::Request(c_opt.time, 2048, 100, 0.1, true, false, true);

    for (auto planner : planners)
        bench->addPlanner(getPlanner(planner, problem));

    bench->setPreRunEvent([&](const ob::PlannerPtr &planner) {
        planner->getSpaceInformation()->getStateSpace()->as<ompl::base::ConstrainedStateSpace>()->clear();
        planner->clear();
    });
    }

void ConstrainedProblem::runBenchmark()
{
    bench->benchmark(request);

    auto now(ompl::time::as_string(ompl::time::now()));
    const std::string filename =
        (boost::format("%1%_%2%_benchmark.log") % now % bench->getExperimentName()).str();

    bench->saveResultsToFile(filename.c_str());
}
