#pragma once

#include <iostream>
#include <fstream>
#include <thread>
#include <boost/format.hpp>

#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/base/spaces/SO3StateSpace.h>
#include <closed_chain_motion_planner/kinematics/panda_tracik.h>
#include <closed_chain_motion_planner/kinematics/panda_model.h>
#include <closed_chain_motion_planner/base/constraints/ConstraintFunction.h>
#include <closed_chain_motion_planner/base/jy_ConstrainedValidStateSampler.h>
#include <closed_chain_motion_planner/planner/stefanBiPRM.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <closed_chain_motion_planner/base/utils.h>

#include <closed_chain_motion_planner/base/jy_ProjectedStateSpace.h>
#include <closed_chain_motion_planner/kinematics/grasping_point.h>

#include <ompl/base/ConstrainedSpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/constraint/ConstrainedStateSpace.h>

#include <moveit/robot_trajectory/robot_trajectory.h>

#include <ompl/tools/benchmark/Benchmark.h>
namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace ot = ompl::tools;
using namespace Eigen;
using namespace std;
enum PLANNER_TYPE
{
    RRT,
    RRTConnect,
    StefanBiPRM
};

struct ConstrainedOptions
{
    double delta;
    double lambda;
    double tolerance1;
    double tolerance2;
    double time;
    unsigned int tries;
    double range;
};

class ConstrainedProblem
{
public:
    ConstrainedProblem(ob::StateSpacePtr space_, ChainConstraintPtr constraint_);
    grasping_point grp;
    void setConstrainedOptions();
    void setStartState();
    void goalSampling();
    void solveOnce();
    void dumpGraph()
    {
        ob::PlannerData data(csi);
        pp->getPlannerData(data);

        std::ofstream graphfile(grp.debug_file_prefix_+ "node_info.graphml");
        data.printGraphML(graphfile);
        graphfile.close();

        std::ofstream graphfile2(grp.debug_file_prefix_ + "graph_info.dot");
        data.printGraphviz(graphfile2);
        graphfile2.close();

        OMPL_INFORM("Dumping planner graph");
    }

    bool sampleIKgoal(ob::State *result, Isometry3d base_obj);
    bool sampleIKgoal(bool start, ob::State *result, Isometry3d base_obj);
    bool sampleIKstart(ob::State *result, Isometry3d base_obj);
    void setObjSpace(Eigen::Vector3d lb, Eigen::Vector3d ub);
    void setObjStartGoal();
    bool solveConnectPose(Eigen::Isometry3d obj_start, Eigen::Ref<Eigen::VectorXd> q_out);

    template <typename _T>
    std::shared_ptr<_T> createPlanner()
    {
        auto &&planner = std::make_shared<_T>(csi);
        return std::move(planner);
    }

    template <typename _T>
    std::shared_ptr<_T> createPlanner2()
    {
        auto &&planner = std::make_shared<_T>(csi, tsi, grp, obj_start_, obj_goal_);
        return std::move(planner);
    }

    template <typename _T>
    std::shared_ptr<_T> createPlanner3()
    {
        auto &&planner = std::make_shared<_T>(csi, tsi, obj_start_, obj_goal_, valid_sampler_);
        return std::move(planner);
    }

    template <typename _T>
    std::shared_ptr<_T> createPlannerRange()
    {
        auto &&planner = createPlanner<_T>();

        if (c_opt.range == 0)
        {
        }
        else
            planner->setRange(c_opt.range);

        return std::move(planner);
    }
    template <typename _T>
    std::shared_ptr<_T> createPlannerRangeProj(const std::string &projection)
    {
        const bool isProj = projection != "";
        auto &&planner = createPlannerRange<_T>();

        if (isProj)
            planner->setProjectionEvaluator(projection);

        return std::move(planner);
    }
    ob::PlannerPtr getPlanner(enum PLANNER_TYPE planner, const std::string &projection = "")
    {
        ob::PlannerPtr p;
        switch (planner)
        {
            case RRT:
                p = createPlannerRange<og::RRT>();
                break;
            case RRTConnect:
                p = createPlannerRange<og::RRTConnect>();
                break;
            case StefanBiPRM:
                p = createPlanner3<stefanBiPRM>();
                break;
        }
        return p;
    }

    void setPlanner(enum PLANNER_TYPE planner, const std::string &projection = "")
    {
        pp = getPlanner(planner, projection);
        ss->setPlanner(pp);
    }
    void _setEnvironment(const std::map<std::string, int> & arm_name_map);
    ob::StateSpacePtr space;
    ChainConstraintPtr constraint;
    
    ob::ConstrainedStateSpacePtr css;
    std::shared_ptr<ob::CompoundStateSpace> Gts;
    ob::ConstrainedSpaceInformationPtr csi;
    ob::SpaceInformationPtr tsi;
    ob::PlannerPtr pp;
    og::SimpleSetupPtr ss;

    struct ConstrainedOptions c_opt;

    std::shared_ptr<ompl::base::GoalStates> goals;
    
    ob::State *obj_start_, *obj_goal_;
    std::map<std::string, int> arm_name_map_;
    std::vector<std::string> arm_names_;
    PandaModel robot_model_;
    std::map<std::string, ArmModelPtr> arm_models_;
    jy_ValidStateSamplerPtr valid_sampler_;

    ot::Benchmark *bench;
    ot::Benchmark::Request request;
    void setupBenchmark(std::vector<enum PLANNER_TYPE> &planners, const std::string &problem);
    void runBenchmark();
protected:
    ompl::RNG rng_;
};
