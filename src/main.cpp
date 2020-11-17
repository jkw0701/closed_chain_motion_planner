

#include <ros/ros.h>

#include <iostream>
#include <fstream>
#include <Eigen/Core>
#include <closed_chain_motion_planner/base/constraints/ConstrainedPlanningCommon.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/ExecuteTrajectoryGoal.h>
#include <moveit_msgs/ExecuteTrajectoryAction.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

using namespace std;

int main(int argc, char **argv)
{
    std::string name_ = "triple_constraint_planning";
    ros::init(argc, argv, name_);
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle node_handle("~");
    std::string obj_name; // = "dumbbell";
    node_handle.getParam("obj_name", obj_name);
    std::cout << obj_name << std::endl;
    ros::WallDuration(1.0).sleep();

    int links = 14;
    auto ss = std::make_shared<KinematicChainSpace>(links);
    auto constraint = std::make_shared<KinematicChainConstraint>(links);
    ConfigPtr config = std::make_shared<grasping_point>();
    config->loadConfig(obj_name);
    ConstrainedProblem cp(ss, constraint, config);
    
    cp.setPlanner(StefanBiPRM);
    // cp.setPlanner(RRTConnect);
    bool goalRegion = false;
    if (!goalRegion)
    {
        while (true)
        {
            cp.goalSampling();
            if (cp.goals->hasStates())
                break;
        }
    }

    OMPL_INFORM("*START* threading");
    cp.solveOnce(goalRegion);

    return 0;
}
