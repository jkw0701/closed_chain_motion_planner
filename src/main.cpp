

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
unsigned int links = 21;

void execute_path(std::string path_name, std::string planning_group, double total_time = 2.5)
{
    moveit::planning_interface::MoveGroupInterface move_group(planning_group);
    moveit_msgs::RobotTrajectory robot_trajectory;
    trajectory_msgs::JointTrajectory joint_trajectory;
    joint_trajectory.joint_names = move_group.getJointNames();
    ifstream path_file(path_name);
    while (path_file)
    {
        trajectory_msgs::JointTrajectoryPoint traj_point;
        bool eof = false;
        for (int j = 0; j < 21; j++)
        {
            double data;
            if (!(path_file >> data))
            {
                // cout << "wrong file: " << j << endl;
                eof = true;
                break;
            }
            traj_point.positions.push_back(data);
        }
        if (eof)
            break;
        // traj_point.time_from_start = ros::Duration();
        joint_trajectory.points.push_back(traj_point);
    }

    int n_traj = joint_trajectory.points.size();
    for (int i = 0; i < n_traj; i++)
        joint_trajectory.points[i].time_from_start = ros::Duration(total_time / n_traj * i);

    robot_trajectory.joint_trajectory = joint_trajectory;

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = robot_trajectory;
    plan.planning_time_ = total_time;
    move_group.execute(plan);
}



int main(int argc, char **argv)
{
    std::string name_ = "tripple_constraint_planning";
    ros::init(argc, argv, name_);
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle node_handle("~");
    ros::WallDuration(1.0).sleep();

    int links = 14;
    grasping_point grp;
    auto ss = std::make_shared<KinematicChainSpace>(links);
    auto constraint = std::make_shared<KinematicChainConstraint>(links, grp);
    ConstrainedProblem cp(ss, constraint);
    // cp.setConstrainedOptions();

    // cp.setPlanner(StefanBiPRM);
    // cp.setPlanner(RRTConnect);
    // while (true)
    // {
    //     cp.goalSampling();
    //     if (cp.goals->hasStates()) // && Gcp.goals->hasStates())
    //         break;
    // }
    OMPL_INFORM("*START* threading");
    // cp.solveOnce();

    cp.setStartState();
    std::vector<enum PLANNER_TYPE> planners = {RRTConnect, StefanBiPRM};
    cp.setupBenchmark(planners, "kinematicchain");
    cp.bench->addExperimentParameter("links", "INTEGER", std::to_string(cp.constraint->getAmbientDimension()));
    cp.runBenchmark();
    
    return 0;
}
