/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Ioan Sucan */
#include <closed_chain_motion_planner/base/constraints/jy_ProjectedStateSpace.h>
#include <closed_chain_motion_planner/planner/RRTSE3.h>

#include <limits>
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"

RRTSE3::RRTSE3(const base::SpaceInformationPtr &si, const ompl::base::SpaceInformationPtr &tsi,
                ob::State *obj_start, ob::State *obj_goal, jy_ValidStateSamplerPtr sampler)
  : base::Planner(si, "RRTSE3"), tsi_(std::move(tsi)), 
      obj_start_(std::move(obj_start)), obj_goal_(std::move(obj_goal)), sampler_(std::move(sampler))
{
    specs_.approximateSolutions = true;
    specs_.directed = true;

    Planner::declareParam<double>("range", this, &RRTSE3::setRange, &RRTSE3::getRange, "0.:1.:10000.");
    Planner::declareParam<double>("goal_bias", this, &RRTSE3::setGoalBias, &RRTSE3::getGoalBias, "0.:.05:1.");
    Planner::declareParam<bool>("intermediate_states", this, &RRTSE3::setIntermediateStates, &RRTSE3::getIntermediateStates,
                                "0,1");

    //addIntermediateStates_ = addIntermediateStates;


    obj_space_ = tsi_->getStateSpace()->as<ob::CompoundStateSpace>()->getSubspace(1);
    obj_sampler_ = obj_space_->allocStateSampler();

    stefan_checker_ = std::make_shared<stefanFCL>();

    nearest_state_ = obj_start_;



//     base::State *xstate = tsi_->allocState();
//    // ob::State *obj_state_ = obj_space_->allocState();
//     ob::State *mid_state_ = obj_space_->allocState();
    
//             ob::State *obj_state_ = obj_space_->allocState();

//             obj_space_->interpolate(nearest_state_, 
//                                     obj_goal_,
//                                     0.3, mid_state_); // -.2
//                //                         std::cout << "1" << std::endl;

//             obj_sampler_->sampleGaussian(obj_state_, mid_state_, 0.2); // 0.25
//              Isometry3d base_obj = StateEigenUtils::StateToIsometry(obj_state_);
//              std::cout << base_obj.translation().transpose() << std::endl;

//             // mean : mid_state_ / std : 0.2

//                   // compute ik                  
//                   if (!sampler_->sampleRandomGoal(StateEigenUtils::StateToIsometry(obj_state_), xstate->as<ob::CompoundStateSpace::StateType>()->components[0]));

//             std::cout << "-----------------" << std::endl;

}

RRTSE3::~RRTSE3()
{
    freeMemory();
}

void RRTSE3::clear()
{
    Planner::clear();
    sampler_.reset();
    freeMemory();
    if (nn_)
        nn_->clear();
    lastGoalMotion_ = nullptr;
    std::cout << "===clear===" << std::endl;
}

void RRTSE3::setup()
{
    Planner::setup();
    tools::SelfConfig sc(si_, getName());
    sc.configurePlannerRange(maxDistance_);

    if (!nn_)
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    nn_->setDistanceFunction([this](const Motion *a, const Motion *b) { return distanceFunction(a, b); });
}

void RRTSE3::freeMemory()
{
    if (nn_)
    {
        std::vector<Motion *> motions;
        nn_->list(motions);
        for (auto &motion : motions)
        {
            if (motion->state != nullptr)
                si_->freeState(motion->state);
            delete motion;
        }
    }
}

// ompl::base::PlannerStatus RRTSE3::solve(const base::PlannerTerminationCondition &ptc)
// {
//     //checkValidity();
//     base::Goal *goal = pdef_->getGoal().get();
//     auto *goal_s = dynamic_cast<base::GoalSampleableRegion *>(goal);

//     while (const base::State *st = pis_.nextStart())
//     {
//       //   auto *motion = new Motion(si_);
//       //   si_->copyState(motion->state, st);
//       //   nn_->add(motion);
//       auto *motion = new Motion(tsi_);
//       ompl::base::State *tst = tsi_->allocState();
//       tst->as<ob::CompoundStateSpace::StateType>()->components[0] = si_->cloneState(st);
//       tst->as<ob::CompoundStateSpace::StateType>()->components[1] = obj_start_;
//       //si_->copyState(motion->state, tst);      
//       motion->state = tsi_->cloneState(tst);
//       nn_->add(motion);
//     }


//     if (nn_->size() == 0)
//     {
//         OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
//         return base::PlannerStatus::INVALID_START;
//     }

// //     if (!sampler_)
// //         sampler_ = si_->allocStateSampler();

//     OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), nn_->size());

//     Motion *solution = nullptr;
//     Motion *approxsol = nullptr;
//     double approxdif = std::numeric_limits<double>::infinity();
//     auto *rmotion = new Motion(tsi_);
//     base::State *rstate = rmotion->state;
//     base::State *xstate = tsi_->allocState();
//    // ob::State *obj_state_ = obj_space_->allocState();
//     ob::State *mid_state_ = obj_space_->allocState();
    

//     while (!ptc)
//     {
//         /* sample random state (with goal biasing) */
//         // if ((goal_s != nullptr) && rng_.uniform01() < goalBias_ && goal_s->canSample())
//         //     goal_s->sampleGoal(rstate);
//         // else
//         // {
//             ob::State *obj_state_ = obj_space_->allocState();

//             obj_space_->interpolate(nearest_state_, 
//                                     obj_goal_,
//                                     0.3, mid_state_); // -.2
//                //                         std::cout << "1" << std::endl;

//             obj_sampler_->sampleGaussian(obj_state_, mid_state_, 0.2); // 0.25
//              Isometry3d base_obj = StateEigenUtils::StateToIsometry(obj_state_);
//             // std::cout << base_obj.translation().transpose() << std::endl;

//             // mean : mid_state_ / std : 0.2
//             if (!stefan_checker_->isValid(obj_state_))
//             {    
//                   continue;
//             }
//             else
//             {
//                   // compute ik                  
//                   if (!sampler_->sampleRandomGoal(StateEigenUtils::StateToIsometry(obj_state_), rstate->as<ob::CompoundStateSpace::StateType>()->components[0]))
//                         continue;
//             }
//         //}
//             rstate->as<ob::CompoundStateSpace::StateType>()->components[1] = obj_state_;

//         /* find closest state in the tree */
//         // double min_dist = 100.0;
//         // int min_index = -1;
//         // for (int i=0;i<nn_.size();i++){

//         //     if (min_dist < )
             
//         // }

//         Motion *nmotion = nn_->nearest(rmotion);
//         base::State *dstate = rstate;

//         /* find state to add */
//          Isometry3d base_obj2 = StateEigenUtils::StateToIsometry(nmotion->state->as<ob::CompoundStateSpace::StateType>()->components[1]);
//         std::cout << base_obj2.translation().transpose() << std::endl;

//         double d = obj_space_->distance(nmotion->state->as<ob::CompoundStateSpace::StateType>()->components[1], rstate->as<ob::CompoundStateSpace::StateType>()->components[1]);
//         if (d > maxDistance_)
//         {
//             obj_space_->interpolate(nmotion->state->as<ob::CompoundStateSpace::StateType>()->components[1], rstate->as<ob::CompoundStateSpace::StateType>()->components[1], maxDistance_ / d, xstate->as<ob::CompoundStateSpace::StateType>()->components[1]);
            
//             dstate = xstate;
//         }

//         //if (si_->checkMotion(nmotion->state->as<ob::CompoundStateSpace::StateType>()->components[0], dstate->as<ob::CompoundStateSpace::StateType>()->components[0]))
//             std::vector<ob::State *> states;
//         if (si_->getStateSpace()->as<jy_ProjectedStateSpace>()->discreteGeodesic(
//                                     nmotion->state->as<ob::CompoundStateSpace::StateType>()->components[0],
//                                     dstate->as<ob::CompoundStateSpace::StateType>()->components[0],
//                                     false, &states))        
//         {
//             //last_state->as<ob::CompoundStateSpace::StateType>()->components[0]           
            
//              std::cout << "1231" << std::endl;
//             // if (addIntermediateStates_)
//             // {
//             // //     std::vector<base::State *> states;
//             // //     const unsigned int count = si_->getStateSpace()->validSegmentCount(nmotion->state, dstate);

//             // //     if (si_->getMotionStates(nmotion->state, dstate, states, count, true, true))
//             // //         si_->freeState(states[0]);

//             // //     for (std::size_t i = 1; i < states.size(); ++i)
//             // //     {
//             // //         auto *motion = new Motion;
//             // //         motion->state = states[i];
//             // //         motion->parent = nmotion;
//             // //         nn_->add(motion);

//             // //         nmotion = motion;
//             // //     }
//             // }
//             // else
//             // {
//                 auto *motion = new Motion(tsi_);
//                 motion->state = tsi_->cloneState(dstate);
//                 motion->parent = nmotion;
//                 nn_->add(motion);

//                 nmotion = motion;
//             // //}

//             double dist = 0.0;
//             bool sat = goal->isSatisfied(nmotion->state->as<ob::CompoundStateSpace::StateType>()->components[0], &dist);

            

//             if (sat)
//             {
//                 approxdif = dist;
//                 solution = nmotion;
//                 break;
//             }
//             if (dist < approxdif)
//             {
//                 approxdif = dist;
//                 approxsol = nmotion;
//             }
//         }
//     }

//     bool solved = false;
//     bool approximate = false;
//     if (solution == nullptr)
//     {
//         solution = approxsol;
//         approximate = true;
//     }

//     if (solution != nullptr)
//     {
//         lastGoalMotion_ = solution;

//         /* construct the solution path */
//         std::vector<Motion *> mpath;
//         while (solution != nullptr)
//         {
//             mpath.push_back(solution);
//             solution = solution->parent;
//         }

//         /* set the solution path */
//         auto path(std::make_shared<og::PathGeometric>(si_));
//         for (int i = mpath.size() - 1; i >= 0; --i)
//             path->append(mpath[i]->state);
//         pdef_->addSolutionPath(path, approximate, approxdif, getName());
//         solved = true;
//     }

//     si_->freeState(xstate);
//     if (rmotion->state != nullptr)
//         si_->freeState(rmotion->state);
//     delete rmotion;

//     OMPL_INFORM("%s: Created %u states", getName().c_str(), nn_->size());

//     return {solved, approximate};
// }


ompl::base::PlannerStatus RRTSE3::solve(const base::PlannerTerminationCondition &ptc)
{
    //checkValidity();
    goal_state_ = si_->allocState();
    Eigen::Map<Eigen::VectorXd> &sol = *goal_state_->as<ob::ConstrainedStateSpace::StateType>();
    sol << -0.15531, -0.0289497, -0.657976, -2.256, -0.0493029, 2.26913, -0.0675558, 0.174092, 0.0786131 ,0.575653 ,-2.12558, -0.0598863, 2.20374 ,1.60813;

    // goal_state_ = si_->cloneState(goals_->getStateCount());
    si_->printState(goal_state_);

    base::Goal *goal = pdef_->getGoal().get();
    auto *goal_s = dynamic_cast<base::GoalSampleableRegion *>(goal);
    //si_->printState(goal);

    while (const base::State *st = pis_.nextStart())
    {
        auto *motion = new Motion(si_);
        si_->copyState(motion->state, st);
        nn_->add(motion);
    }

    if (nn_->size() == 0)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    // if (!sampler_)
    //     sampler_ = si_->allocStateSampler();

    OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), nn_->size());

    Motion *solution = nullptr;
    Motion *approxsol = nullptr;
    double approxdif = std::numeric_limits<double>::infinity();
    auto *rmotion = new Motion(si_);
    base::State *rstate = rmotion->state;
    base::State *xstate = si_->allocState();
    base::State *goalstate = si_->allocState();

    ob::State *mid_state_ = obj_space_->allocState();

    while (!ptc)
    {
        /* sample random state (with goal biasing) */
        // if ((goal_s != nullptr) && rng_.uniform01() < goalBias_ && goal_s->canSample())
        //     goal_s->sampleGoal(rstate);
        // else
        // {
        //    sampler_->sampleUniform(rstate);
            ob::State *obj_state_ = obj_space_->allocState();

            obj_space_->interpolate(nearest_state_, 
                                    obj_goal_,
                                    0.3, mid_state_); // -.2
               //                         std::cout << "1" << std::endl;

            obj_sampler_->sampleGaussian(obj_state_, mid_state_, 0.2); // 0.25
             Isometry3d base_obj = StateEigenUtils::StateToIsometry(obj_state_);
            
            if (!stefan_checker_->isValid(obj_state_))
            {    
                  continue;
            }
            else
            {
                  // goal->as
                  // compute ik                  
                  //if (!sampler_->sampleRandomGoal(StateEigenUtils::StateToIsometry(obj_state_), rstate))
                  if (!sampler_->sampleCalibGoal(StateEigenUtils::StateToIsometry(obj_state_), goal_state_ , rstate))
                        continue;
            }

        //}
         //   std::cout << "base_obj \t" << base_obj.translation().transpose() << std::endl;


        /* find closest state in the tree */
        Motion *nmotion = nn_->nearest(rmotion);
        base::State *dstate = rstate;

        /* find state to add */
        double d = si_->distance(nmotion->state, rstate);

        if (d > maxDistance_)
        {
            si_->getStateSpace()->interpolate(nmotion->state, rstate, maxDistance_ / d, xstate);
            dstate = xstate;
        }

    
        //if (si_->checkMotion(nmotion->state, dstate))
            std::vector<ob::State *> states;
        if (si_->getStateSpace()->as<jy_ProjectedStateSpace>()->discreteGeodesic(
                                    nmotion->state,
                                    dstate,
                                    false, &states)) 
        {

             //std::cout << "1231" << std::endl;
            //  std::vector<base::State *> states;
            //  const unsigned int count = si_->getStateSpace()->validSegmentCount(nmotion->state, dstate);

            //  if (si_->getMotionStates(nmotion->state, dstate, states, count, true, true))
            //      si_->freeState(states[0]);

             for (std::size_t i = 1; i < states.size(); ++i)
             {
                 auto *motion = new Motion;
                 motion->state = states[i];
                 motion->parent = nmotion;
                 nn_->add(motion);

                nmotion = motion;
             }

            double dist = 0.0;
            // std::cout << "----------" << std::endl;
            // si_->printState(nmotion->state);
            // si_->printState(goal_state_);

            bool sat = goal->isSatisfied(nmotion->state, &dist);

            if (dist < 0.4)
            {
                approxdif = dist;
                solution = nmotion;
                break;
            }
            if (dist < approxdif)
            {
                approxdif = dist;
                approxsol = nmotion;
            }
        }
    }

    bool solved = false;
    bool approximate = false;
    if (solution == nullptr)
    {
        solution = approxsol;
        approximate = true;
    }

    if (solution != nullptr)
    {
        lastGoalMotion_ = solution;

        /* construct the solution path */
        std::vector<Motion *> mpath;
        while (solution != nullptr)
        {
            mpath.push_back(solution);
            solution = solution->parent;
        }

        /* set the solution path */
        auto path(std::make_shared<og::PathGeometric>(si_));
        for (int i = mpath.size() - 1; i >= 0; --i)
            path->append(mpath[i]->state);
        pdef_->addSolutionPath(path, approximate, approxdif, getName());
        solved = true;
    }

    si_->freeState(xstate);
    if (rmotion->state != nullptr)
        si_->freeState(rmotion->state);
    delete rmotion;

    OMPL_INFORM("%s: Created %u states", getName().c_str(), nn_->size());

    return {solved, approximate};
}


void RRTSE3::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    std::vector<Motion *> motions;
    if (nn_)
        nn_->list(motions);

    if (lastGoalMotion_ != nullptr)
        data.addGoalVertex(base::PlannerDataVertex(lastGoalMotion_->state));

    for (auto &motion : motions)
    {
        if (motion->parent == nullptr)
            data.addStartVertex(base::PlannerDataVertex(motion->state));
        else
            data.addEdge(base::PlannerDataVertex(motion->parent->state), base::PlannerDataVertex(motion->state));
    }
}
