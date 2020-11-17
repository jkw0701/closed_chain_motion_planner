#pragma once

#include <utility>
#include <mutex>

#include "ompl/base/SpaceInformation.h"
#include "ompl/base/spaces/constraint/ConstrainedStateSpace.h"

#include "ompl/util/ClassForward.h"
#include "ompl/util/Console.h"
#include "ompl/util/Exception.h"

#include <closed_chain_motion_planner/kinematics/panda_model.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <closed_chain_motion_planner/base/constraints/ConstraintFunction.h>
#include <closed_chain_motion_planner/base/constraints/ik_task.h>
#include <closed_chain_motion_planner/kinematics/KinematicChain.h>
#include <closed_chain_motion_planner/kinematics/panda_tracik.h>

#include <closed_chain_motion_planner/base/jy_IKValidityCheck.h>
#include <closed_chain_motion_planner/base/utils.h>

class jy_ValidStateSampler
{
public:
    jy_ValidStateSampler(ob::SpaceInformationPtr si): si_(si)
    {
        // IKvc_ = std::make_shared<IKValidityChecker>();
        ik_task_ = std::make_shared<IKTask>("common_ik_task");
    }

    void setArmModels(const std::map<std::string, ArmModelPtr> &arm_models, const std::vector<std::string> & arm_names)
    {
        arm_models_ = arm_models;
        arm_names_ = arm_names;
        ik_task_->setArmModels(arm_models);
        // IKvc_->setArmNames(arm_names);
    }

    void setStartState(const Eigen::Ref<const Eigen::VectorXd> & q0)
    {
        q0_ = q0;
    }
    
    void setValidityChecker(const KinematicChainValidityCheckerPtr &IKvc)
    {
       IKvc_ = IKvc;
    }


    Eigen::Isometry3d compute_t_wo(const Eigen::Ref<const Eigen::VectorXd> & q0)
    {
        return ik_task_->compute_t_wo(q0);
    }

    Eigen::Isometry3d compute_t_wo(ob::State *state)
    {
        Eigen::Map<Eigen::VectorXd> q0 = *state->as<ob::ConstrainedStateSpace::StateType>();
        // std::cout << q0.transpose() << std::endl;
        return ik_task_->compute_t_wo(q0.segment<7>(0));
    }

    bool sampleCalibGoal(Isometry3d base_obj, ob::State *result)
    {
        std::lock_guard<std::mutex> _(graphMutex_);        
        // IKvc_->moveOBJ("stefan", base_obj);
        Eigen::Map<Eigen::VectorXd> &sol = *result->as<ob::ConstrainedStateSpace::StateType>();       
                
        ik_task_->setTargetPose(base_obj);
        ik_task_->setSigma(sigma_);

        for (int i = 0; i < 2; i++)
        {
            bool success = false;    
            int arm_idx = arm_models_[arm_names_[i]]->index;            
            ik_task_->setArmName(arm_names_[i]);
            // std::cout << "set arm name to "<< arm_names_[i] << std::endl;
            // std::cout << q0_.segment<7>(i * 7).transpose() << std::endl;

            Eigen::VectorXd temp_sol(7);
            if (_solveCalibIK(i, q0_.segment<7>(i * 7), temp_sol))
            {
                sol.segment<7>(i * 7) = temp_sol;
                success = true;
            }
            else
            {
                int iter = 15;
                double prev = std::numeric_limits<double>::infinity();
                while (--iter)
                {          
                    if (_randomsolveCalibIK(i, temp_sol))
                    {
                        double curr = (q0_.segment<7>(i * 7) - temp_sol).norm();
                        if ( curr < prev)
                        {
                            prev = curr;
                            sol.segment<7>(i * 7) = temp_sol;
                        }
                        success = true;                    
                    }
                }
            }            
            if (!success)
                return false;
            
        }
        // return si_->isValid(result);
        return true;
    }

    bool sampleRandomGoal(Isometry3d base_obj, ob::State *result)
    {
        std::lock_guard<std::mutex> _(graphMutex_);        
        // IKvc_->moveOBJ("stefan", base_obj);
        Eigen::Map<Eigen::VectorXd> &sol = *result->as<ob::ConstrainedStateSpace::StateType>();       
                
        ik_task_->setTargetPose(base_obj);
        ik_task_->setSigma(sigma_);

        for (int i = 0; i < 2; i++)
        {
            bool success = false;    
            int arm_idx = arm_models_[arm_names_[i]]->index;            
            ik_task_->setArmName(arm_names_[i]);
            // std::cout << "set arm name to "<< arm_names_[i] << std::endl;
            // std::cout << q0_.segment<7>(i * 7).transpose() << std::endl;

            Eigen::VectorXd temp_sol(7);
            int iter = 15;
            double prev = std::numeric_limits<double>::infinity();
            while (--iter)
            {          
                if (_randomsolveCalibIK(i, temp_sol))
                {
                    sol.segment<7>(i * 7) = temp_sol;
                    success = true;                    
                }
            }
            if (!success)
                return false;
        }
        // return si_->isValid(result);
        return true;
    }
    
    bool sampleCalibGoal(Isometry3d base_obj, ob::State *start_state, ob::State *result)
    {
        std::lock_guard<std::mutex> _(graphMutex_);        
        // IKvc_->moveOBJ("stefan", base_obj);
        Eigen::Map<Eigen::VectorXd> &sol = *result->as<ob::ConstrainedStateSpace::StateType>();     
        Eigen::Map<Eigen::VectorXd> sstart = *start_state->as<ob::ConstrainedStateSpace::StateType>();         
                
        ik_task_->setTargetPose(base_obj);
        ik_task_->setSigma(sigma_);
        for (int i = 0; i < 2; i++)
        {
            bool success = false;    
            int arm_idx = arm_models_[arm_names_[i]]->index;            
            ik_task_->setArmName(arm_names_[i]);            
            Eigen::VectorXd temp_sol(7);
            if (_solveCalibIK(i, sstart.segment<7>(i * 7), temp_sol))
            {
                sol.segment<7>(i * 7) = temp_sol;
                success = true;
            }
            else
            {
                int iter = 15;
                double prev = std::numeric_limits<double>::infinity();
                while (--iter)
                {          
                    if (_randomsolveCalibIK(i, temp_sol))
                    {
                        double curr = (sstart.segment<7>(i * 7) - temp_sol).norm();
                        if ( curr < prev)
                        {
                            prev = curr;
                            sol.segment<7>(i * 7) = temp_sol;
                        }
                        success = true;                    
                    }
                }
            }            
            if (!success)
                return false;
        }
        return si_->isValid(result);
    }


    bool _solveCalibIK(const int arm_idx, VectorXd start, Eigen::Ref<Eigen::VectorXd> solution)
    {
        return ik_task_->solve(start, solution) && IKvc_->IKValid(arm_idx, solution);
    }
    bool _randomsolveCalibIK(const int arm_idx, Eigen::Ref<Eigen::VectorXd> solution)
    {
        return ik_task_->random_solve(solution) && IKvc_->IKValid(arm_idx, solution);
    }
    
    bool solveCalibIK(const int arm_idx, VectorXd start, Isometry3d target, Eigen::Ref<Eigen::VectorXd> solution)
    {
        ik_task_->setArmName(arm_names_[arm_idx]);
        ik_task_->setTargetPose(target);
        ik_task_->setSigma(sigma_);
        return ik_task_->solve(start, solution) && IKvc_->IKValid(arm_idx, solution);
    }
    bool randomsolveCalibIK(const int arm_idx, Isometry3d target, Eigen::Ref<Eigen::VectorXd> solution)
    {
        ik_task_->setArmName(arm_names_[arm_idx]);
        ik_task_->setTargetPose(target);
        ik_task_->setSigma(sigma_);
        
        return ik_task_->random_solve(solution) && IKvc_->IKValid(arm_idx, solution);
    }

    bool sampleCalibGoal(ob::State *obj_state, ob::State *start, ob::State *result)
    {
        Isometry3d base_obj = StateEigenUtils::StateToIsometry(obj_state);
        return sampleCalibGoal(base_obj, start, result);
    }
    

    // std::shared_ptr<IKValidityChecker> IKvc_;
    
private:
    ob::SpaceInformationPtr si_;
    mutable std::mutex graphMutex_;
    double epsilon{0.4};
    IKTaskPtr ik_task_;
    double sigma_ {1.0};
    std::vector<std::string> arm_names_;
    std::map<std::string, ArmModelPtr> arm_models_;
    std::array<Eigen::Isometry3d, 2> t_o7_;
    Eigen::VectorXd q0_;
    KinematicChainValidityCheckerPtr IKvc_;
    
};

typedef std::shared_ptr<jy_ValidStateSampler> jy_ValidStateSamplerPtr;
