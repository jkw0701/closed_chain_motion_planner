#pragma once

#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <memory>
// #include <ompl/geometric/SimpleSetup.h>
// #include <ompl/geometric/PathGeometric.h>
// #include <algorithm>

#include <ompl/base/Constraint.h>
#include <ompl/base/ConstrainedSpaceInformation.h>
#include <ompl/base/spaces/constraint/ConstrainedStateSpace.h>
#include <ompl/base/spaces/constraint/ProjectedStateSpace.h>

#include <closed_chain_motion_planner/kinematics/grasping_point.h>
#include <closed_chain_motion_planner/kinematics/panda_rbdl.h>


using namespace std;
class KinematicChainConstraint : public ompl::base::Constraint
{
public:
    KinematicChainConstraint(unsigned int links, grasping_point grp) : ompl::base::Constraint(links, 2)
    {
        maxIterations = 250;
        lb_ << -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973;
        ub_ << 2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973;
    }

    void setInitialPosition(const Eigen::Ref<const Eigen::VectorXd> init_joint)
    {
        const Eigen::Ref<const Eigen::VectorXd> & arm_seg_1 = init_joint.segment<7>(0);
        const Eigen::Ref<const Eigen::VectorXd> & arm_seg_2 = init_joint.segment<7>(7);

        const Eigen::Isometry3d & t_w71 = arm_models_[0]->t_wb * arm_models_[0]->rbdl_model->getTransform(arm_seg_1);
        const Eigen::Isometry3d & t_w72 = arm_models_[1]->t_wb * arm_models_[1]->rbdl_model->getTransform(arm_seg_2);

        init_chain_ = t_w72.inverse() * t_w71;  
    }

    
    bool jointValid(const Eigen::Ref<const Eigen::VectorXd> &q) const
    {
        double eps =  0.001;
        for (int arm = 0; arm < 2; arm++)
        {
            for (int i=0; i<lb_.size(); i++)
            {
                if (q(arm * 7 + i) < lb_(i) + eps) return false;
                if (q(arm * 7 + i) > ub_(i) - eps) return false;
            }
        }
        return true;
    }

    bool project(Eigen::Ref<Eigen::VectorXd> x) const override
    {
        // Newton's method
        unsigned int iter = 0;
        double norm1 = 0;
        double norm2 = 0;
        double norm3 = 0;
        double norm4 = 0;
        Eigen::VectorXd f(getCoDimension());
        Eigen::MatrixXd j(getCoDimension(), n_);
        function(x, f);
        while ( ( (norm1 = f[0] > tolerance1_)  || (norm2 = f[1]) > tolerance2_ ) && iter++ < maxIterations)
        {
            jacobian(x, j);
            x -= 0.30 * j.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(f);
            function(x, f);
        }
        
        if (jointValid(x) && (norm1 < tolerance1_) && (norm2 < tolerance2_))
        {
            // OMPL_INFORM("ITER : %d  / norm1 : %f  / norm2 : %f  "  , iter, norm1, norm2);
            return true;
        }
        else    
            return false;
    }

    void function(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> out) const override
    {
        const Eigen::Ref<const Eigen::VectorXd> & arm_seg_1 = x.segment<7>(0);
        const Eigen::Ref<const Eigen::VectorXd> & arm_seg_2 = x.segment<7>(7);

        const Eigen::Isometry3d & t_w71 = arm_models_[0]->t_wb * arm_models_[0]->rbdl_model->getTransform(arm_seg_1);
        const Eigen::Isometry3d & t_w72 = arm_models_[1]->t_wb * arm_models_[1]->rbdl_model->getTransform(arm_seg_2);

        const Eigen::Isometry3d & current_chain = t_w72.inverse() * t_w71;  

        Eigen::Quaterniond current_quat(current_chain.linear());
        Eigen::Quaterniond init_quat(init_chain_.linear());

        double err_r = current_quat.angularDistance(init_quat);
        double err_p = (current_chain.translation() - init_chain_.translation()).norm();
        
        out[0] = err_p ;
        out[1] = err_r;
    }    
    
    void setTolerance(const double tolerance1, const double tolerance2)
    {
        if (tolerance1 <= 0 || tolerance2 <= 0)
            throw ompl::Exception("ompl::base::Constraint::setProjectionTolerance(): "
                                  "tolerance must be positive.");
        tolerance1_ = tolerance1;
        tolerance2_ = tolerance2;
        // OMPL_INFORM("Set tolerance to %f and %f", tolerance1_, tolerance2_);
    }

    bool isSatisfied(const Eigen::Ref<const Eigen::VectorXd> &x) const override
    {
        Eigen::VectorXd f(getCoDimension());
        function(x, f);
        // std::cout << f.transpose() << std::endl;
        return f.allFinite() && f[0] <= tolerance1_ && f[1] <= tolerance2_;
    }

    void setArmModels(const ArmModelPtr & arm1, const ArmModelPtr & arm2)
    {
        arm_models_[0] = arm1;
        arm_models_[1] = arm2;
    }
  

protected:
    double tolerance1_, tolerance2_;

private:
    int maxIterations;
    Eigen::Matrix<double, 7, 1> lb_, ub_;
    std::array<ArmModelPtr, 2> arm_models_;
    Eigen::Isometry3d init_chain_;
};


typedef std::shared_ptr<KinematicChainConstraint> ChainConstraintPtr;

