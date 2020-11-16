#pragma once

#include "ompl/base/MotionValidator.h"
#include "ompl/base/PlannerData.h"
#include "ompl/base/StateSampler.h"
#include "ompl/base/ValidStateSampler.h"
#include <ompl/base/Constraint.h>

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/constraint/ConstrainedStateSpace.h>

#include <Eigen/Core>
#include <utility>
namespace ob = ompl::base;
class jy_ProjectedStateSpace;
typedef std::shared_ptr<jy_ProjectedStateSpace> jy_ProjectedStateSpacePtr;

class jy_ProjectedStateSampler : public ob::WrapperStateSampler
{
public:
    jy_ProjectedStateSampler(const jy_ProjectedStateSpace *space, ob::StateSamplerPtr sampler) ;

    void sampleUniform(ob::State *state) override;
    void sampleUniformNear(ob::State *state, const ob::State *near, double distance) override;
    void sampleGaussian(ob::State *state, const ob::State *mean, double stdDev) override;

protected:
    const ob::ConstraintPtr constraint_;
};

class jy_ProjectedStateSpace : public ob::ConstrainedStateSpace
{
public:            
    jy_ProjectedStateSpace(const ob::StateSpacePtr &ambientSpace, const ob::ConstraintPtr &constraint)
    : ob::ConstrainedStateSpace(ambientSpace, constraint)
    {
        setName("Projected" + space_->getName());
    }
    ~jy_ProjectedStateSpace() override = default;

    ob::StateSamplerPtr allocDefaultStateSampler() const override
    {
        return std::make_shared<jy_ProjectedStateSampler>(this, space_->allocDefaultStateSampler());
    }

            /**  Allocate the previously set state sampler for this space. */
    ob::StateSamplerPtr allocStateSampler() const override
    {
        return std::make_shared<jy_ProjectedStateSampler>(this, space_->allocStateSampler());
    }

    bool discreteGeodesic(const ob::State *from, const ob::State *to, bool interpolate = false,
                                  std::vector<ob::State *> *geodesic = nullptr) const override;
};


class jy_MotionValidator : public ob::ConstrainedMotionValidator
{
public:
    jy_MotionValidator(const ob::SpaceInformationPtr &si) : ob::ConstrainedMotionValidator(si)
    {}

    bool checkMotion(const ob::State *s1, const ob::State *s2) const override
    {
        // std::cout << "check motion " << std::endl;
        return ss_.getConstraint()->isSatisfied(s2) &&  ss_.discreteGeodesic(s1, s2, false);
        return ss_.discreteGeodesic(s1, s2, false);
    }
};
 
