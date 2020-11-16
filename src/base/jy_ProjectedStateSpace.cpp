#include <closed_chain_motion_planner/base/jy_ProjectedStateSpace.h>
#include <Eigen/Core>
#include <utility>

jy_ProjectedStateSampler::jy_ProjectedStateSampler(const jy_ProjectedStateSpace *space, ob::StateSamplerPtr sampler)
    : ob::WrapperStateSampler(space, std::move(sampler)), constraint_(space->getConstraint())
{
}

void jy_ProjectedStateSampler::sampleUniform(ob::State *state)
{
    ob::WrapperStateSampler::sampleUniform(state);
    constraint_->project(state);
    space_->enforceBounds(state);
}

void jy_ProjectedStateSampler::sampleUniformNear(ob::State *state, const ob::State *near, const double distance)
{
    ob::WrapperStateSampler::sampleUniformNear(state, near, distance);
    constraint_->project(state);
    space_->enforceBounds(state);
}

void jy_ProjectedStateSampler::sampleGaussian(ob::State *state, const ob::State *mean, const double stdDev)
{
    ob::WrapperStateSampler::sampleGaussian(state, mean, stdDev);
    constraint_->project(state);
    space_->enforceBounds(state);
}


bool jy_ProjectedStateSpace::discreteGeodesic(const ob::State *from, const ob::State *to, bool interpolate,
                                              std::vector<ob::State *> *geodesic) const
{
    if (geodesic != nullptr)
    {
        geodesic->clear();
        geodesic->push_back(cloneState(from));
    }
    const double tolerance = delta_;

    // No need to traverse the manifold if we are already there.
    double dist, step, total = 0;
    if ((dist = distance(from, to)) <= tolerance)
        return true;
    

    const double max = dist * lambda_;

    auto previous = cloneState(from);
    auto scratch = allocState();
    
    auto &&svc = si_->getStateValidityChecker();
    
    do
    {
        ob::WrapperStateSpace::interpolate(previous, to, delta_ / dist, scratch);
        // Project new state onto constraint manifold
        // bool proj = !constraint_->project(scratch);
        // bool val = !(interpolate || svc->isValid(scratch));
        // bool lamstep = (step = distance(previous, scratch)) > lambda_ * tolerance;
        // std::cout  << proj << " " << val << " " << lamstep << std::endl;
        // if (proj || val || lamstep)
       
         if (!constraint_->project(scratch)                  // not on manifold
            || !(interpolate || svc->isValid(scratch))      // not valid
            || (step = distance(previous, scratch)) > lambda_ * delta_)  // deviated
            break;

        // Check if we have wandered too far
        total += step;
        if (total > max)
            break;
        

        // Check if we are no closer than before
        const double newDist = distance(scratch, to);
        if (newDist >= dist)
            break;
        

        dist = newDist;
        copyState(previous, scratch);

        // printState(scratch, std::cout);
        // Store the new state
        if (geodesic != nullptr)
            geodesic->push_back(cloneState(scratch));
        
    } while ((dist >= tolerance)); // && (iter < 50));
    
    freeState(scratch);
    freeState(previous);

    return dist <= tolerance;
}
