

#include <utility>

#include "ompl/base/ScopedState.h"
#include <closed_chain_motion_planner/base/jy_GoalLazySamples.h>
#include "ompl/util/Time.h"

ompl::base::jy_GoalLazySamples::jy_GoalLazySamples(const SpaceInformationPtr &si, jy_GoalSamplingFn samplerFunc, bool autoStart,
                                             double minDist)
  : GoalStates(si)
  , samplerFunc_(std::move(samplerFunc))
  , terminateSamplingThread_(false)
  , samplingThread_(nullptr)
  , samplingAttempts_(0)
  , minDist_(minDist)
{
    type_ = GOAL_LAZY_SAMPLES;
    if (autoStart)
        startSampling();
}

ompl::base::jy_GoalLazySamples::~jy_GoalLazySamples()
{
    stopSampling();
}

void ompl::base::jy_GoalLazySamples::startSampling()
{
    std::lock_guard<std::mutex> slock(lock_);
    if (samplingThread_ == nullptr)
    {
        // OMPL_DEBUG("Starting goal sampling thread");
        terminateSamplingThread_ = false;
        samplingThread_ = new std::thread(&jy_GoalLazySamples::goalSamplingThread, this);
    }
}

void ompl::base::jy_GoalLazySamples::stopSampling()
{
    /* Set termination flag */
    {
        std::lock_guard<std::mutex> slock(lock_);
        if (!terminateSamplingThread_)
        {
            OMPL_DEBUG("Attempting to stop goal sampling thread...");
            terminateSamplingThread_ = true;
        }
    }

    /* Join thread */
    if (samplingThread_ != nullptr)
    {
        samplingThread_->join();
        delete samplingThread_;
        samplingThread_ = nullptr;
    }
}

void ompl::base::jy_GoalLazySamples::goalSamplingThread()
{
    {
        /* Wait for startSampling() to finish assignment
         * samplingThread_ */
        std::lock_guard<std::mutex> slock(lock_);
    }

    if (!si_->isSetup())  // this looks racy
    {
        OMPL_DEBUG("Waiting for space information to be set up before the sampling thread can begin computation...");
        // wait for everything to be set up before performing computation
        while (!terminateSamplingThread_ && !si_->isSetup())
            std::this_thread::sleep_for(time::seconds(0.01));
    }
    unsigned int prevsa = samplingAttempts_;

    if (isSampling() && samplerFunc_)
    {
        // OMPL_DEBUG("Beginning sampling thread computation");
        ScopedState<> s(si_);
        while (isSampling() && samplerFunc_(this, s.get()))
        {
            ++samplingAttempts_;
            if (si_->satisfiesBounds(s.get()) && si_->isValid(s.get()))
            {
                // si_->printState(s.get());
                addStateIfDifferent(s.get(), minDist_);
            }
            else
            {
                // OMPL_DEBUG("Invalid goal candidate");
            }
        }
    }
    else
        OMPL_WARN("Goal sampling thread never did any work.%s",
                  samplerFunc_ ? (si_->isSetup() ? "" : " Space information not set up.") : " No sampling function "
                                                                                            "set.");
    {
        std::lock_guard<std::mutex> slock(lock_);
        terminateSamplingThread_ = true;
    }
    // std::cout << samplingAttempts_ - prevsa << std::endl;
    // if (samplingAttempts_ - prevsa < 3)
    // {
    //     resampling = true;
    // }
    OMPL_DEBUG("Stopped goal sampling thread after %u sampling attempts", samplingAttempts_ - prevsa);
}

bool ompl::base::jy_GoalLazySamples::isSampling() const
{
    std::lock_guard<std::mutex> slock(lock_);
    return !terminateSamplingThread_ && samplingThread_ != nullptr;
}

bool ompl::base::jy_GoalLazySamples::couldSample() const
{
    return canSample() || isSampling();
}

void ompl::base::jy_GoalLazySamples::clear()
{
    std::lock_guard<std::mutex> slock(lock_);
    GoalStates::clear();
}

double ompl::base::jy_GoalLazySamples::distanceGoal(const State *st) const
{
    std::lock_guard<std::mutex> slock(lock_);
    return GoalStates::distanceGoal(st);
}

void ompl::base::jy_GoalLazySamples::sampleGoal(base::State *st) const
{
    std::lock_guard<std::mutex> slock(lock_);
    GoalStates::sampleGoal(st);
}

void ompl::base::jy_GoalLazySamples::setNewStateCallback(const NewStateCallbackFn &callback)
{
    callback_ = callback;
}

void ompl::base::jy_GoalLazySamples::addState(const State *st)
{
    std::lock_guard<std::mutex> slock(lock_);
    GoalStates::addState(st);
}

const ompl::base::State *ompl::base::jy_GoalLazySamples::getState(unsigned int index) const
{
    std::lock_guard<std::mutex> slock(lock_);
    return GoalStates::getState(index);
}

bool ompl::base::jy_GoalLazySamples::hasStates() const
{
    std::lock_guard<std::mutex> slock(lock_);
    return GoalStates::hasStates();
}

std::size_t ompl::base::jy_GoalLazySamples::getStateCount() const
{
    std::lock_guard<std::mutex> slock(lock_);
    return GoalStates::getStateCount();
}

unsigned int ompl::base::jy_GoalLazySamples::maxSampleCount() const
{
    std::lock_guard<std::mutex> slock(lock_);
    return GoalStates::maxSampleCount();
}

bool ompl::base::jy_GoalLazySamples::addStateIfDifferent(const State *st, double minDistance)
{
    const base::State *newState = nullptr;
    bool added = false;
    {
        std::lock_guard<std::mutex> slock(lock_);
        if (GoalStates::distanceGoal(st) > minDistance)
        {
            GoalStates::addState(st);
            added = true;
            if (callback_)
                newState = states_.back();
        }
    }

    // the lock is released at this; if needed, issue a call to the callback
    if (newState != nullptr)
        callback_(newState);
    return added;
}
