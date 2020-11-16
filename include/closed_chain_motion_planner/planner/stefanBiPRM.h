#pragma once

#include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/geometric/planners/prm/ConnectionStrategy.h>
#include <ompl/geometric/planners/PlannerIncludes.h>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/pending/disjoint_sets.hpp>
#include <mutex>
#include <utility>
#include <vector>
#include <map>

#include <ompl/base/spaces/constraint/ConstrainedStateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/ConstrainedSpaceInformation.h>
#include <closed_chain_motion_planner/kinematics/KinematicChain.h>
#include <closed_chain_motion_planner/base/jy_ConstrainedValidStateSampler.h>
#include <closed_chain_motion_planner/base/stefan_planning/stefanFCL.h>
#include <closed_chain_motion_planner/base/utils.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/goals/GoalStates.h>
using namespace ompl;

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace ompl
{
    namespace base
    {
        OMPL_CLASS_FORWARD(OptimizationObjective);
    }
} // namespace ompl

static const unsigned int DEFAULT_NEAREST_NEIGHBORS = 5;

class stefanBiPRM : public ompl::base::Planner
{
public:
    enum GrowState
    {
        TRAPPED,
        SREACHED,
        GREACHED
    };
    struct vertex_state_t
    {
        typedef boost::vertex_property_tag kind;
    };

    struct vertex_total_connection_attempts_t
    {
        typedef boost::vertex_property_tag kind;
    };

    struct vertex_successful_connection_attempts_t
    {
        typedef boost::vertex_property_tag kind;
    };

    using Graph = boost::adjacency_list<
        boost::vecS, boost::vecS, boost::undirectedS,
        boost::property<
            vertex_state_t, base::State *,
            boost::property<
                vertex_total_connection_attempts_t, unsigned long int,
                boost::property<vertex_successful_connection_attempts_t, unsigned long int,
                                boost::property<boost::vertex_predecessor_t, unsigned long int,
                                                boost::property<boost::vertex_rank_t, unsigned long int>>>>>,
        boost::property<boost::edge_weight_t, base::Cost>>;

    /** @brief The type for a vertex in the roadmap. */
    using Vertex = boost::graph_traits<Graph>::vertex_descriptor;
    using VertexPtr = std::shared_ptr<Vertex>;
    /** @brief The type for an edge in the roadmap. */
    using Edge = boost::graph_traits<Graph>::edge_descriptor;

    /** @brief A nearest neighbors data structure for roadmap vertices. */
    using RoadmapNeighbors = std::shared_ptr<NearestNeighbors<Vertex>>;

    /** @brief A function returning the milestones that should be
     * attempted to connect to. */
    using ConnectionStrategy = std::function<const std::vector<Vertex> &(const Vertex)>;
    using ConnectionFilter = std::function<bool(const Vertex &, const Vertex &)>;
    /*  A function that can reject connections.

        This is called after previous connections from the neighbor list
        have been added to the roadmap.
        */

    /** \brief Access to the internal ompl::base::state at each Vertex */
    boost::property_map<Graph, vertex_state_t>::type stateProperty_;

    /** \brief Access to the number of total connection attempts for a vertex */
    boost::property_map<Graph, vertex_total_connection_attempts_t>::type totalConnectionAttemptsProperty_;

    /** \brief Access to the number of successful connection attempts for a vertex */
    boost::property_map<Graph, vertex_successful_connection_attempts_t>::type
        successfulConnectionAttemptsProperty_;

    /** \brief Access to the weights of each Edge */
    boost::property_map<Graph, boost::edge_weight_t>::type weightProperty_;

    /** \brief Data structure that maintains the connected components */
    boost::disjoint_sets<boost::property_map<Graph, boost::vertex_rank_t>::type,
                         boost::property_map<Graph, boost::vertex_predecessor_t>::type>
        disjointSets_;

    /** \brief Constructor */
    stefanBiPRM(const ompl::base::SpaceInformationPtr &si, const ompl::base::SpaceInformationPtr &tsi,
                ob::State *obj_start, ob::State *obj_regrp, jy_ValidStateSamplerPtr sampler);
    ~stefanBiPRM() override;

    void setProblemDefinition(const ompl::base::ProblemDefinitionPtr &pdef) override;
    void setMaxNearestNeighbors(unsigned int k);

    void getPlannerData(ompl::base::PlannerData &data) const override;

    void clearQuery();

    void clear() override;

    /** \brief Set a different nearest neighbors datastructure */
    template <template <typename T> class NN>
    void setNearestNeighbors()
    {
        if (tree_ && tree_->size() != 0)
            OMPL_WARN("Calling setNearestNeighbors will clear all states.");
        clear();
        tree_ = std::make_shared<NN<Vertex>>();
        // connectionStrategy_ = ConnectionStrategy();
        connectionStrategy_ = og::KStrategy<Vertex>(DEFAULT_NEAREST_NEIGHBORS, tree_);

        if (isSetup())
            setup();
    }

    void setup() override;

    const Graph &getRoadmap() const
    {
        return g_;
    }

    /** \brief Return the number of milestones currently in the graph */
    unsigned long int milestoneCount() const
    {
        return boost::num_vertices(g_);
    }

    /** \brief Return the number of edges currently in the graph */
    unsigned long int edgeCount() const
    {
        return boost::num_edges(g_);
    }

    // bool isSatisfied(const ob::State *st) const;
    std::shared_ptr<std::vector<Vertex>> startM_, goalM_;
    bool addedNewSolution_{false};
    VertexPtr nearest_;
    Vertex addedVertex, Sprevious_;
    ob::State *nearest_state_;

    ompl::base::Cost distFromGoal, distFromStart;
    void freeMemory();
    GrowState growTree();
    GrowState growTree(ob::State *obj_state_);

    Vertex addMilestone(ompl::base::State *state);
    Vertex startgoalMilestone(ompl::base::State *tstate);
    Vertex addMidMilestone(ompl::base::State *tstate, Vertex &n);
    void milestonesToAdd(std::vector<std::pair<Vertex, ob::State *>> lists);
    void uniteComponents(Vertex m1, Vertex m2)
    {
        disjointSets_.union_set(m1, m2);
    }
    bool sameComponent(Vertex m1, Vertex m2);
    bool sameComponent_start(Vertex n);
    // void checkForSolution(const ompl::base::PlannerTerminationCondition &ptc, ompl::base::PathPtr &solution);
    void checkForSolution(const ompl::base::PlannerTerminationCondition &ptc, ompl::base::PathPtr &solution);
    bool maybeConstructSolution(const std::vector<Vertex> &starts, const std::vector<Vertex> &goals,
                                ompl::base::PathPtr &solution);
    bool maybeConstructSolution(const std::vector<Vertex> &starts, const Vertex &goal,
                                ompl::base::PathPtr &solution);

    ompl::base::Cost constructApproximateSolution(const std::vector<Vertex> &starts, const std::vector<Vertex> &goals, ompl::base::PathPtr &solution);

    bool addedNewSolution() const;

    ompl::base::PathPtr constructSolution(const Vertex &start, const Vertex &goal);

    ompl::base::Cost costHeuristic(Vertex u, Vertex v) const;

    double distanceFunction(const Vertex a, const Vertex b) const
    {
        /* joint */
        // return si_->distance(stateProperty_[a]->as<ob::CompoundStateSpace::StateType>()->components[0], stateProperty_[b]->as<ob::CompoundStateSpace::StateType>()->components[0]);

        /* SE3 */
        return obj_space_->distance(stateProperty_[a]->as<ob::CompoundStateSpace::StateType>()->components[1], stateProperty_[b]->as<ob::CompoundStateSpace::StateType>()->components[1]);
    }

    ///////////////////////////////////////
    // Planner progress property functions
    std::string getIterationCount() const
    {
        return std::to_string(iterations_);
    }
    std::string getBestCost() const
    {
        return std::to_string(bestCost_.value());
    }
    std::string getMilestoneCountString() const
    {
        return std::to_string(milestoneCount());
    }
    std::string getEdgeCountString() const
    {
        return std::to_string(edgeCount());
    }

    jy_ValidStateSamplerPtr sampler_;

    /** \brief Sampler user for generating random in the state space */
    ompl::base::StateSamplerPtr simpleSampler_;

    /** \brief Nearest neighbors data structure */
    RoadmapNeighbors tree_;
    /** \brief Connectivity graph */
    Graph g_;

    /** \brief Function that returns the milestones to attempt connections with */
    ConnectionStrategy connectionStrategy_;

    /** \brief Random number generator */
    RNG rng_;
    mutable std::mutex graphMutex_;

    /** \brief Objective cost function for stefanBiPRM graph edges */
    ompl::base::OptimizationObjectivePtr opt_;

    //////////////////////////////
    // Planner progress properties
    /** \brief Number of iterations the algorithm performed */
    unsigned long int iterations_{0};
    /** \brief Best cost found so far by algorithm */
    ompl::base::Cost bestCost_{std::numeric_limits<double>::quiet_NaN()};

    // ompl::base::jy_ConstrainedSpaceInformationPtr si_;
    ob::SpaceInformationPtr tsi_;
    ob::State *obj_start_, *obj_goal_;

    std::normal_distribution<> normalDist_{0, 1};
    std::mt19937 generator_;
    grasping_point grp_;

    // ob::Cost closestFromGoal(bool start, Vertex &closest);
    ompl::base::Cost closestFromGoal(std::vector<Vertex> froms, std::vector<Vertex> tos, Vertex &closest);

    std::shared_ptr<stefanFCL> stefan_checker_;
    ob::StateSpacePtr obj_space_;
    ob::StateSamplerPtr obj_sampler_;
    Eigen::Matrix<double, 7, 1> lower_limit, upper_limit;
    double epsilon{0.2};

    ompl::base::PlannerStatus solve(const ompl::base::PlannerTerminationCondition &ptc) override;
    void constructRoadmap(const ompl::base::PlannerTerminationCondition &ptc);
};
