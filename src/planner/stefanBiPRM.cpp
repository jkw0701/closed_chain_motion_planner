
#include <closed_chain_motion_planner/planner/stefanBiPRM.h>

#include <closed_chain_motion_planner/base/jy_ProjectedStateSpace.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/datastructures/PDF.h>
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/tools/config/MagicConstants.h>
#include <boost/graph/astar_search.hpp>
#include <boost/graph/incremental_components.hpp>
#include <boost/property_map/vector_property_map.hpp>
#include <boost/foreach.hpp>
#include <thread>
#include <future>

#include "GoalVisitor.hpp"
#define foreach BOOST_FOREACH
using namespace Eigen;
using namespace std;

stefanBiPRM::stefanBiPRM(const ompl::base::SpaceInformationPtr &si, const ompl::base::SpaceInformationPtr &tsi,
                           ob::State *obj_start, ob::State *obj_goal, jy_ValidStateSamplerPtr sampler)
    : ompl::base::Planner(si, "stefanBiPRM"),
      tsi_(std::move(tsi)), 
      obj_start_(std::move(obj_start)), obj_goal_(std::move(obj_goal)), sampler_(std::move(sampler)),
      stateProperty_(boost::get(vertex_state_t(), g_)),
      totalConnectionAttemptsProperty_(boost::get(vertex_total_connection_attempts_t(), g_)),
      successfulConnectionAttemptsProperty_(boost::get(vertex_successful_connection_attempts_t(), g_)),
      weightProperty_(boost::get(boost::edge_weight, g_)), disjointSets_(boost::get(boost::vertex_rank, g_),
                                                                         boost::get(boost::vertex_predecessor, g_))
{
    specs_.recognizedGoal = ompl::base::GOAL_SAMPLEABLE_REGION;
    specs_.approximateSolutions = true;
    specs_.optimizingPaths = true;
    specs_.multithreaded = true;

    Planner::declareParam<unsigned int>("max_nearest_neighbors", this, &stefanBiPRM::setMaxNearestNeighbors,
                                        std::string("8:1000"));

    addPlannerProgressProperty("iterations INTEGER", [this] {
        return getIterationCount();
    });
    addPlannerProgressProperty("best cost REAL", [this] {
        return getBestCost();
    });
    addPlannerProgressProperty("milestone count INTEGER", [this] {
        return getMilestoneCountString();
    });
    addPlannerProgressProperty("edge count INTEGER", [this] {
        return getEdgeCountString();
    });

    // obj_space_ = std::make_shared<ob::SE3StateSpace>();
    obj_space_ = tsi_->getStateSpace()->as<ob::CompoundStateSpace>()->getSubspace(1);
    obj_sampler_ = obj_space_->allocStateSampler();

    stefan_checker_ = std::make_shared<stefanFCL>();

    startM_ = std::make_shared< std::vector<Vertex> >();
    goalM_ = std::make_shared< std::vector<Vertex> >();
    
    nearest_ = std::make_shared<Vertex>();
    nearest_state_ = obj_start_;
    
}

stefanBiPRM::~stefanBiPRM()
{
    freeMemory();
}


void stefanBiPRM::setup()
{
    Planner::setup();

    if (!tree_)
    {
        specs_.multithreaded = false; 
        tree_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Vertex>(this));
        specs_.multithreaded = true;
        tree_->setDistanceFunction([this](const Vertex a, const Vertex b) { return distanceFunction(a, b); });  
    }
    
    if (!connectionStrategy_)
        connectionStrategy_ = og::KStrategy<Vertex>(DEFAULT_NEAREST_NEIGHBORS, tree_);

    if (pdef_)
    {
        if (pdef_->hasOptimizationObjective())
            opt_ = pdef_->getOptimizationObjective();
        else
        {
            opt_ = std::make_shared<ompl::base::PathLengthOptimizationObjective>(si_);
            opt_->setCostThreshold(opt_->infiniteCost());
        }
    }
    else
    {
        OMPL_INFORM("%s: problem definition is not set, deferring setup completion...", getName().c_str());
        setup_ = false;
    }
    
}

void stefanBiPRM::setMaxNearestNeighbors(unsigned int k)
{
    if (!tree_)
    {
        specs_.multithreaded = false; // temporarily set to false since tree_ is used only in single thread
        tree_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Vertex>(this));
        specs_.multithreaded = true;
        tree_->setDistanceFunction([this](const Vertex a, const Vertex b) {
            return distanceFunction(a, b);
        });
    }
    
    // connectionStrategy_ = ConnectionStrategy();
    connectionStrategy_ = og::KStrategy<Vertex>(k, tree_);
    
    if (isSetup())
        setup();
}

void stefanBiPRM::setProblemDefinition(const ompl::base::ProblemDefinitionPtr &pdef)
{
    ob::Planner::setProblemDefinition(pdef);
    clearQuery();
}

void stefanBiPRM::clearQuery()
{
    // startM_.clear();
    // goalM_.clear();
    pis_.restart();
}

void stefanBiPRM::clear()
{
    ob::Planner::clear();
    sampler_.reset();
    simpleSampler_.reset();
    freeMemory();
    if (tree_)
        tree_->clear();
    
    clearQuery();

    iterations_ = 0;
    bestCost_ = ompl::base::Cost(std::numeric_limits<double>::quiet_NaN());
}

void stefanBiPRM::freeMemory()
{
    foreach (Vertex v, boost::vertices(g_))
        tsi_->freeState(stateProperty_[v]);
    g_.clear();
}

bool stefanBiPRM::maybeConstructSolution(const std::vector<Vertex> &starts, const std::vector<Vertex> &goals,
                                         ompl::base::PathPtr &solution)
{
    // ompl::base::Goal *g = pdef_->getGoal().get();
    ompl::base::Cost sol_cost(opt_->infiniteCost());
    foreach (Vertex start, starts)
    {
        foreach (Vertex goal, goals)
        {
            // we lock because the connected components algorithm is incremental and may change disjointSets_
            graphMutex_.lock();
            bool same_component = sameComponent(start, goal);
            graphMutex_.unlock();
            if (same_component) // && g->isStartGoalPairValid(stateProperty_[goal], stateProperty_[start]))
            {
                ompl::base::PathPtr p = constructSolution(start, goal);
                if (p)
                {
                    ompl::base::Cost pathCost = p->cost(opt_);
                    if (opt_->isCostBetterThan(pathCost, bestCost_))
                        bestCost_ = pathCost;
                    // Check if optimization objective is satisfied
                    if (opt_->isSatisfied(pathCost))
                    {
                        solution = p;
                        OMPL_INFORM("FIND SOLUTION,,,,,,,,,,,,");
                        tsi_->printState(stateProperty_[start]);
                        tsi_->printState(stateProperty_[goal]);
                        return true;
                    }
                    if (opt_->isCostBetterThan(pathCost, sol_cost))
                    {
                        solution = p;
                        sol_cost = pathCost;
                    }
                }
            }
        }
    }

    return false;
}

bool stefanBiPRM::maybeConstructSolution(const std::vector<Vertex> &starts, const Vertex &goal,
                                         ompl::base::PathPtr &solution)
{
    // ompl::base::Goal *g = pdef_->getGoal().get();
    ompl::base::Cost sol_cost(opt_->infiniteCost());
    foreach (Vertex start, starts)
    {
        // we lock because the connected components algorithm is incremental and may change disjointSets_
        graphMutex_.lock();
        bool same_component = sameComponent(start, goal);
        graphMutex_.unlock();
        if (same_component) // && g->isStartGoalPairValid(stateProperty_[goal], stateProperty_[start]))
        {
            ompl::base::PathPtr p = constructSolution(start, goal);
            if (p)
            {
                ompl::base::Cost pathCost = p->cost(opt_);
                if (opt_->isCostBetterThan(pathCost, bestCost_))
                    bestCost_ = pathCost;
                // Check if optimization objective is satisfied
                if (opt_->isSatisfied(pathCost))
                {
                    solution = p;
                    return true;
                }
                if (opt_->isCostBetterThan(pathCost, sol_cost))
                {
                    solution = p;
                    sol_cost = pathCost;
                }
            }
        }
    }

    return false;
}


bool stefanBiPRM::sameComponent(Vertex m1, Vertex m2)
{
    return boost::same_component(m1, m2, disjointSets_);
}

bool stefanBiPRM::sameComponent_start(Vertex n)
{
    foreach (Vertex start, *startM_)
    {
        if (sameComponent(start, n)) return true;
    }
    return false;
}
stefanBiPRM::GrowState stefanBiPRM::growTree()
{
    if (!addedNewSolution_)
    {
        ob::State *obj_state_ = obj_space_->allocState();
        ob::State *mid_state_ = obj_space_->allocState();

        // obj_space_->interpolate(nearest_state_, 
        //                         stateProperty_[Sprevious_]->as<ob::CompoundStateSpace::StateType>()->components[1], 
        //                         0.3, mid_state_); // -.2
        obj_space_->interpolate(nearest_state_, 
                                obj_goal_, 
                                0.3, mid_state_); // -.2
        int iter = 3;
        while (--iter)
        {
            obj_sampler_->sampleGaussian(obj_state_, mid_state_, 0.2); // 0.25
            if (stefan_checker_->isValid(obj_state_)) return growTree(obj_state_);
        }
        return TRAPPED;  
    }
}

stefanBiPRM::GrowState stefanBiPRM::growTree(ob::State *obj_state_)
{
    std::lock_guard<std::mutex> _(graphMutex_);
    bool ik_success = false;    
    ompl::base::State *tstate = tsi_->allocState();
    tstate->as<ob::CompoundStateSpace::StateType>()->components[1] = obj_state_;

    // obj_space_ ->printState(obj_state_, std::cout);
    // graphMutex_.lock();
    Vertex t = boost::add_vertex(g_);
    stateProperty_[t] = tsi_->cloneState(tstate);
    totalConnectionAttemptsProperty_[t] = 1;
    successfulConnectionAttemptsProperty_[t] = 0;
    disjointSets_.make_set(t);
    const std::vector<Vertex> neighbors = connectionStrategy_(t);
    // graphMutex_.unlock();
    foreach (Vertex n, neighbors)
    {
        if (sampler_->sampleCalibGoal(obj_state_, stateProperty_[n]->as<ob::CompoundStateSpace::StateType>()->components[0], 
                                                  stateProperty_[t]->as<ob::CompoundStateSpace::StateType>()->components[0]))        
        {
            ik_success = true;
            break;
        }
    }
    if (ik_success)
    {
        bool add_edge = false;
        std::vector<std::pair<Vertex, ob::State *> > added_lists;
        for (auto n : neighbors)
        {
            // graphMutex_.lock();
            totalConnectionAttemptsProperty_[t]++;
            // std::cout << n << " and added vertex : " << t << std::endl;
            totalConnectionAttemptsProperty_[n]++;
            // graphMutex_.unlock();
            std::vector<ob::State *> states;
            if (si_->getStateSpace()->as<jy_ProjectedStateSpace>()->discreteGeodesic(
                                    stateProperty_[n]->as<ob::CompoundStateSpace::StateType>()->components[0],
                                    stateProperty_[t]->as<ob::CompoundStateSpace::StateType>()->components[0],
                                    false, &states)) 
            {
                // graphMutex_.lock();
                // std::cout << "add edge" << std::endl;
                successfulConnectionAttemptsProperty_[t]++;
                successfulConnectionAttemptsProperty_[n]++;
                const base::Cost weight = opt_->motionCost(stateProperty_[n]->as<ob::CompoundStateSpace::StateType>()->components[0],
                                                        stateProperty_[t]->as<ob::CompoundStateSpace::StateType>()->components[0]);
                const Graph::edge_property_type properties(weight);
                boost::add_edge(n, t, properties, g_);
                uniteComponents(n, t);
                add_edge = true;
                // graphMutex_.unlock();
            }
            
            else 
            {
                if (states.size() > 1 && sameComponent_start(n))
                {                    
                    ob::State *_obj_state = obj_space_->allocState();
                    StateEigenUtils::IsometryToState(sampler_->compute_t_wo(states.back()), _obj_state);
                    
                    double current = obj_space_->distance(stateProperty_[n]->as<ob::CompoundStateSpace::StateType>()->components[1], obj_goal_);
                    double new_dist =  obj_space_->distance(_obj_state, obj_goal_);
                    if (new_dist < current)
                    {
                        ompl::base::State *_mstate = tsi_->allocState();
                        _mstate->as<ob::CompoundStateSpace::StateType>()->components[0] = states.back();
                        _mstate->as<ob::CompoundStateSpace::StateType>()->components[1] = _obj_state;
                        added_lists.push_back(std::make_pair(n, tsi_->cloneState(_mstate)));  
                    }               
                }                
            }
        }

        if (!add_edge)
        {
            tsi_->freeState(stateProperty_[t]);
            boost::clear_vertex(t, g_);
            boost::remove_vertex(t, g_);
            if (!added_lists.empty()) milestonesToAdd(added_lists);
            return TRAPPED;
        }
        tree_->add(t);                
        foreach (Vertex start, *startM_)
        {
            bool same_comp = sameComponent(start, t);
            if (same_comp)
            {
                addedVertex = t;
                if (!added_lists.empty()) milestonesToAdd(added_lists);
                return SREACHED;
            }
        }
        return GREACHED;
    }

    tsi_->freeState(stateProperty_[t]);
    boost::clear_vertex(t, g_);
    boost::remove_vertex(t, g_);
    return TRAPPED;    
}

stefanBiPRM::Vertex stefanBiPRM::addMilestone(ompl::base::State *tstate)
{
    std::lock_guard<std::mutex> _(graphMutex_);    
    Vertex m = boost::add_vertex(g_);
    stateProperty_[m] = tstate;
    totalConnectionAttemptsProperty_[m] = 1;
    successfulConnectionAttemptsProperty_[m] = 0;
    disjointSets_.make_set(m);

    const std::vector<Vertex> &neighbors = connectionStrategy_(m);
    bool add_edge = false;
    foreach (Vertex n, neighbors)
    {
        totalConnectionAttemptsProperty_[m]++;
        totalConnectionAttemptsProperty_[n]++;

        if (si_->checkMotion(si_->cloneState(stateProperty_[n]->as<ob::CompoundStateSpace::StateType>()->components[0]),
                             si_->cloneState(stateProperty_[m]->as<ob::CompoundStateSpace::StateType>()->components[0])))
        {
            successfulConnectionAttemptsProperty_[m]++;
            successfulConnectionAttemptsProperty_[n]++;
            const ompl::base::Cost weight = opt_->motionCost(stateProperty_[n]->as<ob::CompoundStateSpace::StateType>()->components[0],
                                                             stateProperty_[m]->as<ob::CompoundStateSpace::StateType>()->components[0]);
            const Graph::edge_property_type properties(weight);
            boost::add_edge(n, m, properties, g_);
            uniteComponents(n, m);
            add_edge = true;
        }
    }
    if (add_edge) tree_->add(m);
    else 
    {
        tsi_->freeState(stateProperty_[m]);
        boost::clear_vertex(m, g_);
        boost::remove_vertex(m, g_);
    }
    return m;
}

void stefanBiPRM::milestonesToAdd(std::vector<std::pair<Vertex, ob::State *> > lists)
{
    std::cout << "add " << lists.size() << " mid stones" << std::endl;
    for (auto &pair : lists)
    {        
        Vertex mid = boost::add_vertex(g_);
        
        // stateProperty_[mid] = tsi_->cloneState(_mstate);
        stateProperty_[mid] = pair.second;
        totalConnectionAttemptsProperty_[mid] = 1;
        successfulConnectionAttemptsProperty_[mid] = 0;
        disjointSets_.make_set(mid);
        
        totalConnectionAttemptsProperty_[mid]++;
        totalConnectionAttemptsProperty_[pair.first]++;

        successfulConnectionAttemptsProperty_[mid]++;
        successfulConnectionAttemptsProperty_[pair.first]++;
        const ompl::base::Cost weight = opt_->motionCost(stateProperty_[pair.first]->as<ob::CompoundStateSpace::StateType>()->components[0],
                                                        stateProperty_[mid]->as<ob::CompoundStateSpace::StateType>()->components[0]);
        const Graph::edge_property_type properties(weight);
        boost::add_edge(pair.first, mid, properties, g_);
        uniteComponents(pair.first, mid);
        tree_->add(mid);    
    }
}

stefanBiPRM::Vertex stefanBiPRM::startgoalMilestone(ompl::base::State *tstate)
{
    std::lock_guard<std::mutex> _(graphMutex_);
    
    Vertex m = boost::add_vertex(g_);
    stateProperty_[m] = tstate;
    totalConnectionAttemptsProperty_[m] = 1;
    successfulConnectionAttemptsProperty_[m] = 0;
    disjointSets_.make_set(m);
    
    const std::vector<Vertex> &neighbors = connectionStrategy_(m);
    foreach (Vertex n, neighbors)
    {
        totalConnectionAttemptsProperty_[m]++;
        totalConnectionAttemptsProperty_[n]++;

        if (si_->checkMotion(si_->cloneState(stateProperty_[n]->as<ob::CompoundStateSpace::StateType>()->components[0]),
                             si_->cloneState(stateProperty_[m]->as<ob::CompoundStateSpace::StateType>()->components[0])))
        {
            successfulConnectionAttemptsProperty_[m]++;
            successfulConnectionAttemptsProperty_[n]++;
            const ompl::base::Cost weight = opt_->motionCost(stateProperty_[n]->as<ob::CompoundStateSpace::StateType>()->components[0],
                                                             stateProperty_[m]->as<ob::CompoundStateSpace::StateType>()->components[0]);
            const Graph::edge_property_type properties(weight);
            boost::add_edge(n, m, properties, g_);
            uniteComponents(n, m);
            // std::cout << "start and goal add edge??" << std::endl;
        }
    }
    tree_->add(m);
    return m;
}

ompl::base::Cost stefanBiPRM::constructApproximateSolution(const std::vector<Vertex> &starts, const std::vector<Vertex> &goals, ompl::base::PathPtr &solution)
{
    std::lock_guard<std::mutex> _(graphMutex_);
    ompl::base::Cost closestVal(opt_->infiniteCost());
    bool approxPathJustStart = true;

    foreach (Vertex start, starts)
    {
        foreach (Vertex goal, goals)
        {
            ompl::base::Cost heuristicCost(costHeuristic(start, goal));
            if (opt_->isCostBetterThan(heuristicCost, closestVal))
            {
                closestVal = heuristicCost;
                approxPathJustStart = true;
            }
            // if (!g->isStartGoalPairValid(stateProperty_[goal], stateProperty_[start]))
            // {
            //     continue;
            // }
            ompl::base::PathPtr p;
            boost::vector_property_map<Vertex> prev(boost::num_vertices(g_));
            boost::vector_property_map<ompl::base::Cost> dist(boost::num_vertices(g_));
            boost::vector_property_map<ompl::base::Cost> rank(boost::num_vertices(g_));

            try
            {
                // Consider using a persistent distance_map if it's slow
                boost::astar_search(
                    g_, start, [this, goal](Vertex v) { return costHeuristic(v, goal); },
                    boost::predecessor_map(prev)
                        .distance_map(dist)
                        .rank_map(rank)
                        .distance_compare([this](ompl::base::Cost c1, ompl::base::Cost c2) { return opt_->isCostBetterThan(c1, c2); })
                        .distance_combine([this](ompl::base::Cost c1, ompl::base::Cost c2) { return opt_->combineCosts(c1, c2); })
                        .distance_inf(opt_->infiniteCost())
                        .distance_zero(opt_->identityCost())
                        .visitor(AStarGoalVisitor<Vertex>(goal)));
            }
            catch (AStarFoundGoal &)
            {
            }

            Vertex closeToGoal = start;
            for (auto vp = vertices(g_); vp.first != vp.second; vp.first++)
            {
                ompl::base::Cost dist_to_goal(costHeuristic(*vp.first, goal));
                if (opt_->isFinite(rank[*vp.first]) && opt_->isCostBetterThan(dist_to_goal, closestVal))
                {
                    closeToGoal = *vp.first;
                    closestVal = dist_to_goal;
                    approxPathJustStart = false;
                }
            }
            if (closeToGoal != start)
            {
                // std::cout << "goal ::::::::::::::::::::::::::;" << std::endl;
                // tsi_->printState(stateProperty_[closeToGoal]);
                auto p(std::make_shared<og::PathGeometric>(si_));
                for (Vertex pos = closeToGoal; prev[pos] != pos; pos = prev[pos])
                    p->append(stateProperty_[pos]->as<ob::CompoundStateSpace::StateType>()->components[0]);
                p->append(stateProperty_[start]->as<ob::CompoundStateSpace::StateType>()->components[0]);
                p->reverse();

                solution = p;
            }
        }
    }
    if (approxPathJustStart)
    {
        return opt_->infiniteCost();
    }
    return closestVal;
}

ompl::base::PathPtr stefanBiPRM::constructSolution(const Vertex &start, const Vertex &goal)
{
    std::lock_guard<std::mutex> _(graphMutex_);
    boost::vector_property_map<Vertex> prev(boost::num_vertices(g_));
    
    try
    {
        // Consider using a persistent distance_map if it's slow
        boost::astar_search(
            g_, start, [this, goal](Vertex v) { return costHeuristic(v, goal); },
            boost::predecessor_map(prev)
                .distance_compare([this](base::Cost c1, base::Cost c2) { return opt_->isCostBetterThan(c1, c2); })
                .distance_combine([this](base::Cost c1, base::Cost c2) { return opt_->combineCosts(c1, c2); })
                .distance_inf(opt_->infiniteCost())
                .distance_zero(opt_->identityCost())
                .visitor(AStarGoalVisitor<Vertex>(goal)));
    }

    catch (AStarFoundGoal &)
    {
    }

    if (prev[goal] == goal)
    {
        std::vector<Vertex> lists;
        tree_->list(lists);
        for (auto i : lists)
            std::cout << i << " ";
        std::cout << std::endl;
        ob::PlannerData data(si_);
        getPlannerData(data);
        std::ofstream graphfile("/home/jiyeong/catkin_ws/debug.graphml");
        data.printGraphML(graphfile);
        graphfile.close();

        std::ofstream graphfile2("/home/jiyeong/catkin_ws/debug.dot");
        data.printGraphviz(graphfile2);
        graphfile2.close();
        throw Exception(name_, "Could not find solution path");
    }

    auto p(std::make_shared<ompl::geometric::PathGeometric>(si_));
    for (Vertex pos = goal; prev[pos] != pos; pos = prev[pos])
        p->append(stateProperty_[pos]->as<ob::CompoundStateSpace::StateType>()->components[0]);
    p->append(stateProperty_[start]->as<ob::CompoundStateSpace::StateType>()->components[0]);
    p->reverse();

    return p;
}

void stefanBiPRM::getPlannerData(ompl::base::PlannerData &data) const
{
    ob::Planner::getPlannerData(data);

    for (unsigned long i : *startM_)
        data.addStartVertex(
            ompl::base::PlannerDataVertex(stateProperty_[i]->as<ob::CompoundStateSpace::StateType>()->components[0], const_cast<stefanBiPRM *>(this)->disjointSets_.find_set(i)));

    for (unsigned long i : *goalM_)
        data.addGoalVertex(
            ompl::base::PlannerDataVertex(stateProperty_[i]->as<ob::CompoundStateSpace::StateType>()->components[0], const_cast<stefanBiPRM *>(this)->disjointSets_.find_set(i)));

    // Adding edges and all other vertices simultaneously
    foreach (const Edge e, boost::edges(g_))
    {
        const Vertex v1 = boost::source(e, g_);
        const Vertex v2 = boost::target(e, g_);
        data.addEdge(ompl::base::PlannerDataVertex(stateProperty_[v1]->as<ob::CompoundStateSpace::StateType>()->components[0]), ompl::base::PlannerDataVertex(stateProperty_[v2]->as<ob::CompoundStateSpace::StateType>()->components[0]));

        // Add the reverse edge, since we're constructing an undirected roadmap
        data.addEdge(ompl::base::PlannerDataVertex(stateProperty_[v2]->as<ob::CompoundStateSpace::StateType>()->components[0]), ompl::base::PlannerDataVertex(stateProperty_[v1]->as<ob::CompoundStateSpace::StateType>()->components[0]));

        // Add tags for the newly added vertices
        data.tagState(stateProperty_[v1]->as<ob::CompoundStateSpace::StateType>()->components[0], const_cast<stefanBiPRM *>(this)->disjointSets_.find_set(v1));
        data.tagState(stateProperty_[v2]->as<ob::CompoundStateSpace::StateType>()->components[0], const_cast<stefanBiPRM *>(this)->disjointSets_.find_set(v2));
    }
}

ompl::base::Cost stefanBiPRM::costHeuristic(Vertex u, Vertex v) const
{
    double d = obj_space_->distance(stateProperty_[u]->as<ob::CompoundStateSpace::StateType>()->components[1],
                                    stateProperty_[v]->as<ob::CompoundStateSpace::StateType>()->components[1]);
    ompl::base::Cost cost(d);
    return cost;
}

ompl::base::Cost stefanBiPRM::closestFromGoal(std::vector<Vertex> froms, std::vector<Vertex> tos, Vertex &closest)
{
    std::lock_guard<std::mutex> _(graphMutex_);
    // std::vector<Vertex> froms, tos;

    ompl::base::Cost closestVal(opt_->infiniteCost());
    foreach (Vertex from, froms)
    {
        foreach (Vertex to, tos)
        {
            ompl::base::Cost heuristicCost(costHeuristic(from, to));
            if (opt_->isCostBetterThan(heuristicCost, closestVal))
                closestVal = heuristicCost;

            boost::vector_property_map<Vertex> prev(boost::num_vertices(g_));
            boost::vector_property_map<ompl::base::Cost> dist(boost::num_vertices(g_));
            boost::vector_property_map<ompl::base::Cost> rank(boost::num_vertices(g_));
            try
            {
                boost::astar_search(
                    g_, from, [this, to](Vertex v) { return costHeuristic(v, to); },
                    boost::predecessor_map(prev)
                        .distance_map(dist)
                        .rank_map(rank)
                        .distance_compare(
                            [this](ompl::base::Cost c1, ompl::base::Cost c2) { return opt_->isCostBetterThan(c1, c2); })
                        .distance_combine([this](ompl::base::Cost c1, ompl::base::Cost c2) { return opt_->combineCosts(c1, c2); })
                        .distance_inf(opt_->infiniteCost())
                        .distance_zero(opt_->identityCost())
                        .visitor(AStarGoalVisitor<Vertex>(to)));
            }
            catch (AStarFoundGoal &)
            {
            }
            Vertex temp = from;
            for (auto vp = vertices(g_); vp.first != vp.second; vp.first++)
            {
                ompl::base::Cost dist_to_goal(costHeuristic(*vp.first, to));
                if (opt_->isFinite(rank[*vp.first]) && opt_->isCostBetterThan(dist_to_goal, closestVal))
                {
                    temp = *vp.first;
                    closestVal = dist_to_goal;
                }
            }
            closest = temp;
            break;
        }
    }
    return closestVal;
}

void stefanBiPRM::checkForSolution(const ompl::base::PlannerTerminationCondition &ptc, ompl::base::PathPtr &solution)
{
    Sprevious_ = goalM_->front();
    double prev_distFromGoal = obj_space_->distance(obj_start_, obj_goal_);
    distFromGoal = opt_->infiniteCost();
    distFromStart = opt_->infiniteCost();
    
    int tree_size = tree_->size();

    double prev_distFromStart = prev_distFromGoal;

    ompl::base::State *tst_goal = tsi_->allocState();
    ompl::base::State *tst_start = tsi_->allocState();

    bool start = true;
    distFromGoal = closestFromGoal(*startM_, *goalM_, *nearest_);
    while (!ptc && !addedNewSolution_)
    {
        if (tree_->size() > tree_size + 3)
        {
            // OMPL_INFORM("calculate distance~");
            tree_size = tree_->size();    
            distFromGoal = closestFromGoal(*startM_, *goalM_, *nearest_);
            // std::cout <<" ======= distance to goal : " << distFromGoal << std::endl;
            /* Update Goal State */
            if (distFromGoal.value() < prev_distFromGoal - 0.1)
            {
                std::cout <<" ======= distance to goal : " << distFromGoal << std::endl;
                nearest_state_ = stateProperty_[*nearest_]->as<ob::CompoundStateSpace::StateType>()->components[1];
                prev_distFromGoal = distFromGoal.value();
                bool goal_suc = sampler_->sampleCalibGoal(StateEigenUtils::StateToIsometry(obj_goal_), 
                                                            stateProperty_[*nearest_]->as<ob::CompoundStateSpace::StateType>()->components[0], 
                                                            tst_goal->as<ob::CompoundStateSpace::StateType>()->components[0]);
                if (goal_suc)
                {
                    tst_goal->as<ob::CompoundStateSpace::StateType>()->components[1] = obj_goal_;
                    const ob::State *con_goal = tst_goal;
                    OMPL_INFORM(" Add new goal ");
                    // tsi_->printState(tst_goal);
                    goalM_->push_back(startgoalMilestone(tsi_->cloneState(con_goal)));
                }
                ob::State *obj_state_ = obj_space_->allocState();
                ob::State *tstate = tsi_->allocState();
                for (int i = 1; i < 10; i++)
                {
                    obj_space_->interpolate(stateProperty_[*nearest_]->as<ob::CompoundStateSpace::StateType>()->components[1],
                                            obj_goal_, 0.1 * i, obj_state_);
                    // obj_space_->interpolate(stateProperty_[*nearest_]->as<ob::CompoundStateSpace::StateType>()->components[1],
                    //                         stateProperty_[Sprevious_]->as<ob::CompoundStateSpace::StateType>()->components[1],
                    //                         0.1 * i, obj_state_);
                    Isometry3d base_obj = StateEigenUtils::StateToIsometry(obj_state_);
                    if (stefan_checker_->isValid(base_obj) && 
                        sampler_->sampleCalibGoal(base_obj, stateProperty_[*nearest_]->as<ob::CompoundStateSpace::StateType>()->components[0], 
                                                            tstate->as<ob::CompoundStateSpace::StateType>()->components[0]))
                    {
                        // std::cout << "goal interpolate success / step size : " << 0.1 * i << std::endl;
                        tstate->as<ob::CompoundStateSpace::StateType>()->components[1] = obj_state_;
                        addMilestone(tsi_->cloneState(tstate));
                    }
                    else break;
                }
            }
        
            distFromStart = closestFromGoal(*goalM_, *startM_, Sprevious_);        
            if (distFromStart.value() < prev_distFromStart - 0.1)
            {
                prev_distFromStart = distFromStart.value();
                std::cout << "=======distance to start : " << distFromStart.value() << std::endl;    
                bool start_suc = sampler_->sampleCalibGoal(StateEigenUtils::StateToIsometry(obj_start_), 
                                                            stateProperty_[Sprevious_]->as<ob::CompoundStateSpace::StateType>()->components[0], 
                                                            tst_start->as<ob::CompoundStateSpace::StateType>()->components[0]);
                if (start_suc)
                {
                    tst_start->as<ob::CompoundStateSpace::StateType>()->components[1] = obj_start_;
                    const ob::State *con_start = tst_start;
                    OMPL_INFORM(" Add new start ");
                    // tsi_->printState(con_start);
                    startM_->push_back(startgoalMilestone(tsi_->cloneState(con_start)));
                }
                // ob::State *obj_state_ = obj_space_->allocState();
                // ob::State *tstate = tsi_->allocState();
                // for (int i = 1; i < 10; i++)
                // {
                //     obj_space_->interpolate(stateProperty_[Sprevious_]->as<ob::CompoundStateSpace::StateType>()->components[1],
                //                             obj_start_, 0.1 * i, obj_state_);
                //     Isometry3d base_obj = StateEigenUtils::StateToIsometry(obj_state_);
                //     if (stefan_checker_->isValid(base_obj) &&
                //         sampler_->sampleGoal(base_obj, stateProperty_[Sprevious_]->as<ob::CompoundStateSpace::StateType>()->components[0], 
                //                                             tstate->as<ob::CompoundStateSpace::StateType>()->components[0]))
                //     {
                //         // std::cout << "start interpolate success / step size : " << 0.1 * i << std::endl;
                //         tstate->as<ob::CompoundStateSpace::StateType>()->components[1] = obj_state_;
                //         addMilestone(tsi_->cloneState(tstate));
                //     }
                //     else break;
                // }
            }
            
            // start = !start;
        }
        addedNewSolution_ = maybeConstructSolution(*startM_, *goalM_, solution);
        if (!addedNewSolution_)
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        else
        {
            nearest_state_ = obj_goal_;
            distFromGoal = ob::Cost(0.0);
        }
    
    }
}



ompl::base::PlannerStatus stefanBiPRM::solve(const ompl::base::PlannerTerminationCondition &ptc)
{
    // checkValidity();
    auto *goal = dynamic_cast<ompl::base::GoalSampleableRegion *>(pdef_->getGoal().get());
    if (goal == nullptr)
    {
        OMPL_ERROR("%s: Unknown type of goal", getName().c_str());
        return ompl::base::PlannerStatus::UNRECOGNIZED_GOAL_TYPE;
    }
    while (const base::State *st = pis_.nextStart())
    {
        ompl::base::State *tst = tsi_->allocState();
        tst->as<ob::CompoundStateSpace::StateType>()->components[0] = si_->cloneState(st);
        tst->as<ob::CompoundStateSpace::StateType>()->components[1] = obj_start_;
        startM_->push_back(startgoalMilestone(tst));
    }

    
    if (startM_->empty())
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return ompl::base::PlannerStatus::INVALID_START;
    }

    if (!goal->couldSample())
    {
        OMPL_ERROR("%s: Insufficient states in sampleable goal region", getName().c_str());
        return ompl::base::PlannerStatus::INVALID_GOAL;
    }
    
    if (goal->maxSampleCount() > goalM_->size() || goalM_->empty())
    {
        const base::State *st = goalM_->empty() ? pis_.nextGoal(ptc) : pis_.nextGoal();
        ompl::base::State *tst = tsi_->allocState();
        tst->as<ob::CompoundStateSpace::StateType>()->components[0] = si_->cloneState(st);
        tst->as<ob::CompoundStateSpace::StateType>()->components[1] = obj_goal_;
        goalM_->push_back(startgoalMilestone(tst));
    }

    distFromGoal = opt_->infiniteCost();    
    
    ompl::base::PathPtr sol_start;
    std::thread slnThreadStart([this, &ptc, &sol_start] 
                { checkForSolution(ptc, sol_start); });

    ompl::base::PlannerTerminationCondition ptcOrSolutionFound([this, &ptc] 
    { return ptc || (addedNewSolution_); }); 
    
    constructRoadmap(ptcOrSolutionFound);

    slnThreadStart.join();

    bool approximate = false;

    if (sol_start)
    {
        ompl::base::PlannerSolution psol(sol_start);
        psol.setPlannerName(getName());
        psol.setOptimized(opt_, bestCost_, addedNewSolution_);
        pdef_->addSolutionPath(psol);
    }
    else
    {
        // Return an approximate solution.
        ompl::base::Cost diff = constructApproximateSolution(*startM_, *goalM_, sol_start);
        if (!opt_->isFinite(diff))
        {
            OMPL_INFORM("Closest path is still start and goal");
            return ompl::base::PlannerStatus::TIMEOUT;
        }
        OMPL_INFORM("Using approximate solution, heuristic cost-to-go is %f", diff.value());
        pdef_->addSolutionPath(sol_start, true, diff.value(), getName());
        return ompl::base::PlannerStatus::APPROXIMATE_SOLUTION;
    }
    return sol_start ? ompl::base::PlannerStatus::EXACT_SOLUTION : ompl::base::PlannerStatus::TIMEOUT;
}

void stefanBiPRM::constructRoadmap(const ompl::base::PlannerTerminationCondition &ptc)
{
    // checkValidity();
    // Gplanner->checkValidity();

    OMPL_INFORM("START CONSTRUCT ROADMAP!");
    ob::State *mid_state_ = obj_space_->allocState();
    // double newDist = distanceBetweenTrees_;
    double prev_S;
    prev_S = distFromGoal.value();
    while (!ptc())
    {
        growTree();
        // std::cout << growTree() <<std::endl;
    }
    OMPL_INFORM("FINISH CONSTRUCT ROADMAP");
}
