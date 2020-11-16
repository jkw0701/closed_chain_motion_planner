#pragma once
#include <boost/graph/astar_search.hpp>

namespace
{
    struct AStarFoundGoal
    {
    };  // exception for termination

    // visitor that terminates when we find the goal
    // V is the vertex type
    template <typename V>
    class AStarGoalVisitor : public boost::default_astar_visitor
    {
    public:
        AStarGoalVisitor(const V &goal) : goal_(goal)
        {
        }

        // G is the graph type
        template <typename G>
        void examine_vertex(const V &u, const G & /*unused*/)
        {
            if (u == goal_)
                throw AStarFoundGoal();
        }

    private:
        V goal_;
    };
}

