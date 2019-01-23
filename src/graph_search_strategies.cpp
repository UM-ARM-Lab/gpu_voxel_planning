#include "graph_search_strategies.hpp"



// using namespace GVP;
namespace GVP
{


    GraphSearchStrategy::GraphSearchStrategy(const std::string &filename) :
        graph(filename), initialized(false) {}

    void GraphSearchStrategy::initialize(const Scenario &scenario)
    {
        cur_node = graph.addVertexAndEdges(scenario.getState().getCurConfig().asVector());
        goal_node = graph.addVertexAndEdges(VictorRightArmConfig(scenario.goal_config).asVector());
        initialized = true;
    }


    Path GraphSearchStrategy::applyTo(Scenario &scenario)
    {
        if(!initialized)
        {
            initialize(scenario);
        }
        const VictorRightArmConfig &current(scenario.getState().getCurConfig());
        VictorRightArmConfig expected(graph.GetNodeImmutable(cur_node).GetValueImmutable());
        VictorRightArmConfig next;
            
        if(current == expected)
        {
            std::vector<NodeIndex> node_path = lazySp(cur_node, goal_node, scenario.getState());
            next = graph.GetNodeImmutable(node_path[1]).GetValueImmutable();
            prev_node = cur_node;
            cur_node = node_path[1];
        }
        else
        {
            std::cout << "Off path, returning to graph\n";
            next = graph.GetNodeImmutable(prev_node).GetValueImmutable();
            graph.GetEdgeMutable(prev_node, cur_node).SetValidity(arc_dijkstras::EDGE_VALIDITY::INVALID);
                
            cur_node = prev_node;
        }

        return interpolate(current, next, discretization);
    }



    void GraphSearchStrategy::computeSweptVolume(State &s, arc_dijkstras::GraphEdge &e)
    {
        PROFILE_START("ComputeSweptVolume");
        VictorRightArmConfig q_start(graph.GetNodeImmutable(e.GetFromIndex()).GetValueImmutable());
        VictorRightArmConfig q_end(graph.GetNodeImmutable(e.GetToIndex()).GetValueImmutable());
        GVP::Path path = interpolate(q_start, q_end, discretization);

        DenseGrid swept_volume;
        for(const auto &config: path)
        {
            s.robot.set(config.asMap());
            swept_volume.add(&s.robot.occupied_space);
        }
        PROFILE_RECORD("ComputeSweptVolume");
        precomputed_swept_volumes[arc_dijkstras::getHashable(e)] = swept_volume;
    }


    DenseGrid GraphSearchStrategy::getSweptVolume(State &s, arc_dijkstras::GraphEdge &e)
    {
        PROFILE_START("GetSweptVolume");
        if(!precomputed_swept_volumes.count(arc_dijkstras::getHashable(e)))
        {
            computeSweptVolume(s, e);
        }
            
        PROFILE_RECORD("GetSweptVolume");
        return DenseGrid(precomputed_swept_volumes[arc_dijkstras::getHashable(e)]);
    }


    double GraphSearchStrategy::calcEdgeProbability(State &s, arc_dijkstras::GraphEdge &e)
    {
        return s.calcProbFree(getSweptVolume(s, e));
    }

    bool GraphSearchStrategy::checkEdge(arc_dijkstras::GraphEdge &e, State &s)
    {
        bool valid = !getSweptVolume(s, e).overlapsWith(&s.known_obstacles);
        e.SetValidity(valid ? arc_dijkstras::EDGE_VALIDITY::VALID :
                      arc_dijkstras::EDGE_VALIDITY::INVALID);
        return valid;
    }

    double GraphSearchStrategy::evaluateEdge(arc_dijkstras::GraphEdge &e, State &s)
    {
        if(e.GetValidity() == arc_dijkstras::EDGE_VALIDITY::UNKNOWN)
        {
            checkEdge(e, s);
        }
        return calculateEdgeWeight(s, e);
    }

    std::vector<NodeIndex> GraphSearchStrategy::lazySp(NodeIndex start, NodeIndex goal, State &s)
    {
        const auto eval_fn =
            [&] (arc_dijkstras::Graph<std::vector<double>> &g, arc_dijkstras::GraphEdge &e)
            {
                return evaluateEdge(e, s);
            };

        auto result = arc_dijkstras::LazySP<std::vector<double>>::PerformLazySP(
            graph, start, goal, &distanceHeuristic, eval_fn, true);
        return result.first;
    }






    /**********************************
     **  Optimistic Graph Search
     ********************************/
    double OptimisticGraphSearch::calculateEdgeWeight(State &s, arc_dijkstras::GraphEdge &e)
    {
        return e.GetWeight();
    }




     /**********************************
     **  ParetoCost Graph Search
     ********************************/
    double ParetoCostGraphSearch::calculateEdgeWeight(State &s, arc_dijkstras::GraphEdge &e)
    {
        double p_cost = -std::log(calcEdgeProbability(s, e));
        double l_cost = e.GetWeight();
        return l_cost + alpha * p_cost;

    }

}
