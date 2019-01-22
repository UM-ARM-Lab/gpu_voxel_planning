#ifndef GRAPH_SEARCH_STRATEGIES_HPP
#define GRAPH_SEARCH_STRATEGIES_HPP

#include "victor_halton_roadmap.hpp"
#include "state.hpp"
#include "strategies.hpp"
#include <graph_planner/dijkstras_addons.hpp>
#include <cmath>
#include <arc_utilities/timing.hpp>

namespace GVP
{

    typedef int64_t NodeIndex;



    
    class GraphSearchStrategy : public Strategy
    {
    public:        
        Roadmap graph;
        NodeIndex prev_node;
        NodeIndex cur_node;
        NodeIndex goal_node;
        bool initialized;
        double discretization = 0.02;

        std::map<arc_dijkstras::HashableEdge, ProbGrid> precomputed_swept_volumes;

        GraphSearchStrategy(const std::string &filename) : graph(filename), initialized(false) {}

        virtual double calculateEdgeWeight(State &s, arc_dijkstras::GraphEdge &e) = 0;

        void initialize(const Scenario &scenario)
        {
            cur_node = graph.addVertexAndEdges(scenario.getState().getCurConfig().asVector());
            goal_node = graph.addVertexAndEdges(VictorRightArmConfig(scenario.goal_config).asVector());
            initialized = true;
        }

        
        GVP::Path applyTo(Scenario &scenario)
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


        void computeSweptVolume(State &s, arc_dijkstras::GraphEdge &e)
        {
            PROFILE_START("ComputeSweptVolume");
            VictorRightArmConfig q_start(graph.GetNodeImmutable(e.GetFromIndex()).GetValueImmutable());
            VictorRightArmConfig q_end(graph.GetNodeImmutable(e.GetToIndex()).GetValueImmutable());
            GVP::Path path = interpolate(q_start, q_end, discretization);

            ProbGrid swept_volume;
            for(const auto &config: path)
            {
                s.robot.set(config.asMap());
                swept_volume.add(&s.robot.occupied_space);
            }
            PROFILE_RECORD("ComputeSweptVolume");
            precomputed_swept_volumes[arc_dijkstras::getHashable(e)] = swept_volume;
        }

        ProbGrid getSweptVolume(State &s, arc_dijkstras::GraphEdge &e)
        {
            PROFILE_START("GetSweptVolume");
            if(!precomputed_swept_volumes.count(arc_dijkstras::getHashable(e)))
            {
                computeSweptVolume(s, e);
            }
            
            PROFILE_RECORD("GetSweptVolume");
            return precomputed_swept_volumes[arc_dijkstras::getHashable(e)];
        }


        double calcEdgeProbability(State &s, arc_dijkstras::GraphEdge &e)
        {
            return s.calcProbFree(getSweptVolume(s, e));
        }

        
        /* Checks an edge against known obstacles to see if there is a collision.
         * Changes the graph edge validity accordingly
         */
        bool checkEdge(arc_dijkstras::GraphEdge &e, State &s)
        {
            bool valid = !getSweptVolume(s, e).overlapsWith(&s.known_obstacles);
            e.SetValidity(valid ? arc_dijkstras::EDGE_VALIDITY::VALID :
                          arc_dijkstras::EDGE_VALIDITY::INVALID);
            return valid;
        }
        


        double evaluateEdge(arc_dijkstras::GraphEdge &e,
                            State &s)
        {
            if(e.GetValidity() == arc_dijkstras::EDGE_VALIDITY::UNKNOWN)
            {
                checkEdge(e, s);
            }
            return calculateEdgeWeight(s, e);
        }


        std::vector<NodeIndex> lazySp(NodeIndex start, NodeIndex goal, State &s)
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
    };


    
    class OptimisticGraphSearch : public GraphSearchStrategy
    {
    public:        

        OptimisticGraphSearch(const std::string &filename) : GraphSearchStrategy(filename) {}

        virtual double calculateEdgeWeight(State &s, arc_dijkstras::GraphEdge &e) override
        {
            return e.GetWeight();
        }

    };        



    class ParetoCostGraphSearch : public GraphSearchStrategy
    {
    public:        
        double alpha = 1.0;
        ParetoCostGraphSearch(const std::string &filename) : GraphSearchStrategy(filename) {}

        
        virtual double calculateEdgeWeight(State &s, arc_dijkstras::GraphEdge &e) override
        {
            double p_cost = -std::log(calcEdgeProbability(s, e));
            double l_cost = e.GetWeight();
            return l_cost + alpha * p_cost;
        }
    };        
}

#endif


