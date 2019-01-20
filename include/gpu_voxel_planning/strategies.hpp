#ifndef STRATEGIES_HPP
#define STRATEGIES_HPP

#include "victor_halton_roadmap.hpp"
#include "scenarios.hpp"
#include "path_utils_addons.hpp"
#include "gvp_graph_search.hpp"

namespace GVP
{
    class Strategy
    {
    public:
        virtual GVP::Path applyTo(Scenario &scenario) = 0;
    };


    class GraphSearchStrategy : public Strategy
    {
    public:        
        Roadmap graph;
        NodeIndex prev_node;
        NodeIndex cur_node;
        NodeIndex goal_node;
        bool initialized;

        GraphSearchStrategy(const std::string &filename) : graph(filename), initialized(false) {}

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
                std::vector<NodeIndex> node_path = planPath(cur_node, goal_node, graph, scenario.getState());
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

            return interpolate(current, next, 0.02);

        }
    };
}

#endif
