#include "graph_search_strategies.hpp"



// using namespace GVP;
namespace GVP
{

    GraphSearchStrategy::GraphSearchStrategy(const std::string &graph_filepath,
                                             const std::string& swept_volumes_filepath) :
        swept_volumes_filepath(swept_volumes_filepath),
        graph_filepath(graph_filepath),
        graph(graph_filepath),
        initialized(false)
    {
        precomputed_swept_volumes.loadFromFile(swept_volumes_filepath, graph.getNodes().size());
    }
    
    GraphSearchStrategy::GraphSearchStrategy(const std::string &graph_filepath) :
        swept_volumes_filepath(""), graph_filepath(graph_filepath), 
        graph(graph_filepath), initialized(false) {}

    GraphSearchStrategy::GraphSearchStrategy() :
        GraphSearchStrategy("/home/bradsaund/catkin_ws/src/gpu_voxel_planning/graphs/halton_100k.graph",
                            "/home/bradsaund/catkin_ws/src/gpu_voxel_planning/graphs/swept_volumes_100k.map"){}
        

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
        VictorRightArmConfig expected(graph.getNode(cur_node).getValue());
        VictorRightArmConfig next;
            
        if(current == expected)
        {
            std::vector<NodeIndex> node_path = lazySp(cur_node, goal_node, scenario.getState());
            next = graph.getNode(node_path[1]).getValue();
            prev_node = cur_node;
            cur_node = node_path[1];
        }
        else
        {
            next = graph.getNode(prev_node).getValue();
            graph.getEdge(prev_node, cur_node).setValidity(arc_dijkstras::EDGE_VALIDITY::INVALID);
                
            cur_node = prev_node;
        }

        return interpolate(current, next, discretization);
    }



    void GraphSearchStrategy::computeSweptVolume(State &s, arc_dijkstras::GraphEdge &e)
    {
        PROFILE_START("ComputeSweptVolume");
        VictorRightArmConfig q_start(graph.getNode(e.getFromIndex()).getValue());
        VictorRightArmConfig q_end(graph.getNode(e.getToIndex()).getValue());
        GVP::Path path = interpolate(q_start, q_end, discretization);

        DenseGrid swept_volume;
        for(const auto &config: path)
        {
            s.robot.set(config.asMap());
            swept_volume.add(&s.robot.occupied_space);
        }
        PROFILE_RECORD("ComputeSweptVolume");
        precomputed_swept_volumes[arc_dijkstras::getSortedHashable(e)] = swept_volume;
    }


    DenseGrid GraphSearchStrategy::getSweptVolume(State &s, arc_dijkstras::GraphEdge &e)
    {
        PROFILE_START("GetSweptVolume");
        arc_dijkstras::HashableEdge e_hashed = arc_dijkstras::getSortedHashable(e);
        if(!precomputed_swept_volumes.count(e_hashed))
        {
            computeSweptVolume(s, e);
        }
            
        PROFILE_RECORD("GetSweptVolume");
        return DenseGrid(precomputed_swept_volumes[e_hashed]);
    }





    bool GraphSearchStrategy::checkEdge(arc_dijkstras::GraphEdge &e, State &s)
    {
        bool valid = !getSweptVolume(s, e).overlapsWith(&s.known_obstacles);
        e.setValidity(valid ? arc_dijkstras::EDGE_VALIDITY::VALID :
                      arc_dijkstras::EDGE_VALIDITY::INVALID);
        return valid;
    }

    double GraphSearchStrategy::evaluateEdge(arc_dijkstras::GraphEdge &e, State &s)
    {
        if(e.getValidity() == arc_dijkstras::EDGE_VALIDITY::UNKNOWN)
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
        if(result.second == std::numeric_limits<double>::infinity())
        {
            std::cout << "No path found on graph\n";
        }
        return result.first;
    }


    void GraphSearchStrategy::saveToFile(std::string filename)
    {
        precomputed_swept_volumes.saveToFile(filename);
    }





    /**********************************
     **  Optimistic Graph Search
     ********************************/
    double OptimisticGraphSearch::calculateEdgeWeight(State &s, arc_dijkstras::GraphEdge &e)
    {
        return e.getWeight();
    }


    std::string OptimisticGraphSearch::getName() const
    {
        return "Optimistic Graph Search";
    }


     /**********************************
     **  ParetoCost Graph Search
     ********************************/
    double ParetoCostGraphSearch::calculateEdgeWeight(State &s, arc_dijkstras::GraphEdge &e)
    {
        double edge_probability = s.calcProbFree(getSweptVolume(s, e));
        double p_cost = -std::log(edge_probability);
        double l_cost = e.getWeight();
        return l_cost + alpha * p_cost;
    }

    std::string ParetoCostGraphSearch::getName() const
    {
        std::stringstream ss;
        ss << "ParetoCost Graph Search: alpha = " << alpha;
        return ss.str();
    }



    
    /************************************
     **  UnknownSpaceCost Graph Search
     ***********************************/
    double UnknownSpaceCostGraphSearch::calculateEdgeWeight(State &s, arc_dijkstras::GraphEdge &e)
    {
        DenseGrid sv = getSweptVolume(s, e);
        double edge_probability = s.calcProbFree(sv);

        double unknown_vox = sv.countOccupied() - sv.collideWith(&s.known_free);
        double p_cost = -alpha*std::log(edge_probability);
        double l_cost = e.getWeight();
        double unknown_cost = free_cost * unknown_vox;
        std::cout << "Edge traverses " << unknown_cost << " unknown voxels\n";
        return l_cost + p_cost + unknown_cost;
    }

    std::string UnknownSpaceCostGraphSearch::getName() const
    {
        std::stringstream ss;
        ss << "UnknownSpaceCost Graph Search: cost = " << free_cost;
        return ss.str();
    }

}
