#include "strategies/graph_search_strategies.hpp"



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
        try
        {
            precomputed_swept_volumes.loadFromFile(swept_volumes_filepath, graph.getNodes().size());
        }
        catch(const std::runtime_error& e)
        {
            std::cout << "Could not load precomputed swept volumes from file\n";
        }
    }
    
    GraphSearchStrategy::GraphSearchStrategy(const std::string &graph_filepath) :
        swept_volumes_filepath(""), graph_filepath(graph_filepath), 
        graph(graph_filepath), initialized(false) {}

    GraphSearchStrategy::GraphSearchStrategy() :
        // GraphSearchStrategy("/home/bradsaund/catkin_ws/src/gpu_voxel_planning/graphs/halton_100k.graph",
        //                     "/home/bradsaund/catkin_ws/src/gpu_voxel_planning/graphs/swept_volumes_100k.map"){}
        GraphSearchStrategy("/home/bradsaund/catkin_ws/src/gpu_voxel_planning/graphs/halton_100k.graph",
                            "/home/bradsaund/catkin_ws/src/gpu_voxel_planning/graphs/swept_volumes_100k.map"){}
        

    void GraphSearchStrategy::initialize(const Scenario &scenario)
    {
        cur_node = graph.addVertexAndEdges(scenario.getState().getCurConfig().asVector());
        goal_node = graph.addVertexAndEdges(VictorRightArmConfig(scenario.goal_config).asVector());
        std::cout << "Initial (node " << cur_node << ") and Goal (node " << goal_node << ") vertices added\n";
        initialized = true;
    }            


    Path GraphSearchStrategy::applyTo(Scenario &scenario, GpuVoxelRvizVisualizer& viz)
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
            std::vector<NodeIndex> node_path = plan(cur_node, goal_node, scenario.getState(), viz);
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

    std::vector<NodeIndex> GraphSearchStrategy::plan(NodeIndex start, NodeIndex goal, State &s,
                                                     GpuVoxelRvizVisualizer& viz)
    {
        return lazySp(start, goal, s);
    }


    DenseGrid GraphSearchStrategy::computeSweptVolume(State &s, arc_dijkstras::GraphEdge &e)
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
        return swept_volume;
    }

    void GraphSearchStrategy::storeSweptVolume(const arc_dijkstras::GraphEdge &e,
                                               const DenseGrid &swept_volume)
    {
        precomputed_swept_volumes[arc_dijkstras::getSortedHashable(e)] = swept_volume;
    }


    DenseGrid GraphSearchStrategy::getSweptVolume(State &s, arc_dijkstras::GraphEdge &e)
    {
        PROFILE_START("GetSweptVolume");
        arc_dijkstras::HashableEdge e_hashed = arc_dijkstras::getSortedHashable(e);
        if(!precomputed_swept_volumes.count(e_hashed))
        {
            storeSweptVolume(e, computeSweptVolume(s, e));
        }
            
        PROFILE_RECORD("GetSweptVolume");
        return DenseGrid(precomputed_swept_volumes[e_hashed]);
    }





    bool GraphSearchStrategy::checkEdge(arc_dijkstras::GraphEdge &e, State &s)
    {
        PROFILE_START("EdgeCheck");
        bool valid = !getSweptVolume(s, e).overlapsWith(&s.known_obstacles);
        e.setValidity(valid ? arc_dijkstras::EDGE_VALIDITY::VALID :
                      arc_dijkstras::EDGE_VALIDITY::INVALID);
        PROFILE_RECORD("EdgeCheck");
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

        const auto heuristic_fn =
            [] (std::vector<double> q1, std::vector<double> q2)
            {
                return EigenHelpers::Distance(q1, q2);
            };
        
        auto result = arc_dijkstras::LazySP<std::vector<double>>::PerformBiLazySP(
            graph, start, goal, heuristic_fn, eval_fn);
        if(result.second == std::numeric_limits<double>::infinity())
        {
            std::cout << "No path found on graph\n";
        }
        PROFILE_RECORD_DOUBLE("lazySP path cost ", result.second);
        std::cout << "LazySP path cost " << result.second << "\n";
        return result.first;
    }


    void GraphSearchStrategy::saveToFile(std::string filename)
    {
        precomputed_swept_volumes.saveToFile(filename);
    }





    /**********************************
     **  Omniscient Graph Search
     ********************************/
    double OmniscientGraphSearch::calculateEdgeWeight(State &s, arc_dijkstras::GraphEdge &e)
    {
        return e.getWeight();
    }

    std::string OmniscientGraphSearch::getName() const
    {
        return "Omniscient Graph Search";
    }
    
    Path OmniscientGraphSearch::applyTo(Scenario &scenario, GpuVoxelRvizVisualizer& viz)
    {
        if(!initialized)
        {
            initialize(scenario);
        }

        const VictorRightArmConfig &current(scenario.getState().getCurConfig());
        VictorRightArmConfig expected(graph.getNode(cur_node).getValue());
        VictorRightArmConfig next;

        Path path;
        std::vector<NodeIndex> node_path = plan(cur_node, goal_node, scenario.getState(), viz);

        if(node_path.size() <= 1)
        {
            std::cerr << "Path of less than 2 nodes found\n";
            assert(false);
        }
        
        for(size_t i=0; i<node_path.size()-1; i++)
        {
            Path segment = interpolate(graph.getNode(node_path[i]).getValue(),
                                       graph.getNode(node_path[i+1]).getValue(),
                                       discretization);
                                       
            path.insert(path.end(), segment.begin(), segment.end());
        }
        return path;
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
    /* This is just a Best First search using the the collision measure belief with additional term
     *  Should be implemented in the belief
     */ 
    // double UnknownSpaceCostGraphSearch::calculateEdgeWeight(State &s, arc_dijkstras::GraphEdge &e)
    // {
    //     DenseGrid sv = getSweptVolume(s, e);
    //     double edge_probability = s.calcProbFree(sv);

    //     double unknown_vox = sv.countOccupied() - sv.collideWith(&s.known_free);
    //     double p_cost = -alpha*std::log(edge_probability);
    //     double l_cost = e.getWeight();
    //     double unknown_cost = free_cost * unknown_vox;
    //     std::cout << "Edge traverses " << unknown_cost << " unknown voxels\n";
    //     return l_cost + p_cost + unknown_cost;
    // }

    // std::string UnknownSpaceCostGraphSearch::getName() const
    // {
    //     std::stringstream ss;
    //     ss << "UnknownSpaceCost Graph Search: cost = " << free_cost;
    //     return ss.str();
    // }



    
    /********************************
     **   AStar Graph Search
     *******************************/
    double AStarGraphSearch::calculateEdgeWeight(State &s, arc_dijkstras::GraphEdge &e)
    {
        return e.getWeight();
    }

    std::string AStarGraphSearch::getName() const
    {
        return "AStar Optimistic Graph Search";
    }
    
    std::vector<NodeIndex> AStarGraphSearch::plan(NodeIndex start, NodeIndex goal, State &s,
                                                  GpuVoxelRvizVisualizer& viz)
    {
        using namespace arc_dijkstras;
        const auto edge_validity_check_fn =
            [&] (Graph<std::vector<double>> &g, GraphEdge &e)
            {
                return checkEdge(e, s);
            };
        const auto distance_fn = [&] (const Graph<std::vector<double>>& search_graph, 
                                      const GraphEdge& edge)
            {
                UNUSED(search_graph);
                return edge.getWeight();
            };

        auto result = AstarLogging<std::vector<double>>::PerformLazyAstar(
            graph, start, goal, edge_validity_check_fn, distance_fn, &distanceHeuristic, true);
        if(result.second == std::numeric_limits<double>::infinity())
        {
            std::cout << "No path found on graph\n";
        }
        return result.first;
    }

    DenseGrid AStarGraphSearch::getSweptVolume(State &s, arc_dijkstras::GraphEdge &e)
    {
        return computeSweptVolume(s, e);
    }



    /***********************************
     **             HOP               **
     **  Averaging over Clairvoyance  **
     **********************************/
    double HOPGraphSearch::calculateEdgeWeight(State &s, arc_dijkstras::GraphEdge &e)
    {
        if(e.getValidity() == arc_dijkstras::EDGE_VALIDITY::INVALID)
        {
            return std::numeric_limits<double>::infinity();
        }
        return e.getWeight();
    }

    std::vector<NodeIndex> HOPGraphSearch::plan(NodeIndex start, NodeIndex goal, State &s,
                                                GpuVoxelRvizVisualizer& viz)
    {
        
        std::map<NodeIndex, double> actions;
        using pair_type = decltype(actions)::value_type;
        Roadmap orig_graph = graph;

        for(int i=0; i<num_samples; i++)
        {
            std::cout << "Checking sample " << i << "\n\n";
            graph = orig_graph;
            // DenseGrid g1 = s.bel->sampleState();
            // g1.merge(&s.known_obstacles);
            
            // viz.grid_pub.publish(visualizeDenseGrid(g1, viz.global_frame,
            //                                         "sampled_world", makeColor(0, 0, 1.0, 1.0)));
            
            
            State sampled_state(s.robot);
            sampled_state.robot_self_collide_obstacles = s.robot_self_collide_obstacles;
            sampled_state.known_obstacles = s.bel->sampleState();
            sampled_state.known_obstacles.add(&s.known_obstacles);
            viz.vizGrid(sampled_state.known_obstacles, "sampled_world", makeColor(0, 0, 1.0, 1.0));

            VictorRightArmConfig goal_config(graph.getNode(goal).getValue());
            sampled_state.robot.set(goal_config.asMap());
            if(sampled_state.robot.occupied_space.overlapsWith(&s.known_obstacles))
            {
                continue;
            }

            auto result = lazySp(start, goal, sampled_state);

            if(result.size() < 2)
            {
                continue;
            }

            viz.vizEEPath(interpolate(s.getCurConfig(), graph.getNode(result[1]).getValue(), 0.1),
                          "sampledPath");

            NodeIndex a = result[1];
            std::cout << "Best action: " << a << "\n";
            if(actions.count(a) == 0)
            {
                actions[a] = 0;
            }
            actions[a] += 1.0;

            DenseGrid sv = getSweptVolume(s, graph.getEdge(start, a));
            viz.grid_pub.publish(visualizeDenseGrid(sv, viz.global_frame,
                                                    "swept volume", makeColor(1, 1, 0, 0.7)));
            std::cout << "Edge eval: " << evaluateEdge(graph.getEdge(start, a), sampled_state) << "\n";
            std::cout << "Edge Check: " << checkEdge(graph.getEdge(start, a), sampled_state) << "\n";
            // arc_helpers::WaitForInput();

        }
        graph = orig_graph;

        for(auto it: actions)
        {
            std::cout << it.first << ": " << it.second << "\n";
        }


        auto pr = std::max_element(actions.begin(), actions.end(),
                                   [](const pair_type &p1, const pair_type &p2)
                                   {return p1.second < p2.second;});
        return std::vector<NodeIndex>{start, pr->first};

    }

    std::string HOPGraphSearch::getName() const
    {
        return "HOP Graph Search";
    }
    
    

}
