#include "strategies/graph_search_strategies.hpp"
#include "robot/robot_helpers.hpp"



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
            std::cout << "Loading swept volumes...";
            precomputed_swept_volumes.loadFromFile(swept_volumes_filepath, graph.getNodes().size());
            std::cout << "loaded\n";
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
        // GraphSearchStrategy("/home/bradsaund/catkin_ws/src/gpu_voxel_planning/graphs/halton_100k.graph",
        //                     "/home/bradsaund/catkin_ws/src/gpu_voxel_planning/graphs/swept_volumes_100k.map"){}
        GraphSearchStrategy("/home/bradsaund/catkin_ws/src/gpu_voxel_planning/graphs/halton_10k.graph",
                            "/home/bradsaund/catkin_ws/src/gpu_voxel_planning/graphs/swept_volumes_10k.map"){}
        

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
            graph.getEdge(cur_node, prev_node).setValidity(arc_dijkstras::EDGE_VALIDITY::INVALID);
                
            cur_node = prev_node;
        }

        return interpolate(current, next, discretization);
    }

    std::vector<NodeIndex> GraphSearchStrategy::plan(NodeIndex start, NodeIndex goal, State &s,
                                                     GpuVoxelRvizVisualizer& viz)
    {
        return lazySp(start, goal, s, graph);
    }


    DenseGrid GraphSearchStrategy::computeSweptVolume(State &s, const arc_dijkstras::GraphEdge &e)
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


    DenseGrid GraphSearchStrategy::getSweptVolume(State &s, const arc_dijkstras::GraphEdge &e)
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

    std::vector<NodeIndex> GraphSearchStrategy::lazySp(NodeIndex start, NodeIndex goal, State &s,
                                                       Roadmap &rm)
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

        PROFILE_START("lazysp_successful");
        PROFILE_START("lazysp_no_path_found");
        
        auto result = arc_dijkstras::LazySP<std::vector<double>>::PerformBiLazySP(
            rm, start, goal, heuristic_fn, eval_fn);
        if(result.second == std::numeric_limits<double>::infinity())
        {
            std::cout << "No path found on graph\n";
        }
        PROFILE_RECORD_DOUBLE("lazySP path cost ", result.second);
        std::cout << "LazySP path cost " << result.second << "\n";

        if(result.second < std::numeric_limits<double>::max())
        {
            PROFILE_RECORD("lazysp_successful");
        }
        else
        {
            PROFILE_RECORD("lazysp_no_path_found");
        }
        
        return result.first;
    }


    void GraphSearchStrategy::saveToFile(std::string filename)
    {
        precomputed_swept_volumes.saveToFile(filename);
    }





    /**********************************
     **  Omniscient Graph Search
     ********************************/
    double OmniscientGraphSearch::calculateEdgeWeight(State &s, const arc_dijkstras::GraphEdge &e)
    {
        return e.getWeight();
    }

    std::string OmniscientGraphSearch::getName() const
    {
        return "Omniscient";
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
    double OptimisticGraphSearch::calculateEdgeWeight(State &s, const arc_dijkstras::GraphEdge &e)
    {
        if(s.bel->calcProbFree(getSweptVolume(s, e)) > 0)
        {
            return e.getWeight();
        }
        return std::numeric_limits<double>::infinity();
    }

    std::string OptimisticGraphSearch::getName() const
    {
        return "Optimistic";
    }

    
     /**********************************
     **  ParetoCost Graph Search
     ********************************/
    double ParetoCostGraphSearch::calculateEdgeWeight(State &s, const arc_dijkstras::GraphEdge &e)
    {
        double edge_probability = s.calcProbFree(getSweptVolume(s, e));
        double p_cost = -std::log(edge_probability);
        double l_cost = e.getWeight();
        return l_cost + alpha * p_cost;
    }

    std::string ParetoCostGraphSearch::getName() const
    {
        std::stringstream ss;
        ss << "ParetoCosta" << alpha;
        return ss.str();
    }



    
    /************************************
     **  UnknownSpaceCost Graph Search
     ***********************************/
    /* This is just a Best First search using the the collision measure belief with additional term
     *  Should be implemented in the belief
     */ 
    // double UnknownSpaceCostGraphSearch::calculateEdgeWeight(State &s, const arc_dijkstras::GraphEdge &e)
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
    double AStarGraphSearch::calculateEdgeWeight(State &s, const arc_dijkstras::GraphEdge &e)
    {
        return e.getWeight();
    }

    std::string AStarGraphSearch::getName() const
    {
        return "AStar Optimistic";
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

    DenseGrid AStarGraphSearch::getSweptVolume(State &s, const arc_dijkstras::GraphEdge &e)
    {
        return computeSweptVolume(s, e);
    }






    /***********************************
     **        Thompson               **
     **********************************/
    double ThompsonGraphSearch::calculateEdgeWeight(State &s, const arc_dijkstras::GraphEdge &e)
    {
        if(e.isInvalid())
        {
            return std::numeric_limits<double>::infinity();
        }
        return e.getWeight();
    }


    State ThompsonGraphSearch::sampleValidState()
    {
        
    }

    std::vector<NodeIndex> ThompsonGraphSearch::plan(NodeIndex start, NodeIndex goal, State &s,
                                                GpuVoxelRvizVisualizer& viz)
    {
        PROFILE_START("Thompson_plan");

        while(true)
        {
            Roadmap graph_cpy = graph;
            State sampled_state = s.sample();
            
            PROFILE_START("Viz_sample");
            viz.vizGrid(sampled_state.known_obstacles, "sampled_world", makeColor(0, 0, 1.0, 1.0));
            PROFILE_RECORD("Viz_sample");
            
            VictorRightArmConfig goal_config(graph.getNode(goal).getValue());
            sampled_state.robot.set(goal_config.asMap());
            if(sampled_state.robot.occupied_space.overlapsWith(&sampled_state.known_obstacles))
            {
                continue;
            }

            auto result = lazySp(start, goal, sampled_state, graph_cpy);

            if(result.size() < 2)
            {
                continue;
            }

            PROFILE_START("Viz_sample_ee_path");
            viz.vizEEPath(interpolate(s.getCurConfig(), graph.getNode(result[1]).getValue(), 0.1),
                          "sampledPath", 0, makeColor(0.0, 0.0, 1.0));
            PROFILE_RECORD("Viz_sample_ee_path");
            return result;
                
        }

    }

    std::string ThompsonGraphSearch::getName() const
    {
        return "Thompson";
    }







    /***********************************
     **             HOP               **
     **  Averaging over Clairvoyance  **
     **********************************/
    double HOPGraphSearch::calculateEdgeWeight(State &s, const arc_dijkstras::GraphEdge &e)
    {
        if(e.isInvalid())
        {
            return std::numeric_limits<double>::infinity();
        }
        return e.getWeight();
    }

    std::vector<NodeIndex> HOPGraphSearch::plan(NodeIndex start, NodeIndex goal, State &s,
                                                GpuVoxelRvizVisualizer& viz)
    {
        PROFILE_START("HOP_plan");
        std::map<NodeIndex, double> actions;
        using pair_type = decltype(actions)::value_type;
        Roadmap graph_cpy = graph;

        for(int i=0; i<num_samples; i++)
        {
            // std::cout << "Checking sample " << i << "\n\n";

            State sampled_state = s.sample();
            
            PROFILE_START("Viz_sample");
            viz.vizGrid(sampled_state.known_obstacles, "sampled_world", makeColor(0, 0, 1.0, 1.0));
            PROFILE_RECORD("Viz_sample");

            VictorRightArmConfig goal_config(graph.getNode(goal).getValue());
            sampled_state.robot.set(goal_config.asMap());
            if(sampled_state.robot.occupied_space.overlapsWith(&sampled_state.known_obstacles))
            {
                continue;
            }

            PROFILE_START("Copy_graph");
            graph_cpy = graph;
            PROFILE_RECORD("Copy_graph");

            auto result = lazySp(start, goal, sampled_state, graph_cpy);

            if(result.size() < 2)
            {
                continue;
            }

            PROFILE_START("Viz_sample_ee_path");
            viz.vizEEPath(interpolate(s.getCurConfig(), graph.getNode(result[1]).getValue(), 0.1),
                          "sampledPath", 0, makeColor(0.0, 0.0, 1.0));
            PROFILE_RECORD("Viz_sample_ee_path");

            NodeIndex a = result[1];
            // std::cout << "Best action: " << a << "\n";
            if(actions.count(a) == 0)
            {
                actions[a] = 0;
            }
            actions[a] += 1.0;

            PROFILE_START("Viz sample sv");
            DenseGrid sv = getSweptVolume(s, graph.getEdge(start, a));
            viz.vizGrid(sv, "swept volume", makeColor(1, 1, 0, 0.7));
            PROFILE_RECORD("Viz sample sv");
            // std::cout << "Edge eval: " << evaluateEdge(graph.getEdge(start, a), sampled_state) << "\n";
            // std::cout << "Edge Check: " << checkEdge(graph.getEdge(start, a), sampled_state) << "\n";
            // arc_helpers::WaitForInput();

        }

        for(auto it: actions)
        {
            std::cout << it.first << ": " << it.second << "\n";
        }


        auto pr = std::max_element(actions.begin(), actions.end(),
                                   [](const pair_type &p1, const pair_type &p2)
                                   {return p1.second < p2.second;});
        PROFILE_RECORD("HOP_plan");
        return std::vector<NodeIndex>{start, pr->first};

    }

    std::string HOPGraphSearch::getName() const
    {
        return "HOP";
    }



    /***********************************
     **             ORO               **
     **      Optimistic Rollout       **
     **********************************/
    double OROGraphSearch::calculateEdgeWeight(State &s, const arc_dijkstras::GraphEdge &e)
    {
        if(e.isInvalid())
        {
            return std::numeric_limits<double>::infinity();
        }
        if(s.calcProbFree(getSweptVolume(s, e)) <= 0)
        {
            return std::numeric_limits<double>::infinity();
        }
        return e.getWeight();
    }

    bool OROGraphSearch::pathExists(NodeIndex start, NodeIndex goal, State &s)
    {
        VictorRightArmConfig goal_config(graph.getNode(goal).getValue());
        s.robot.set(goal_config.asMap());
        if(s.robot.occupied_space.overlapsWith(&s.known_obstacles))
        {
            return false;
        }

        std::cout << "Checking if path exists\n";
        arc_dijkstras::EvaluatedEdges invalid_edges;
        // auto path = lazySp(start, goal, s, graph);
        auto path = lazySpForRollout(start, goal, s, graph, invalid_edges);
        std::cout << "Check complete\n";
        std::cout << "Path is " << PrettyPrint::PrettyPrint(path) << "\n";
        std::cout << "Path size is " << path.size() << "\n";
        return path.size() > 0;
    }

    std::vector<NodeIndex> OROGraphSearch::lazySpForRollout(NodeIndex start, NodeIndex goal, State &s,
                                                            Roadmap &rm,
                                                            arc_dijkstras::EvaluatedEdges& additional_invalid)
    {
        const auto eval_fn =
            [&] (const arc_dijkstras::Graph<std::vector<double>> &g, const arc_dijkstras::GraphEdge &e)
            {
                auto hashed_edge = arc_dijkstras::getSortedHashable(e);
                if(additional_invalid.count(hashed_edge))
                {
                    return std::numeric_limits<double>::infinity();
                }

                if(getSweptVolume(s, e).overlapsWith(&s.known_obstacles))
                {
                    additional_invalid[hashed_edge] = std::numeric_limits<double>::infinity();
                    return std::numeric_limits<double>::infinity();
                }

                return calculateEdgeWeight(s, e);
            };

        const auto heuristic_fn =
            [] (const std::vector<double>& q1, const std::vector<double>& q2)
            {
                return EigenHelpers::Distance(q1, q2);
            };

        PROFILE_START("lazysp_successful");
        PROFILE_START("lazysp_no_path_found");

        auto result = arc_dijkstras::LazySP<std::vector<double>>::PerformBiLazySP(
            rm, start, goal, heuristic_fn, eval_fn);
        if(result.second == std::numeric_limits<double>::infinity())
        {
            std::cout << "No path found on graph\n";
            std::cout << "LazySP path size " << result.first.size() << "\n";
        }
        PROFILE_RECORD_DOUBLE("lazySP path cost ", result.second);
        std::cout << "LazySP for Rollout path cost " << result.second << "\n";

        if(result.second < std::numeric_limits<double>::max())
        {
            PROFILE_RECORD("lazysp_successful");
        }
        else
        {
            PROFILE_RECORD("lazysp_no_path_found");
        }
        
        return result.first;
    }


    std::vector<NodeIndex> OROGraphSearch::getPossibleActions(State& state, NodeIndex cur,
        GpuVoxelRvizVisualizer& viz)
    {
        std::vector<NodeIndex> possible_actions;
        for(const auto& e: graph.getNode(cur).getOutEdges())
        {
            DenseGrid sv = getSweptVolume(state, e);

            if(calculateEdgeWeight(state, e) >= std::numeric_limits<double>::max())
            {
                
                // viz.vizGrid(sv, "Possible edge to try", makeColor(1.0, 0.5, 0.5));
                // std::cout << "This edge is invalid\n";
                // arc_helpers::WaitForInput();
                continue;
            }
            // viz.vizGrid(sv, "Possible edge to try", makeColor(1.0, 0.5, 0.5));
            // std::cout << "Possibly valid action\n";
            // arc_helpers::WaitForInput();

            possible_actions.push_back(e.getToIndex());
        }
        return possible_actions;
    }

    double OROGraphSearch::simulateTransition(State& state, const Roadmap& rm, const DenseGrid& occupied,
                                              NodeIndex& cur, NodeIndex next,
                                              arc_dijkstras::EvaluatedEdges& additional_invalid,
                                              GpuVoxelRvizVisualizer& viz)
    {
        // std::cout << "Simulating Transition\n";
        const auto& e = rm.getEdge(cur, next);
        // std::cout << "Edge " << e << " is invalid? " << e.isInvalid() << "\n";
        // if(checkEdge(e, state))


        DenseGrid sv = getSweptVolume(state, e);
        viz.vizGrid(sv, "transition", makeColor(1, 0.5, 0.5, 0.7));
        if(!sv.overlapsWith(&occupied))
        {
            PROFILE_START("simulate_belief_update_free");
            state.bel->updateFreeSpace(getSweptVolume(state, e));
            PROFILE_RECORD("simulate_belief_update_free");
            const auto q1 = rm.getNode(cur).getValue();
            const auto q2 = rm.getNode(next).getValue();
            cur = next;
            return EigenHelpers::Distance(q1, q2);;
        }

        // std::cout << "Simulated transition collided\n";
        
        const VictorRightArmConfig c1(rm.getNode(cur).getValue());
        const VictorRightArmConfig c2(rm.getNode(next).getValue());
        auto path = interpolate(c1, c2, discretization);


        double d = 0;
        for(const auto &c: path)
        {
            state.robot.set(c.asMap());
            if(state.robot.occupied_space.overlapsWith(&occupied))
            {
                PROFILE_START("simulate_belief_update_collision");
                state.bel->updateCollisionSpace(state.robot, getFirstLinkInCollision(state.robot, occupied));
                PROFILE_RECORD("simulate_belief_update_collision");
                additional_invalid[arc_dijkstras::getSortedHashable(e)] =
                    std::numeric_limits<double>::infinity();
                // e.setValidity(arc_dijkstras::EDGE_VALIDITY::INVALID);
                // rm.getReverseEdge(e).setValidity(arc_dijkstras::EDGE_VALIDITY::INVALID);

                state.bel->viz(viz);
                std::cout << "Transition resulted in a collision\n";
                // arc_helpers::WaitForInput();
                
                break;
            }

            PROFILE_START("simulate_belief_update_free_before_collision");
            state.bel->updateFreeSpace(state.robot.occupied_space);
            PROFILE_RECORD("simulate_belief_update_free_before_collision");
            d += discretization; // Approximation of true cost accumulated
        }
        
        return 2*d;
    }

    double OROGraphSearch::rolloutOptimistic(State& state, Roadmap& rm, const DenseGrid& occupied,
                                             NodeIndex cur, NodeIndex goal,
                                             arc_dijkstras::EvaluatedEdges& additional_invalid,
                                             GpuVoxelRvizVisualizer& viz)
    {

        PROFILE_START("Rollout");
        std::cout << "New rolling out\n";
        double rollout_cost = 0;
        while(cur != goal)
        {
            PROFILE_START("rollout lazysp");
            auto path = lazySpForRollout(cur, goal, state, rm, additional_invalid);
            PROFILE_RECORD("rollout lazysp");

            if(path.size() < 2)
            {
                std::cout << "No path found. Rollout failed\n";
                return std::numeric_limits<double>::infinity();
            }

            PROFILE_START("Rollout viz sv");
            DenseGrid sv = getSweptVolume(state, graph.getEdge(cur, path[1]));
            viz.vizGrid(sv, "swept volume", makeColor(1, 1, 0, 0.7));
            PROFILE_RECORD("Rollout viz sv");

            PROFILE_START("Simulate transition in rollout");
            rollout_cost += simulateTransition(state, rm, occupied, cur, path[1],
                                               additional_invalid,
                                               viz);
            PROFILE_RECORD("Simulate transition in rollout");
            // arc_helpers::WaitForInput();
            // if(cur == path[1])
            // {
            //     std::cout << "Transition succeeded\n";
            // }
            // else
            // {
            //     std::cout << "Transition " << cur << " -> " << path[1] <<  " failed\n";
            //     std::cout << "path size: " << path.size() << "\n";
            // }
        }

        std::cout << "Rollout reached goal\n";
        PROFILE_RECORD("Rollout");
        return rollout_cost;
    }

    std::vector<NodeIndex> OROGraphSearch::plan(NodeIndex start, NodeIndex goal, State &s,
                                                GpuVoxelRvizVisualizer& viz)
    {
        std::cout << "ORO plan\n";
        PROFILE_START("ORO_plan");
        std::map<NodeIndex, double> actions;
        using pair_type = decltype(actions)::value_type;

        auto all_actions = getPossibleActions(s, start, viz);
        std::cout << "There are " << all_actions.size() << " possible actions to try\n";

        if(all_actions.size() == 0)
        {
            throw std::logic_error("No possible valid actions to try next\n");
        }

        for(int i=0; i<num_samples; i++)
        {
            // std::cout << "Checking sample " << i << "\n\n";
            State sampled_state = s.sample();
            sampled_state.bel = s.bel->clone();

            PROFILE_START("Viz_sample");
            viz.vizGrid(sampled_state.known_obstacles, "sampled_world", makeColor(0, 0, 1.0, 1.0));
            PROFILE_RECORD("Viz_sample");

            if(!pathExists(start, goal, sampled_state))
            {
                std::cout << "Path check failed. Resampling\n";
                continue;
            }
            std::cout << "There is a valid path. Rolling out\n";

            for(auto initial_action: all_actions)
            {
                std::cout << "Initial action: " << initial_action << "\n";
                State rollout_state(s);
                rollout_state.bel = s.bel->clone();
                // PROFILE_START("Copy_graph");
                // Roadmap rollout_graph = graph;
                arc_dijkstras::EvaluatedEdges invalid_edges_during_rollout;
                // PROFILE_RECORD("Copy_graph");
                NodeIndex cur = start;

                PROFILE_START("Simulate transition first");
                double rollout_cost = simulateTransition(rollout_state, graph,
                                                         sampled_state.known_obstacles,
                                                         cur, initial_action,
                                                         invalid_edges_during_rollout,
                                                         viz);
                PROFILE_RECORD("Simulate transition first");

                rollout_cost += rolloutOptimistic(rollout_state, graph,
                                                  sampled_state.known_obstacles,
                                                  cur, goal, invalid_edges_during_rollout,
                                                  viz);

                if(actions.count(initial_action) == 0)
                {
                    actions[initial_action] = 0;
                }
                actions[initial_action] += rollout_cost;

            }

            
            


            // PROFILE_START("Viz_sample_ee_path");
            // viz.vizEEPath(interpolate(s.getCurConfig(), graph.getNode(result[1]).getValue(), 0.1),
            //               "sampledPath");
            // PROFILE_RECORD("Viz_sample_ee_path");


            // PROFILE_START("Viz sample sv");
            // DenseGrid sv = getSweptVolume(s, graph.getEdge(start, a));
            // viz.vizGrid(sv, "swept volume", makeColor(1, 1, 0, 0.7));
            // PROFILE_RECORD("Viz sample sv");
            // std::cout << "Edge eval: " << evaluateEdge(graph.getEdge(start, a), sampled_state) << "\n";
            // std::cout << "Edge Check: " << checkEdge(graph.getEdge(start, a), sampled_state) << "\n";
            // arc_helpers::WaitForInput();

        }

        for(auto it: actions)
        {
            std::cout << it.first << ": " << it.second << "\n";
        }


        auto pr = std::min_element(actions.begin(), actions.end(),
                                   [](const pair_type &p1, const pair_type &p2)
                                   {return p1.second < p2.second;});
        PROFILE_RECORD("ORO_plan");
        return std::vector<NodeIndex>{start, pr->first};

    }

    std::string OROGraphSearch::getName() const
    {
        return "ORO";
    }
}
