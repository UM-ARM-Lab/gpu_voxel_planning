#include "strategies/iterative_deepening_strategy.hpp"
#include "path_utils_addons.hpp"
#include <algorithm>
#include <random>
#include "gvp_exceptions.hpp"


const std::string basepath = "/home/bradsaund/catkin_ws/src/gpu_voxel_planning/graphs/ID/";

using namespace GVP;

IterativeDeepeningStrategy::IterativeDeepeningStrategy(const std::string &graph_filepath,
                                                       const std::string& swept_volumes_filepath) :
    
    LayeredGraphStrategy(graph_filepath, swept_volumes_filepath),
    id_graph(graph_filepath)
{
    try
    {
        precomputed_swept_volumes.loadFromFile(swept_volumes_filepath, id_graph.getNodes().size());
        std::cout << "Loaded " << precomputed_swept_volumes.size() << " swept volumes\n";
    }
    catch(const std::runtime_error& e)
    {
        std::cout << "Could not load precomputed swept volumes from file\n";
    }
}

        
void IterativeDeepeningStrategy::initialize(Scenario &scenario)
{
    addStartAndGoalToGraph(scenario);
    initialized = true;
}

void IterativeDeepeningStrategy::addStartAndGoalToGraph(const Scenario &scenario)
{
    int orig_edge_count = id_graph.countEdges();
    // for(int depth = 3; depth <= id_graph.depth; depth++)
    // for(int depth = 13; depth <= 14; depth++)
    int depth = 1;
    DepthNode start(depth, scenario.getState().getCurConfig().asVector());
    DepthNode goal(depth, VictorRightArmConfig(scenario.goal_config).asVector());
    NodeIndex start_id = id_graph.addVertexAndEdges(start);
    NodeIndex goal_id = id_graph.addVertexAndEdges(goal);

    std::cout << "Initial (node " << start_id << ") and Goal (node " << goal_id << ") vertices added\n";
    cur_node = start_id;
    goal_node = goal_id;
    std::cout << "Start and goal added " << id_graph.countEdges() - orig_edge_count << " edges\n";
}

std::vector<NodeIndex> IterativeDeepeningStrategy::plan(NodeIndex start, NodeIndex goal, State &s)
{
    std::vector<double> start_q = getNodeValue(start);
    std::vector<double> goal_q = getNodeValue(goal);

    for(int depth = 1; depth <= id_graph.depth; depth++)
    {
        DepthNode start_node(depth, start_q);
        DepthNode goal_node(depth, goal_q);
        NodeIndex start_id = id_graph.addVertexAndEdges(start_node);
        NodeIndex goal_id = id_graph.addVertexAndEdges(goal_node);        

        std::vector<NodeIndex> path = lazySp(start_id, goal_id, s);
        if(path.size() > 0)
        {
            std::cout << "Path found on layer " << depth << "\n";
            return path;
        }
    }
    
    std::cout << "No path found";
    throw SearchError("Path not found");
}

std::vector<NodeIndex> IterativeDeepeningStrategy::lazySp(NodeIndex start, NodeIndex goal, State &s)
{
    const auto eval_fn =
        [&] (arc_dijkstras::Graph<std::vector<double>> &g, arc_dijkstras::GraphEdge &e)
        {
            return evaluateEdge(e, s);
        };

    const auto heuristic_fn = [&] (const std::vector<double> &n1,
                                   const std::vector<double> &n2)
        {
            return distanceHeuristic(n1, n2);
        };

    auto selector =
        [&] (std::vector<int64_t> path,
             arc_dijkstras::Graph<std::vector<double>>& g,
             const arc_dijkstras::EvaluatedEdges &evaluatedEdges)
        {
            // return forwardPrecomputedSelector(path, g, evaluatedEdges);
            return arc_dijkstras::LazySP<std::vector<double>>::ForwardSelector(path, g, evaluatedEdges);
            // return arc_dijkstras::LazySP<std::vector<double>>::BisectionSelector(path, g, evaluatedEdges);
        };
        
    
    std::cout << "Performing lazysp\n";
    auto result = arc_dijkstras::LazySP<std::vector<double>>::PerformBiLazySP(
        id_graph, start, goal, heuristic_fn, eval_fn, selector);
    // auto result = arc_dijkstras::LazySP<std::vector<double>>::PerformLazySP(
    //     id_graph, goal, start, heuristic_fn, eval_fn, selector);
    // std::reverse(result.first.begin(), result.first.end());

    std::cout << "LazySP finished\n";
    
    if(result.second == std::numeric_limits<double>::infinity())
    {
        std::cout << "No path found on graph\n";
    } else
    {
        std::cout << "Path: " << PrettyPrint::PrettyPrint(result.first) << "\n";
    }
    
    
    PROFILE_RECORD_DOUBLE("lazySP path cost ", result.second);
    std::cout << "LazySP path cost " << result.second << "\n";

    return result.first;
}


double IterativeDeepeningStrategy::distanceHeuristic(const std::vector<double> &raw1,
                                                     const std::vector<double> &raw2) const
{

    DepthNode d1(raw1);
    DepthNode d2(raw2);
    std::string depth_logging_name = "EdgeHeuristic depth=" +
        std::to_string(d1.depth);
    PROFILE_START(depth_logging_name);
    PROFILE_RECORD(depth_logging_name);

    return EigenHelpers::Distance(d1.q, d2.q);
}





IDSearch::IDSearch(bool use_precomputed, int graph_num) :
    IterativeDeepeningStrategy(basepath + "seed" + std::to_string(graph_num) + ".graph",
                               use_precomputed ? basepath + "sv" + std::to_string(graph_num) + ".map" : ""),
    use_precomputed(use_precomputed)
{
}

std::string IDSearch::getName() const
{
    std::string type = (use_precomputed ? "_precomputed" : "");
    return "Iterative_Deepening" + type;
}

double IDSearch::calculateEdgeWeight(State &s, arc_dijkstras::GraphEdge &e)
{
    return e.getWeight();
}

