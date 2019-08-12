#include "strategies/selective_densification_strategies.hpp"
#include "path_utils_addons.hpp"
#include <algorithm>
#include <random>
#include "gvp_exceptions.hpp"


const std::string basepath = "/home/bradsaund/catkin_ws/src/gpu_voxel_planning/graphs/SD/";

using namespace GVP;

SelectiveDensificationStrategy::SelectiveDensificationStrategy(const std::string &graph_filepath,
                                                               const std::string& swept_volumes_filepath) :
    initialized(false),
    graph_filepath(graph_filepath),
    swept_volumes_filepath(swept_volumes_filepath),
    sd_graph(graph_filepath)
{
    try
    {
        precomputed_swept_volumes.loadFromFile(swept_volumes_filepath, sd_graph.getNodes().size());
        std::cout << "Loaded " << precomputed_swept_volumes.size() << " swept volumes\n";
    }
    catch(const std::runtime_error& e)
    {
        std::cout << "Could not load precomputed swept volumes from file\n";
    }

    setMode(EdgeCheckMode::FAST);
}



// SelectiveDensificationStrategy::SelectiveDensificationStrategy(const std::string &graph_filepath) :
//     graph_filepath(graph_filepath),
//     swept_volumes_filepath(""), 
//     sd_graph(graph_filepath), initialized(false)
// {
//     std::cout << "Loading graph without precomputed swept volumes\n";
// }


// SelectiveDensificationStrategy::SelectiveDensificationStrategy() :
//     SelectiveDensificationStrategy("/home/bradsaund/catkin_ws/src/gpu_voxel_planning/graphs/SD_100k.graph",
//                                    "/home/bradsaund/catkin_ws/src/gpu_voxel_planning/graphs/swept_volumes_SD_100k.map"){}
// SelectiveDensificationStrategy::SelectiveDensificationStrategy() :
//     SelectiveDensificationStrategy("/home/bradsaund/catkin_ws/src/gpu_voxel_planning/graphs/SD_2_16.graph",
//                                    "/home/bradsaund/catkin_ws/src/gpu_voxel_planning/graphs/swept_volumes_SD_2_16.map"){}


void SelectiveDensificationStrategy::setMode(EdgeCheckMode mode_)
{
    mode = mode_;
}
        

void SelectiveDensificationStrategy::initialize(Scenario &scenario)
{
    addStartAndGoalToGraph(scenario);
    // connectStartAndGoalToGraph(scenario);
    initialized = true;
}

void SelectiveDensificationStrategy::addStartAndGoalToGraph(const Scenario &scenario)
{
    int orig_edge_count = sd_graph.countEdges();
    for(int depth = 3; depth <= sd_graph.depth; depth++)
    // for(int depth = 13; depth <= 14; depth++)
    {
        // int depth = sd_graph.depth;
        DepthNode start(depth, scenario.getState().getCurConfig().asVector());
        DepthNode goal(depth, VictorRightArmConfig(scenario.goal_config).asVector());
        NodeIndex start_id = sd_graph.addVertexAndEdges(start);
        NodeIndex goal_id = sd_graph.addVertexAndEdges(goal);

        std::cout << "Initial (node " << start_id << ") and Goal (node " << goal_id << ") vertices added\n";
        // if(depth == 0)
        {
            cur_node = start_id;
            goal_node = goal_id;
            std::cout << "This is the start and goal\n";
        }
    }

    std::cout << "Start and goal added " << sd_graph.countEdges() - orig_edge_count << " edges\n";
}

NodeIndex SelectiveDensificationStrategy::connectToGraph(Scenario &scenario, const VictorRightArmConfig &q)
{
    int depth = sd_graph.depth;
    // int depth = 3;
    DepthNode d(depth, q.asVector());
    double r = sd_graph.radiusAtDepth(depth);
    std::cout << "radius is: " << r << "\n";
    std::vector<int> vs = sd_graph.getVerticesWithinRadius(d.toRaw(), r);
    std::cout << "Vertices within radius: " << PrettyPrint::PrettyPrint(vs) << "\n";
    std::cout << "Size: " << vs.size() << "\n";

    for(int v: vs)
    {
        if(sd_graph.getNodeValue(v).depth != depth)
        {
            continue;
        }
        std::vector<double> q_near = sd_graph.getNodeValue(v).q;
        // std::cout << PrettyPrint::PrettyPrint(q_near) << "\n";
        // std::cout << PrettyPrint::PrettyPrint(q.asVector()) << "\n";
        std::cout << "Distance: " << EigenHelpers::Distance(q_near, q.asVector()) << "\n";
        if(checkPathFast(q, VictorRightArmConfig(q_near), scenario.getState()))
        {
            return v;
        }
    }

    throw SearchError("No path found to graph");
    
    //TODO: check straight line path from q to every node within radius in increasing order of distance. Return as soon as a valid path is found 
    assert(false && "Not implemented yet");
}

void SelectiveDensificationStrategy::connectStartAndGoalToGraph(Scenario &scenario)
{
    // std::vector<int> v = sd_graph.getVerticesWithinRadius(scenario.getState().getCurConfig().asVector(), sd_graph.r_disc);
    // std::cout << "Vertices within radius: " << PrettyPrint::PrettyPrint(v) << "\n";
    cur_node = connectToGraph(scenario, scenario.getState().getCurConfig());
    goal_node = connectToGraph(scenario, VictorRightArmConfig(scenario.goal_config));

    start_to_graph = interpolate(scenario.getState().getCurConfig().asVector(),
                                 sd_graph.getNodeValue(cur_node).q,
                                 discretization);

    graph_to_goal = interpolate(sd_graph.getNodeValue(goal_node).q,
                                scenario.goal_config,
                                discretization);
    return;

    //TODO: set start_to_graph, graph_to_goal using connectToGraph
    assert(false && "Not implemented yet");
}


Path SelectiveDensificationStrategy::applyTo(Scenario &scenario, GpuVoxelRvizVisualizer& viz_)
{
    viz = &viz_;
    PROFILE_START("PathLength");
    if(!initialized)
    {
        initialize(scenario);
    }

    PROFILE_START("Plan");

    std::vector<NodeIndex> node_path = plan(cur_node, goal_node, scenario.getState());
    PROFILE_RECORD("Plan");

    if(node_path.size() <= 1)
    {
        std::cerr << "Path of less than 2 nodes found\n";
        assert(false);
    }

    Path path = start_to_graph;
    for(size_t i=0; i<node_path.size()-1; i++)
    {
        Path segment = interpolate(sd_graph.getNodeValue(node_path[i]).q,
                                   sd_graph.getNodeValue(node_path[i+1]).q,
                                   discretization);
                                       
        path.insert(path.end(), segment.begin(), segment.end());
    }
    path.insert(path.end(), graph_to_goal.begin(), graph_to_goal.end());

    PROFILE_RECORD_DOUBLE("PathLength", PathUtils::length(toPathUtilsPath(path)));

    PROFILE_RECORD_DOUBLE("SetRobotConfig before smoothing",
                           arc_utilities::Profiler::getData("Set robot config").size());

    std::mt19937 rng;
    rng.seed(42);


    for(int i=0; i<30; i++)
    {
        PROFILE_START("Smooth");
        path = smooth(path, scenario.getState(), discretization, rng);
        PROFILE_RECORD_DOUBLE("PathLength", PathUtils::length(toPathUtilsPath(path)));
        PROFILE_RECORD("Smooth");
    }

    std::cout << "Smoothed path cost " << PathUtils::length(toPathUtilsPath(path)) << "\n\n";

    return GVP::densify(path, discretization);
}


std::vector<NodeIndex> SelectiveDensificationStrategy::plan(NodeIndex start, NodeIndex goal, State &s)
{
    return lazySp(start, goal, s);
    // return astar(start, goal, s);
}


DenseGrid SelectiveDensificationStrategy::computeSweptVolume(State &s, arc_dijkstras::GraphEdge &e)
{
    PROFILE_START("ComputeSweptVolume");
    VictorRightArmConfig q_start(sd_graph.getNodeValue(e.getFromIndex()).q);
    VictorRightArmConfig q_end(sd_graph.getNodeValue(e.getToIndex()).q);
    GVP::Path path = interpolate(q_start, q_end, discretization);

    DenseGrid swept_volume;
    for(const auto &config: path)
    {
        PROFILE_START("Config Added to Swept Volume");
        s.robot.set(config.asMap());
        swept_volume.add(&s.robot.occupied_space);
        PROFILE_RECORD("Config Added to Swept Volume");
    }
    PROFILE_RECORD("ComputeSweptVolume");
    return swept_volume;
}

void SelectiveDensificationStrategy::storeSweptVolume(const arc_dijkstras::GraphEdge &e,
                                           const DenseGrid &swept_volume)
{
    precomputed_swept_volumes[arc_dijkstras::getSortedHashable(e)] = swept_volume;
}


DenseGrid SelectiveDensificationStrategy::getSweptVolume(State &s, arc_dijkstras::GraphEdge &e)
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




bool SelectiveDensificationStrategy::checkPathFast(VictorRightArmConfig q_start, VictorRightArmConfig q_end,
    State &s)
{
    GVP::Path path = interpolate(q_start, q_end, discretization);

    auto rng = std::default_random_engine{};
    std::shuffle(std::begin(path), std::end(path), rng);

    for(const auto &config: path)
    {

        s.robot.set(config.asMap());
        if(s.robot.occupied_space.overlapsWith(&s.known_obstacles) ||
           s.robot.occupied_space.overlapsWith(&s.robot_self_collide_obstacles))
            // if(s.robot.occupied_space.overlapsWith(&s.known_obstacles))
            // if(!s.isPossiblyValid(config))
        {
            return false;
        }
    }
    return true;
}

bool SelectiveDensificationStrategy::checkEdgeFast(arc_dijkstras::GraphEdge &e, State &s)
{
    PROFILE_START("CheckEdgeFast Valid");
    PROFILE_START("CheckEdgeFast Invalid");
    std::string depth_logging_name = "CheckEdgeFast depth=" +
        std::to_string(sd_graph.getNodeValue(e.getFromIndex()).depth);
    PROFILE_START(depth_logging_name);

    VictorRightArmConfig q_start(sd_graph.getNodeValue(e.getFromIndex()).q);
    VictorRightArmConfig q_end(sd_graph.getNodeValue(e.getToIndex()).q);

    bool edge_valid = checkPathFast(q_start, q_end, s);
    
    PROFILE_RECORD(depth_logging_name);

    if(!edge_valid)
    {
        PROFILE_RECORD("CheckEdgeFast Invalid");
        e.setValidity(arc_dijkstras::EDGE_VALIDITY::INVALID);
        
        // For some reason, adding this makes planning take much longer
        sd_graph.getReverseEdge(e).setValidity(arc_dijkstras::EDGE_VALIDITY::INVALID);
        return false;
    }
    
    e.setValidity(arc_dijkstras::EDGE_VALIDITY::VALID);
    sd_graph.getReverseEdge(e).setValidity(arc_dijkstras::EDGE_VALIDITY::VALID);
    PROFILE_RECORD("CheckEdgeFast Valid");

    return true;
}


bool SelectiveDensificationStrategy::checkEdgeAndStore(arc_dijkstras::GraphEdge &e, State &s)
{
    PROFILE_START("CheckEdgeAndStore");
    PROFILE_START("CheckEdgeAndStore get sv");
    DenseGrid sv = getSweptVolume(s, e);
    PROFILE_RECORD("CheckEdgeAndStore get sv");

    PROFILE_START("CheckEdgeAndStore grid overlap");
    bool valid = !sv.overlapsWith(&s.robot_self_collide_obstacles) &&
        !sv.overlapsWith(&s.known_obstacles);
    // bool valid = !sv.overlapsWith(&s.known_obstacles);


    e.setValidity(valid ? arc_dijkstras::EDGE_VALIDITY::VALID :
                  arc_dijkstras::EDGE_VALIDITY::INVALID);

    sd_graph.getReverseEdge(e).setValidity(valid ? arc_dijkstras::EDGE_VALIDITY::VALID :
                                           arc_dijkstras::EDGE_VALIDITY::INVALID);
    
    PROFILE_RECORD("CheckEdgeAndStore");
    return valid;
}


void SelectiveDensificationStrategy::vizEdge(arc_dijkstras::GraphEdge &e)
{

    VictorRightArmConfig q_start(sd_graph.getNodeValue(e.getFromIndex()).q);
    VictorRightArmConfig q_end(sd_graph.getNodeValue(e.getToIndex()).q);
    std::vector<VictorRightArmConfig> dense_edge = interpolate(q_start, q_end, discretization);

    auto color = makeColor(0,1,1);
    if(e.isInvalid())
    {
        color = makeColor(1,0,0);
    }
    viz->vizEEPath(dense_edge,
                   "checked_edge", vized_id++, color);
}

bool SelectiveDensificationStrategy::checkEdge(arc_dijkstras::GraphEdge &e, State &s)
{
    PROFILE_START("CheckEdge");
    arc_dijkstras::HashableEdge e_hashed = arc_dijkstras::getSortedHashable(e);

    if(precomputed_swept_volumes.count(e_hashed))
    {
        bool valid = checkEdgeAndStore(e, s); //This will run fast, since there is already a swept volume
        PROFILE_RECORD("CheckEdge");
        if(VISUALIZE)
        {
            vizEdge(e);
        }
        return valid;
    }

    PROFILE_START("CheckEdge From Scratch");
    bool valid = false;

    std::cout << "Checking edge from scratch: " << e << "\n";
    
    if(mode == EdgeCheckMode::STORE)
    {
        valid = checkEdgeAndStore(e, s);
    }
    else if(mode == EdgeCheckMode::FAST)
    {
        valid = checkEdgeFast(e, s);
    }
    else
    {
        assert(false && "invalid mode");
    }
    PROFILE_RECORD("CheckEdge From Scratch");
    PROFILE_RECORD("CheckEdge");
    if(VISUALIZE)
    {
        vizEdge(e);
    }
    return valid;
}

double SelectiveDensificationStrategy::evaluateEdge(arc_dijkstras::GraphEdge &e, State &s)
{
    // std::cout << "evaluating edge " << e.getFromIndex() << "->" << e.getToIndex() << "\n";
    if(e.getValidity() == arc_dijkstras::EDGE_VALIDITY::UNKNOWN)
    {
        checkEdge(e, s);
    }
    return calculateEdgeWeight(s, e);
}

std::vector<int> SelectiveDensificationStrategy::forwardPrecomputedSelector(
    std::vector<int64_t> path,
    arc_dijkstras::Graph<std::vector<double>>& g,
    const arc_dijkstras::EvaluatedEdges &evaluatedEdges)
{
    using namespace arc_dijkstras;
    for(int i=0; i<(int)path.size()-1; i++)
    {
        GraphEdge &e = g.getNode(path[i]).getEdgeTo(path[i+1]);
        if(e.getValidity() == EDGE_VALIDITY::INVALID)
        {
            std::cout << "Forward selector encountered edge alredy known to be invalid: ( " <<
                e.getFromIndex() << ", " << e.getToIndex() << ")\n";
            assert(false);
        }
        if(precomputed_swept_volumes.count(getSortedHashable(e)) &&
           evaluatedEdges.count(getHashable(e)) == 0)
        {
            return std::vector<int>{i};
        }
    }
    for(int i=0; i<(int)path.size()-1; i++)
    {
        GraphEdge &e = g.getNode(path[i]).getEdgeTo(path[i+1]);
        if(e.getValidity() == EDGE_VALIDITY::INVALID)
        {
            std::cout << "Forward selector encountered edge alredy known to be invalid: ( " <<
                e.getFromIndex() << ", " << e.getToIndex() << ")\n";
            assert(false);
        }
        if(evaluatedEdges.count(getHashable(e)) == 0)
        {
            return std::vector<int>{i};
        }
    }
    return std::vector<int>();
}

std::vector<NodeIndex> SelectiveDensificationStrategy::lazySp(NodeIndex start, NodeIndex goal, State &s)
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
    // const auto heuristic_cons_fn = [&] (const std::vector<double> &n1,
    //                                     const std::vector<double> &n2)
    //     {
    //         return EigenHelpers::Distance(n1, n2);
    //     };

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
        sd_graph, start, goal, heuristic_fn, eval_fn, selector);
    // auto result = arc_dijkstras::LazySP<std::vector<double>>::PerformLazySP(
    //     sd_graph, goal, start, heuristic_fn, eval_fn, selector);
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


std::vector<NodeIndex> SelectiveDensificationStrategy::astar(NodeIndex start, NodeIndex goal, State &s)
{
    const auto edge_check_fn =
        [&] (arc_dijkstras::Graph<std::vector<double>> &g, arc_dijkstras::GraphEdge &e)
        {
            if(e.getValidity() == arc_dijkstras::EDGE_VALIDITY::UNKNOWN);
            {
                evaluateEdge(e, s);
            }
            return e.getValidity() == arc_dijkstras::EDGE_VALIDITY::VALID;
        };

    const auto heuristic_fn = [&] (const std::vector<double> &n1,
                                   const std::vector<double> &n2)
        {
            return distanceHeuristic(n1, n2);
        };

    const auto dist_fn = [&] (const arc_dijkstras::Graph<std::vector<double>> &g,
                              const arc_dijkstras::GraphEdge &e)
        {
            return e.getWeight();
        };
    
    std::cout << "Performing astar search\n";
    auto result = arc_dijkstras::AstarLogging<std::vector<double>>::PerformLazyAstar(
        sd_graph, start, goal, edge_check_fn, dist_fn, heuristic_fn, true);

    std::cout << "astar finished\n";
    
    if(result.second == std::numeric_limits<double>::infinity())
    {
        std::cout << "No path found on graph\n";
    }
    
    PROFILE_RECORD_DOUBLE("lazySP path cost ", result.second);
    std::cout << "astar path cost " << result.second << "\n";

    return result.first;
}


void SelectiveDensificationStrategy::saveToFile(std::string filename)
{
    precomputed_swept_volumes.saveToFile(filename);
}





/**********************************
 **  Omniscient Graph Search
 ********************************/
OmniscientSDGraphSearch::OmniscientSDGraphSearch(bool use_precomputed, double c_p, int graph_num) :
    SelectiveDensificationStrategy(basepath + "seed" + std::to_string(graph_num) + ".graph",
                                   use_precomputed ? basepath + "sv" + std::to_string(graph_num) + ".map" : ""),
    use_precomputed(use_precomputed),
    c_p(c_p)
{
}

double OmniscientSDGraphSearch::calculateEdgeWeight(State &s, arc_dijkstras::GraphEdge &e)
{
    return e.getWeight();
}

std::string OmniscientSDGraphSearch::getName() const
{
    std::string type = (use_precomputed ? "_precomputed" : "");
    PROFILE_RECORD_DOUBLE("c_p", c_p);
    return "Omniscient_SD_Graph_Search" + type;
}

double OmniscientSDGraphSearch::distanceHeuristic(const std::vector<double> &raw1,
                                                  const std::vector<double> &raw2) const
{

    DepthNode d1(raw1);
    DepthNode d2(raw2);
    // std::cout << "Calling dist heuristic with depth " << d1.depth << "\n";
     // std::pow(2, d1.depth);
    std::string depth_logging_name = "EdgeHeuristic depth=" +
        std::to_string(d1.depth);
    PROFILE_START(depth_logging_name);
    PROFILE_RECORD(depth_logging_name);

    return EigenHelpers::Distance(d1.q, d2.q)*(1+c_p*std::pow(2, d1.depth));
    // return EigenHelpers::Distance(d1.q, d2.q);
}

    


/****************************
 **  Dense Graph Search   ***
 ***************************/
DenseGraphSearch::DenseGraphSearch(bool use_precomputed, int graph_num) :
    OmniscientSDGraphSearch(use_precomputed, 0.0, graph_num)
{
}

std::string DenseGraphSearch::getName() const
{
    std::string type = (use_precomputed ? "_precomputed" : "");
    return "Dense_Graph_Search" + type;
}

double DenseGraphSearch::distanceHeuristic(const std::vector<double> &raw1,
                                           const std::vector<double> &raw2) const
{

    DepthNode d1(raw1);
    DepthNode d2(raw2);
    // std::cout << "Calling dist heuristic with depth " << d1.depth << "\n";
     // std::pow(2, d1.depth);
    std::string depth_logging_name = "EdgeHeuristic depth=" +
        std::to_string(d1.depth);
    PROFILE_START(depth_logging_name);
    PROFILE_RECORD(depth_logging_name);

    return EigenHelpers::Distance(d1.q, d2.q);
}
