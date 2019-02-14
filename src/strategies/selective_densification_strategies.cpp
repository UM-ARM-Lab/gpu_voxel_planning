#include "strategies/selective_densification_strategies.hpp"


using namespace GVP;

SelectiveDensificationStrategy::SelectiveDensificationStrategy(const std::string &graph_filepath,
                                                               const std::string& swept_volumes_filepath) :
    swept_volumes_filepath(swept_volumes_filepath),
    graph_filepath(graph_filepath),
    sd_graph(graph_filepath),
    initialized(false)
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
}



SelectiveDensificationStrategy::SelectiveDensificationStrategy(const std::string &graph_filepath) :
    swept_volumes_filepath(""), graph_filepath(graph_filepath), 
    sd_graph(graph_filepath), initialized(false) {}


SelectiveDensificationStrategy::SelectiveDensificationStrategy() :
    SelectiveDensificationStrategy("/home/bradsaund/catkin_ws/src/gpu_voxel_planning/graphs/SD_100k.graph",
                                   "/home/bradsaund/catkin_ws/src/gpu_voxel_planning/graphs/swept_volumes_SD_100k.map"){}
        

void SelectiveDensificationStrategy::initialize(const Scenario &scenario)
{
    // for(int depth = 0; depth < sd_graph.depth; depth++)
    {
        int depth = sd_graph.depth - 1;
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
    initialized = true;
}            


Path SelectiveDensificationStrategy::applyTo(Scenario &scenario)
{
    if(!initialized)
    {
        initialize(scenario);
    }

    Path path;
    std::vector<NodeIndex> node_path = plan(cur_node, goal_node, scenario.getState());

    if(node_path.size() <= 1)
    {
        std::cerr << "Path of less than 2 nodes found\n";
        assert(false);
    }
        
    for(size_t i=0; i<node_path.size()-1; i++)
    {
        Path segment = interpolate(sd_graph.getNodeValue(node_path[i]).q,
                                   sd_graph.getNodeValue(node_path[i+1]).q,
                                   discretization);
                                       
        path.insert(path.end(), segment.begin(), segment.end());
    }
    return path;
}


std::vector<NodeIndex> SelectiveDensificationStrategy::plan(NodeIndex start, NodeIndex goal, State &s)
{
    return lazySp(start, goal, s);
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





bool SelectiveDensificationStrategy::checkEdge(arc_dijkstras::GraphEdge &e, State &s)
{
    PROFILE_START("EdgeCheck");
    bool valid = !getSweptVolume(s, e).overlapsWith(&s.known_obstacles);
    e.setValidity(valid ? arc_dijkstras::EDGE_VALIDITY::VALID :
                  arc_dijkstras::EDGE_VALIDITY::INVALID);
    PROFILE_RECORD("EdgeCheck");
    return valid;
}

double SelectiveDensificationStrategy::evaluateEdge(arc_dijkstras::GraphEdge &e, State &s)
{
    if(e.getValidity() == arc_dijkstras::EDGE_VALIDITY::UNKNOWN)
    {
        checkEdge(e, s);
    }
    return calculateEdgeWeight(s, e);
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
            return sd_graph.distanceHeuristic(n1, n2);
        };
    std::cout << "Performing lazysp\n";
    auto result = arc_dijkstras::LazySP<std::vector<double>>::PerformLazySP(
        sd_graph, start, goal, heuristic_fn, eval_fn, true);

    std::cout << "LazySP finished\n";
    
    if(result.second == std::numeric_limits<double>::infinity())
    {
        std::cout << "No path found on graph\n";
    }
    return result.first;
}


void SelectiveDensificationStrategy::saveToFile(std::string filename)
{
    precomputed_swept_volumes.saveToFile(filename);
}





/**********************************
 **  Omniscient Graph Search
 ********************************/
double OmniscientSDGraphSearch::calculateEdgeWeight(State &s, arc_dijkstras::GraphEdge &e)
{
    return e.getWeight();
}

std::string OmniscientSDGraphSearch::getName() const
{
    return "Omniscient SD Graph Search";
}
    
