#include "gpu_voxel_planning/strategies/layered_graph_strategy.hpp"


using namespace GVP;

LayeredGraphStrategy::LayeredGraphStrategy(const std::string &graph_filepath,
                                           const std::string& swept_volumes_filepath) :
    graph_filepath(graph_filepath),
    swept_volumes_filepath(swept_volumes_filepath)
{
    setMode(EdgeCheckMode::FAST);
}

void LayeredGraphStrategy::setMode(EdgeCheckMode mode_)
{
    mode = mode_;
}

bool LayeredGraphStrategy::checkPathFast(VictorRightArmConfig q_start, VictorRightArmConfig q_end,
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


void LayeredGraphStrategy::saveToFile(std::string filename)
{
    precomputed_swept_volumes.saveToFile(filename);
}

DenseGrid LayeredGraphStrategy::getSweptVolume(State &s, arc_dijkstras::GraphEdge &e)
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


DenseGrid LayeredGraphStrategy::computeSweptVolume(State &s, arc_dijkstras::GraphEdge &e)
{
    PROFILE_START("ComputeSweptVolume");
    VictorRightArmConfig q_start(getStart(e));
    VictorRightArmConfig q_end(getEnd(e));
    
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

void LayeredGraphStrategy::storeSweptVolume(const arc_dijkstras::GraphEdge &e,
                                           const DenseGrid &swept_volume)
{
    precomputed_swept_volumes[arc_dijkstras::getSortedHashable(e)] = swept_volume;
}

bool LayeredGraphStrategy::checkEdgeFast(arc_dijkstras::GraphEdge &e, State &s)
{
    PROFILE_START("CheckEdgeFast Valid");
    PROFILE_START("CheckEdgeFast Invalid");
    std::string depth_logging_name = "CheckEdgeFast depth=" + std::to_string(getDepth(e));
    PROFILE_START(depth_logging_name);

    VictorRightArmConfig q_start(getStart(e));
    VictorRightArmConfig q_end(getEnd(e));

    bool edge_valid = checkPathFast(q_start, q_end, s);
    
    PROFILE_RECORD(depth_logging_name);

    if(!edge_valid)
    {
        PROFILE_RECORD("CheckEdgeFast Invalid");
        e.setValidity(arc_dijkstras::EDGE_VALIDITY::INVALID);
        
        // For some reason, adding this makes planning take much longer
        getReverseEdge(e).setValidity(arc_dijkstras::EDGE_VALIDITY::INVALID);
        return false;
    }
    
    e.setValidity(arc_dijkstras::EDGE_VALIDITY::VALID);
    getReverseEdge(e).setValidity(arc_dijkstras::EDGE_VALIDITY::VALID);
    PROFILE_RECORD("CheckEdgeFast Valid");

    return true;
}


bool LayeredGraphStrategy::checkEdgeAndStore(arc_dijkstras::GraphEdge &e, State &s)
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

    getReverseEdge(e).setValidity(valid ? arc_dijkstras::EDGE_VALIDITY::VALID :
                                  arc_dijkstras::EDGE_VALIDITY::INVALID);
    
    PROFILE_RECORD("CheckEdgeAndStore");
    return valid;
}


void LayeredGraphStrategy::vizEdge(arc_dijkstras::GraphEdge &e)
{

    VictorRightArmConfig q_start(getStart(e));
    VictorRightArmConfig q_end(getEnd(e));
    std::vector<VictorRightArmConfig> dense_edge = interpolate(q_start, q_end, discretization);

    auto color = makeColor(0,1,1);
    if(e.isInvalid())
    {
        color = makeColor(1,0,0);
    }
    viz->vizEEPath(dense_edge,
                   "checked_edge", vized_id++, color);
}

bool LayeredGraphStrategy::checkEdge(arc_dijkstras::GraphEdge &e, State &s)
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


double LayeredGraphStrategy::evaluateEdge(arc_dijkstras::GraphEdge &e, State &s)
{
    if(e.getValidity() == arc_dijkstras::EDGE_VALIDITY::UNKNOWN)
    {
        checkEdge(e, s);
    }
    return calculateEdgeWeight(s, e);
}



Path LayeredGraphStrategy::applyTo(Scenario &scenario, GpuVoxelRvizVisualizer& viz_)
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
        Path segment = interpolate(getNodeValue(node_path[i]), getNodeValue(node_path[i+1]),
                                   discretization);
                                       
        path.insert(path.end(), segment.begin(), segment.end());
    }
    path.insert(path.end(), graph_to_goal.begin(), graph_to_goal.end());

    PROFILE_RECORD_DOUBLE("PathLength", PathUtils::length(toPathUtilsPath(path)));

    PROFILE_RECORD_DOUBLE("SetRobotConfig before smoothing",
                           arc_utilities::Profiler::getData("Set robot config").size());

    std::mt19937 rng;
    rng.seed(42);


    if(SMOOTH)
    {
        for(int i=0; i<30; i++)
        {
            PROFILE_START("Smooth");
            path = smooth(path, scenario.getState(), discretization, rng);
            PROFILE_RECORD_DOUBLE("PathLength", PathUtils::length(toPathUtilsPath(path)));
            PROFILE_RECORD("Smooth");
        }
        std::cout << "Smoothed path cost " << PathUtils::length(toPathUtilsPath(path)) << "\n\n";
    }


    return GVP::densify(path, discretization);
}

