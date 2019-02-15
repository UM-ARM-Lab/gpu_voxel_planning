#include "prob_map.hpp"
#include <gpu_voxels/helpers/GeometryGeneration.h>
#include "common_names.hpp"
#include <arc_utilities/timing.hpp>
#include <arc_utilities/arc_helpers.hpp>

#include <ros/ros.h>
#include "gpu_voxel_rviz_visualization.hpp"
#include "robot_model.hpp"
#include "state.hpp"
#include "scenario_tester.hpp"
#include "strategies/graph_search_strategies.hpp"
#include "path_utils_addons.hpp"
#include "urdf_model.hpp"
#include "strategies/victor_selective_densification.hpp"
#include "strategies/selective_densification_strategies.hpp"
#include "strategies/ompl_strategies.hpp"

using namespace GVP;


int main(int argc, char* argv[])
{
    icl_core::logging::initialize(argc, argv);
    ros::init(argc, argv, "wip_roadmap");
    ros::NodeHandle n;
    GpuVoxelRvizVisualizer viz(n);

    std::string graph_filepath = "/home/bradsaund/catkin_ws/src/gpu_voxel_planning/graphs/SD_100k.graph";

    // SDRoadmap rm;
    // rm.saveToFile(graph_filepath);


    ros::Duration(1.0).sleep();
    // GVP::VictorRightArm victor_right;
    // GVP::VictorLeftArmAndBase victor_left;
    // TableWithBox scenario(true, true, true);
    SlottedWall scenario(true);

    // AStarGraphSearch strat;
    // OmniscientGraphSearch strat;
    // OmniscientGraphSearch strat("/home/bradsaund/catkin_ws/src/gpu_voxel_planning/graphs/halton_100k.graph");
    OmniscientSDGraphSearch strat;
    // OmniscientSDGraphSearch strat(graph_filepath);
    // RRT_Strategy strat;

    // testAngles(scenario, viz);
    // return 1;
    
    
    SimulationScenarioTester tester(scenario, n);
    std::cout << "Attempting strategy\n";
    tester.attemptStrategy(strat);
    // strat.saveToFile();

    // viz.vizEEGraph(strat.graph);
    
    PROFILE_PRINT_SUMMARY_FOR_ALL();
    std::string filename = "sim_timing_" + arc_helpers::GetCurrentTimeAsString();
    PROFILE_WRITE_SUMMARY_FOR_ALL(filename);
    PROFILE_WRITE_ALL_FEWER_THAN(filename, 10000);
    
}
