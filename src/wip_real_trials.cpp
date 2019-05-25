#include "maps/prob_map.hpp"
#include <gpu_voxels/helpers/GeometryGeneration.h>
#include "common_names.hpp"
#include <arc_utilities/timing.hpp>
#include <arc_utilities/arc_helpers.hpp>

#include <ros/ros.h>
#include "ros_interface/ros_interface.hpp"
#include "state.hpp"
#include "scenario_tester.hpp"
#include "strategies/graph_search_strategies.hpp"
#include "path_utils_addons.hpp"
#include "scenarios/real_scenario.hpp"

using namespace GVP;




int main(int argc, char* argv[])
{
    icl_core::logging::initialize(argc, argv);
    ros::init(argc, argv, "wip_roadmap");
    ros::NodeHandle n;

    // std::string graph_filepath = "/home/bradsaund/catkin_ws/src/gpu_voxel_planning/graphs/SD_100k.graph";


    BeliefParams bp(BeliefType::CHS);
    // BeliefParams bp(BeliefType::IID, std::vector<double>{0,0,0}, 0.1);
    // BeliefParams bp(BeliefType::Obstacle, std::vector<double>{0,0,0}, 0.1);
    // BeliefParams bp(BeliefType::Bonkers, std::vector<double>{0,0,0}, 0.05);
    // BeliefParams bp(BeliefType::MoEObstacle, std::vector<double>{0,0,0}, 0.05);
    // BeliefParams bp(BeliefType::MoEBonkers, std::vector<double>{0,0,0}, 0.05);

    ros::Duration(1.0).sleep();
    // RealTable scenario(bp);
    RealEmpty scenario(bp);

    RealScenarioTester tester(scenario, n);

    // OptimisticGraphSearch strat;
    // ThompsonGraphSearch strat;
    // HOPGraphSearch strat;
    // OROGraphSearch strat;
    ParetoCostGraphSearch strat(1.0);


    std::cout << "Attempting strategy\n";
    tester.attemptStrategy(strat);
    // strat.saveToFile();

    // strat.saveToFile("/home/bradsaund/catkin_ws/src/gpu_voxel_planning/graphs/swept_volumes_100k.map");

    // viz.vizEEGraph(strat.graph);
    // viz.vizEESDGraph(strat.sd_graph);
    
    PROFILE_PRINT_SUMMARY_FOR_ALL();
    std::string filename = "sim_timing_" + arc_helpers::GetCurrentTimeAsString();
    PROFILE_WRITE_SUMMARY_FOR_ALL(filename);
    PROFILE_WRITE_ALL_FEWER_THAN(filename, 10000);

}
