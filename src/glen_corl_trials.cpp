#include "gpu_voxel_planning/maps/prob_map.hpp"
#include <gpu_voxels/helpers/GeometryGeneration.h>
#include "gpu_voxel_planning/common_names.hpp"
#include <arc_utilities/timing.hpp>
#include <arc_utilities/arc_helpers.hpp>

#include <ros/ros.h>
#include "gpu_voxel_planning/ros_interface/ros_interface.hpp"
#include "gpu_voxel_planning/state.hpp"
#include "gpu_voxel_planning/scenario_tester.hpp"
#include "gpu_voxel_planning/strategies/graph_search_strategies.hpp"
#include "gpu_voxel_planning/path_utils_addons.hpp"
// #include "strategies/victor_selective_densification.hpp"
// #include "strategies/selective_densification_strategies.hpp"
// #include "strategies/ompl_strategies.hpp"

using namespace GVP;


int main(int argc, char* argv[])
{
    icl_core::logging::initialize(argc, argv);
    ros::init(argc, argv, "wip_roadmap");
    ros::NodeHandle n;



    BeliefParams bp(BeliefType::CHS);
    // BeliefParams bp(BeliefType::IID, std::vector<double>{0,0,0}, 0.1);
    // BeliefParams bp(BeliefType::Obstacle, std::vector<double>{0,0,0}, 0.1);
    // BeliefParams bp(BeliefType::Bonkers, std::vector<double>{0,0,0}, 0.05);
    // BeliefParams bp(BeliefType::MoEObstacle, std::vector<double>{0,0,0}, 0.05);
    // BeliefParams bp(BeliefType::MoEBonkers, std::vector<double>{0,0,0}, 0.05);

    ros::Duration(1.0).sleep();
    // GVP::VictorRightArm victor_right;
    // GVP::VictorLeftArmAndBase victor_left;
//    TableWithBox scenario(bp, true, true, false);
    // SlottedWall scenario(bp);
    // Bookshelf scenario(bp);
    GlenScenario1 scenario(bp);

    SimulationScenarioTester tester(scenario, n);

    // OptimisticGraphSearch strat;
    // ThompsonGraphSearch strat;
    // HOPGraphSearch strat;
    // OROGraphSearch strat;
    // QMDP strat;
    ParetoCostGraphSearch strat(1.0);

    // arc_helpers::WaitForInput();


    std::cout << "Attempting strategy\n";
    tester.attemptStrategy(strat);
    // strat.saveToFile();

    // strat.saveToFile("/home/bradsaund/catkin_ws/src/gpu_voxel_planning/graphs/swept_volumes_10k.map");

    // viz.vizEEGraph(strat.graph);
    // viz.vizEESDGraph(strat.sd_graph);

    PROFILE_PRINT_SUMMARY_FOR_ALL();
    std::string filename = "sim_timing_" + arc_helpers::GetCurrentTimeAsString();
    PROFILE_WRITE_SUMMARY_FOR_ALL(filename);
    PROFILE_WRITE_ALL_FEWER_THAN(filename, 10000);
}
