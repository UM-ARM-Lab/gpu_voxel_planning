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
#include "gpu_voxel_planning/scenarios/real_scenario.hpp"
#include <arm_video_recorder/TriggerVideoRecording.h>

using namespace GVP;



int main(int argc, char* argv[])
{
    icl_core::logging::initialize(argc, argv);
    ros::init(argc, argv, "wip_roadmap");
    ros::NodeHandle n;
    ros::ServiceClient video_recorder =
        n.serviceClient<arm_video_recorder::TriggerVideoRecording>("video_recorder");


        

    // std::string graph_filepath = ros::package::getPath("gpu_voxel_planning")  + "/graphs/SD_100k.graph";


    BeliefParams bp(BeliefType::CHS);
    // BeliefParams bp(BeliefType::Obstacle, std::vector<double>{0,0,0}, 0.1);
    // BeliefParams bp(BeliefType::Bonkers, std::vector<double>{0,0,0}, 0.05);
    // BeliefParams bp(BeliefType::MoEObstacle, std::vector<double>{0,0,0}, 0.1);
    // BeliefParams bp(BeliefType::MoEBonkers, std::vector<double>{0,0,0}, 0.05);


    
    // BeliefParams bp(BeliefType::Obstacle, std::vector<double>{0.1,0.1,0.1}, 0.4);

    // BeliefParams bp(BeliefType::MoEObstacle, std::vector<double>{0,0,0}, 0.1);
    // BeliefParams bp(BeliefType::MoEObstacle, std::vector<double>{0.1,0.1,0.1}, 0.4);
    // BeliefParams bp(BeliefType::MoEBonkers, std::vector<double>{0,0,0}, 0.05);

    ros::Duration(1.0).sleep();
    // RealTable scenario(bp);
    RealEmpty scenario(bp);

    RealScenarioTester tester(scenario, n);

    // OptimisticGraphSearch strat;
    CollisionMeasure strat(1.0);
    // ThompsonGraphSearch strat;
    // HOPGraphSearch strat;
     

    std::string filename = scenario.getName() + "_" + strat.getName() + "_" +
        scenario.belief_name + "_" +
        arc_helpers::GetCurrentTimeAsString();

    arm_video_recorder::TriggerVideoRecording srv;
    srv.request.filename=filename + ".mp4";
    srv.request.record = true;
    srv.request.timeout_in_sec = 30*60.0;

    video_recorder.call(srv);


    std::cout << "Attempting strategy\n";
    tester.attemptStrategy(strat);
    // strat.saveToFile();


    // viz.vizEEGraph(strat.graph);
    // viz.vizEESDGraph(strat.sd_graph);

    srv.request.record = false;
    video_recorder.call(srv);
    
    PROFILE_PRINT_SUMMARY_FOR_ALL();
    PROFILE_WRITE_SUMMARY_FOR_ALL(filename);
    PROFILE_WRITE_ALL_FEWER_THAN(filename, 100);

    ros::Duration(3.0).sleep();
    tester.reversePath();
}
