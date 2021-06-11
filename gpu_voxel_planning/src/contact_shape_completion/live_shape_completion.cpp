#include <arm_video_recorder/TriggerVideoRecording.h>
#include <gpu_voxels/helpers/GeometryGeneration.h>
#include <ros/ros.h>

#include <arc_utilities/arc_helpers.hpp>
#include <arc_utilities/timing.hpp>
#include <gpu_voxel_planning/strategies/contact_shape_completion_strategies.hpp>

#include "gpu_voxel_planning/common_names.hpp"
#include "gpu_voxel_planning/maps/prob_map.hpp"
#include "gpu_voxel_planning/path_utils_addons.hpp"
#include "gpu_voxel_planning/ros_interface/ros_interface.hpp"
#include "gpu_voxel_planning/scenario_tester.hpp"
#include "gpu_voxel_planning/scenarios/real_scenario.hpp"
#include "gpu_voxel_planning/state.hpp"
#include "gpu_voxel_planning/strategies/graph_search_strategies.hpp"

using namespace GVP;

int main(int argc, char* argv[]) {
  icl_core::logging::initialize(argc, argv);
  ros::init(argc, argv, "live_demo");
  ros::NodeHandle n;

  // std::string graph_filepath = ros::package::getPath("gpu_voxel_planning") + "/graphs/SD_100k.graph";

  BeliefParams bp(BeliefType::ShapeCompletion);


  ros::Duration(1.0).sleep();
//  RealTable scenario(bp);
  RealShapeRequestScenario scenario(bp);

  RealScenarioTester tester(scenario, n);

  OptimismIG strat;

  std::cout << "Attempting strategy\n";
  tester.attemptStrategy(strat);

  // strat.saveToFile();

  // strat.saveToFile(ros::package::getPath("gpu_voxel_planning") + "/graphs/swept_volumes_100k.map");

  ros::Duration(3.0).sleep();
  std::cout << "Reversing path\n";
  tester.reversePath();
}
