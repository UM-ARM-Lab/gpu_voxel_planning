#include <gpu_voxels/helpers/GeometryGeneration.h>
#include <ros/package.h>
#include <ros/ros.h>

#include <arc_utilities/arc_helpers.hpp>
#include <arc_utilities/timing.hpp>

#include "gpu_voxel_planning/common_names.hpp"
#include "gpu_voxel_planning/scenario_tester.hpp"
#include "gpu_voxel_planning/state.hpp"
#include "gpu_voxel_planning/strategies/graph_search_strategies.hpp"
#include "gpu_voxel_planning/strategies/contact_shape_completion_strategies.hpp"



using namespace GVP;

void test(ros::NodeHandle &n, SimulationScenario &scenario, GraphSearchStrategy &strategy) {
  PROFILE_REINITIALIZE(0, 0);
  std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~\nTesting:\n" << scenario.getName() << "\n";
  std::cout << strategy.getName() << "\n";
  std::cout << scenario.belief_name << "\n";

  SimulationScenarioTester tester(scenario, n);
  ros::Duration(1.0).sleep();
  tester.attemptStrategy(strategy);

  std::string filename = scenario.getName() + "_" + strategy.getName() + "_" + scenario.belief_name + "_" +
                         arc_helpers::GetCurrentTimeAsString();
  //  PROFILE_WRITE_SUMMARY_FOR_ALL(filename);
  //  PROFILE_WRITE_ALL_FEWER_THAN(filename, 100);
  //    std::string package_path = ros::package::getPath("gpu_voxel_planning");
  strategy.saveToFile(ros::package::getPath("gpu_voxel_planning") + "/graphs/swept_volumes_10k.map");
}


std::vector<std::function<std::shared_ptr<SimulationScenario>(void)>> getScenarioFactories(BeliefParams &bp) {
  std::vector<std::function<std::shared_ptr<SimulationScenario>(void)>> factories;

  factories.emplace_back([&]() { return std::make_shared<ShapeRequestScenario>(bp); });
  //  factories.push_back([&]() { return std::make_shared<TableWithBox>(bp, false, true, false); });
  //  factories.push_back([&]() { return std::make_shared<Bookshelf>(bp); });
  return factories;
}

std::vector<std::function<std::shared_ptr<GraphSearchStrategy>()>> getStrategyFactories() {
  std::vector<std::function<std::shared_ptr<GraphSearchStrategy>(void)>> factories;
//  factories.push_back([]() { return std::make_shared<OptimisticGraphSearch>(); });
  // factories.push_back([](){ return std::make_shared<CollisionMeasure>(1.0);});
  // factories.push_back([](){ return std::make_shared<CollisionMeasure>(10.0);});
  // factories.push_back([](){ return std::make_shared<ThompsonGraphSearch>();});
  // factories.push_back([](){ return std::make_shared<HOPGraphSearch>();});
  // factories.push_back([](){ return std::make_shared<QMDP>();});
  // factories.push_back([](){ return std::make_shared<OROGraphSearch>();});

  factories.emplace_back([]() {return std::make_shared<OptimismIG>(); });
  return factories;
}

std::vector<BeliefParams> getBeliefParams() {
  std::vector<BeliefParams> bps;
  //  bps.emplace_back(BeliefType::CHS);
  //   bps.emplace_back(BeliefType::Obstacle, std::vector<double>{0,0,0}, 0.1);
  //   bps.emplace_back(BeliefType::Obstacle, std::vector<double>{0.1,0.1,0.1}, 0.4);
  // bps.emplace_back(BeliefType::Bonkers, std::vector<double>{0,0,0}, 0.05);
  // bps.emplace_back(BeliefType::MoEObstacle, std::vector<double>{0,0,0}, 0.1);
  // bps.emplace_back(BeliefType::MoEObstacle, std::vector<double>{0.1,0.1,0.1}, 0.4);
  // bps.emplace_back(BeliefType::MoEBonkers, std::vector<double>{0,0,0}, 0.05);
  bps.emplace_back(BeliefType::ShapeCompletion);
  return bps;
}

void testAll(ros::NodeHandle &n) {
  for (int i = 0; i < 1; i++) {
    for (auto bp : getBeliefParams()) {
      for (const auto &scenario_factory : getScenarioFactories(bp)) {
        for (const auto &strategy_factory : getStrategyFactories()) {
          test(n, *scenario_factory(), *strategy_factory());
        }
      }
    }
  }
}

int main(int argc, char *argv[]) {
  icl_core::logging::initialize(argc, argv);
  ros::init(argc, argv, "graph_publisher");
  ros::NodeHandle n;

  ros::Duration(1.0).sleep();

  // test1(n);
  // test2(n);
  testAll(n);
}
