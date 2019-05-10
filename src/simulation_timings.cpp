#include "maps/prob_map.hpp"
#include <gpu_voxels/helpers/GeometryGeneration.h>
#include "common_names.hpp"
#include <arc_utilities/timing.hpp>
#include <arc_utilities/arc_helpers.hpp>

#include <ros/ros.h>
#include "ros_interface/gpu_voxel_rviz_visualization.hpp"
#include "robot_model.hpp"
#include "state.hpp"
#include "scenario_tester.hpp"
#include "strategies/graph_search_strategies.hpp"
#include "path_utils_addons.hpp"

using namespace GVP;



void test(ros::NodeHandle &n, SimulationScenario &scenario, Strategy &strategy)
{
    SimulationScenarioTester tester(scenario, n);
    tester.attemptStrategy(strategy);
}

void test1(ros::NodeHandle &n)
{
    TableWithBox scenario(true, false, false);
    OptimisticGraphSearch strat;
    test(n, scenario, strat);
}

void test2(ros::NodeHandle &n)
{
    TableWithBox scenario(true, true, false);
    ParetoCostGraphSearch strat(1.0);
    test(n, scenario, strat);
}

std::vector<std::function<std::shared_ptr<SimulationScenario>(void)>> getScenarioFactories()
{
    std::vector<std::function<std::shared_ptr<SimulationScenario>(void)>> factories;
    factories.push_back([](){ return std::make_shared<TableWithBox>(true, true, false);});
    factories.push_back([](){ return std::make_shared<TableWithBox>(true, false, false);});
    return factories;
}

std::vector<std::function<std::shared_ptr<Strategy>(void)>> getStrategyFactories()
{
    std::vector<std::function<std::shared_ptr<Strategy>(void)>> factories;
    factories.push_back([](){ return std::make_shared<OptimisticGraphSearch>();});
    factories.push_back([](){ return std::make_shared<ParetoCostGraphSearch>(1.0);});
    factories.push_back([](){ return std::make_shared<ParetoCostGraphSearch>(10.0);});
    return factories;
}


void testAll(ros::NodeHandle &n)
{
    for(auto scenario_factory:getScenarioFactories())
    {
        for(auto strategy_factory: getStrategyFactories())
        {
            test(n, *scenario_factory(), *strategy_factory());
        }
    }
}



int main(int argc, char* argv[])
{
    icl_core::logging::initialize(argc, argv);
    ros::init(argc, argv, "graph_publisher");
    ros::NodeHandle n;

    ros::Duration(1.0).sleep();

    // test1(n);
    // test2(n);
    testAll(n);

    
    std::string filename = "sim_timing_" + arc_helpers::GetCurrentTimeAsString();
    PROFILE_WRITE_SUMMARY_FOR_ALL(filename);
}
