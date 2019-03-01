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
#include "strategies/selective_densification_strategies.hpp"
#include "strategies/ompl_strategies.hpp"


using namespace GVP;




void test(ros::NodeHandle &n, SimulationScenario &scenario, Strategy &strategy)
{
    SimulationScenarioTester tester(scenario, n);
    tester.attemptStrategy(strategy);
}


std::vector<std::function<std::shared_ptr<SimulationScenario>(void)>> getScenarioFactories()
{
    std::vector<std::function<std::shared_ptr<SimulationScenario>(void)>> factories;
    factories.push_back([](){ return std::make_shared<TableWithBox>(true, true, true);});
    factories.push_back([](){ return std::make_shared<Bookshelf>(true);});
    factories.push_back([](){ return std::make_shared<SlottedWall>(true);});
    return factories;
}

std::vector<std::function<std::shared_ptr<Strategy>(void)>> getStrategyFactories()
{
    std::vector<std::function<std::shared_ptr<Strategy>(void)>> factories;
    //todo, load different versions based on precomputedness
    factories.push_back([](){
            return std::make_shared<OmniscientSDGraphSearch>(false);}); //not using precomputed
    factories.push_back([](){
            return std::make_shared<OmniscientSDGraphSearch>(true);}); //using precomputed
    factories.push_back([](){
            return std::make_shared<DenseGraphSearch>(false);}); //using precomputed
    factories.push_back([](){
            return std::make_shared<DenseGraphSearch>(true);}); //not precomputed
    for(int i=0; i<5; i++)
    {
        factories.push_back([](){ return std::make_shared<RRT_Strategy>();});
        factories.push_back([](){ return std::make_shared<BIT_Strategy>();});
    }
    return factories;
}


void testAll(ros::NodeHandle &n)
{
    for(auto scenario_factory:getScenarioFactories())
    {
        for(auto strategy_factory: getStrategyFactories())
        {
            PROFILE_REINITIALIZE(0,0);
            auto scenario_ptr = scenario_factory();
            auto strategy_ptr = strategy_factory();
            test(n, *scenario_ptr, *strategy_ptr);
            
            std::string filename = scenario_ptr->getName() + " " +
                strategy_ptr->getName() + " "+ arc_helpers::GetCurrentTimeAsString();
            PROFILE_WRITE_SUMMARY_FOR_ALL(filename);
            PROFILE_WRITE_ALL_FEWER_THAN(filename, 150);
        }
    }
}


void preparePrecomputed(ros::NodeHandle &n)
{
    for(auto scenario_factory:getScenarioFactories())
    {
        {
            OmniscientSDGraphSearch strat(true);
            strat.setMode(SelectiveDensificationStrategy::EdgeCheckMode::STORE);
            test(n, *scenario_factory(), strat);
            strat.saveToFile();
        }

        {
            DenseGraphSearch strat(true);
            strat.setMode(SelectiveDensificationStrategy::EdgeCheckMode::STORE);
            test(n, *scenario_factory(), strat);
            strat.saveToFile();
        }
    }

}



int main(int argc, char* argv[])
{
    icl_core::logging::initialize(argc, argv);
    ros::init(argc, argv, "graph_publisher");
    ros::NodeHandle n;

    ros::Duration(1.0).sleep();

    preparePrecomputed(n);
    testAll(n);

    
    // std::string filename = "sim_timing_" + arc_helpers::GetCurrentTimeAsString();

}
