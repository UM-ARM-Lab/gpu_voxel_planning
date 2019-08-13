#include "maps/prob_map.hpp"
#include <gpu_voxels/helpers/GeometryGeneration.h>
#include "common_names.hpp"
#include <arc_utilities/timing.hpp>
#include <arc_utilities/arc_helpers.hpp>

#include <ros/ros.h>
#include "ros_interface/gpu_voxel_rviz_visualization.hpp"
#include "robot/robot_model.hpp"
#include "state.hpp"
#include "scenario_tester.hpp"
#include "strategies/graph_search_strategies.hpp"
#include "path_utils_addons.hpp"
#include "strategies/selective_densification_strategies.hpp"
#include "strategies/ompl_strategies.hpp"


using namespace GVP;

std::vector<double> c_ps{100.0, 10.0, 1.0, 0.1, 0.01, 0.001, 0.0001, 0.00001, 0.0};
static const double chosen_cp = 1;
// std::vector<double> c_ps{1.0};



void test(ros::NodeHandle &n, SimulationScenario &scenario, Strategy &strategy)
{
    SimulationScenarioTester tester(scenario, n);
    std::cout << "Testing strat\n";
    tester.attemptStrategy(strategy);
    std::cout << "Finished test\n";
}


std::vector<std::function<std::shared_ptr<SimulationScenario>(void)>> getScenarioFactories()
{
    BeliefParams bp(BeliefType::Deterministic);
    std::vector<std::function<std::shared_ptr<SimulationScenario>(void)>> factories;
    factories.push_back([bp](){ return std::make_shared<TableWithBox>(bp, true, true, true);});
    // factories.push_back([bp](){ return std::make_shared<Bookshelf>(bp);});
    // factories.push_back([bp](){ return std::make_shared<SlottedWall>(bp);});
    return factories;
}

std::vector<std::function<std::shared_ptr<SelectiveDensificationStrategy>(void)>>
getPrecomputedStrategyFactories()
{
    std::vector<std::function<std::shared_ptr<SelectiveDensificationStrategy>(void)>> factories;
    // for(double c_p: c_ps)
    // {
    //     factories.push_back([c_p](){
    //             return std::make_shared<OmniscientSDGraphSearch>(true, c_p, 0);}); //using precomputed
    // }

    for(int i=0; i<10; i++)
    {
        factories.push_back([i](){
                return std::make_shared<OmniscientSDGraphSearch>(true, chosen_cp, i);}); //using precomputed
        // factories.push_back([i](){
        //         return std::make_shared<DenseGraphSearch>(true, i);}); //not precomputed
    }


    // factories.push_back([](){
    //         return std::make_shared<OmniscientSDGraphSearch>(true, chosen_cp, 1);}); //using precomputed
    // factories.push_back([i](){
    //         return std::make_shared<DenseGraphSearch>(true, i);}); //not precomputed


    return factories;
}

std::vector<std::function<std::shared_ptr<Strategy>(void)>> getStrategyFactories()
{
    std::vector<std::function<std::shared_ptr<Strategy>(void)>> factories;
    for(auto f: getPrecomputedStrategyFactories())
    {
        factories.push_back(f);
    }

    // for(double c_p: c_ps)
    // {
    //     factories.push_back([c_p](){
    //             return std::make_shared<OmniscientSDGraphSearch>(false, c_p, 0);}); //not using precomputed
    // }

    // for(int i=0; i<10; i++)
    // {
    //     factories.push_back([i](){
    //             return std::make_shared<OmniscientSDGraphSearch>(false, chosen_cp, i);}); //using precomputed
    //     factories.push_back([i](){
    //             return std::make_shared<DenseGraphSearch>(false, i);}); //not precomputed
    // }
                
    
    // for(int i=0; i<10; i++)
    // {
    //     factories.push_back([](){ return std::make_shared<RRT_Strategy>();});
    //     factories.push_back([](){ return std::make_shared<BIT_Strategy>();});
    // }
    

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
            
            std::string filename = "wip_" + scenario_ptr->getName() + " " +
                strategy_ptr->getName() + " "+ arc_helpers::GetCurrentTimeAsString();
            PROFILE_WRITE_SUMMARY_FOR_ALL(filename);
            PROFILE_WRITE_ALL_FEWER_THAN(filename, 150);
            PROFILE_PRINT_SUMMARY_FOR_ALL();
        }
    }
}


void preparePrecomputed(ros::NodeHandle &n)
{
    std::cout << "Precomputing swept volumes\n";
    for(auto scenario_factory:getScenarioFactories())
    {
        for(auto f: getPrecomputedStrategyFactories())
        {
            auto strat_ptr = f();
            strat_ptr->setMode(SelectiveDensificationStrategy::EdgeCheckMode::STORE);
            test(n, *scenario_factory(), *strat_ptr);
            strat_ptr->saveToFile();
        }
    }
}



int main(int argc, char* argv[])
{
    icl_core::logging::initialize(argc, argv);
    ros::init(argc, argv, "selective_densification_experiments");
    ros::NodeHandle n;

    if(VISUALIZE)
    {
        std::cout << "\n!! WARNING !!\n\n Visualiziation on, which means timings are not accurate\n";
    }

    // preparePrecomputed(n);
    testAll(n);
}
