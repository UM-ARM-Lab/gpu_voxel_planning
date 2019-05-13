#include "scenario_tester.hpp"



using namespace GVP;


/*******************************
 *  Simulation Scenario Tester
*******************************/
bool SimulationScenarioTester::attemptPath(const std::vector<VictorRightArmConfig> &path)
{
    for(const auto &c:path)
    {
        if(!scenario.getSimulationState().move(c, scenario.getTrueObstacles(), ri))
        {
            scenario.viz(ri.viz);
            return false;
        }
        PROFILE_START("Viz_scenario");
        scenario.viz(ri.viz);
        PROFILE_RECORD("Viz_scenario");
        // ros::Duration(0.01).sleep();
        ros::Duration(0.001).sleep();
    }
    return true;
}

bool SimulationScenarioTester::attemptStrategy(Strategy &strategy)
{
    scenario.viz(ri.viz);
    scenario.initFakeVictor(ri);
    try{
        scenario.validate();
    }
    catch(std::invalid_argument &e)
    {
        scenario.viz(ri.viz);
        throw e;
    }
    
    std::string name = getName(strategy);
    while(!scenario.completed())
    {
        PROFILE_START(name + " Planning Time");
        std::vector<VictorRightArmConfig> path;
        try
        {
            path = strategy.applyTo(scenario, ri.viz);
        }
        catch(std::runtime_error &e)
        {
            std::cout << "No path found\n";
            return false;
        }
        PROFILE_RECORD(name + " Planning Time");

        ri.viz.vizEEPath(path, "Path Found");
        // std::cout << "path found with " << path.size() << " verts\n";
        
        PROFILE_START(name + " Motion Time");
        attemptPath(path);
        PROFILE_RECORD(name + " Motion Time");

    }
    std::cout << "Reached Goal with cost " << scenario.getState().accumulated_cost << "\n";
    PROFILE_RECORD_DOUBLE("accumulated_cost", scenario.getState().accumulated_cost);
    return true;
}

std::string SimulationScenarioTester::getName(const Strategy &strategy) const
{
    return scenario.getName() + ", " +  strategy.getName();
}





/*******************************
 *    Real Scenario Tester
*******************************/
bool RealScenarioTester::attemptPath(const std::vector<VictorRightArmConfig> &path)
{
    for(const auto &c:path)
    {
        if(!scenario.getRealState().move(c, ri))
        {
            scenario.viz(ri.viz);
            return false;
        }
        PROFILE_START("Viz_scenario");
        scenario.viz(ri.viz);
        PROFILE_RECORD("Viz_scenario");
        // ros::Duration(0.01).sleep();
        ros::Duration(0.001).sleep();
    }
    return true;
}

bool RealScenarioTester::attemptStrategy(Strategy &strategy)
{
    std::cout << "Attempt Strategy\n";
    std::cout << "Viz Scenario\n";
    scenario.viz(ri.viz);
    std::cout << "Init Fake Victor\n";
    scenario.initFakeVictor(ri);
    try{
        std::cout << "Validate\n";
        scenario.validate();
    }
    catch(std::invalid_argument &e)
    {
        scenario.viz(ri.viz);
        throw e;
    }

    std::cout << "getname\n";
    std::string name = getName(strategy);
    while(!scenario.completed())
    {
        PROFILE_START(name + " Planning Time");
        std::vector<VictorRightArmConfig> path;
        try
        {
            std::cout << "plan\n";
            path = strategy.applyTo(scenario, ri.viz);
        }
        catch(std::runtime_error &e)
        {
            std::cout << "No path found\n";
            return false;
        }
        PROFILE_RECORD(name + " Planning Time");

        ri.viz.vizEEPath(path, "Path Found");
        // std::cout << "path found with " << path.size() << " verts\n";
        
        PROFILE_START(name + " Motion Time");
        attemptPath(path);
        PROFILE_RECORD(name + " Motion Time");

    }
    std::cout << "Reached Goal with cost " << scenario.getState().accumulated_cost << "\n";
    PROFILE_RECORD_DOUBLE("accumulated_cost", scenario.getState().accumulated_cost);
    return true;
}

std::string RealScenarioTester::getName(const Strategy &strategy) const
{
    return scenario.getName() + ", " +  strategy.getName();
}
