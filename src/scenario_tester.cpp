#include "scenario_tester.hpp"



using namespace GVP;


bool SimulationScenarioTester::attemptPath(const std::vector<VictorRightArmConfig> &path)
{
    for(const auto &c:path)
    {
        if(!scenario.getSimulationState().move(c, scenario.getTrueObstacles()))
        {
            viz.vizScenario(scenario);
            return false;
        }
        viz.vizScenario(scenario);
        // ros::Duration(0.01).sleep();
        ros::Duration(0.001).sleep();
    }
    return true;
}

bool SimulationScenarioTester::attemptStrategy(Strategy &strategy)
{
    viz.vizScenario(scenario);
    try{
        scenario.validate();
    }
    catch(std::invalid_argument &e)
    {
        viz.vizScenario(scenario);
        throw e;
    }
    
    std::string name = getName(strategy);
    while(!scenario.completed())
    {
        PROFILE_START(name + " Planning Time");
        std::vector<VictorRightArmConfig> path;
        try
        {
            path = strategy.applyTo(scenario);
        }
        catch(std::runtime_error &e)
        {
            std::cout << "No path found\n";
            return false;
        }
        PROFILE_RECORD(name + " Planning Time");

        viz.vizEEPath(path, "Path Found");
        std::cout << "path found with " << path.size() << " verts\n";
        
        PROFILE_START(name + " Motion Time");
        attemptPath(path);
        PROFILE_RECORD(name + " Motion Time");

    }
    std::cout << "Reached Goal\n";
    return true;
}

std::string SimulationScenarioTester::getName(const Strategy &strategy) const
{
    return scenario.getName() + ", " +  strategy.getName();
}
