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
        ros::Duration(0.01).sleep();
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
        const std::vector<VictorRightArmConfig> path = strategy.applyTo(scenario);
        PROFILE_RECORD(name + " Planning Time");
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
