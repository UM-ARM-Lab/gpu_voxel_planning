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
    while(!scenario.completed())
    {
        attemptPath(strategy.applyTo(scenario));
    }
    std::cout << "Reached Goal\n";
    return true;
}

