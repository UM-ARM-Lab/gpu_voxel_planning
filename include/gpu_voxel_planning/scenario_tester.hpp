#ifndef SCENARIO_TESTER_HPP
#define SCENARIO_TESTER_HPP

#include "scenarios/simulation_scenarios.hpp"
#include "strategies/strategies.hpp"
#include "ros_interface/ros_interface.hpp"
#include <ros/ros.h>


namespace GVP
{
    class SimulationScenarioTester
    {
    public:
        SimulationScenario &scenario;
        RosInterface ri;
        ros::NodeHandle &n;
        

        SimulationScenarioTester(SimulationScenario &scenario, ros::NodeHandle &n) :
            scenario(scenario), n(n), ri(n)
        {
        }

        bool attemptPath(const std::vector<VictorRightArmConfig> &path);

        bool attemptStrategy(Strategy &strategy);

        std::string getName(const Strategy &strategy) const;
    };
}

#endif
