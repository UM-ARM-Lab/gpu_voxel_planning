#ifndef SCENARIO_TESTER_HPP
#define SCENARIO_TESTER_HPP

#include "scenarios/simulation_scenarios.hpp"
#include "scenarios/real_scenario.hpp"
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

        int num_path_attempts;
        

        SimulationScenarioTester(SimulationScenario &scenario, ros::NodeHandle &n) :
            scenario(scenario), n(n), ri(n), num_path_attempts(0)
        {
        }

        bool attemptPath(const std::vector<VictorRightArmConfig> &path);

        bool attemptStrategy(Strategy &strategy);

        std::string getName(const Strategy &strategy) const;
    };


    class RealScenarioTester
    {
    public:
        RealScenario &scenario;
        RosInterface ri;
        ros::NodeHandle &n;
        

        RealScenarioTester(RealScenario &scenario, ros::NodeHandle &n) :
            scenario(scenario), n(n), ri(n)
        {
        }

        bool attemptPath(const std::vector<VictorRightArmConfig> &path);

        bool attemptStrategy(Strategy &strategy);

        std::string getName(const Strategy &strategy) const;
    };
}

#endif
