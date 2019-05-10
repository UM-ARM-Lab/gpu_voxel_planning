#ifndef SCENARIO_TESTER_HPP
#define SCENARIO_TESTER_HPP

#include "scenarios.hpp"
#include "strategies/strategies.hpp"
#include "ros_interface/gpu_voxel_rviz_visualization.hpp"
#include <ros/ros.h>


namespace GVP
{
    class SimulationScenarioTester
    {
    public:
        SimulationScenario &scenario;
        GpuVoxelRvizVisualizer viz;
        ros::NodeHandle &n;
        

        SimulationScenarioTester(SimulationScenario &scenario, ros::NodeHandle &n) :
            scenario(scenario), n(n), viz(n)
        {
        }

        bool attemptPath(const std::vector<VictorRightArmConfig> &path);

        bool attemptStrategy(Strategy &strategy);

        std::string getName(const Strategy &strategy) const;
    };
}

#endif
