#ifndef SCENARIO_TESTER_HPP
#define SCENARIO_TESTER_HPP

#include "scenarios.hpp"
#include "strategies.hpp"
#include "gpu_voxel_rviz_visualization.hpp"
#include <ros/ros.h>


namespace GVP
{
    class ScenarioTester
    {
    public:
        Scenario &scenario;
        GpuVoxelRvizVisualizer viz;
        ros::NodeHandle &n;
        

        ScenarioTester(Scenario &scenario, ros::NodeHandle &n) : scenario(scenario), n(n), viz(n)
        {
        }


        bool attemptPath(const std::vector<VictorRightArmConfig> &path)
        {
            for(const auto &c:path)
            {
                if(!scenario.getState().move(c, scenario.getTrueObstacles()))
                {
                    viz.vizScenario(scenario);
                    return false;
                }
                viz.vizScenario(scenario);
                ros::Duration(0.01).sleep();
            }
            return true;
        }



        bool attemptStrategy(Strategy &strategy)
        {
            while(!scenario.completed())
            {
                attemptPath(strategy.applyTo(scenario));
            }
            std::cout << "Reached Goal\n";
            return true;
        }
    };
}

#endif
