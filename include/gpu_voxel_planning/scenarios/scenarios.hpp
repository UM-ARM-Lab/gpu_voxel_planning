#ifndef GVP_SCENARIOS_HPP
#define GVP_SCENARIOS_HPP


#include "gpu_voxel_planning/state.hpp"
#include "gpu_voxel_planning/ros_interface/gpu_voxel_rviz_visualization.hpp"

namespace GVP
{
    class Scenario
    {
    public:
        VictorRightArm victor;
        robot::JointValueMap goal_config;
        virtual State& getState() = 0;
        virtual const State& getState() const = 0;
        virtual std::string getName() const = 0;
        
        virtual bool completed() const
        {
            return VictorRightArmConfig(getState().current_config) == VictorRightArmConfig(goal_config);
        }

        virtual void viz(const GpuVoxelRvizVisualizer& viz)
        {
        }

        Scenario(){}
    };
}

#endif
