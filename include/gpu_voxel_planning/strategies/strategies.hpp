#ifndef GVP_STRATEGIES_HPP
#define GVP_STRATEGIES_HPP

#include "scenarios.hpp"
#include "ros_interface/gpu_voxel_rviz_visualization.hpp"

namespace GVP
{
    class Strategy
    {
    public:
        virtual Path applyTo(Scenario &scenario, GpuVoxelRvizVisualizer& viz) = 0;
        virtual std::string getName() const = 0;
    };


}

#endif
