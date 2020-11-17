#ifndef GVP_STRATEGIES_HPP
#define GVP_STRATEGIES_HPP

#include "gpu_voxel_planning/scenarios/scenarios.hpp"
#include "gpu_voxel_planning/ros_interface/gpu_voxel_rviz_visualization.hpp"
#include "gpu_voxel_planning/path_utils_addons.hpp"

namespace GVP
{
    class Strategy
    {
    public:
        virtual GVP::Path applyTo(Scenario &scenario, GpuVoxelRvizVisualizer& viz) = 0;
        virtual std::string getName() const = 0;
    };


}

#endif
