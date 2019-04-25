#ifndef STRATEGIES_HPP
#define STRATEGIES_HPP

#include "victor_halton_roadmap.hpp"
#include "scenarios.hpp"
#include "path_utils_addons.hpp"
#include "gpu_voxel_rviz_visualization.hpp"

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
