#ifndef MOCK_GPU_VOXEL_RVIZ_VISUALIZATION_HPP
#define MOCK_GPU_VOXEL_RVIZ_VISUALIZATION_HPP
#include "ros_interface/gpu_voxel_rviz_visualization.hpp"

namespace GVP
{
    class MockGpuVoxelRvizVisualizer : public GpuVoxelRvizVisualizer
    {
    public:
        MockGpuVoxelRvizVisualizer() {}

        void vizEEPosition(const std::vector<double> config) override
        {}

        void vizEEPath(const std::vector<GVP::VictorRightArmConfig> path_config,
                       std::string path_name) override
        {}

        void vizGrid(const DenseGrid& grid, const std::string& name, const std_msgs::ColorRGBA& color)
            const override
        {}

        void vizChs(const std::vector<DenseGrid> &chss, const std::string &ns = "chs")
            const override
        {}

                    
    };
}



#endif //MOCK_GPU_VOXEL_RVIZ_VISUALIZATION_HPP
